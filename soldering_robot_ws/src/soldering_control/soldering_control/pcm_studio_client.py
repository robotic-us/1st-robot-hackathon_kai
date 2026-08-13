"""Maintenance access to the PCM's USB Studio SDO endpoint.

This is deliberately separate from the normal ``phorce`` motion API.  Its
default command is read-only.  Persistent zero-setting requires an explicit
axis confirmation and is rejected unless PCM reports a parked, servo-off,
motion-idle, online axis with an open settings-write window.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import fcntl
import json
import math
import os
import secrets
import select
import struct
import termios
import time
from typing import Optional
import zlib


STUDIO_DOP_NODE_ID = 0x51
MSG_SDO_REQ = 1
MSG_SDO_RSP = 2

OD_MOTION = 0x5F00
OD_ZERO = 0x5F01
OD_POSTURE = 0x5F03
OD_AXIS = 0x5F07
OD_SERVO = 0x5F08
OD_SETTINGS = 0x5F0B
OD_SESSION = 0x5F0C
OD_ZERO_SNAPSHOT = 0x5F0D

SUB_ZERO_SET_MASK = 0x0D
SUB_BOOT_POSTURE = 0x01
SUB_MOTION_PLAY = 0x01
SUB_MOTION_RELOAD = 0x02
SUB_RELOAD_RESULT = 0x03
SUB_RELOAD_BUSY = 0x04
SUB_AXIS_CONFIGURED = 0x01
SUB_AXIS_LIVE = 0x03
SUB_AXIS_LATCHED = 0x04
SUB_SERVO_SET = 0x01
SUB_SERVO_STATE = 0x02
SUB_MOTION_BUSY = 0x05
SUB_WRITE_WINDOW = 0x03
SUB_WRITE_RESULT = 0x04
SUB_ZERO_SNAPSHOT_HEADER = 0x01
SUB_ZERO_SNAPSHOT_DATA = 0x02

WINDOW_OPEN = 1 << 0
WINDOW_REASONS = {
    1 << 1: "not_parking",
    1 << 2: "servo_on",
    1 << 3: "teaching",
    1 << 4: "motion_running",
    1 << 5: "usb_transition",
    1 << 6: "sd_owned_by_host",
    1 << 7: "flash_save_in_progress",
    1 << 8: "motion_reload",
    1 << 9: "boot_error",
}

SETTINGS_RESULT_NAMES = {
    1: "applied",
    0: "pending",
    -1: "rejected_range",
    -2: "rejected_window",
    -3: "rejected_axis_offline",
}
SETTINGS_OBJ_AXIS = 2
SETTINGS_OBJ_ZERO = 7
SAVED_BIT_AXIS = 1 << 4
RELOAD_RESULT_NAMES = {
    1: "success",
    0: "pending",
    -1: "rejected_motion_running",
    -2: "sd_acquire_timeout",
    -3: "sd_mount_failed",
    -4: "csv_parse_or_rescan_failed",
}
SERVO_STATE_NAMES = {
    0: "off_parked",
    1: "on",
    2: "transition",
    3: "unavailable",
    4: "unconfigured",
}

SESSION_GUARD = 827147088
SESSION_MODE_STORAGE = 2
SESSION_MODE_LIVE = 4
SESSION_PHASE_READY = 5
SESSION_PHASE_REJECTED = 6
SESSION_PHASE_FAULT = 255
SESSION_FLAG_FW_OWNS_SD = 1 << 2


class ProtocolError(RuntimeError):
    """The peer returned an invalid frame or SDO response."""


class SdoAbort(RuntimeError):
    """PCM rejected an SDO operation."""

    def __init__(self, index: int, subindex: int, code: int):
        self.index = index
        self.subindex = subindex
        self.code = code
        super().__init__(
            f"SDO abort 0x{code:08X} at 0x{index:04X}:{subindex:02X}"
        )


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE used by AGR DOP (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (
                crc << 1
            ) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray(b"\x00")
    code_index = 0
    code = 1
    for value in data:
        if value == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(value)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        index += 1
        end = index + code - 1
        if end > len(data):
            raise ProtocolError("truncated COBS frame")
        out.extend(data[index:end])
        index = end
        if code < 0xFF and index < len(data):
            out.append(0)
    return bytes(out)


def build_dop_frame(msg_type: int, node_id: int, sequence: int,
                    payload: bytes) -> bytes:
    raw = struct.pack(
        "<BBH", msg_type, node_id & 0x7F, sequence & 0xFFFF
    ) + payload
    raw += struct.pack("<H", crc16_ccitt(raw))
    return cobs_encode(raw) + b"\x00"


@dataclass(frozen=True)
class DopFrame:
    msg_type: int
    node_id: int
    sequence: int
    payload: bytes


def parse_dop_frame(encoded: bytes) -> DopFrame:
    raw = cobs_decode(encoded)
    if len(raw) < 6:
        raise ProtocolError("DOP frame shorter than header plus CRC")
    body, received_crc = raw[:-2], struct.unpack("<H", raw[-2:])[0]
    if crc16_ccitt(body) != received_crc:
        raise ProtocolError("DOP CRC mismatch")
    msg_type, node_id, sequence = struct.unpack_from("<BBH", body)
    return DopFrame(msg_type, node_id, sequence, body[4:])


@dataclass(frozen=True)
class SdoResponse:
    index: int
    subindex: int
    data: bytes = b""
    abort_code: int = 0
    is_write_ack: bool = False


def parse_sdo_response(payload: bytes) -> SdoResponse:
    if len(payload) < 4:
        raise ProtocolError("SDO response is shorter than four bytes")
    command, index, subindex = struct.unpack_from("<BHB", payload)
    remainder = payload[4:]
    if command & 0xE0 == 0x80:
        code = struct.unpack_from("<I", remainder.ljust(4, b"\x00"))[0]
        return SdoResponse(index, subindex, abort_code=code)
    if command & 0xE0 == 0x60:
        return SdoResponse(index, subindex, is_write_ack=True)
    if command & 0x03 == 0x03:  # legacy expedited upload
        unused = (command >> 2) & 0x03
        return SdoResponse(index, subindex, remainder[:4 - unused])
    if len(remainder) < 2:
        raise ProtocolError("explicit-length SDO response has no length")
    size = struct.unpack_from("<H", remainder)[0]
    if size > len(remainder) - 2:
        raise ProtocolError("explicit-length SDO response is truncated")
    return SdoResponse(index, subindex, remainder[2:2 + size])


class LinuxCdcPort:
    """Small stdlib-only Linux USB-CDC transport (CDC ignores baud rate)."""

    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attributes = termios.tcgetattr(self.fd)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attributes[3] = 0
        baud = getattr(termios, "B921600", termios.B115200)
        attributes[4] = baud
        attributes[5] = baud
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
        # pyserial asserts these on open.  PCM's LIVE session treats DTR as the
        # CDC link-presence signal, so a plain termios open must do it too.
        modem_bits = termios.TIOCM_DTR | termios.TIOCM_RTS
        try:
            fcntl.ioctl(
                self.fd, termios.TIOCMBIS, struct.pack("I", modem_bits)
            )
        except OSError:
            # Some CDC implementations do not expose modem control.  They
            # also do not gate traffic on it, so continuing is appropriate.
            pass
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def write_all(self, data: bytes, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        view = memoryview(data)
        while view:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"write timeout on {self.path}")
            _, writable, _ = select.select([], [self.fd], [], remaining)
            if not writable:
                continue
            written = os.write(self.fd, view)
            view = view[written:]

    def read_some(self, timeout: float) -> bytes:
        readable, _, _ = select.select([self.fd], [], [], max(timeout, 0.0))
        return os.read(self.fd, 8192) if readable else b""

    def __enter__(self) -> "LinuxCdcPort":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()


class PcmStudioClient:
    def __init__(self, port: LinuxCdcPort, node_id: int = STUDIO_DOP_NODE_ID,
                 timeout: float = 1.5):
        self.port = port
        self.node_id = node_id
        self.timeout = timeout
        self.sequence = 0
        self.rx_buffer = bytearray()

    def _request(
        self, index: int, subindex: int, data: Optional[bytes]
    ) -> bytes:
        if data is None:
            sdo = struct.pack("<BHB", 0x40, index, subindex)
        else:
            if not 1 <= len(data) <= 60:
                raise ValueError("SDO write data must contain 1..60 bytes")
            sdo = struct.pack("<BHBH", 0x21, index, subindex, len(data)) + data
        frame = build_dop_frame(
            MSG_SDO_REQ, self.node_id, self.sequence, sdo
        )
        self.sequence = (self.sequence + 1) & 0xFFFF
        self.port.write_all(frame, self.timeout)

        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            while b"\x00" in self.rx_buffer:
                encoded, _, tail = self.rx_buffer.partition(b"\x00")
                self.rx_buffer = bytearray(tail)
                if not encoded:
                    continue
                try:
                    dop = parse_dop_frame(bytes(encoded))
                except ProtocolError:
                    continue
                if dop.msg_type != MSG_SDO_RSP or dop.node_id != self.node_id:
                    continue
                response = parse_sdo_response(dop.payload)
                if (response.index, response.subindex) != (index, subindex):
                    continue
                if response.abort_code:
                    raise SdoAbort(index, subindex, response.abort_code)
                if data is not None and not response.is_write_ack:
                    raise ProtocolError("expected SDO write acknowledgement")
                return response.data
            chunk = self.port.read_some(deadline - time.monotonic())
            if chunk:
                self.rx_buffer.extend(chunk)
                if len(self.rx_buffer) > 65536:
                    del self.rx_buffer[:-4096]
        raise TimeoutError(f"no response from 0x{index:04X}:{subindex:02X}")

    def read(self, index: int, subindex: int) -> bytes:
        return self._request(index, subindex, None)

    def write(self, index: int, subindex: int, data: bytes) -> None:
        self._request(index, subindex, data)

    def read_u8(self, index: int, subindex: int) -> int:
        data = self.read(index, subindex)
        if len(data) != 1:
            raise ProtocolError(f"expected u8, received {len(data)} bytes")
        return data[0]

    def read_i8(self, index: int, subindex: int) -> int:
        data = self.read(index, subindex)
        if len(data) != 1:
            raise ProtocolError(f"expected i8, received {len(data)} bytes")
        return struct.unpack("<b", data)[0]

    def read_u16(self, index: int, subindex: int) -> int:
        data = self.read(index, subindex)
        if len(data) != 2:
            raise ProtocolError(f"expected u16, received {len(data)} bytes")
        return struct.unpack("<H", data)[0]

    def read_u32(self, index: int, subindex: int) -> int:
        data = self.read(index, subindex)
        if len(data) != 4:
            raise ProtocolError(f"expected u32, received {len(data)} bytes")
        return struct.unpack("<I", data)[0]


def decode_write_result(raw: int) -> dict[str, object]:
    result = struct.unpack("<b", bytes([raw & 0xFF]))[0]
    return {
        "raw": raw,
        "result": result,
        "result_name": SETTINGS_RESULT_NAMES.get(result, "unknown"),
        "object": (raw >> 8) & 0xFF,
        "sequence": (raw >> 16) & 0xFFFF,
    }


def axis_list(mask: int) -> list[int]:
    return [axis for axis in range(12) if mask & (1 << axis)]


def read_zero_snapshot(client: PcmStudioClient) -> dict[str, object]:
    """Read one generation-stable, CRC-checked persistent zero snapshot."""
    header_layout = struct.Struct("<BBHIHHI")
    first_raw = client.read(OD_ZERO_SNAPSHOT, SUB_ZERO_SNAPSHOT_HEADER)
    data = client.read(OD_ZERO_SNAPSHOT, SUB_ZERO_SNAPSHOT_DATA)
    second_raw = client.read(OD_ZERO_SNAPSHOT, SUB_ZERO_SNAPSHOT_HEADER)
    if len(first_raw) != header_layout.size or len(second_raw) != header_layout.size:
        raise ProtocolError("unexpected zero snapshot header length")
    if first_raw != second_raw:
        raise RuntimeError("zero snapshot changed while it was being read")
    schema, count, known, generation, total_len, flags, expected_crc = (
        header_layout.unpack(first_raw)
    )
    if schema != 1 or count != 12 or total_len != 48 or len(data) != 48:
        raise ProtocolError("unsupported zero snapshot layout")
    actual_crc = zlib.crc32(data) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ProtocolError(
            f"zero snapshot CRC mismatch: 0x{actual_crc:08X} != "
            f"0x{expected_crc:08X}"
        )
    offsets = list(struct.unpack("<12f", data))
    if any(not math.isfinite(value) for value in offsets):
        raise ProtocolError("zero snapshot contains a non-finite value")
    return {
        "schema_revision": schema,
        "axis_count": count,
        "known_mask": known,
        "known_axes": axis_list(known),
        "generation": generation,
        "flags": flags,
        "crc32": expected_crc,
        "offsets_rad": offsets,
    }


def window_reasons(mask: int) -> list[str]:
    return [name for bit, name in WINDOW_REASONS.items() if mask & bit]


def ensure_live_session(client: PcmStudioClient,
                        timeout: float = 15.0) -> dict[str, object]:
    """Hand the safely-unmounted USB SD volume back to PCM.

    The same CDC handle must stay open after LIVE_BEGIN.  Some PCM firmware
    revisions stop responding if the host closes and reopens CDC in LIVE mode.
    """
    hello_data = client.read(OD_SESSION, 1)
    status_data = client.read(OD_SESSION, 3)
    if len(hello_data) != 12 or len(status_data) != 28:
        raise ProtocolError("unexpected PCM session ABI length")
    hello = struct.unpack("<HBBII", hello_data)
    status = struct.unpack("<HBBBBHIIIII", status_data)
    if hello[:3] != (1, 1, 1) or status[0] != 1:
        raise ProtocolError("unsupported PCM session ABI revision")
    if (
        status[1] == SESSION_MODE_LIVE
        and status[5] & SESSION_FLAG_FW_OWNS_SD
    ):
        return {
            "already_live": True,
            "mode": status[1],
            "phase": status[2],
            "flags": status[5],
            "media_generation": status[10],
        }
    if status[1] != SESSION_MODE_STORAGE:
        raise RuntimeError(
            f"PCM is not in attachable storage mode (mode={status[1]})"
        )

    # Studio seeds this monotonic command token from Unix time.  PCM retains a
    # last-seen token that is not necessarily exposed by generation_echo after
    # reconnect, so a small local counter can be rejected as stale (reason 4).
    wall_clock_generation = int(time.time()) & 0xFFFFFFFF
    generation = (
        max(status[7] + 1, status[10] + 1, wall_clock_generation)
        & 0xFFFFFFFF
    ) or 1
    nonce = secrets.randbits(64) or 1
    nonce_low = nonce & 0xFFFFFFFF
    nonce_high = nonce >> 32
    payload = struct.pack(
        "<HBBIIIII",
        1,  # ABI revision
        1,  # LIVE_BEGIN
        1,  # HOST_RELEASE_CONFIRMED
        hello[4],
        generation,
        nonce_low,
        nonce_high,
        SESSION_GUARD,
    )
    client.write(OD_SESSION, 2, payload)
    deadline = time.monotonic() + timeout
    last = status
    while time.monotonic() < deadline:
        data = client.read(OD_SESSION, 3)
        if len(data) != 28:
            raise ProtocolError("unexpected PCM session status length")
        last = struct.unpack("<HBBBBHIIIII", data)
        matches = (
            last[0] == 1
            and last[3] == 1
            and last[6] == hello[4]
            and last[7] == generation
            and last[8] == nonce_low
            and last[9] == nonce_high
        )
        if matches and last[2] == SESSION_PHASE_READY:
            if (
                last[1] != SESSION_MODE_LIVE
                or not last[5] & SESSION_FLAG_FW_OWNS_SD
            ):
                raise RuntimeError("PCM LIVE_READY flags violate the ABI")
            return {
                "already_live": False,
                "mode": last[1],
                "phase": last[2],
                "flags": last[5],
                "media_generation": last[10],
            }
        if matches and last[2] in (
            SESSION_PHASE_REJECTED,
            SESSION_PHASE_FAULT,
        ):
            raise RuntimeError(
                f"PCM rejected LIVE_BEGIN (reason={last[4]})"
            )
        time.sleep(0.15)
    raise TimeoutError(f"PCM LIVE_BEGIN timed out; last_status={last}")


def read_status(client: PcmStudioClient) -> dict[str, object]:
    configured = client.read_u16(OD_AXIS, SUB_AXIS_CONFIGURED)
    live = client.read_u16(OD_AXIS, SUB_AXIS_LIVE)
    latched = client.read_u16(OD_AXIS, SUB_AXIS_LATCHED)
    zero_set = client.read_u16(OD_ZERO, SUB_ZERO_SET_MASK)
    servo = client.read_u8(OD_SERVO, SUB_SERVO_STATE)
    motion_busy = client.read_u8(OD_MOTION, SUB_MOTION_BUSY)
    window = client.read_u16(OD_SETTINGS, SUB_WRITE_WINDOW)
    write_result = client.read_u32(OD_SETTINGS, SUB_WRITE_RESULT)
    return {
        "configured_mask": configured,
        "configured_axes": axis_list(configured),
        "live_mask": live,
        "live_axes": axis_list(live),
        "latched_mask": latched,
        "latched_axes": axis_list(latched),
        "zero_set_mask": zero_set,
        "zero_set_axes": axis_list(zero_set),
        "servo_state": servo,
        "servo_state_name": SERVO_STATE_NAMES.get(servo, "unknown"),
        "motion_busy": bool(motion_busy),
        "write_window": window,
        "write_window_open": bool(window & WINDOW_OPEN),
        "write_window_reasons": window_reasons(window),
        "last_write": decode_write_result(write_result),
    }


def save_detected_axis_config(
    client: PcmStudioClient,
    expected_live_mask: int,
    apply_timeout: float = 8.0,
) -> dict[str, object]:
    """Persist exactly the axes PCM currently detects, as Phorce Studio does."""
    if not 0 < expected_live_mask < (1 << 12):
        raise ValueError("expected live mask must use bits 0..11 and be non-zero")

    before = wait_for_rebase_window(client, timeout=apply_timeout)
    live_mask = int(before["live_mask"])
    if live_mask != expected_live_mask:
        raise RuntimeError(
            "live axis mask changed: expected "
            f"0x{expected_live_mask:03X}, observed 0x{live_mask:03X}"
        )

    baseline_sequence = int(before["last_write"]["sequence"])
    client.write(
        OD_AXIS, SUB_AXIS_CONFIGURED, struct.pack("<H", live_mask)
    )

    deadline = time.monotonic() + apply_timeout
    observed: Optional[dict[str, object]] = None
    while time.monotonic() < deadline:
        try:
            raw = client.read_u32(OD_SETTINGS, SUB_WRITE_RESULT)
        except TimeoutError:
            continue
        result = decode_write_result(raw)
        if (
            result["sequence"] != baseline_sequence
            and result["object"] == SETTINGS_OBJ_AXIS
        ):
            observed = result
            if result["result"] != 0:
                break
        time.sleep(0.1)
    if observed is None:
        raise TimeoutError("PCM did not publish an axis-config processing result")
    if observed["result"] != 1:
        raise RuntimeError(
            "PCM rejected axis configuration: "
            + str(observed["result_name"])
        )

    persisted = wait_for_rebase_window(client, timeout=apply_timeout)
    configured_mask = int(persisted["configured_mask"])
    if configured_mask != live_mask:
        raise RuntimeError(
            "axis configuration read-back mismatch: wrote "
            f"0x{live_mask:03X}, read 0x{configured_mask:03X}"
        )
    saved_mask = client.read_u16(OD_SETTINGS, 1)
    if not saved_mask & SAVED_BIT_AXIS:
        raise RuntimeError("axis configuration applied but saved bit is not set")
    return {
        "configured_mask": configured_mask,
        "configured_axes": axis_list(configured_mask),
        "live_mask": int(persisted["live_mask"]),
        "live_axes": list(persisted["live_axes"]),
        "saved_settings_mask": saved_mask,
        "write_result": observed,
        "servo_left_off": True,
        "motion_executed": False,
    }


def set_current_as_zero(client: PcmStudioClient, axis: int,
                        apply_timeout: float = 8.0) -> dict[str, object]:
    if not 0 <= axis < 12:
        raise ValueError("axis must be in 0..11")
    before = read_status(client)
    failures = []
    if axis not in before["live_axes"]:
        failures.append(f"axis {axis} is not online")
    if before["servo_state"] != 0:
        failures.append("servo is not OFF/parked")
    if before["motion_busy"]:
        failures.append("motion is running")
    if not before["write_window_open"]:
        reasons = ", ".join(before["write_window_reasons"]) or "unknown"
        failures.append(f"settings write window is closed ({reasons})")
    if failures:
        raise RuntimeError("unsafe zero-set state: " + "; ".join(failures))

    baseline_sequence = int(before["last_write"]["sequence"])
    client.write(OD_ZERO, axis + 1, struct.pack("<f", 0.0))

    deadline = time.monotonic() + apply_timeout
    observed: Optional[dict[str, object]] = None
    while time.monotonic() < deadline:
        try:
            raw = client.read_u32(OD_SETTINGS, SUB_WRITE_RESULT)
        except TimeoutError:
            # Flash persistence can briefly suppress Studio SDO replies.
            # Keep the operation bounded by the overall apply deadline.
            continue
        result = decode_write_result(raw)
        if (
            result["sequence"] != baseline_sequence
            and result["object"] == SETTINGS_OBJ_ZERO
        ):
            observed = result
            if result["result"] != 0:
                break
        time.sleep(0.1)
    if observed is None:
        raise TimeoutError("PCM did not publish a zero-set processing result")
    if observed["result"] != 1:
        raise RuntimeError(
            "PCM rejected zero-set: " + str(observed["result_name"])
        )

    zero_mask = client.read_u16(OD_ZERO, SUB_ZERO_SET_MASK)
    if not zero_mask & (1 << axis):
        raise RuntimeError(
            "zero-set applied but its persistent bitmap is not set"
        )
    return {
        "axis": axis,
        "target_rad": 0.0,
        "write_result": observed,
        "zero_set_mask": zero_mask,
        "zero_set_axes": axis_list(zero_mask),
    }


def wait_for_rebase_window(client: PcmStudioClient,
                           timeout: float = 8.0) -> dict[str, object]:
    deadline = time.monotonic() + timeout
    last_status: Optional[dict[str, object]] = None
    while time.monotonic() < deadline:
        try:
            last_status = read_status(client)
        except TimeoutError:
            # Flash persistence and USB ownership transitions can briefly
            # suppress Studio SDO replies.  Keep the overall wait bounded.
            time.sleep(0.15)
            continue
        if (
            last_status["servo_state"] == 0
            and not last_status["motion_busy"]
            # Some firmware reports OPEN together with a transient reason
            # bit (notably flash_save_in_progress).  Only mask==OPEN is a
            # genuinely writable window for the next persistent zero.
            and last_status["write_window"] == WINDOW_OPEN
        ):
            return last_status
        time.sleep(0.15)
    reasons = [] if last_status is None else last_status["write_window_reasons"]
    raise TimeoutError(
        "relative-rebase window did not open"
        + (f" ({', '.join(reasons)})" if reasons else "")
    )


def stop_and_servo_off_for_rebase(
    client: PcmStudioClient, allow_interrupt_motion: bool,
    allow_torque_off: bool,
) -> dict[str, object]:
    before = read_status(client)
    if before["motion_busy"] and not allow_interrupt_motion:
        raise RuntimeError(
            "motion is active; wait for completion or explicitly pass "
            "--allow-interrupt-motion"
        )
    if before["servo_state"] != 0 and not allow_torque_off:
        raise RuntimeError(
            "servo is active; explicitly pass --allow-torque-off"
        )

    # Motion ID 0 is only an interruption command.  Sending it while PCM is
    # already idle can leave the firmware's stop latch asserted and prevent a
    # same-session software re-arm after changing the user zero.
    if before["motion_busy"]:
        client.write(OD_MOTION, SUB_MOTION_PLAY, struct.pack("<H", 0))
    if before["servo_state"] != 0:
        client.write(OD_SERVO, SUB_SERVO_SET, b"\x00")
    return wait_for_rebase_window(client)


def reload_motion_catalog(client: PcmStudioClient, slot: int,
                          timeout: float = 16.0) -> dict[str, object]:
    if not 0 <= slot <= 50:
        raise ValueError("reload slot must be in 0..50")
    client.write(
        OD_MOTION, SUB_MOTION_RELOAD, struct.pack("<H", slot)
    )
    deadline = time.monotonic() + timeout
    saw_busy = False
    result = 0
    while time.monotonic() < deadline:
        busy = client.read_u8(OD_MOTION, SUB_RELOAD_BUSY)
        result = client.read_i8(OD_MOTION, SUB_RELOAD_RESULT)
        saw_busy = saw_busy or bool(busy)
        if not busy and result != 0:
            break
        time.sleep(0.15)
    else:
        raise TimeoutError("PCM motion catalog reload did not finish")
    if result != 1:
        raise RuntimeError(
            "motion reload failed: "
            + RELOAD_RESULT_NAMES.get(result, f"unknown_{result}")
        )
    return {
        "requested_slot": slot,
        "rescanned_all_slots": True,
        "saw_busy": saw_busy,
        "result": result,
        "result_name": RELOAD_RESULT_NAMES[result],
    }


def relative_rebase(
    client: PcmStudioClient,
    axes: list[int],
    allow_interrupt_motion: bool,
    allow_torque_off: bool,
    arm_after: bool,
) -> dict[str, object]:
    if not axes:
        raise ValueError("at least one axis is required")
    if len(set(axes)) != len(axes):
        raise ValueError("axes must not contain duplicates")
    ready = stop_and_servo_off_for_rebase(
        client, allow_interrupt_motion, allow_torque_off
    )
    rebased = []
    for axis in axes:
        rebased.append(set_current_as_zero(client, axis))
    # SETTINGS_RESULT=applied means the request was accepted, but PCM may
    # still be persisting Flash.  Do not arm or start another operation until
    # the transient flash_save_in_progress reason has fully cleared.
    persisted = wait_for_rebase_window(client)
    arm_result = arm_from_zero_pose(client, axes) if arm_after else None
    return {
        "mode": "persistent_flash_relative_rebase",
        "ready_status": ready,
        "rebased": rebased,
        "persisted_status": persisted,
        "csv_touched": False,
        "motion_catalog_reloaded": False,
        "arm": arm_result,
        "servo_left_off": not arm_after,
    }


def verify_zero_boot_pose(client: PcmStudioClient,
                          axes: list[int]) -> dict[str, object]:
    data = client.read(OD_POSTURE, SUB_BOOT_POSTURE)
    if not data:
        raise RuntimeError("automatic arming requires a saved boot pose")
    option = data[0]
    # Option 2 is the firmware's built-in user-zero pose and therefore needs
    # no angle payload.  Current firmware still reads it back as 49 bytes.
    if option == 2:
        return {
            "option": option,
            "meaning": "user_zero",
            "verified_axes": axes,
        }
    if len(data) != 49:
        raise RuntimeError(
            "automatic arming requires boot-pose option 2, or option 3 "
            "with 12 angles"
        )
    angles = list(struct.unpack("<12f", data[1:]))
    if option != 3:
        raise RuntimeError(
            "automatic arming blocked: boot-pose option must be 2 or 3"
        )
    nonzero = [axis for axis in axes if abs(angles[axis]) > 1e-4]
    if nonzero:
        raise RuntimeError(
            "automatic arming blocked: boot pose is not 0 rad on axes "
            + ", ".join(map(str, nonzero))
        )
    return {"option": option, "angles_rad": angles, "verified_axes": axes}


def arm_from_zero_pose(client: PcmStudioClient, axes: list[int],
                       timeout: float = 10.0) -> dict[str, object]:
    boot_pose = verify_zero_boot_pose(client, axes)
    client.write(OD_SERVO, SUB_SERVO_SET, b"\x01")
    deadline = time.monotonic() + timeout
    states = []
    while time.monotonic() < deadline:
        # Boot-pose option 2 may occupy PCM for roughly three seconds while
        # servo activation settles.  CDC can temporarily omit SDO responses;
        # one per-request timeout is not an arming failure as long as the
        # overall deadline has not expired.
        try:
            state = client.read_u8(OD_SERVO, SUB_SERVO_STATE)
        except TimeoutError:
            continue
        if not states or states[-1] != state:
            states.append(state)
        if state == 1:
            return {
                "software_armed": True,
                "boot_pose": boot_pose,
                "observed_states": states,
            }
        if state in (3, 4):
            raise RuntimeError(
                "software arming rejected: "
                + SERVO_STATE_NAMES.get(state, str(state))
            )
        time.sleep(0.2)
    raise TimeoutError(
        "software arming did not reach servo ON; states="
        + repr(states)
    )


def _format_json(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read PCM Studio status or persist current position as zero"
        )
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--timeout", type=float, default=1.5)
    parser.add_argument(
        "--attach-live",
        action="store_true",
        help="return an already-unmounted USB SD volume to PCM first",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("status", help="read-only PCM maintenance status")
    axes = commands.add_parser(
        "save-detected-axis-config",
        help="persist PCM's currently detected axes exactly as Studio does",
    )
    axes.add_argument(
        "--expect-live-mask", type=lambda value: int(value, 0), required=True
    )
    axes.add_argument(
        "--confirm-live-mask", type=lambda value: int(value, 0), required=True
    )
    axes.add_argument(
        "--i-understand-axis-config-is-persistent",
        action="store_true",
        help="required acknowledgement of the persistent axis configuration",
    )
    zero = commands.add_parser(
        "set-current-as-zero", help="persist the current pose as 0 rad"
    )
    zero.add_argument("--axis", type=int, required=True)
    zero.add_argument(
        "--confirm-axis", type=int, required=True,
        help="must exactly match --axis",
    )
    zero.add_argument(
        "--i-understand-motion-references-will-shift",
        action="store_true",
        help="required acknowledgement of persistent coordinate change",
    )
    rebase = commands.add_parser(
        "relative-rebase",
        help="stop, torque off and persist current pose as relative origin",
    )
    rebase.add_argument("--axes", type=int, nargs="+", required=True)
    rebase.add_argument(
        "--confirm-axes", type=int, nargs="+", required=True,
        help="must exactly match --axes",
    )
    rebase.add_argument(
        "--allow-torque-off", action="store_true",
        help="allow software servo OFF before rebasing",
    )
    rebase.add_argument(
        "--allow-interrupt-motion", action="store_true",
        help="allow immediate torque-off if PCM still reports motion busy",
    )
    rebase.add_argument(
        "--arm-after", action="store_true",
        help="software-arm after verifying boot-pose option 3 is 0 rad",
    )
    rebase.add_argument(
        "--i-understand-repeated-flash-writes",
        action="store_true",
        help="required acknowledgement for relative-teaching use",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.timeout <= 0 or not math.isfinite(args.timeout):
        raise SystemExit("--timeout must be a positive finite number")
    if args.command == "set-current-as-zero":
        if args.axis != args.confirm_axis:
            raise SystemExit("--confirm-axis must exactly match --axis")
        if not args.i_understand_motion_references_will_shift:
            raise SystemExit(
                "persistent write blocked: pass "
                "--i-understand-motion-references-will-shift"
            )
    if args.command == "save-detected-axis-config":
        if args.expect_live_mask != args.confirm_live_mask:
            raise SystemExit(
                "--confirm-live-mask must exactly match --expect-live-mask"
            )
        if not args.i_understand_axis_config_is_persistent:
            raise SystemExit(
                "persistent axis configuration blocked: pass "
                "--i-understand-axis-config-is-persistent"
            )
    if args.command == "relative-rebase":
        if args.axes != args.confirm_axes:
            raise SystemExit("--confirm-axes must exactly match --axes")
        if not args.i_understand_repeated_flash_writes:
            raise SystemExit(
                "persistent rebase blocked: pass "
                "--i-understand-repeated-flash-writes"
            )

    try:
        with LinuxCdcPort(args.port) as port:
            client = PcmStudioClient(port, timeout=args.timeout)
            session = (
                ensure_live_session(client) if args.attach_live else None
            )
            if args.command == "status":
                result = read_status(client)
            elif args.command == "save-detected-axis-config":
                result = save_detected_axis_config(
                    client, args.expect_live_mask
                )
            elif args.command == "set-current-as-zero":
                result = set_current_as_zero(client, args.axis)
            else:
                result = relative_rebase(
                    client,
                    args.axes,
                    args.allow_interrupt_motion,
                    args.allow_torque_off,
                    args.arm_after,
                )
            if session is not None:
                result["session"] = session
        print(_format_json(result))
        return 0
    except (
        OSError,
        ProtocolError,
        SdoAbort,
        TimeoutError,
        RuntimeError,
    ) as exc:
        print(_format_json({"ok": False, "error": str(exc)}))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
