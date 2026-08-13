import struct

import pytest

from soldering_control.pcm_studio_client import (
    MSG_SDO_REQ,
    STUDIO_DOP_NODE_ID,
    ProtocolError,
    build_dop_frame,
    cobs_decode,
    cobs_encode,
    crc16_ccitt,
    decode_write_result,
    parse_dop_frame,
    parse_sdo_response,
    RELOAD_RESULT_NAMES,
    verify_zero_boot_pose,
    wait_for_rebase_window,
    SESSION_GUARD,
)


@pytest.mark.parametrize(
    "data", [b"", b"\x00", b"abc", b"a\x00b", bytes(range(256))]
)
def test_cobs_round_trip(data):
    assert cobs_decode(cobs_encode(data)) == data


def test_crc16_ccitt_check_value():
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_zero_write_wire_layout():
    sdo = struct.pack("<BHBH", 0x21, 0x5F01, 8, 4) + struct.pack("<f", 0.0)
    encoded = build_dop_frame(MSG_SDO_REQ, STUDIO_DOP_NODE_ID, 0x1234, sdo)
    frame = parse_dop_frame(encoded[:-1])
    assert frame.msg_type == MSG_SDO_REQ
    assert frame.node_id == 0x51
    assert frame.sequence == 0x1234
    assert frame.payload.hex() == "21015f08040000000000"


def test_parse_explicit_length_read_response():
    response = parse_sdo_response(
        struct.pack("<BHBH", 0x41, 0x5F07, 3, 2) + b"\x80\x00"
    )
    assert response.data == b"\x80\x00"
    assert not response.abort_code


def test_parse_rejects_truncated_explicit_response():
    with pytest.raises(ProtocolError):
        parse_sdo_response(
            struct.pack("<BHBH", 0x41, 0x5F07, 3, 4) + b"\x80\x00"
        )


def test_decode_settings_write_result():
    decoded = decode_write_result((42 << 16) | (7 << 8) | 1)
    assert decoded == {
        "raw": 0x002A0701,
        "result": 1,
        "result_name": "applied",
        "object": 7,
        "sequence": 42,
    }


def test_reload_result_contract():
    assert RELOAD_RESULT_NAMES[1] == "success"
    assert RELOAD_RESULT_NAMES[-4] == "csv_parse_or_rescan_failed"
    assert struct.pack("<I", SESSION_GUARD) == b"PCM1"


class _ReadOnlyFakeClient:
    def __init__(self, data):
        self.data = data

    def read(self, _index, _subindex):
        return self.data


def test_verify_zero_boot_pose():
    data = bytes([3]) + struct.pack("<12f", *([0.0] * 12))
    result = verify_zero_boot_pose(_ReadOnlyFakeClient(data), [7])
    assert result["option"] == 3
    assert result["verified_axes"] == [7]


def test_verify_builtin_zero_boot_pose():
    data = bytes([2]) + struct.pack("<12f", *([0.0] * 12))
    result = verify_zero_boot_pose(_ReadOnlyFakeClient(data), [7])
    assert result == {
        "option": 2,
        "meaning": "user_zero",
        "verified_axes": [7],
    }


def test_verify_zero_boot_pose_rejects_nonzero_axis():
    angles = [0.0] * 12
    angles[7] = 0.1
    data = bytes([3]) + struct.pack("<12f", *angles)
    with pytest.raises(RuntimeError, match="not 0 rad"):
        verify_zero_boot_pose(_ReadOnlyFakeClient(data), [7])


def test_rebase_window_waits_for_flash_save_reason(monkeypatch):
    states = iter(
        [
            {
                "servo_state": 0,
                "motion_busy": False,
                "write_window": 129,
                "write_window_reasons": ["flash_save_in_progress"],
            },
            {
                "servo_state": 0,
                "motion_busy": False,
                "write_window": 1,
                "write_window_reasons": [],
            },
        ]
    )
    monkeypatch.setattr(
        "soldering_control.pcm_studio_client.read_status",
        lambda _client: next(states),
    )
    monkeypatch.setattr(
        "soldering_control.pcm_studio_client.time.sleep",
        lambda _seconds: None,
    )

    result = wait_for_rebase_window(object(), timeout=1.0)

    assert result["write_window"] == 1
