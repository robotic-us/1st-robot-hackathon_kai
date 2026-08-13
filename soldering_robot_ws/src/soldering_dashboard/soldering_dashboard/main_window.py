"""Main PyQt5 operator window for control, perception, and training."""

from __future__ import annotations

from collections import deque
from datetime import datetime
import math
import os
import time
import webbrowser

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from soldering_interfaces.srv import PcmCommand

from .ros_worker import RosWorker
from .wandb_worker import WandbWorker


GREEN = "#31c48d"
YELLOW = "#f4b942"
RED = "#f05252"
BLUE = "#5aa9ff"
MUTED = "#8795a8"


def _finite_text(value: float, decimals: int = 2) -> str:
    return f"{value:.{decimals}f}" if math.isfinite(value) else "—"


class Badge(QtWidgets.QLabel):
    def __init__(self, title: str):
        super().__init__(f"{title}: UNKNOWN")
        self.title = title
        self.set_state("UNKNOWN", MUTED)

    def set_state(self, text: str, color: str) -> None:
        self.setText(f"{self.title}: {text}")
        self.setStyleSheet(
            "QLabel {"
            f"color: {color}; border: 1px solid {color};"
            "padding: 5px 10px; border-radius: 8px; font-weight: 700;"
            "}"
        )


class DashboardWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Soldering Robot Operations Dashboard")
        self.resize(1500, 920)
        self._motor_latest: dict[int, dict] = {}
        self._history = {
            motor_id: {
                key: deque(maxlen=1200)
                for key in (
                    "t",
                    "target",
                    "position",
                    "velocity",
                    "current",
                    "dob",
                    "temperature",
                )
            }
            for motor_id in range(1, 7)
        }
        self._started_monotonic = time.monotonic()
        self._latest_wandb_url = ""
        self._last_pcm = {}
        self._last_bridge = {}
        self._shutdown_done = False

        self.ros_worker = RosWorker(self)
        self.wandb_worker = WandbWorker(self, poll_s=5.0)
        self._build_ui()
        self._connect_signals()
        self._apply_theme()
        self.ros_worker.start()
        self.wandb_worker.start()
        self.wandb_worker.set_target(
            self.wandb_entity.text(), self.wandb_project.text()
        )
        self.graph_timer = QtCore.QTimer(self)
        self.graph_timer.timeout.connect(self._refresh_motor_graphs)
        self.graph_timer.start(100)

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        root_layout = QtWidgets.QVBoxLayout(root)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(10)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("AUTOMATIC SOLDERING / OPERATIONS")
        title.setObjectName("title")
        header.addWidget(title)
        header.addStretch(1)
        self.badges = {
            name: Badge(name)
            for name in ("ROS", "ETHERCAT", "ARM", "PCM", "VISION", "W&B")
        }
        for badge in self.badges.values():
            header.addWidget(badge)
        root_layout.addLayout(header)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_motor_tab(), "모터 / 제어")
        self.tabs.addTab(self._build_topic_tab(), "ROS 토픽")
        self.tabs.addTab(self._build_vision_tab(), "비전")
        self.tabs.addTab(self._build_wandb_tab(), "학습 / W&B")
        self.tabs.addTab(self._build_log_tab(), "통합 로그")
        root_layout.addWidget(self.tabs, 1)
        self.setCentralWidget(root)
        self.statusBar().showMessage(
            "대시보드는 하드웨어 스택을 자동 시작하지 않습니다. 물리 E-Stop은 별도입니다."
        )

    def _build_motor_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        split = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        upper = QtWidgets.QWidget()
        upper_layout = QtWidgets.QHBoxLayout(upper)
        self.motor_table = QtWidgets.QTableWidget(6, 15)
        self.motor_table.setHorizontalHeaderLabels(
            [
                "Motor",
                "Axis",
                "Link",
                "Valid",
                "OPER",
                "Fault",
                "Target °",
                "Position °",
                "Velocity °/s",
                "Current A",
                "DOB A",
                "Load",
                "Temp °C",
                "Bus V",
                "Age ms",
            ]
        )
        self.motor_table.verticalHeader().setVisible(False)
        self.motor_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.motor_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.motor_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeToContents
        )
        self.motor_table.horizontalHeader().setStretchLastSection(True)
        for row in range(6):
            self.motor_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(row + 1)))
            for column in range(1, 15):
                self.motor_table.setItem(row, column, QtWidgets.QTableWidgetItem("—"))
        upper_layout.addWidget(self.motor_table, 4)
        upper_layout.addWidget(self._build_pcm_control(), 1)
        split.addWidget(upper)

        graph_widget = QtWidgets.QWidget()
        graph_layout = QtWidgets.QVBoxLayout(graph_widget)
        selector = QtWidgets.QHBoxLayout()
        selector.addWidget(QtWidgets.QLabel("그래프 모터"))
        self.motor_selector = QtWidgets.QComboBox()
        self.motor_selector.addItems([f"Motor {i}" for i in range(1, 7)])
        selector.addWidget(self.motor_selector)
        selector.addStretch(1)
        self.setup_label = QtWidgets.QLabel("Setup: waiting")
        selector.addWidget(self.setup_label)
        graph_layout.addLayout(selector)
        self.position_plot = pg.PlotWidget(title="Target / Position (deg)")
        self.velocity_plot = pg.PlotWidget(title="Velocity (deg/s)")
        self.electrical_plot = pg.PlotWidget(
            title="Current / DOB (A) / Temperature (°C)"
        )
        for plot in (self.position_plot, self.velocity_plot, self.electrical_plot):
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("bottom", "elapsed", units="s")
            graph_layout.addWidget(plot)
        self.target_curve = self.position_plot.plot(pen=pg.mkPen(YELLOW, width=2), name="target")
        self.position_curve = self.position_plot.plot(pen=pg.mkPen(BLUE, width=2), name="position")
        self.position_plot.addLegend()
        self.velocity_curve = self.velocity_plot.plot(pen=pg.mkPen(GREEN, width=2))
        self.current_curve = self.electrical_plot.plot(
            pen=pg.mkPen("#b794f4", width=2), name="current"
        )
        self.dob_curve = self.electrical_plot.plot(
            pen=pg.mkPen("#22d3ee", width=2), name="DOB current"
        )
        self.temperature_curve = self.electrical_plot.plot(
            pen=pg.mkPen(RED, width=2), name="temperature"
        )
        self.electrical_plot.addLegend()
        split.addWidget(graph_widget)
        split.setSizes([330, 530])
        layout.addWidget(split)
        return page

    def _build_pcm_control(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("PCM 명령 (잠금 기본)")
        form = QtWidgets.QFormLayout(box)
        self.pcm_state_label = QtWidgets.QLabel("disconnected / no status")
        self.pcm_state_label.setWordWrap(True)
        form.addRow("상태", self.pcm_state_label)
        self.axes_edit = QtWidgets.QLineEdit("7")
        self.slot_spin = QtWidgets.QSpinBox()
        self.slot_spin.setRange(1, 50)
        self.slot_spin.setValue(2)
        form.addRow("Axes", self.axes_edit)
        form.addRow("Slot", self.slot_spin)
        self.command_unlock = QtWidgets.QCheckBox(
            "명령 활성화 — 주변과 물리 E-Stop 확인"
        )
        self.command_unlock.toggled.connect(self._update_command_lock)
        form.addRow(self.command_unlock)
        button_grid = QtWidgets.QGridLayout()
        self.arm_button = QtWidgets.QPushButton("PCM ARM")
        self.play_button = QtWidgets.QPushButton("슬롯 1회 재생")
        self.stop_button = QtWidgets.QPushButton(
            "PCM 정지/자세유지\n(E-Stop 아님)"
        )
        self.reconnect_button = QtWidgets.QPushButton("USB 재연결")
        self.command_buttons = [
            self.arm_button,
            self.play_button,
            self.stop_button,
            self.reconnect_button,
        ]
        for button in self.command_buttons:
            button.setEnabled(False)
        button_grid.addWidget(self.arm_button, 0, 0)
        button_grid.addWidget(self.play_button, 0, 1)
        button_grid.addWidget(self.stop_button, 1, 0)
        button_grid.addWidget(self.reconnect_button, 1, 1)
        form.addRow(button_grid)
        warning = QtWidgets.QLabel(
            "STOP/cancel은 모션만 멈추고 서보 토크를 유지합니다. "
            "물리 E-Stop이 아닙니다."
        )
        warning.setWordWrap(True)
        warning.setObjectName("warning")
        form.addRow(warning)
        return box

    def _build_topic_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        self.topic_table = QtWidgets.QTableWidget(0, 7)
        self.topic_table.setHorizontalHeaderLabels(
            ["Topic", "Publishers", "Subscribers", "Hz", "Age ms", "State", "판정"]
        )
        self.topic_table.verticalHeader().setVisible(False)
        self.topic_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.topic_table.horizontalHeader().setSectionResizeMode(
            0, QtWidgets.QHeaderView.Stretch
        )
        for column in range(1, 7):
            self.topic_table.horizontalHeader().setSectionResizeMode(
                column, QtWidgets.QHeaderView.ResizeToContents
            )
        layout.addWidget(self.topic_table)
        hint = QtWidgets.QLabel(
            "ACTIVE는 발행자 존재와 최근 메시지 수신을 모두 확인한 "
            "상태입니다. PUBLISHER ONLY/STALE은 토픽 이름만 존재하거나 "
            "데이터가 멈춘 경우입니다."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return page

    def _build_vision_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(page)
        self.image_label = QtWidgets.QLabel("/soldering/vision/annotated 대기 중")
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.image_label.setMinimumSize(800, 500)
        self.image_label.setStyleSheet("background:#090d13; border:1px solid #344054;")
        layout.addWidget(self.image_label, 4)
        side = QtWidgets.QGroupBox("관측 상태")
        side_layout = QtWidgets.QFormLayout(side)
        self.vision_fields = {}
        labels = (
            ("backend", "Detector"),
            ("valid", "Geometry valid"),
            ("calibrated", "Calibrated"),
            ("objects", "Objects"),
            ("process_class", "Process class"),
            ("process_confidence", "Confidence"),
            ("inference_ms", "Inference ms"),
            ("image_age_ms", "Image age ms"),
            ("wire_error", "Wire error mm"),
            ("iron_to_joint_mm", "Nozzle→joint mm"),
            ("solder_to_joint_mm", "Solder→joint mm"),
            ("occluded", "Occluded"),
        )
        for key, label in labels:
            value = QtWidgets.QLabel("—")
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.vision_fields[key] = value
            side_layout.addRow(label, value)
        layout.addWidget(side, 1)
        return page

    def _build_wandb_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Entity"))
        self.wandb_entity = QtWidgets.QLineEdit(
            os.environ.get("WANDB_ENTITY", "donghyeok8649-kaist")
        )
        controls.addWidget(self.wandb_entity)
        controls.addWidget(QtWidgets.QLabel("Project"))
        self.wandb_project = QtWidgets.QLineEdit(
            os.environ.get("WANDB_PROJECT", "soldering-vision")
        )
        controls.addWidget(self.wandb_project)
        self.wandb_refresh_button = QtWidgets.QPushButton("조회 / 새로고침")
        controls.addWidget(self.wandb_refresh_button)
        self.wandb_open_button = QtWidgets.QPushButton("브라우저에서 열기")
        self.wandb_open_button.setEnabled(False)
        controls.addWidget(self.wandb_open_button)
        layout.addLayout(controls)

        self.wandb_status_label = QtWidgets.QLabel("W&B worker starting")
        layout.addWidget(self.wandb_status_label)
        summary_layout = QtWidgets.QHBoxLayout()
        self.wandb_run_label = QtWidgets.QLabel("Run: —")
        self.wandb_epoch_label = QtWidgets.QLabel("Epoch: —")
        self.wandb_loss_label = QtWidgets.QLabel("Loss: —")
        self.wandb_accuracy_label = QtWidgets.QLabel("Val accuracy: —")
        for label in (
            self.wandb_run_label,
            self.wandb_epoch_label,
            self.wandb_loss_label,
            self.wandb_accuracy_label,
        ):
            label.setObjectName("metric")
            summary_layout.addWidget(label)
        layout.addLayout(summary_layout)
        self.loss_plot = pg.PlotWidget(title="ConvNeXt training loss")
        self.accuracy_plot = pg.PlotWidget(title="Validation accuracy")
        for plot in (self.loss_plot, self.accuracy_plot):
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("bottom", "epoch")
            layout.addWidget(plot)
        self.loss_curve = self.loss_plot.plot(pen=pg.mkPen(YELLOW, width=2), symbol="o")
        self.accuracy_curve = self.accuracy_plot.plot(pen=pg.mkPen(GREEN, width=2), symbol="o")
        self.wandb_details = QtWidgets.QPlainTextEdit()
        self.wandb_details.setReadOnly(True)
        self.wandb_details.setMaximumHeight(150)
        layout.addWidget(self.wandb_details)
        return page

    def _build_log_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        bar = QtWidgets.QHBoxLayout()
        self.log_level = QtWidgets.QComboBox()
        self.log_level.addItem("INFO 이상", 20)
        self.log_level.addItem("WARN 이상", 30)
        self.log_level.addItem("ERROR 이상", 40)
        self.log_level.addItem("DEBUG 포함", 10)
        self.log_filter = QtWidgets.QLineEdit()
        self.log_filter.setPlaceholderText("node 또는 메시지 필터")
        clear_button = QtWidgets.QPushButton("지우기")
        clear_button.clicked.connect(lambda: self.log_view.clear())
        bar.addWidget(self.log_level)
        bar.addWidget(self.log_filter, 1)
        bar.addWidget(clear_button)
        layout.addLayout(bar)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(5000)
        layout.addWidget(self.log_view)
        return page

    def _connect_signals(self) -> None:
        worker = self.ros_worker
        worker.motor_telemetry.connect(self._on_motor_telemetry)
        worker.topic_health.connect(self._on_topic_health)
        worker.armed.connect(self._on_armed)
        worker.setup_state.connect(self._on_setup_state)
        worker.bridge_status.connect(self._on_bridge_status)
        worker.motion_window.connect(self._on_motion_window)
        worker.pcm_status.connect(self._on_pcm_status)
        worker.vision_status.connect(self._on_vision_status)
        worker.image_frame.connect(self._on_image)
        worker.log_line.connect(self._on_log)
        worker.command_result.connect(self._on_command_result)
        worker.worker_status.connect(self._on_ros_worker_status)

        self.arm_button.clicked.connect(
            lambda: self._submit_pcm(PcmCommand.Request.ARM)
        )
        self.play_button.clicked.connect(
            lambda: self._submit_pcm(PcmCommand.Request.PLAY_SLOT)
        )
        self.stop_button.clicked.connect(
            lambda: self._submit_pcm(PcmCommand.Request.STOP)
        )
        self.reconnect_button.clicked.connect(
            lambda: self._submit_pcm(PcmCommand.Request.RECONNECT)
        )

        self.wandb_worker.run_update.connect(self._on_wandb_update)
        self.wandb_worker.worker_status.connect(self._on_wandb_status)
        self.wandb_refresh_button.clicked.connect(self._refresh_wandb)
        self.wandb_open_button.clicked.connect(self._open_wandb)

    def _apply_theme(self) -> None:
        pg.setConfigOptions(
            antialias=True, background="#111827", foreground="#d0d5dd"
        )
        self.setStyleSheet(
            "QWidget { background:#111827; color:#e5e7eb; font-size:13px; }"
            "QMainWindow { background:#0b1220; }"
            "QLabel#title { font-size:20px; font-weight:800; color:#f8fafc; }"
            "QLabel#metric { border:1px solid #344054; border-radius:8px; "
            "padding:10px; font-weight:700; }"
            "QLabel#warning { color:#f4b942; }"
            "QTabWidget::pane, QGroupBox { border:1px solid #344054; border-radius:6px; }"
            "QTabBar::tab { background:#182230; padding:9px 18px; margin-right:2px; }"
            "QTabBar::tab:selected { background:#245ea8; }"
            "QTableWidget, QPlainTextEdit, QLineEdit, QComboBox, QSpinBox { "
            "background:#0f172a; border:1px solid #344054; }"
            "QHeaderView::section { background:#1d2939; color:#f8fafc; padding:6px; border:0; }"
            "QPushButton { background:#245ea8; border:0; border-radius:5px; "
            "padding:8px; font-weight:700; }"
            "QPushButton:hover { background:#3478c9; }"
            "QPushButton:disabled { background:#344054; color:#667085; }"
            "QCheckBox { padding:6px 0; }"
        )

    @QtCore.pyqtSlot(object)
    def _on_motor_telemetry(self, data: dict) -> None:
        motor_id = int(data["motor_id"])
        if not 1 <= motor_id <= 6:
            return
        self._motor_latest[motor_id] = data
        row = motor_id - 1
        now = time.monotonic()
        age_ms = max(0.0, (now - data["received_monotonic"]) * 1000.0)
        values = [
            str(data["axis_index"]),
            "YES" if data["connected"] else "NO",
            "YES" if data["valid"] else "NO",
            "YES" if data["oper"] else "NO",
            "YES" if data["fault"] else "NO",
            _finite_text(math.degrees(data["target_position_rad"]), 3),
            _finite_text(math.degrees(data["filtered_position_rad"]), 3),
            _finite_text(math.degrees(data["velocity_rad_s"]), 3),
            _finite_text(data["current_a"], 2),
            _finite_text(data["disturbance_current_a"], 2),
            {
                0: "UNKNOWN",
                1: "MOVING",
                2: "HOLDING",
                3: "NO EVIDENCE",
            }.get(data["load_state"], "INVALID"),
            _finite_text(data["temperature_c"], 1),
            _finite_text(data["bus_voltage_v"], 1),
            _finite_text(age_ms, 0),
        ]
        for column, value in enumerate(values, start=1):
            item = self.motor_table.item(row, column)
            item.setText(value)
        color = RED if data["fault"] else GREEN if data["valid"] and data["oper"] else YELLOW
        for column in (2, 3, 4, 5):
            self.motor_table.item(row, column).setForeground(QtGui.QColor(color))

        history = self._history[motor_id]
        history["t"].append(now - self._started_monotonic)
        history["target"].append(math.degrees(data["target_position_rad"]))
        history["position"].append(math.degrees(data["filtered_position_rad"]))
        history["velocity"].append(math.degrees(data["velocity_rad_s"]))
        history["current"].append(data["current_a"])
        history["dob"].append(data["disturbance_current_a"])
        history["temperature"].append(data["temperature_c"])

    def _refresh_motor_graphs(self) -> None:
        motor_id = self.motor_selector.currentIndex() + 1
        history = self._history[motor_id]
        if not history["t"]:
            return
        x = list(history["t"])
        self.target_curve.setData(x, list(history["target"]))
        self.position_curve.setData(x, list(history["position"]))
        self.velocity_curve.setData(x, list(history["velocity"]))
        self.current_curve.setData(x, list(history["current"]))
        self.dob_curve.setData(x, list(history["dob"]))
        self.temperature_curve.setData(x, list(history["temperature"]))

    @QtCore.pyqtSlot(object)
    def _on_topic_health(self, rows) -> None:
        self.topic_table.setRowCount(len(rows))
        feedback_active = False
        vision_active = False
        for row_index, health in enumerate(rows):
            age_ms = health.age_s * 1000.0 if math.isfinite(health.age_s) else math.inf
            label = {
                "active": "ACTIVE",
                "stale": "STALE",
                "publisher_only": "PUBLISHER ONLY",
                "missing": "MISSING",
            }[health.state]
            explanation = {
                "active": "발행 및 최근 수신",
                "stale": "최근 데이터 없음",
                "publisher_only": "발행자는 있으나 미수신",
                "missing": "발행자 없음",
            }[health.state]
            values = [
                health.name,
                str(health.publishers),
                str(health.subscribers),
                _finite_text(health.rate_hz, 1),
                _finite_text(age_ms, 0),
                label,
                explanation,
            ]
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                if column == 5:
                    color = (
                        GREEN
                        if health.state == "active"
                        else RED
                        if health.state == "missing"
                        else YELLOW
                    )
                    item.setForeground(QtGui.QColor(color))
                self.topic_table.setItem(row_index, column, item)
            if health.name == "/phorce/feedback":
                feedback_active = health.state == "active"
            if health.name == "/soldering/geometry_observation":
                vision_active = health.state == "active"
        self.badges["ROS"].set_state(
            "FEEDBACK" if feedback_active else "NO DATA",
            GREEN if feedback_active else RED,
        )
        if not vision_active:
            self.badges["VISION"].set_state("NO DATA", YELLOW)

    @QtCore.pyqtSlot(bool)
    def _on_armed(self, armed: bool) -> None:
        self.badges["ARM"].set_state(
            "ARMED" if armed else "DISARMED", GREEN if armed else RED
        )

    @QtCore.pyqtSlot(str)
    def _on_setup_state(self, state: str) -> None:
        self.setup_label.setText(f"Setup: {state}")

    @QtCore.pyqtSlot(object)
    def _on_bridge_status(self, status: dict) -> None:
        self._last_bridge = status
        if status["estop_active"]:
            self.badges["ETHERCAT"].set_state("E-STOP", RED)
        elif status["ethercat_operational"]:
            self.badges["ETHERCAT"].set_state("OP", GREEN)
        else:
            self.badges["ETHERCAT"].set_state(status["mode"].upper(), YELLOW)

    @QtCore.pyqtSlot(object)
    def _on_motion_window(self, status: dict) -> None:
        if status["busy"]:
            self.statusBar().showMessage(
                f"PCM motion busy: active slot {status['active_slot']}"
            )

    @QtCore.pyqtSlot(object)
    def _on_pcm_status(self, status: dict) -> None:
        self._last_pcm = status
        state = str(status.get("state", "unknown"))
        connected = bool(status.get("connected", False))
        detail = str(status.get("detail", ""))
        self.pcm_state_label.setText(f"{state} / {detail}")
        self.badges["PCM"].set_state(
            state.upper(), GREEN if connected and state != "fault" else RED
        )

    @QtCore.pyqtSlot(object)
    def _on_vision_status(self, status: dict) -> None:
        self.badges["VISION"].set_state(
            "VALID" if status["valid"] else "UNREADY",
            GREEN if status["valid"] else YELLOW,
        )
        values = {
            "backend": status["backend"],
            "valid": str(status["valid"]),
            "calibrated": str(status["calibrated"]),
            "objects": str(status["objects"]),
            "process_class": status["process_class"] or "disabled",
            "process_confidence": _finite_text(status["process_confidence"], 3),
            "inference_ms": _finite_text(status["inference_ms"], 1),
            "image_age_ms": _finite_text(status["image_age_ms"], 1),
            "wire_error": f"{status['wire_error_x_mm']:.2f}, {status['wire_error_y_mm']:.2f}",
            "iron_to_joint_mm": _finite_text(status["iron_to_joint_mm"], 2),
            "solder_to_joint_mm": _finite_text(status["solder_to_joint_mm"], 2),
            "occluded": str(status["occluded"]),
        }
        for key, value in values.items():
            self.vision_fields[key].setText(value)

    @QtCore.pyqtSlot(object)
    def _on_image(self, rgb: np.ndarray) -> None:
        height, width, channels = rgb.shape
        image = QtGui.QImage(
            rgb.data,
            width,
            height,
            channels * width,
            QtGui.QImage.Format_RGB888,
        ).copy()
        pixmap = QtGui.QPixmap.fromImage(image)
        self.image_label.setPixmap(
            pixmap.scaled(
                self.image_label.size(),
                QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation,
            )
        )

    @QtCore.pyqtSlot(object)
    def _on_log(self, entry: dict) -> None:
        level = int(entry.get("level", 20))
        if level < int(self.log_level.currentData()):
            return
        name = str(entry.get("name", "unknown"))
        message = str(entry.get("message", ""))
        needle = self.log_filter.text().strip().lower()
        if needle and needle not in f"{name} {message}".lower():
            return
        level_name = {
            10: "DEBUG",
            20: "INFO",
            30: "WARN",
            40: "ERROR",
            50: "FATAL",
        }.get(level, str(level))
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendPlainText(
            f"{timestamp} {level_name:<5} [{name}] {message}"
        )

    def _axes(self) -> list[int] | None:
        try:
            axes = [
                int(value.strip())
                for value in self.axes_edit.text().split(",")
                if value.strip()
            ]
        except ValueError:
            return None
        if (
            not axes
            or len(set(axes)) != len(axes)
            or any(axis < 0 or axis > 11 for axis in axes)
        ):
            return None
        return axes

    def _update_command_lock(self, checked: bool) -> None:
        for button in self.command_buttons:
            button.setEnabled(checked)

    def _submit_pcm(self, operation: int) -> None:
        if not self.command_unlock.isChecked():
            return
        axes = self._axes()
        if axes is None:
            QtWidgets.QMessageBox.warning(self, "Axes 오류", "Axes는 중복 없는 0..11 정수 목록이어야 합니다.")
            return
        operation_name = {
            PcmCommand.Request.ARM: "ARM",
            PcmCommand.Request.PLAY_SLOT: "PLAY SLOT",
            PcmCommand.Request.STOP: "STOP (not E-Stop)",
            PcmCommand.Request.RECONNECT: "RECONNECT",
        }[operation]
        if operation in (PcmCommand.Request.ARM, PcmCommand.Request.PLAY_SLOT):
            answer = QtWidgets.QMessageBox.question(
                self,
                "실물 동작 확인",
                f"{operation_name} 요청을 PCM 서비스로 보낼까요?\n주변과 물리 E-Stop을 확인해야 합니다.",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
        command = {
            "operation": operation,
            "slot_id": self.slot_spin.value(),
            "axes": axes,
            "repeat": 1 if operation == PcmCommand.Request.PLAY_SLOT else 0,
        }
        self.ros_worker.enqueue_pcm_command(command)
        self.command_unlock.setChecked(False)
        self._on_log(
            {"level": 20, "name": "dashboard", "message": f"submitted {operation_name}: {command}"}
        )

    @QtCore.pyqtSlot(object)
    def _on_command_result(self, result: dict) -> None:
        level = 20 if result.get("accepted") else 40
        self._on_log(
            {"level": level, "name": "pcm_command", "message": str(result)}
        )
        self.statusBar().showMessage(str(result.get("message", result)), 8000)

    @QtCore.pyqtSlot(str)
    def _on_ros_worker_status(self, status: str) -> None:
        if status == "running":
            self.badges["ROS"].set_state("WAITING", YELLOW)
        elif status.startswith("error"):
            self.badges["ROS"].set_state("ERROR", RED)
            self._on_log({"level": 40, "name": "ros_worker", "message": status})

    def _refresh_wandb(self) -> None:
        self.wandb_worker.set_target(
            self.wandb_entity.text(), self.wandb_project.text()
        )

    @QtCore.pyqtSlot(str)
    def _on_wandb_status(self, status: str) -> None:
        self.wandb_status_label.setText(status)
        color = (
            GREEN
            if status.startswith("connected")
            else RED
            if status.startswith("error")
            else YELLOW
        )
        self.badges["W&B"].set_state(status.split(":", 1)[0].upper(), color)

    @QtCore.pyqtSlot(object)
    def _on_wandb_update(self, data: dict) -> None:
        history = data.get("history", {})
        epochs = history.get("epoch", [])
        losses = history.get("train/loss", [])
        accuracies = history.get("validation/accuracy", [])
        self.loss_curve.setData(epochs, losses)
        self.accuracy_curve.setData(epochs, accuracies)
        self.wandb_run_label.setText(
            f"Run: {data.get('name', '—')} / {data.get('state', '—')}"
        )
        if epochs:
            self.wandb_epoch_label.setText(f"Epoch: {epochs[-1]:.0f}")
        finite_losses = [value for value in losses if math.isfinite(value)]
        finite_accuracies = [
            value for value in accuracies if math.isfinite(value)
        ]
        self.wandb_loss_label.setText(
            f"Loss: {finite_losses[-1]:.5f}" if finite_losses else "Loss: —"
        )
        self.wandb_accuracy_label.setText(
            f"Val accuracy: {finite_accuracies[-1]:.4f}"
            if finite_accuracies
            else "Val accuracy: —"
        )
        self._latest_wandb_url = str(data.get("url", ""))
        self.wandb_open_button.setEnabled(bool(self._latest_wandb_url))
        config = data.get("config", {})
        summary = data.get("summary", {})
        system = data.get("system_metrics", {})
        lines = [
            f"URL: {self._latest_wandb_url or '—'}",
            f"Architecture: {config.get('architecture', '—')}",
            f"Classes: {config.get('classes', '—')}",
            f"Train images: {config.get('train_images', '—')}",
            f"Validation images: {config.get('validation_images', '—')}",
            "Best/summary accuracy: "
            f"{summary.get('validation/accuracy', summary.get('val_accuracy', '—'))}",
            f"GPU metrics available: {sum(1 for key in system if 'gpu' in key.lower())}",
        ]
        self.wandb_details.setPlainText("\n".join(lines))

    def _open_wandb(self) -> None:
        if self._latest_wandb_url:
            webbrowser.open(self._latest_wandb_url)

    def shutdown_workers(self) -> None:
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self.graph_timer.stop()
        self.ros_worker.stop()
        self.wandb_worker.stop()
        self.ros_worker.wait(3000)
        self.wandb_worker.wait(7000)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.shutdown_workers()
        event.accept()
