"""ACT采集上位机主窗口。"""

from __future__ import annotations

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum
import math
from pathlib import Path
import sys
import time

import numpy as np

from PySide6.QtCore import QEvent, QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QCloseEvent, QKeyEvent, QPainter, QPen, QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)

from episode_buffer import EpisodeBuffer, save_episode, save_validated_episode
from protocol import (
    ActFrame,
    ActMotorState,
    CommandId,
    Packet,
    PacketParser,
    PacketType,
    ProtocolError,
    ResultCode,
    decode_act_frame,
    decode_command_result,
    decode_status,
    encode_capture,
    encode_enable,
    encode_home,
    encode_move,
    encode_ping,
    encode_select,
    encode_stop,
    encode_status_request,
)
from serial_service import PortDescriptor, SerialService, SerialSettings
from tentacle_view import DirectionDial, TentacleViewport


APP_STYLE = """
QWidget {
    background: #17212b;
    color: #d7e1ea;
    font-family: "Microsoft YaHei UI";
    font-size: 13px;
}
QMainWindow { background: #17212b; }
QFrame#topBar, QFrame#bottomBar {
    background: #202c38;
    border: 1px solid #3c5366;
}
QFrame#card {
    background: #202c38;
    border: 1px solid #3c5366;
    border-radius: 8px;
}
QFrame#subCard {
    background: #182632;
    border: 1px solid #33495a;
    border-radius: 6px;
}
QLabel#title { font-size: 17px; font-weight: 700; color: #f2f7fb; }
QLabel#sectionTitle { font-size: 14px; font-weight: 700; color: #72d7ee; }
QLabel#muted { color: #91a3b5; }
QLabel#value { color: #f2f7fb; font-weight: 700; }
QLabel#timer { color: #f2f7fb; font-size: 27px; font-weight: 700; font-family: Consolas; }
QLabel#statusGood { color: #62e5b5; font-weight: 700; }
QLabel#statusWait { color: #f3c969; font-weight: 700; }
QLabel#statusError { color: #ff7b87; font-weight: 700; }
QLabel#statusOff { color: #91a3b5; }
QPushButton {
    min-height: 32px;
    background: #2b3b4b;
    border: 1px solid #4b6579;
    border-radius: 5px;
    padding: 2px 12px;
}
QPushButton:hover { background: #385267; border-color: #66d9ef; }
QPushButton:pressed { background: #243746; }
QPushButton:disabled { color: #718392; background: #263440; border-color: #344653; }
QPushButton#primary { background: #177d98; border-color: #66d9ef; font-weight: 700; }
QPushButton#danger { background: #8a2d38; border-color: #d25a66; color: white; font-weight: 700; }
QPushButton#danger:disabled { background: #3e2930; border-color: #65404a; color: #8e737a; }
QPushButton#controlEnableReady { background: #126f89; border: 2px solid #66d9ef; color: #ffffff; font-weight: 700; }
QPushButton#controlHolding { background: #9a6420; border: 2px solid #ffd166; color: #ffffff; font-weight: 700; }
QPushButton#controlEnabled:disabled { background: #17694f; border: 2px solid #62e5b5; color: #dffff4; font-weight: 700; }
QPushButton#controlStopActive { background: #9b3440; border: 2px solid #ff7b87; color: #ffffff; font-weight: 700; }
QPushButton#parameterLocked { background: #78551f; border: 2px solid #f3c969; color: #fff0ba; font-weight: 700; }
QPushButton#parameterLocked:disabled { background: #3e382a; border-color: #766643; color: #aa9a72; }
QPushButton#parameterEditable { background: #17694f; border: 2px solid #62e5b5; color: #e3fff5; font-weight: 700; }
QComboBox, QLineEdit, QPlainTextEdit {
    min-height: 31px;
    background: #14212c;
    border: 1px solid #476177;
    border-radius: 4px;
    padding: 2px 8px;
    selection-background-color: #167e9b;
}
QComboBox:disabled, QLineEdit:disabled, QPlainTextEdit:disabled { color: #7d8f9d; }
QComboBox QAbstractItemView { background: #202c38; border: 1px solid #4b6579; }
QSlider::groove:horizontal { height: 6px; background: #3b5365; border-radius: 3px; }
QSlider::handle:horizontal { width: 16px; margin: -5px 0; background: #66d9ef; border-radius: 8px; }
QSlider:disabled::handle:horizontal { background: #536a76; }
QScrollArea#deviceScroll { border: none; background: transparent; }
QScrollBar:vertical {
    width: 10px;
    background: #17212b;
    margin: 2px 0;
}
QScrollBar::handle:vertical {
    min-height: 32px;
    background: #476177;
    border-radius: 5px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""


def _card() -> QFrame:
    frame = QFrame()
    frame.setObjectName("card")
    return frame


def _subcard() -> QFrame:
    frame = QFrame()
    frame.setObjectName("subCard")
    return frame


def _section_title(text: str) -> QLabel:
    label = QLabel(text)
    label.setObjectName("sectionTitle")
    return label


def _status_label(text: str, state: str = "off") -> QLabel:
    label = QLabel(text)
    label.setObjectName(
        {
            "good": "statusGood",
            "wait": "statusWait",
            "error": "statusError",
        }.get(state, "statusOff")
    )
    label.setFixedHeight(22)
    return label


@dataclass(slots=True)
class PendingRequest:
    command_id: CommandId
    payload: bytes
    retry_allowed: bool
    retries: int = 0
    select_target: int | None = None
    enable_target: bool | None = None
    capture_target: bool | None = None


class CaptureUiState(IntEnum):
    IDLE = 0
    STARTING = 1
    CAPTURING = 2
    STOPPING = 3
    REVIEW = 4
    SAVING = 5


class TofHeatmap(QWidget):
    """轻量8×8距离热力图；只负责降频显示，不参与原始帧缓存。"""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(480, 480)
        self._distance = np.zeros(64, dtype=np.int16)
        self._valid = np.zeros(64, dtype=bool)

    def set_frame(self, frame: ActFrame) -> None:
        self._distance = frame.distance_mm.copy()
        self._valid = np.isin(frame.target_status, (5, 9)) & (self._distance > 0)
        self.update()

    def paintEvent(self, _event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)
        side = min(self.width(), self.height()) - 24
        cell = side / 8.0
        left = (self.width() - side) / 2.0
        top = (self.height() - side) / 2.0
        painter.setFont(self.font())
        for row in range(8):
            for column in range(8):
                index = row * 8 + column
                rect = QRectF(left + column * cell, top + row * cell, cell, cell)
                if self._valid[index]:
                    distance = int(self._distance[index])
                    normalized = max(0.0, min(1.0, (distance - 100.0) / 900.0))
                    color = QColor.fromHsvF(0.02 + normalized * 0.55, 0.72, 0.90)
                    text = str(distance)
                else:
                    color = QColor("#263440")
                    text = "—"
                painter.fillRect(rect, color)
                painter.setPen(QPen(QColor("#17212b"), 1))
                painter.drawRect(rect)
                painter.setPen(QColor("#f4f7fa") if self._valid[index] else QColor("#748796"))
                painter.drawText(rect, Qt.AlignCenter, text)


class MainWindow(QMainWindow):
    REQUEST_TIMEOUT_MS = 400

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Flexible Tentacle ACT")
        self.resize(1500, 900)
        self.setMinimumSize(1280, 760)

        self.serial = SerialService(self)
        self.parser = PacketParser()
        self.pending: PendingRequest | None = None
        self._ping_confirmed = False
        self._h7_synced = False
        self._confirmed_tentacle = 1
        self._control_enabled = False
        self._parameters_locked = True
        self._target_angle = 0.0
        self._target_bend = 0
        self._target_stiffness = 0
        self._move_dirty = False
        self._enable_hold_active = False
        self._enable_hold_elapsed_ms = 0
        self._space_shortcut_active = False
        self._ports: list[PortDescriptor] = []
        self._logs: deque[str] = deque(maxlen=300)
        self._alert_history: deque[str] = deque(maxlen=100)
        self._current_alert = "无活动告警"
        self._current_alert_severity = "off"
        self._current_alert_source = ""
        self._last_status = None
        self._capture_state = CaptureUiState.IDLE
        self._episode: EpisodeBuffer | None = None
        self._episode_result: str | None = None
        self._last_frame_monotonic = 0.0
        self._last_capture_status_query = 0.0
        self._latest_display_frame: ActFrame | None = None
        self._displayed_frame_seq = -1
        self._capture_timeout_reported = False
        self._orphan_stop_active = False
        self._capture_parser_start = (0, 0, 0, 0)
        self._save_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="act-save")
        self._save_future: Future | None = None
        self._tof_dialog: QDialog | None = None
        self._tof_heatmap: TofHeatmap | None = None
        self._diagnostic_dialog: QDialog | None = None
        self._diagnostic_text: QPlainTextEdit | None = None
        self._quality_dialog: QDialog | None = None
        self._quality_values: dict[str, QLabel] = {}
        self._quality_alert: QLabel | None = None
        self._quality_history: QPlainTextEdit | None = None

        self._request_timer = QTimer(self)
        self._request_timer.setSingleShot(True)
        self._request_timer.timeout.connect(self._on_request_timeout)
        self._heartbeat_timer = QTimer(self)
        self._heartbeat_timer.setInterval(200)
        self._heartbeat_timer.timeout.connect(self._heartbeat)
        self._move_timer = QTimer(self)
        self._move_timer.setInterval(50)
        self._move_timer.timeout.connect(self._send_latest_move)
        self._enable_hold_timer = QTimer(self)
        self._enable_hold_timer.setSingleShot(True)
        self._enable_hold_timer.setInterval(800)
        self._enable_hold_timer.timeout.connect(self._enable_hold_complete)
        self._enable_hold_progress = QTimer(self)
        self._enable_hold_progress.setInterval(100)
        self._enable_hold_progress.timeout.connect(self._update_enable_hold_progress)
        self._capture_ui_timer = QTimer(self)
        self._capture_ui_timer.setInterval(100)
        self._capture_ui_timer.timeout.connect(self._update_capture_ui)
        self._save_poll_timer = QTimer(self)
        self._save_poll_timer.setInterval(100)
        self._save_poll_timer.timeout.connect(self._poll_save_result)

        self._build_ui()
        self._connect_signals()
        self._set_protocol_controls(False)
        application = QApplication.instance()
        if application is not None:
            application.installEventFilter(self)
        self._heartbeat_timer.start()
        self._move_timer.start()
        self._capture_ui_timer.start()

    def _build_ui(self) -> None:
        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(7)
        outer.addWidget(self._build_top_bar())

        body = QHBoxLayout()
        body.setSpacing(7)
        body.addWidget(self._build_device_panel())
        body.addWidget(self._build_teaching_panel(), 1)
        body.addWidget(self._build_episode_panel())
        outer.addLayout(body, 1)
        outer.addWidget(self._build_bottom_bar())
        self.setCentralWidget(root)

    def _build_top_bar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("topBar")
        bar.setFixedHeight(52)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(14, 7, 10, 7)
        title = QLabel("Flexible Tentacle ACT")
        title.setObjectName("title")
        layout.addWidget(title)
        layout.addStretch(1)
        self.top_serial = _status_label("● 串口未连接")
        self.top_h7 = _status_label("● H7未同步")
        self.top_tentacle = _status_label("T1 未确认")
        self.top_episode = _status_label("Episode：空闲")
        for widget in (self.top_serial, self.top_h7, self.top_tentacle, self.top_episode):
            widget.setMinimumWidth(115)
            layout.addWidget(widget)
        self.top_alert = _status_label("● 无活动告警")
        self.top_alert.setFixedWidth(210)
        layout.addWidget(self.top_alert)
        self.emergency_button = QPushButton("紧急停止")
        self.emergency_button.setObjectName("danger")
        self.emergency_button.setEnabled(False)
        self.emergency_button.setToolTip("立即发送ACT STOP；快捷键 Esc")
        layout.addWidget(self.emergency_button)
        return bar

    def _build_device_panel(self) -> QWidget:
        panel = _card()
        panel.setMinimumWidth(250)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(11, 10, 11, 10)
        layout.setSpacing(7)
        layout.addWidget(_section_title("设备与控制"))
        layout.addWidget(QLabel("串口连接"))

        self.port_combo = QComboBox()
        self.port_combo.setMinimumContentsLength(22)
        self.port_combo.addItem("正在扫描串口…", None)
        layout.addWidget(self.port_combo)

        row = QHBoxLayout()
        self.refresh_button = QPushButton("刷新")
        self.connect_button = QPushButton("连接")
        self.connect_button.setEnabled(False)
        row.addWidget(self.refresh_button)
        row.addWidget(self.connect_button)
        layout.addLayout(row)

        layout.addWidget(QLabel("波特率"))
        self.baud_combo = QComboBox()
        self.baud_combo.setEditable(True)
        for baud in (115200, 230400, 460800, 921600, 1000000, 1500000, 2000000):
            self.baud_combo.addItem(str(baud), baud)
        self.baud_combo.setCurrentText("1000000")
        layout.addWidget(self.baud_combo)

        self.serial_state = _status_label("● 串口未连接")
        self.h7_state = _status_label("● H7未同步")
        layout.addWidget(self.serial_state)
        layout.addWidget(self.h7_state)

        self.advanced_button = QPushButton("⚙ 高级串口设置")
        layout.addWidget(self.advanced_button)
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("color:#345064")
        layout.addWidget(separator)

        layout.addWidget(QLabel("当前触手"))
        self.tentacle_combo = QComboBox()
        for tentacle in range(1, 5):
            self.tentacle_combo.addItem(f"T{tentacle}", tentacle)
        self.tentacle_combo.setEnabled(False)
        layout.addWidget(self.tentacle_combo)
        self.tentacle_protocol_state = _status_label("● 等待H7确认")
        self.runtime_state = _status_label("○ 运行状态：固件未提供")
        layout.addWidget(self.tentacle_protocol_state)
        layout.addWidget(self.runtime_state)

        layout.addSpacing(4)
        layout.addWidget(QLabel("控制权"))
        self.control_owner = _status_label("当前：未启用")
        layout.addWidget(self.control_owner)
        self.enable_control_button = QPushButton("按住 0.8 秒启用（Space）")
        self.stop_control_button = QPushButton("■ 停止控制（Space）")
        self.enable_control_button.setToolTip("鼠标长按或按住Space 800ms启用控制")
        self.stop_control_button.setToolTip("点击或在控制已启用时按Space正常停止")
        self.enable_control_button.setEnabled(False)
        self.stop_control_button.setEnabled(False)
        layout.addWidget(self.enable_control_button)
        layout.addWidget(self.stop_control_button)
        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setObjectName("deviceScroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setFixedWidth(280)
        scroll.setWidget(panel)
        return scroll

    def _build_teaching_panel(self) -> QWidget:
        panel = _card()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(11, 11, 11, 11)
        layout.setSpacing(8)
        layout.addWidget(_section_title("示教控制"))

        upper = QHBoxLayout()
        upper.setSpacing(8)
        parameters = _subcard()
        parameters.setFixedWidth(190)
        parameter_layout = QVBoxLayout(parameters)
        parameter_layout.setContentsMargins(12, 11, 12, 11)
        parameter_layout.addWidget(_section_title("控制参数"))
        self.parameter_target = QLabel("当前对象：T1")
        self.parameter_target.setObjectName("value")
        parameter_layout.addWidget(self.parameter_target)
        parameter_layout.addSpacing(8)
        parameter_layout.addWidget(QLabel("控制角度"))
        self.angle_value = QLabel("0°")
        self.angle_value.setObjectName("value")
        parameter_layout.addWidget(self.angle_value)
        self.bend_value = QLabel("Bend  0%")
        parameter_layout.addWidget(self.bend_value)
        self.bend_slider = QSlider(Qt.Horizontal)
        self.bend_slider.setRange(0, 100)
        self.bend_slider.setEnabled(False)
        parameter_layout.addWidget(self.bend_slider)
        self.stiff_value = QLabel("Stiffness  0%")
        parameter_layout.addWidget(self.stiff_value)
        self.stiff_slider = QSlider(Qt.Horizontal)
        self.stiff_slider.setRange(0, 100)
        self.stiff_slider.setEnabled(False)
        parameter_layout.addWidget(self.stiff_slider)
        parameter_layout.addStretch(1)
        self.parameter_lock = QPushButton("🔒 参数已锁定")
        self.parameter_lock.setEnabled(False)
        parameter_layout.addWidget(self.parameter_lock)

        pose = _subcard()
        pose_layout = QVBoxLayout(pose)
        pose_layout.setContentsMargins(5, 5, 5, 5)
        pose_title = _section_title("实际触手姿态")
        pose_layout.addWidget(pose_title)
        self.pose_view = TentacleViewport(tentacle_index=0, vertical_layout=True)
        self.pose_view.set_state(0.0, 0.0, online=False, active=False)
        self.pose_view.setToolTip("等待H7实际姿态反馈；滚轮仅调整显示缩放")
        pose_layout.addWidget(self.pose_view, 1)
        upper.addWidget(parameters)
        upper.addWidget(pose, 1)
        layout.addLayout(upper, 11)

        dial = _subcard()
        dial_layout = QVBoxLayout(dial)
        dial_layout.setContentsMargins(5, 5, 5, 5)
        self.direction_dial = DirectionDial()
        self.direction_dial.locked = True
        self.direction_dial.setToolTip("拖动调整方向；滚轮调整Bend，Shift+滚轮步长为5")
        dial_layout.addWidget(self.direction_dial, 1)
        dial_info = QHBoxLayout()
        dial_info.addStretch(1)
        self.dial_value = QLabel("方向 0°    X:100  Y:0")
        self.dial_value.setObjectName("value")
        dial_info.addWidget(self.dial_value)
        self.home_button = QPushButton("回正")
        self.home_button.setEnabled(False)
        dial_info.addWidget(self.home_button)
        dial_info.addStretch(1)
        dial_layout.addLayout(dial_info)
        layout.addWidget(dial, 9)
        return panel

    def _build_episode_panel(self) -> QWidget:
        panel = _card()
        panel.setFixedWidth(320)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(13, 12, 13, 12)
        layout.setSpacing(9)
        layout.addWidget(_section_title("Episode采集"))
        self.capture_state = _status_label("● 空闲")
        layout.addWidget(self.capture_state)
        self.capture_timer_label = QLabel("00:00.00")
        self.capture_timer_label.setObjectName("timer")
        self.capture_timer_label.setAlignment(Qt.AlignCenter)
        self.capture_timer_label.setFixedHeight(48)
        layout.addWidget(self.capture_timer_label)

        stats = QGridLayout()
        for row, (name, value) in enumerate(
            (("帧数", "0"), ("频率", "—"), ("丢帧", "0"), ("无效", "0"))
        ):
            stats.addWidget(QLabel(name), row, 0)
            value_label = QLabel(value)
            value_label.setObjectName("value")
            stats.addWidget(value_label, row, 1)
            if row == 0:
                self.capture_frame_label = value_label
            elif row == 1:
                self.capture_rate_label = value_label
            elif row == 2:
                self.capture_drop_label = value_label
            else:
                self.capture_invalid_label = value_label
        layout.addLayout(stats)

        row = QHBoxLayout()
        self.start_capture_button = QPushButton("开始采集")
        self.stop_capture_button = QPushButton("停止采集")
        self.start_capture_button.setEnabled(False)
        self.stop_capture_button.setEnabled(False)
        row.addWidget(self.start_capture_button)
        row.addWidget(self.stop_capture_button)
        layout.addLayout(row)

        layout.addWidget(QLabel("Episode结果"))
        result_row = QHBoxLayout()
        self.success_button = QPushButton("成功")
        self.failure_button = QPushButton("失败")
        self.success_button.setCheckable(True)
        self.failure_button.setCheckable(True)
        self.success_button.setEnabled(False)
        self.failure_button.setEnabled(False)
        result_row.addWidget(self.success_button)
        result_row.addWidget(self.failure_button)
        layout.addLayout(result_row)

        form = QFormLayout()
        self.position_combo = QComboBox()
        self.position_combo.addItems(("左侧", "中间", "右侧"))
        self.distance_combo = QComboBox()
        self.distance_combo.addItems(("近", "中等", "远"))
        self.tilt_combo = QComboBox()
        self.tilt_combo.addItems(("左倾", "竖直", "右倾"))
        for widget in (self.position_combo, self.distance_combo, self.tilt_combo):
            widget.setEnabled(False)
        form.addRow("位置", self.position_combo)
        form.addRow("距离", self.distance_combo)
        form.addRow("倾斜", self.tilt_combo)
        layout.addLayout(form)
        layout.addWidget(QLabel("备注"))
        self.notes = QPlainTextEdit()
        self.notes.setEnabled(False)
        self.notes.setMinimumHeight(90)
        layout.addWidget(self.notes, 1)
        save_row = QHBoxLayout()
        self.save_button = QPushButton("保存")
        self.delete_button = QPushButton("删除")
        self.save_button.setEnabled(False)
        self.delete_button.setEnabled(False)
        save_row.addWidget(self.save_button)
        save_row.addWidget(self.delete_button)
        layout.addLayout(save_row)
        return panel

    def _build_bottom_bar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("bottomBar")
        bar.setFixedHeight(44)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(8, 4, 10, 4)
        layout.setSpacing(6)
        self.tof_button = QPushButton("ToF热力图")
        self.quality_button = QPushButton("数据质量")
        self.diagnostic_button = QPushButton("协议诊断")
        self.tof_button.setEnabled(False)
        self.quality_button.setEnabled(True)
        layout.addWidget(self.tof_button)
        layout.addWidget(self.quality_button)
        layout.addWidget(self.diagnostic_button)
        layout.addStretch(1)
        self.bottom_message = QLabel("等待连接")
        self.bottom_message.setObjectName("muted")
        self.bottom_message.setFixedWidth(360)
        self.bottom_message.setWordWrap(False)
        layout.addWidget(self.bottom_message)
        self.rx_tx_label = QLabel("RX 0 B   TX 0 B")
        self.crc_label = QLabel("CRC 0")
        self.drop_label = QLabel("Drop 0")
        for widget in (self.rx_tx_label, self.crc_label, self.drop_label):
            widget.setMinimumWidth(85)
            layout.addWidget(widget)
        return bar

    def _connect_signals(self) -> None:
        self.refresh_button.clicked.connect(self.serial.refresh_ports)
        self.connect_button.clicked.connect(self._toggle_connection)
        self.advanced_button.clicked.connect(self._show_advanced_settings)
        self.tentacle_combo.activated.connect(self._select_tentacle)
        self.enable_control_button.pressed.connect(self._begin_enable_hold)
        self.enable_control_button.released.connect(self._cancel_enable_hold)
        self.stop_control_button.clicked.connect(self._request_disable_control)
        self.emergency_button.clicked.connect(self._request_emergency_stop)
        self.parameter_lock.clicked.connect(self._toggle_parameter_lock)
        self.direction_dial.angle_changed.connect(self._direction_changed)
        self.direction_dial.bend_wheel.connect(self._bend_wheel)
        self.bend_slider.valueChanged.connect(self._bend_changed)
        self.stiff_slider.valueChanged.connect(self._stiffness_changed)
        self.home_button.clicked.connect(self._request_home)
        self.start_capture_button.clicked.connect(self._request_start_capture)
        self.stop_capture_button.clicked.connect(self._request_stop_capture)
        self.success_button.clicked.connect(lambda: self._select_episode_result("success"))
        self.failure_button.clicked.connect(lambda: self._select_episode_result("failure"))
        self.save_button.clicked.connect(self._save_episode)
        self.delete_button.clicked.connect(self._delete_episode)
        self.tof_button.clicked.connect(self._show_tof_heatmap)
        self.diagnostic_button.clicked.connect(self._show_protocol_diagnostics)
        self.quality_button.clicked.connect(self._show_data_quality)
        self.serial.ports_changed.connect(self._update_ports)
        self.serial.status_changed.connect(self._serial_status_changed)
        self.serial.reconnecting.connect(self._serial_reconnecting)
        self.serial.received.connect(self._receive_bytes)
        self.serial.transmitted.connect(self._transmitted)
        self.serial.error.connect(self._serial_error)

    def _set_protocol_controls(self, synced: bool) -> None:
        blocking_request = (
            self.pending is not None
            and self.pending.command_id != CommandId.STATUS
        )
        command_ready = synced and not blocking_request
        episode_clear = self._capture_state == CaptureUiState.IDLE
        self.tentacle_combo.setEnabled(
            command_ready and not self._control_enabled and episode_clear
        )
        self.enable_control_button.setEnabled(command_ready and not self._control_enabled)
        self.stop_control_button.setEnabled(synced and self._control_enabled)
        self.emergency_button.setEnabled(self.serial.is_open)
        self.parameter_lock.setEnabled(command_ready and self._control_enabled)
        self._update_control_button_visuals(command_ready)
        self._apply_parameter_interlocks()
        self._update_capture_interlocks()

    def _update_capture_interlocks(self) -> None:
        """统一维护采集、标注和保存按钮，防止状态交叉覆盖。"""

        # STATUS只是后台姿态/状态刷新，不能让采集按钮每200ms失能一次。
        command_idle = (
            self.pending is None
            or self.pending.command_id == CommandId.STATUS
        )
        synced = self.serial.is_open and self._h7_synced
        self.start_capture_button.setEnabled(
            synced and command_idle and self._capture_state == CaptureUiState.IDLE
        )
        self.stop_capture_button.setEnabled(
            synced and command_idle and self._capture_state == CaptureUiState.CAPTURING
        )
        reviewing = self._capture_state == CaptureUiState.REVIEW
        for widget in (
            self.success_button,
            self.failure_button,
            self.position_combo,
            self.distance_combo,
            self.tilt_combo,
            self.notes,
            self.delete_button,
        ):
            widget.setEnabled(reviewing)
        self.save_button.setEnabled(
            reviewing
            and self._episode is not None
            and self._episode.has_frames
            and self._episode_result is not None
        )

    def _apply_parameter_interlocks(self) -> None:
        """按照同步、控制权和参数锁统一更新全部运动输入。"""

        editable = (
            self._h7_synced
            and self._control_enabled
            and (
                self.pending is None
                or self.pending.command_id == CommandId.STATUS
            )
            and not self._parameters_locked
        )
        self.bend_slider.setEnabled(editable)
        self.stiff_slider.setEnabled(editable)
        self.direction_dial.locked = not editable
        self.home_button.setEnabled(editable)
        self.parameter_lock.setText(
            "🔒 参数已锁定（L解锁）"
            if self._parameters_locked else "✓ 参数可编辑（L锁定）"
        )
        self.parameter_lock.setObjectName(
            "parameterLocked" if self._parameters_locked else "parameterEditable"
        )
        self.parameter_lock.setToolTip(
            "点击或按L解锁参数"
            if self._parameters_locked else "点击或按L立即锁定参数"
        )
        self._repolish(self.parameter_lock)

    def _update_control_button_visuals(self, command_ready: bool) -> None:
        """统一显示控制启用、长按进度和停止状态。"""

        if self._control_enabled:
            self.enable_control_button.setText("✓ 控制已启用")
            self.enable_control_button.setObjectName("controlEnabled")
            self.stop_control_button.setText("■ 停止控制（Space）")
            self.stop_control_button.setObjectName("controlStopActive")
        elif self._enable_hold_active:
            self.enable_control_button.setObjectName("controlHolding")
            self.stop_control_button.setText("■ 停止控制（Space）")
            self.stop_control_button.setObjectName("")
        elif self.pending is not None and self.pending.command_id == CommandId.ENABLE:
            self.enable_control_button.setText("正在等待H7确认…")
            self.enable_control_button.setObjectName("controlHolding")
            self.stop_control_button.setObjectName("")
        else:
            self.enable_control_button.setText("按住 0.8 秒启用（Space）")
            self.enable_control_button.setObjectName("controlEnableReady" if command_ready else "")
            self.stop_control_button.setText("■ 停止控制（Space）")
            self.stop_control_button.setObjectName("")
        self._repolish(self.enable_control_button)
        self._repolish(self.stop_control_button)

    def _set_control_enabled(self, enabled: bool) -> None:
        """采用H7确认状态更新控制权，不根据按钮点击提前假设成功。"""

        self._control_enabled = bool(enabled)
        if not enabled:
            self._parameters_locked = True
            self._move_dirty = False
        self.control_owner.setText("当前：已启用" if enabled else "当前：未启用")
        self.control_owner.setObjectName("statusGood" if enabled else "statusOff")
        self._set_protocol_controls(self._h7_synced)
        self._refresh_dynamic_styles()
        self._update_data_quality()

    def _begin_enable_hold(self) -> None:
        """开始800ms长按计时，并给出可见的剩余时间反馈。"""

        if (
            not self._h7_synced
            or self._control_enabled
            or (
                self.pending is not None
                and self.pending.command_id != CommandId.STATUS
            )
        ):
            return
        self._enable_hold_active = True
        self._enable_hold_elapsed_ms = 0
        self.enable_control_button.setText("继续按住 0.8s")
        self.enable_control_button.setObjectName("controlHolding")
        self._repolish(self.enable_control_button)
        self._enable_hold_timer.start()
        self._enable_hold_progress.start()

    def _update_enable_hold_progress(self) -> None:
        if not self._enable_hold_active:
            self._enable_hold_progress.stop()
            return
        self._enable_hold_elapsed_ms = min(700, self._enable_hold_elapsed_ms + 100)
        remaining = (800 - self._enable_hold_elapsed_ms) / 1000.0
        self.enable_control_button.setText(f"继续按住 {remaining:.1f}s")

    def _cancel_enable_hold(self) -> None:
        if not self._enable_hold_active:
            return
        self._enable_hold_active = False
        self._enable_hold_timer.stop()
        self._enable_hold_progress.stop()
        if (
            not self._control_enabled
            and (
                self.pending is None
                or self.pending.command_id == CommandId.STATUS
            )
        ):
            self._update_control_button_visuals(True)
            self.bottom_message.setText("启动已取消：需要持续按住800ms")

    def _enable_hold_complete(self) -> None:
        if not self._enable_hold_active:
            return
        self._enable_hold_active = False
        self._enable_hold_progress.stop()
        self.enable_control_button.setText("正在请求控制…")
        self.enable_control_button.setObjectName("controlHolding")
        self._repolish(self.enable_control_button)
        if self.pending is not None and self.pending.command_id == CommandId.STATUS:
            self._clear_pending()
        self._send_request(
            CommandId.ENABLE,
            encode_enable(True),
            retry_allowed=False,
            enable_target=True,
        )

    def _request_disable_control(self) -> None:
        """正常停止当前ACT控制；与最高优先级紧急停止分开。"""

        if not self._h7_synced or not self._control_enabled:
            return
        if self.pending is not None and self.pending.command_id != CommandId.STATUS:
            self.bottom_message.setText("当前命令尚未完成，请使用紧急停止")
            return
        if self.pending is not None:
            self._clear_pending()
        self._parameters_locked = True
        self._move_dirty = False
        self._set_bottom_message("正在停止ACT控制")
        self._send_request(
            CommandId.ENABLE,
            encode_enable(False),
            retry_allowed=True,
            enable_target=False,
        )

    def _request_emergency_stop(self) -> None:
        """抢占普通请求并发送ACT STOP，保证STOP不被状态查询阻塞。"""

        if not self.serial.is_open:
            return
        if self.pending is not None:
            self._request_timer.stop()
            self.pending = None
        self._parameters_locked = True
        self._move_dirty = False
        self.bottom_message.setText("正在执行紧急停止")
        self._send_request(CommandId.STOP, encode_stop(), retry_allowed=True)

    def _toggle_parameter_lock(self) -> None:
        if not self._h7_synced:
            self._set_bottom_message("L无效：请先连接并同步H7")
            return
        if not self._control_enabled:
            self._set_bottom_message("L无效：请先长按Space启用ACT控制")
            return
        if self.pending is not None and self.pending.command_id != CommandId.STATUS:
            self._set_bottom_message("L暂不可用：正在等待当前命令完成")
            return
        self._parameters_locked = not self._parameters_locked
        self._apply_parameter_interlocks()
        self.bottom_message.setText(
            "参数已锁定" if self._parameters_locked else "参数已解锁，可以调整目标"
        )

    def _direction_changed(self, angle: float) -> None:
        if self.direction_dial.locked:
            return
        self._target_angle = angle % 360.0
        self._update_target_display()
        self._move_dirty = True

    def _bend_changed(self, value: int) -> None:
        self._target_bend = int(value)
        self._update_target_display()
        if self.bend_slider.isEnabled():
            self._move_dirty = True

    def _stiffness_changed(self, value: int) -> None:
        self._target_stiffness = int(value)
        self._update_target_display()
        if self.stiff_slider.isEnabled():
            self._move_dirty = True

    def _bend_wheel(self, steps: int, fast: bool) -> None:
        if self.direction_dial.locked:
            return
        step = 5 if fast else 1
        self.bend_slider.setValue(self.bend_slider.value() + steps * step)

    def _target_xy(self) -> tuple[int, int]:
        radians = math.radians(self._target_angle)
        return (
            int(round(math.cos(radians) * 100.0)),
            int(round(math.sin(radians) * 100.0)),
        )

    def _update_target_display(self) -> None:
        x, y = self._target_xy()
        self.angle_value.setText(f"{self._target_angle:.0f}°")
        self.bend_value.setText(f"Bend  {self._target_bend}%")
        self.stiff_value.setText(f"Stiffness  {self._target_stiffness}%")
        self.dial_value.setText(
            f"方向 {self._target_angle:.0f}°    X:{x}  Y:{y}"
        )

    def _send_latest_move(self) -> None:
        """最多20Hz发送一次最新目标；MOVE不进入ACK等待队列。"""

        if (
            not self._move_dirty
            or not self.serial.is_open
            or not self._h7_synced
            or not self._control_enabled
            or self._parameters_locked
            or (
                self.pending is not None
                and self.pending.command_id != CommandId.STATUS
            )
        ):
            return
        x, y = self._target_xy()
        payload = encode_move(x, y, self._target_bend, self._target_stiffness)
        if self.serial.write(payload):
            self._move_dirty = False

    def _request_home(self) -> None:
        if (
            not self._h7_synced
            or not self._control_enabled
            or self._parameters_locked
            or (
                self.pending is not None
                and self.pending.command_id != CommandId.STATUS
            )
        ):
            return
        if self.pending is not None:
            self._clear_pending()
        self._move_dirty = False
        self._send_request(CommandId.HOME, encode_home(), retry_allowed=False)

    def _set_capture_state(self, state: CaptureUiState, message: str) -> None:
        self._capture_state = state
        style = {
            CaptureUiState.IDLE: "statusOff",
            CaptureUiState.STARTING: "statusWait",
            CaptureUiState.CAPTURING: "statusGood",
            CaptureUiState.STOPPING: "statusWait",
            CaptureUiState.REVIEW: "statusGood",
            CaptureUiState.SAVING: "statusWait",
        }[state]
        self.capture_state.setText(f"● {message}")
        self.capture_state.setObjectName(style)
        top_text = {
            CaptureUiState.IDLE: "Episode：空闲",
            CaptureUiState.STARTING: "Episode：启动中",
            CaptureUiState.CAPTURING: "Episode：采集中",
            CaptureUiState.STOPPING: "Episode：停止中",
            CaptureUiState.REVIEW: "Episode：等待保存",
            CaptureUiState.SAVING: "Episode：保存中",
        }[state]
        self.top_episode.setText(top_text)
        self.top_episode.setObjectName(style)
        for widget in (self.capture_state, self.top_episode):
            widget.style().unpolish(widget)
            widget.style().polish(widget)
        self._set_protocol_controls(self._h7_synced)

    def _request_start_capture(self) -> None:
        """建立本地Episode后请求H7开启ACT_FRAME，确保首帧不会丢失。"""

        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            baud = 0
        if baud != 1_000_000:
            self._report_alert(
                "完整ACT_FRAME采集必须使用1000000波特率",
                "error",
                "capture_baud",
            )
            return
        if (
            not self._h7_synced
            or (
                self.pending is not None
                and self.pending.command_id != CommandId.STATUS
            )
            or self._capture_state != CaptureUiState.IDLE
        ):
            return
        if self.pending is not None:
            # 用户采集操作优先于后台STATUS；迟到的STATUS仍可安全解析。
            self._clear_pending()
        self._episode = EpisodeBuffer(
            self._confirmed_tentacle,
            0,
            0,
        )
        stats = self.parser.stats
        self._capture_parser_start = (
            stats.crc_errors,
            stats.length_errors,
            stats.version_errors,
            stats.discarded_bytes,
        )
        self._episode_result = None
        self.success_button.setChecked(False)
        self.failure_button.setChecked(False)
        self._last_frame_monotonic = 0.0
        self._last_capture_status_query = 0.0
        self._latest_display_frame = None
        self._displayed_frame_seq = -1
        self._capture_timeout_reported = False
        self._orphan_stop_active = False
        self._set_capture_state(CaptureUiState.STARTING, "正在请求H7开始采集")
        self._set_bottom_message("正在开启ACT_FRAME数据流")
        if not self._send_request(
            CommandId.CAPTURE,
            encode_capture(True),
            retry_allowed=False,
            capture_target=True,
        ):
            self._episode = None
            self._set_capture_state(CaptureUiState.IDLE, "空闲")
            self._report_alert("采集启动命令发送失败", "error", "capture_start")

    def _request_stop_capture(self) -> None:
        """只停止ACT_FRAME采集，不隐式停止电机控制。"""

        if (
            not self._h7_synced
            or (
                self.pending is not None
                and self.pending.command_id != CommandId.STATUS
            )
            or self._capture_state != CaptureUiState.CAPTURING
        ):
            return
        if self.pending is not None:
            # 停止采集同样优先，避免后台STATUS造成一次无反馈点击。
            self._clear_pending()
        self._set_capture_state(CaptureUiState.STOPPING, "正在停止采集")
        self._set_bottom_message("正在停止采集；ACT电机控制保持当前状态")
        if not self._send_request(
            CommandId.CAPTURE,
            encode_capture(False),
            retry_allowed=True,
            capture_target=False,
        ):
            self._set_capture_state(CaptureUiState.CAPTURING, "采集中")
            self._report_alert("采集停止命令发送失败", "error", "capture_stop")

    def _select_episode_result(self, result: str) -> None:
        if self._capture_state != CaptureUiState.REVIEW:
            return
        self._episode_result = result
        self.success_button.setChecked(result == "success")
        self.failure_button.setChecked(result == "failure")
        self._set_bottom_message("Episode结果：成功" if result == "success" else "Episode结果：失败")
        self._update_capture_interlocks()

    def _enter_episode_review(self, reason: str = "") -> None:
        self._orphan_stop_active = False
        if self._episode is None:
            self._set_capture_state(CaptureUiState.IDLE, "空闲")
            return
        if reason:
            self._episode.mark_interrupted(reason)
        self._set_capture_state(CaptureUiState.REVIEW, "等待标注并保存")
        self._set_bottom_message(
            f"采集完成，共{self._episode.frame_count}帧；请选择结果后保存"
        )

    def _save_episode(self) -> None:
        if (
            self._capture_state != CaptureUiState.REVIEW
            or self._episode is None
            or not self._episode.has_frames
            or self._episode_result is None
            or self._save_future is not None
        ):
            return
        self._episode.set_annotation(
            self._episode_result,
            self.position_combo.currentText(),
            self.distance_combo.currentText(),
            self.tilt_combo.currentText(),
            self.notes.toPlainText(),
        )
        stats = self.parser.stats
        crc0, length0, version0, discarded0 = self._capture_parser_start
        descriptor = self.port_combo.currentData()
        snapshot = self._episode.snapshot(
            {
                "serial": {
                    "baud": int(self.baud_combo.currentText()),
                    "port": descriptor.name if isinstance(descriptor, PortDescriptor) else "",
                },
                "pc_protocol_errors": {
                    "crc": max(0, stats.crc_errors - crc0),
                    "length": max(0, stats.length_errors - length0),
                    "version": max(0, stats.version_errors - version0),
                    "discarded_bytes": max(0, stats.discarded_bytes - discarded0),
                },
            }
        )
        self._set_capture_state(CaptureUiState.SAVING, "正在后台保存")
        self._save_future = self._save_executor.submit(
            save_validated_episode, snapshot, self._episode_root()
        )
        self._save_poll_timer.start()

    def _poll_save_result(self) -> None:
        future = self._save_future
        if future is None or not future.done():
            return
        self._save_poll_timer.stop()
        self._save_future = None
        try:
            saved_path, compatibility = future.result()
        except Exception as exc:
            self._set_capture_state(CaptureUiState.REVIEW, "保存失败，可重试")
            self._report_alert(f"Episode保存失败：{exc}", "error", "episode_save")
            return
        self._episode = None
        self._episode_result = None
        self.notes.clear()
        self.success_button.setChecked(False)
        self.failure_button.setChecked(False)
        self._reset_capture_summary()
        self._set_capture_state(CaptureUiState.IDLE, "空闲")
        if not compatibility.schema_valid:
            self._report_alert(
                "Episode已保存，但训练格式校验失败：" + "; ".join(compatibility.errors),
                "error",
                "training_compatibility",
            )
        elif compatibility.valid_window_count == 0:
            detail = "; ".join(compatibility.warnings) or "没有有效训练窗口"
            self._report_alert(
                f"Episode已保存，但暂不可训练：{detail}",
                "warning",
                "training_compatibility",
            )
        else:
            self._set_bottom_message(
                f"Episode已保存且训练校验通过：{compatibility.valid_window_count}个有效窗口；{saved_path}"
            )

    def _delete_episode(self) -> None:
        if self._capture_state != CaptureUiState.REVIEW or self._episode is None:
            return
        answer = QMessageBox.question(
            self,
            "删除未保存Episode",
            f"确认删除当前未保存的{self._episode.frame_count}帧数据？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self._episode.clear()
        self._episode = None
        self._episode_result = None
        self.notes.clear()
        self.success_button.setChecked(False)
        self.failure_button.setChecked(False)
        self._reset_capture_summary()
        self._set_capture_state(CaptureUiState.IDLE, "空闲")
        self._set_bottom_message("未保存Episode已删除")

    @staticmethod
    def _episode_root() -> Path:
        if getattr(sys, "frozen", False):
            project_root = Path(sys.executable).resolve().parent
        else:
            project_root = Path(__file__).resolve().parents[1]
        return project_root / "datasets" / "raw"

    def _reset_capture_summary(self) -> None:
        self.capture_timer_label.setText("00:00.00")
        self.capture_frame_label.setText("0")
        self.capture_rate_label.setText("—")
        self.capture_drop_label.setText("0")
        self.capture_invalid_label.setText("0")

    def _handle_act_frame(self, frame: ActFrame) -> None:
        if self._orphan_stop_active:
            return
        self._update_actual_pose(
            ActMotorState(
                tentacle=frame.tentacle,
                valid_flags=frame.valid_flags,
                status_age_ms=frame.status_age_ms,
                actual_q=frame.actual_q,
                action_q=frame.action_q,
            )
        )
        if self._capture_state not in (
            CaptureUiState.STARTING,
            CaptureUiState.CAPTURING,
            CaptureUiState.STOPPING,
        ) or self._episode is None:
            return
        try:
            appended = self._episode.append(frame)
        except ValueError as exc:
            self._report_alert(str(exc), "error", "capture_frame")
            return
        if not appended:
            return
        self._last_frame_monotonic = time.monotonic()
        self._capture_timeout_reported = False
        self._latest_display_frame = frame
        self.tof_button.setEnabled(True)
        if frame.fault:
            self._report_alert("ACT_FRAME报告所选触手电机FAULT", "error", "frame_fault")

    def _update_actual_pose(self, motor: ActMotorState) -> None:
        """用H7实际电机快照刷新姿态；该路径与Episode采集状态完全独立。"""

        if motor.tentacle != self._confirmed_tentacle:
            return
        if motor.position_valid:
            angle, bend = self._pose_from_actual_q(motor.actual_q)
            self.pose_view.set_state(
                angle,
                bend,
                online=True,
                active=self._control_enabled and not motor.fault,
            )
            self.pose_view.setToolTip(
                f"T{motor.tentacle} 实际位置反馈，状态年龄 {motor.status_age_ms} ms；滚轮仅调整显示缩放"
            )
        else:
            self.pose_view.set_state(0.0, 0.0, online=False, active=False)
            self.pose_view.setToolTip(
                f"T{motor.tentacle} 实际坐标尚未恢复可信；状态年龄 {motor.status_age_ms} ms"
            )

    @staticmethod
    def _pose_from_actual_q(actual_q: np.ndarray) -> tuple[float, float]:
        cable_x = (-26.0, -71.0, 97.0)
        cable_y = (97.0, -71.0, -26.0)
        values = [float(value) for value in actual_q]
        mean_q = sum(values) / 3.0
        differential = [value - mean_q for value in values]
        vector_x = sum(value * axis for value, axis in zip(differential, cable_x))
        vector_y = sum(value * axis for value, axis in zip(differential, cable_y))
        angle = 0.0 if abs(vector_x) + abs(vector_y) < 1e-6 else math.degrees(math.atan2(vector_y, vector_x)) % 360.0
        amplitude_q = math.hypot(vector_x, vector_y) / 15000.0
        bend = max(0.0, min(100.0, amplitude_q * 10000.0 / 27000.0))
        return angle, bend

    def _update_capture_ui(self) -> None:
        episode = self._episode
        if episode is not None:
            elapsed = episode.elapsed_seconds
            minutes = int(elapsed // 60)
            seconds = elapsed - minutes * 60
            self.capture_timer_label.setText(f"{minutes:02d}:{seconds:05.2f}")
            quality = episode.quality()
            self.capture_frame_label.setText(str(quality.frame_count))
            self.capture_rate_label.setText(
                f"{quality.receive_hz:.1f} Hz" if quality.frame_count > 1 else "—"
            )
            h7_drop = max(0, episode.end_h7_drop - episode.start_h7_drop)
            self.capture_drop_label.setText(
                f"序号 {quality.sequence_gap_count} / H7 {h7_drop}"
            )
            self.capture_invalid_label.setText(str(quality.invalid_position_count))
        frame = self._latest_display_frame
        if (
            frame is not None
            and frame.frame_seq != self._displayed_frame_seq
            and self._tof_heatmap is not None
        ):
            self._tof_heatmap.set_frame(frame)
            self._displayed_frame_seq = frame.frame_seq
        silence_seconds = (
            time.monotonic() - self._last_frame_monotonic
            if self._last_frame_monotonic > 0
            else (episode.elapsed_seconds if episode is not None else 0.0)
        )
        if (
            self._capture_state == CaptureUiState.CAPTURING
            and silence_seconds > 2.0
            and not self._capture_timeout_reported
        ):
            self._capture_timeout_reported = True
            self._report_alert("超过2秒未收到ACT_FRAME，请检查ToF和串口带宽", "warning", "frame_timeout")

    def _show_tof_heatmap(self) -> None:
        if self._tof_dialog is not None:
            self._tof_dialog.raise_()
            self._tof_dialog.activateWindow()
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("ToF 8×8实时热力图")
        dialog.resize(620, 650)
        layout = QVBoxLayout(dialog)
        layout.addWidget(QLabel("数值单位：mm；无效区域显示为 —"))
        heatmap = TofHeatmap()
        layout.addWidget(heatmap, 1)
        close_button = QPushButton("关闭")
        close_button.clicked.connect(dialog.close)
        layout.addWidget(close_button)
        if self._latest_display_frame is not None:
            heatmap.set_frame(self._latest_display_frame)
        dialog.finished.connect(self._tof_closed)
        self._tof_dialog = dialog
        self._tof_heatmap = heatmap
        dialog.show()

    def _tof_closed(self) -> None:
        self._tof_dialog = None
        self._tof_heatmap = None

    def _toggle_connection(self) -> None:
        if self.serial.desired_open:
            self.serial.close()
            self.connect_button.setText("连接")
            return
        descriptor = self.port_combo.currentData()
        if not isinstance(descriptor, PortDescriptor):
            self.bottom_message.setText("请先选择真实串口")
            return
        try:
            baud = int(self.baud_combo.currentText().strip())
        except ValueError:
            self.bottom_message.setText("波特率必须是整数")
            return
        if baud <= 0:
            self.bottom_message.setText("波特率必须大于0")
            return
        self._reset_protocol_state("正在打开串口")
        self.connect_button.setText("取消连接")
        self.port_combo.setEnabled(False)
        self.baud_combo.setEnabled(False)
        self.serial.open(SerialSettings(descriptor.name, baud))
        if baud != 1_000_000:
            self._report_alert(
                "当前波特率仅建议用于小包调试，ACT采集需切换到1000000",
                "warning",
                "serial_baud",
            )

    def _update_ports(self, ports: list[PortDescriptor]) -> None:
        previous = self.port_combo.currentData()
        previous_name = previous.name if isinstance(previous, PortDescriptor) else ""
        self._ports = ports
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        if not ports:
            self.port_combo.addItem("未发现可用串口", None)
        else:
            for descriptor in ports:
                self.port_combo.addItem(descriptor.display_name, descriptor)
                details = []
                if descriptor.manufacturer:
                    details.append(descriptor.manufacturer)
                if descriptor.vendor_id is not None and descriptor.product_id is not None:
                    details.append(f"VID:PID {descriptor.vendor_id:04X}:{descriptor.product_id:04X}")
                if descriptor.serial_number:
                    details.append(f"SN {descriptor.serial_number}")
                self.port_combo.setItemData(
                    self.port_combo.count() - 1,
                    "\n".join(details),
                    Qt.ToolTipRole,
                )
            index = next((i for i, port in enumerate(ports) if port.name == previous_name), 0)
            self.port_combo.setCurrentIndex(index)
        self.port_combo.blockSignals(False)
        self.connect_button.setEnabled(bool(ports) or self.serial.desired_open)

    def _serial_status_changed(self, opened: bool, message: str) -> None:
        self._log(message)
        if opened:
            self.top_serial.setText("● 串口已打开")
            self.top_serial.setObjectName("statusGood")
            self.serial_state.setText("● 串口已打开")
            self.serial_state.setObjectName("statusGood")
            self.connect_button.setText("断开")
            self._set_bottom_message("串口已打开，正在同步H7")
            self._reset_protocol_state("等待H7响应")
            QTimer.singleShot(80, self._start_handshake)
        else:
            self._reset_protocol_state(message)
            self.top_serial.setText("● 串口未连接")
            self.top_serial.setObjectName("statusOff")
            self.serial_state.setText("● 串口未连接")
            self.serial_state.setObjectName("statusOff")
            self.port_combo.setEnabled(not self.serial.desired_open)
            self.baud_combo.setEnabled(not self.serial.desired_open)
            self.connect_button.setText("取消重连" if self.serial.desired_open else "连接")
            self._set_bottom_message(message)
        self._refresh_dynamic_styles()
        self._update_data_quality()

    def _serial_reconnecting(self, message: str) -> None:
        self._reset_protocol_state(message)
        self.top_serial.setText("● 串口重连中")
        self.top_serial.setObjectName("statusWait")
        self.serial_state.setText("● 串口重连中")
        self.serial_state.setObjectName("statusWait")
        self.connect_button.setText("取消重连")
        self._report_alert(message, "warning", "serial_reconnect")
        self._log(message)
        self._refresh_dynamic_styles()

    def _start_handshake(self) -> None:
        if not self.serial.is_open or self.pending is not None:
            return
        self._ping_confirmed = False
        self._set_h7_synced(False, "● H7同步中")
        self._send_request(CommandId.PING, encode_ping(), retry_allowed=True)

    def _send_request(
        self,
        command_id: CommandId,
        payload: bytes,
        retry_allowed: bool,
        select_target: int | None = None,
        enable_target: bool | None = None,
        capture_target: bool | None = None,
    ) -> bool:
        if self.pending is not None or not self.serial.is_open:
            return False
        if not self.serial.write(payload):
            return False
        self.pending = PendingRequest(
            command_id,
            payload,
            retry_allowed,
            0,
            select_target,
            enable_target,
            capture_target,
        )
        self._request_timer.start(self.REQUEST_TIMEOUT_MS)
        self._set_protocol_controls(self._h7_synced)
        self._log(f"TX {payload.decode('ascii').strip()}")
        return True

    def _clear_pending(self) -> PendingRequest | None:
        pending = self.pending
        self.pending = None
        self._request_timer.stop()
        self._set_protocol_controls(self._h7_synced)
        return pending

    def _on_request_timeout(self) -> None:
        pending = self.pending
        if pending is None:
            return
        if pending.retry_allowed and pending.retries == 0 and self.serial.is_open:
            pending.retries += 1
            self.serial.write(pending.payload)
            self._request_timer.start(self.REQUEST_TIMEOUT_MS)
            self._log(f"RETRY {pending.command_id.name}")
            return
        failed = self._clear_pending()
        if failed and failed.command_id == CommandId.SELECT:
            self._restore_confirmed_tentacle()
        if failed and failed.command_id == CommandId.CAPTURE:
            frame_recent = (
                self._last_frame_monotonic > 0
                and time.monotonic() - self._last_frame_monotonic < 1.0
            )
            if failed.capture_target and self._episode is not None and self._episode.has_frames:
                self._set_capture_state(CaptureUiState.CAPTURING, "采集中（ACK丢失）")
                self._report_alert(
                    "CAPTURE启动ACK超时，但ACT_FRAME持续到达，继续采集",
                    "warning",
                    "capture_ack",
                )
                return
            if failed.capture_target:
                self._episode = None
                self._set_capture_state(CaptureUiState.IDLE, "启动超时")
            elif frame_recent:
                self._set_capture_state(CaptureUiState.CAPTURING, "停止未确认，仍在采集")
                self._report_alert(
                    "CAPTURE停止响应超时且数据仍在到达，请重试停止",
                    "warning",
                    "capture_ack",
                )
                return
            else:
                self._enter_episode_review("CAPTURE停止响应超时")
        self._enable_hold_active = False
        self._enable_hold_timer.stop()
        self._enable_hold_progress.stop()
        self._set_h7_synced(False, "● H7无响应")
        self._report_alert(
            f"{pending.command_id.name}等待响应超时",
            "error",
            "protocol_timeout",
        )
        self._log(f"TIMEOUT {pending.command_id.name}")

    def _heartbeat(self) -> None:
        if not self.serial.is_open or self.pending is not None:
            return
        if self._h7_synced:
            if self._capture_state == CaptureUiState.CAPTURING:
                now = time.monotonic()
                no_frame_duration = (
                    now - self._last_frame_monotonic
                    if self._last_frame_monotonic > 0
                    else (self._episode.elapsed_seconds if self._episode is not None else 0.0)
                )
                if (
                    no_frame_duration > 3.0
                    and now - self._last_capture_status_query > 3.0
                ):
                    self._last_capture_status_query = now
                    self._send_request(
                        CommandId.STATUS,
                        encode_status_request(),
                        retry_allowed=False,
                    )
                return
            self._send_request(CommandId.STATUS, encode_status_request(), retry_allowed=False)
        else:
            self._start_handshake()

    def _receive_bytes(self, payload: bytes) -> None:
        crc_before = self.parser.stats.crc_errors
        for packet in self.parser.feed(payload):
            self._handle_packet(packet)
        crc_delta = self.parser.stats.crc_errors - crc_before
        if crc_delta > 0:
            self._report_alert(
                f"检测到{crc_delta}个CRC错误，累计{self.parser.stats.crc_errors}",
                "warning",
                "protocol_crc",
            )
        self._update_counters()

    def _handle_packet(self, packet: Packet) -> None:
        try:
            if packet.packet_type == PacketType.ACK:
                result = decode_command_result(packet.data)
                self._handle_result(result.command_id, result.result, is_error=False)
            elif packet.packet_type == PacketType.ERROR:
                result = decode_command_result(packet.data)
                self._handle_result(result.command_id, result.result, is_error=True)
            elif packet.packet_type == PacketType.STATUS:
                status = decode_status(packet.data)
                self._handle_status(status)
            elif packet.packet_type == PacketType.FRAME:
                self._handle_act_frame(decode_act_frame(packet.data))
            else:
                self._log(f"RX UNKNOWN type={packet.packet_type} len={len(packet.data)}")
        except ProtocolError as exc:
            self._log(f"PAYLOAD ERROR {exc}")
            self._report_alert(str(exc), "warning", "protocol_payload")

    def _handle_result(self, command_id: int, result: int, is_error: bool) -> None:
        self._log(f"RX {'ERROR' if is_error else 'ACK'} cmd={command_id} result={result}")
        if command_id == int(CommandId.MOVE):
            if is_error or result != ResultCode.OK:
                self._report_alert(
                    f"MOVE被H7拒绝：{self._result_description(result)}",
                    "error" if result in (ResultCode.FAULT, ResultCode.TX_FAILED) else "warning",
                    "h7_move",
                )
                if result == ResultCode.DISABLED:
                    self._set_control_enabled(False)
            return
        pending = self.pending
        if pending is None or command_id != int(pending.command_id):
            self._log("忽略未匹配当前请求的命令结果")
            return
        finished = self._clear_pending()
        if is_error or result != ResultCode.OK:
            if finished and finished.command_id == CommandId.SELECT:
                self._restore_confirmed_tentacle()
            if finished and finished.command_id == CommandId.ENABLE:
                self._update_control_button_visuals(self._h7_synced)
            if finished and finished.command_id == CommandId.CAPTURE:
                if finished.capture_target:
                    self._episode = None
                    self._reset_capture_summary()
                    self._set_capture_state(CaptureUiState.IDLE, "启动失败")
                else:
                    self._set_capture_state(CaptureUiState.CAPTURING, "停止失败，仍在采集")
            self._report_alert(
                f"H7拒绝{pending.command_id.name}：{self._result_description(result)}",
                "error" if result in (ResultCode.FAULT, ResultCode.TX_FAILED) else "warning",
                "h7_command",
            )
            return
        if pending.command_id == CommandId.PING:
            self._ping_confirmed = True
            QTimer.singleShot(
                0,
                lambda: self._send_request(
                    CommandId.STATUS,
                    encode_status_request(),
                    retry_allowed=False,
                ),
            )
        elif pending.command_id == CommandId.SELECT:
            QTimer.singleShot(
                0,
                lambda: self._send_request(
                    CommandId.STATUS,
                    encode_status_request(),
                    retry_allowed=False,
                ),
            )
        elif pending.command_id == CommandId.ENABLE:
            self._set_control_enabled(bool(finished and finished.enable_target))
            self.bottom_message.setText(
                "ACT控制已启用，参数仍保持锁定"
                if self._control_enabled
                else "ACT控制已正常停止"
            )
            QTimer.singleShot(0, self._request_status_if_idle)
        elif pending.command_id == CommandId.HOME:
            self._reset_target_values()
            self.bottom_message.setText("HOME命令已由H7确认")
            QTimer.singleShot(0, self._request_status_if_idle)
        elif pending.command_id == CommandId.STOP:
            self._set_control_enabled(False)
            self.bottom_message.setText("紧急停止已由H7确认")
            QTimer.singleShot(0, self._request_status_if_idle)
        elif pending.command_id == CommandId.CAPTURE:
            if finished and finished.capture_target:
                self._set_capture_state(CaptureUiState.CAPTURING, "采集中")
                self._set_bottom_message("ACT_FRAME采集已启动")
            else:
                self._set_capture_state(CaptureUiState.STOPPING, "等待H7确认停止")
            QTimer.singleShot(0, self._request_status_if_idle)

    @staticmethod
    def _result_description(result: int) -> str:
        descriptions = {
            int(ResultCode.BAD_COMMAND): "命令不支持",
            int(ResultCode.BAD_ARGUMENT): "参数越界或格式错误",
            int(ResultCode.BUSY): "其他控制来源正在占用",
            int(ResultCode.NOT_READY): "所选触手尚未ONLINE/READY",
            int(ResultCode.FAULT): "所选触手存在FAULT",
            int(ResultCode.DISABLED): "ACT控制未启用或已被底层释放",
            int(ResultCode.TX_FAILED): "H7向CAN提交失败",
        }
        return descriptions.get(int(result), f"未知结果码 {result}")

    def _request_status_if_idle(self) -> None:
        if self.serial.is_open and self._h7_synced and self.pending is None:
            self._send_request(
                CommandId.STATUS,
                encode_status_request(),
                retry_allowed=False,
            )

    def _reset_target_values(self) -> None:
        """HOME确认后将上位机目标恢复为明确的零状态。"""

        self._target_angle = 0.0
        self._target_bend = 0
        self._target_stiffness = 0
        self._move_dirty = False
        self.direction_dial.set_angle(0.0)
        self.bend_slider.blockSignals(True)
        self.stiff_slider.blockSignals(True)
        self.bend_slider.setValue(0)
        self.stiff_slider.setValue(0)
        self.bend_slider.blockSignals(False)
        self.stiff_slider.blockSignals(False)
        self._update_target_display()

    def _handle_status(self, status) -> None:
        self._log(
            f"RX STATUS T{status.selected_tentacle} control={status.control_enabled} "
            f"capture={status.capture_state} sent={status.sent_frame_count} "
            f"drop={status.dropped_frame_count}"
        )
        if self.pending is not None and self.pending.command_id == CommandId.STATUS:
            self._clear_pending()
        if not 1 <= status.selected_tentacle <= 4:
            self._report_alert("STATUS中的触手编号无效", "warning", "protocol_status")
            return
        was_control_enabled = self._control_enabled
        previous_status = self._last_status
        self._last_status = status
        if self._episode is not None:
            self._episode.set_h7_counters(
                status.sent_frame_count,
                status.dropped_frame_count,
            )
        self._confirmed_tentacle = status.selected_tentacle
        self._restore_confirmed_tentacle()
        self.pose_view.set_tentacle(status.selected_tentacle - 1)
        self.parameter_target.setText(f"当前对象：T{status.selected_tentacle}")
        self._set_control_enabled(bool(status.control_enabled))
        if status.motor is not None:
            self._update_actual_pose(status.motor)
        if was_control_enabled and not status.control_enabled:
            self._report_alert(
                "H7已释放ACT控制，参数已重新锁定",
                "warning",
                "control_state",
            )
        self.drop_label.setText(f"Drop {status.dropped_frame_count}")
        if status.capture_state == 1:
            if self._capture_state == CaptureUiState.STARTING:
                self._set_capture_state(CaptureUiState.CAPTURING, "采集中")
            elif self._capture_state == CaptureUiState.STOPPING and self._episode is not None:
                self._set_capture_state(CaptureUiState.CAPTURING, "H7仍在采集")
                self._report_alert("H7未进入采集停止状态，请重新停止", "warning", "capture_state")
            elif self._capture_state in (CaptureUiState.IDLE, CaptureUiState.REVIEW):
                self._report_alert(
                    "检测到H7残留采集流，正在停止以避免混入旧Episode",
                    "warning",
                    "capture_orphan",
                )
                QTimer.singleShot(0, self._stop_orphan_capture)
        elif self._capture_state == CaptureUiState.STOPPING:
            self._orphan_stop_active = False
            self._enter_episode_review()
        elif self._capture_state in (
            CaptureUiState.STARTING,
            CaptureUiState.CAPTURING,
        ):
            self._enter_episode_review("H7意外结束采集")
            self._report_alert("H7采集状态意外变为空闲", "warning", "capture_state")
        if (
            previous_status is not None
            and status.dropped_frame_count > previous_status.dropped_frame_count
        ):
            self._report_alert(
                f"H7丢帧计数增加到{status.dropped_frame_count}",
                "warning",
                "h7_drop",
            )
        self.tentacle_protocol_state.setText(f"● T{status.selected_tentacle} 已由H7确认")
        self.tentacle_protocol_state.setObjectName("statusGood")
        self.top_tentacle.setText(f"T{status.selected_tentacle} 已选择")
        self.top_tentacle.setObjectName("statusGood")
        if self._ping_confirmed:
            self._set_h7_synced(True, "● H7已同步")
            if self._current_alert_source in {
                "serial",
                "serial_reconnect",
                "protocol_timeout",
            }:
                self._clear_current_alert()
            self._set_bottom_message("通信与单触手控制协议正常")
        self._refresh_dynamic_styles()
        self._update_data_quality()

    def _stop_orphan_capture(self) -> None:
        if (
            not self.serial.is_open
            or not self._h7_synced
            or self.pending is not None
            or self._last_status is None
            or self._last_status.capture_state == 0
        ):
            return
        self._set_capture_state(CaptureUiState.STOPPING, "正在清理H7残留采集")
        self._orphan_stop_active = True
        self._send_request(
            CommandId.CAPTURE,
            encode_capture(False),
            retry_allowed=True,
            capture_target=False,
        )

    def _select_tentacle(self, _index: int) -> None:
        target = int(self.tentacle_combo.currentData())
        if (
            not self._h7_synced
            or self._control_enabled
            or (
                self.pending is not None
                and self.pending.command_id != CommandId.STATUS
            )
        ):
            self._restore_confirmed_tentacle()
            return
        if self.pending is not None:
            self._clear_pending()
        self.tentacle_combo.setEnabled(False)
        self.bottom_message.setText(f"等待H7确认T{target}")
        self._send_request(
            CommandId.SELECT,
            encode_select(target),
            retry_allowed=False,
            select_target=target,
        )

    def _restore_confirmed_tentacle(self) -> None:
        self.tentacle_combo.blockSignals(True)
        index = self.tentacle_combo.findData(self._confirmed_tentacle)
        if index >= 0:
            self.tentacle_combo.setCurrentIndex(index)
        self.tentacle_combo.blockSignals(False)

    def _set_h7_synced(self, synced: bool, text: str) -> None:
        self._h7_synced = synced
        if not synced:
            self._set_control_enabled(False)
        object_name = "statusGood" if synced else ("statusWait" if self.serial.is_open else "statusOff")
        self.top_h7.setText(text)
        self.top_h7.setObjectName(object_name)
        self.h7_state.setText(text)
        self.h7_state.setObjectName(object_name)
        self._set_protocol_controls(synced)
        self._refresh_dynamic_styles()
        self._update_data_quality()

    def _reset_protocol_state(self, reason: str) -> None:
        if self._capture_state in (
            CaptureUiState.STARTING,
            CaptureUiState.CAPTURING,
            CaptureUiState.STOPPING,
        ):
            if self._episode is not None and self._episode.has_frames:
                self._episode.mark_interrupted(reason)
                self._set_capture_state(CaptureUiState.REVIEW, "连接中断，数据待处理")
            else:
                self._episode = None
                self._set_capture_state(CaptureUiState.IDLE, "连接中断")
        self._request_timer.stop()
        self.pending = None
        self.parser.clear()
        self._last_status = None
        self._ping_confirmed = False
        self._enable_hold_active = False
        self._enable_hold_timer.stop()
        self._enable_hold_progress.stop()
        self.enable_control_button.setText("按住 0.8 秒启用（Space）")
        self._set_h7_synced(False, "● H7未同步")
        self.tentacle_protocol_state.setText("● 等待H7确认")
        self.tentacle_protocol_state.setObjectName("statusOff")
        self.top_tentacle.setText(f"T{self._confirmed_tentacle} 未确认")
        self.top_tentacle.setObjectName("statusOff")
        self.pose_view.set_tentacle(self._confirmed_tentacle - 1)
        self.pose_view.set_state(0.0, 0.0, online=False, active=False)
        self._set_bottom_message(reason)
        self._refresh_dynamic_styles()
        self._update_data_quality()

    def _transmitted(self, _payload: bytes) -> None:
        self._update_counters()

    def _serial_error(self, message: str) -> None:
        if message:
            self._log(f"SERIAL ERROR {message}")
            self._report_alert(message, "error", "serial")

    def _update_counters(self) -> None:
        self.rx_tx_label.setText(f"RX {self.serial.rx_bytes} B   TX {self.serial.tx_bytes} B")
        self.crc_label.setText(f"CRC {self.parser.stats.crc_errors}")
        self._update_diagnostic_dialog()
        self._update_data_quality()

    def _show_advanced_settings(self) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle("高级串口设置")
        dialog.setModal(True)
        layout = QFormLayout(dialog)
        for name, value in (
            ("数据位", "8"),
            ("停止位", "1"),
            ("校验", "无"),
            ("流控", "无"),
            ("自动重连", "用户保持连接时开启"),
        ):
            label = QLabel(value)
            label.setObjectName("value")
            layout.addRow(name, label)
        close_button = QPushButton("关闭")
        close_button.clicked.connect(dialog.accept)
        layout.addRow(close_button)
        dialog.exec()

    def _show_data_quality(self) -> None:
        """显示实时通讯质量、H7摘要以及可追溯的告警历史。"""

        if self._quality_dialog is not None:
            self._quality_dialog.raise_()
            self._quality_dialog.activateWindow()
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("ACT数据质量与故障记录")
        dialog.resize(760, 560)
        layout = QVBoxLayout(dialog)
        layout.addWidget(_section_title("当前状态"))

        self._quality_alert = _status_label("● 无活动告警")
        self._quality_alert.setMinimumWidth(500)
        layout.addWidget(self._quality_alert)

        grid = QGridLayout()
        rows = (
            ("serial", "串口"),
            ("h7", "H7协议"),
            ("tentacle", "当前触手"),
            ("control", "ACT控制"),
            ("capture", "采集状态"),
            ("traffic", "收发字节"),
            ("packets", "有效包"),
            ("crc", "CRC错误"),
            ("format", "长度/版本错误"),
            ("discarded", "丢弃字节"),
            ("h7_frames", "H7发送/丢弃"),
            ("pc_gaps", "PC序号缺口/重复"),
            ("receive_rate", "ACT_FRAME接收频率"),
            ("position_quality", "电机位置无效/过期"),
            ("tof_quality", "ToF有效区域"),
        )
        self._quality_values = {}
        for row, (key, title) in enumerate(rows):
            grid.addWidget(QLabel(title), row, 0)
            value = QLabel("—")
            value.setObjectName("value")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            grid.addWidget(value, row, 1)
            self._quality_values[key] = value
        grid.setColumnStretch(1, 1)
        layout.addLayout(grid)

        layout.addWidget(_section_title("告警历史（最多100条）"))
        self._quality_history = QPlainTextEdit()
        self._quality_history.setReadOnly(True)
        self._quality_history.document().setMaximumBlockCount(100)
        layout.addWidget(self._quality_history, 1)

        buttons = QHBoxLayout()
        acknowledge = QPushButton("确认当前提示")
        clear_history = QPushButton("清空历史")
        close_button = QPushButton("关闭")
        acknowledge.clicked.connect(self._acknowledge_current_alert)
        clear_history.clicked.connect(self._clear_alert_history)
        close_button.clicked.connect(dialog.close)
        buttons.addWidget(acknowledge)
        buttons.addWidget(clear_history)
        buttons.addStretch(1)
        buttons.addWidget(close_button)
        layout.addLayout(buttons)

        dialog.finished.connect(self._quality_closed)
        self._quality_dialog = dialog
        self._update_data_quality()
        dialog.show()

    def _quality_closed(self) -> None:
        self._quality_dialog = None
        self._quality_values = {}
        self._quality_alert = None
        self._quality_history = None

    def _set_bottom_message(self, message: str) -> None:
        self.bottom_message.setText(message)
        self.bottom_message.setToolTip(message)

    def _report_alert(self, message: str, severity: str, source: str) -> None:
        """统一更新顶部、底部和故障历史，避免各入口显示不一致。"""

        normalized = severity if severity in ("warning", "error") else "info"
        duplicate = (
            self._current_alert == message
            and self._current_alert_severity == normalized
            and self._current_alert_source == source
        )
        self._current_alert = message
        self._current_alert_severity = normalized
        self._current_alert_source = source
        if not duplicate:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self._alert_history.append(
                f"{timestamp}  {normalized.upper():7s}  [{source}] {message}"
            )

        prefix = "🔴" if normalized == "error" else ("🟡" if normalized == "warning" else "●")
        short = message if len(message) <= 22 else message[:21] + "…"
        self.top_alert.setText(f"{prefix} {short}")
        self.top_alert.setToolTip(message)
        self.top_alert.setObjectName(
            "statusError" if normalized == "error" else (
                "statusWait" if normalized == "warning" else "statusGood"
            )
        )
        self._set_bottom_message(message)
        self._refresh_dynamic_styles()
        self._update_data_quality()

    def _acknowledge_current_alert(self) -> None:
        """只确认PC提示，不清除或伪造H7侧真实故障状态。"""

        self._clear_current_alert()
        self._set_bottom_message("提示已确认；H7真实故障仍需等待状态恢复")

    def _clear_current_alert(self) -> None:
        """清除PC侧当前提示；告警历史仍保留用于追溯。"""

        self._current_alert = "无活动告警"
        self._current_alert_severity = "off"
        self._current_alert_source = ""
        self.top_alert.setText("● 无活动告警")
        self.top_alert.setToolTip("")
        self.top_alert.setObjectName("statusOff")
        self._refresh_dynamic_styles()
        self._update_data_quality()

    def _clear_alert_history(self) -> None:
        self._alert_history.clear()
        self._update_data_quality()

    def _update_data_quality(self) -> None:
        if self._quality_dialog is None:
            return
        stats = self.parser.stats
        status = self._last_status
        episode_quality = self._episode.quality() if self._episode is not None else None
        values = {
            "serial": "已打开" if self.serial.is_open else "未连接",
            "h7": "已同步" if self._h7_synced else "未同步",
            "tentacle": f"T{self._confirmed_tentacle}",
            "control": "已启用" if self._control_enabled else "未启用",
            "capture": self.capture_state.text().removeprefix("● "),
            "traffic": f"RX {self.serial.rx_bytes} B / TX {self.serial.tx_bytes} B",
            "packets": str(stats.valid_packets),
            "crc": str(stats.crc_errors),
            "format": f"长度 {stats.length_errors} / 版本 {stats.version_errors}",
            "discarded": str(stats.discarded_bytes),
            "h7_frames": (
                f"发送 {status.sent_frame_count} / 丢弃 {status.dropped_frame_count}"
                if status is not None else "尚未收到STATUS"
            ),
            "pc_gaps": (
                f"缺口 {episode_quality.sequence_gap_count} / 重复 {episode_quality.duplicate_count}"
                if episode_quality is not None else "尚未开始Episode"
            ),
            "receive_rate": (
                f"{episode_quality.receive_hz:.2f} Hz"
                if episode_quality is not None and episode_quality.frame_count > 1 else "—"
            ),
            "position_quality": (
                f"无效 {episode_quality.invalid_position_count} / 过期 {episode_quality.stale_position_count}"
                if episode_quality is not None else "—"
            ),
            "tof_quality": (
                f"有效 {episode_quality.valid_zone_ratio * 100.0:.1f}%"
                if episode_quality is not None and episode_quality.frame_count > 0 else "—"
            ),
        }
        for key, text in values.items():
            label = self._quality_values.get(key)
            if label is not None:
                label.setText(text)

        if self._quality_alert is not None:
            self._quality_alert.setText(
                "● 无活动告警" if self._current_alert_severity == "off"
                else self._current_alert
            )
            self._quality_alert.setObjectName(
                "statusError" if self._current_alert_severity == "error" else (
                    "statusWait" if self._current_alert_severity == "warning" else "statusOff"
                )
            )
            self._quality_alert.style().unpolish(self._quality_alert)
            self._quality_alert.style().polish(self._quality_alert)
        if self._quality_history is not None:
            self._quality_history.setPlainText("\n".join(self._alert_history))
            cursor = self._quality_history.textCursor()
            cursor.movePosition(QTextCursor.End)
            self._quality_history.setTextCursor(cursor)

    def _show_protocol_diagnostics(self) -> None:
        if self._diagnostic_dialog is not None:
            self._diagnostic_dialog.raise_()
            self._diagnostic_dialog.activateWindow()
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("ACT协议诊断")
        dialog.resize(680, 430)
        layout = QVBoxLayout(dialog)
        text = QPlainTextEdit()
        text.setReadOnly(True)
        layout.addWidget(text)
        close_button = QPushButton("关闭")
        close_button.clicked.connect(dialog.close)
        layout.addWidget(close_button)
        dialog.finished.connect(self._diagnostic_closed)
        self._diagnostic_dialog = dialog
        self._diagnostic_text = text
        self._update_diagnostic_dialog()
        dialog.show()

    def _diagnostic_closed(self) -> None:
        self._diagnostic_dialog = None
        self._diagnostic_text = None

    def _update_diagnostic_dialog(self) -> None:
        if self._diagnostic_text is None:
            return
        stats = self.parser.stats
        header = (
            f"RX bytes: {self.serial.rx_bytes}\n"
            f"TX bytes: {self.serial.tx_bytes}\n"
            f"Valid packets: {stats.valid_packets}\n"
            f"CRC errors: {stats.crc_errors}\n"
            f"Length errors: {stats.length_errors}\n"
            f"Version errors: {stats.version_errors}\n"
            f"Discarded bytes: {stats.discarded_bytes}\n"
            f"Buffered bytes: {self.parser.buffered_bytes}\n\n"
        )
        self._diagnostic_text.setPlainText(header + "\n".join(self._logs))
        cursor = self._diagnostic_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self._diagnostic_text.setTextCursor(cursor)

    def _log(self, line: str) -> None:
        self._logs.append(line)
        self._update_diagnostic_dialog()

    def _refresh_dynamic_styles(self) -> None:
        for widget in (
            self.top_serial,
            self.top_h7,
            self.top_tentacle,
            self.top_alert,
            self.serial_state,
            self.h7_state,
            self.tentacle_protocol_state,
            self.control_owner,
        ):
            widget.style().unpolish(widget)
            widget.style().polish(widget)

    @staticmethod
    def _repolish(widget: QWidget) -> None:
        """对象名改变后立即刷新局部样式，避免按钮颜色延迟到下一次重绘。"""

        widget.style().unpolish(widget)
        widget.style().polish(widget)
        widget.update()

    def keyPressEvent(self, event: QKeyEvent) -> None:  # type: ignore[override]
        if event.key() == Qt.Key_Escape and not event.isAutoRepeat():
            self._request_emergency_stop()
            event.accept()
            return
        super().keyPressEvent(event)

    def eventFilter(self, watched, event) -> bool:  # type: ignore[override]
        """提供不受按钮焦点影响的Space/L/Esc全局安全快捷键。"""

        event_type = event.type()
        if event_type in (QEvent.ApplicationDeactivate, QEvent.WindowDeactivate):
            if self._space_shortcut_active:
                self._space_shortcut_active = False
                self._cancel_enable_hold()
            return super().eventFilter(watched, event)

        if event_type not in (QEvent.KeyPress, QEvent.KeyRelease):
            return super().eventFilter(watched, event)
        if event.isAutoRepeat():
            return event.key() in (Qt.Key_Space, Qt.Key_L, Qt.Key_Escape)

        if event_type == QEvent.KeyRelease and event.key() == Qt.Key_Space:
            if self._space_shortcut_active:
                self._space_shortcut_active = False
                self._cancel_enable_hold()
                event.accept()
                return True
            return super().eventFilter(watched, event)

        if event_type != QEvent.KeyPress:
            return super().eventFilter(watched, event)
        if event.key() == Qt.Key_Escape:
            self._request_emergency_stop()
            event.accept()
            return True
        if self._control_shortcut_blocked():
            return super().eventFilter(watched, event)
        if event.modifiers() != Qt.NoModifier:
            return super().eventFilter(watched, event)

        if event.key() == Qt.Key_L:
            self._toggle_parameter_lock()
            event.accept()
            return True
        if event.key() == Qt.Key_Space:
            self._space_shortcut_active = True
            if self._control_enabled:
                self._request_disable_control()
            elif not self._h7_synced:
                self._set_bottom_message("Space无效：请先连接并同步H7")
            elif self.pending is not None and self.pending.command_id != CommandId.STATUS:
                self._set_bottom_message("Space暂不可用：正在等待当前命令完成")
            else:
                self._begin_enable_hold()
            event.accept()
            return True
        return super().eventFilter(watched, event)

    @staticmethod
    def _control_shortcut_blocked() -> bool:
        """文本编辑和下拉选择期间不抢占Space/L，防止输入被控制快捷键截获。"""

        focus = QApplication.focusWidget()
        return isinstance(focus, (QLineEdit, QPlainTextEdit, QComboBox))

    def closeEvent(self, event: QCloseEvent) -> None:  # type: ignore[override]
        if self._save_future is not None and not self._save_future.done():
            QMessageBox.information(self, "正在保存", "Episode正在保存，请完成后再退出。")
            event.ignore()
            return
        if self._episode is not None and self._episode.has_frames:
            answer = QMessageBox.question(
                self,
                "存在未保存数据",
                "当前Episode尚未正式保存。退出前将写入datasets/recovery，是否继续？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            try:
                recovery_snapshot = self._episode.snapshot(
                    {"recovery": True, "recovery_reason": "application_exit"}
                )
                recovery_path = save_episode(
                    recovery_snapshot,
                    self._episode_root().parent / "recovery",
                )
                self._log(f"RECOVERY SAVED {recovery_path}")
            except Exception as exc:
                QMessageBox.critical(self, "恢复数据保存失败", str(exc))
                event.ignore()
                return
        if self.serial.is_open and self._capture_state in (
            CaptureUiState.STARTING,
            CaptureUiState.CAPTURING,
            CaptureUiState.STOPPING,
        ):
            self.serial.write(encode_capture(False))
        self._heartbeat_timer.stop()
        self._move_timer.stop()
        self._request_timer.stop()
        self._enable_hold_timer.stop()
        self._enable_hold_progress.stop()
        self._capture_ui_timer.stop()
        self._save_poll_timer.stop()
        application = QApplication.instance()
        if application is not None:
            application.removeEventFilter(self)
        self.serial.shutdown()
        self._save_executor.shutdown(wait=False, cancel_futures=False)
        event.accept()
