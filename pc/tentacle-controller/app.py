from __future__ import annotations

import codecs
import math
from pathlib import Path
import re
import time

from PySide6.QtCore import QEvent, QObject, QPoint, QSize, QSettings, QTimer, Qt, Signal
from PySide6.QtGui import QAction, QColor, QFont, QIntValidator, QKeyEvent, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDockWidget, QDoubleSpinBox, QFrame, QGridLayout,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QMainWindow, QMessageBox,
    QPushButton, QPlainTextEdit, QScrollArea, QSizePolicy, QSlider, QSplitter,
    QSpinBox, QStackedWidget, QStyle, QTabWidget, QToolButton, QVBoxLayout, QWidget,
)

from command_store import CommandDefinition, CommandStore
from serial_service import SerialService, SerialSettings
from tentacle_view import DirectionDial, TentacleViewport


APP_STYLE = """
QMainWindow, QWidget { background: #17212b; color: #d7e1ea; font-size: 13px; }
QFrame#sidebar, QFrame#modebar, QFrame#panel { background: #202c38; border: 1px solid #334454; border-radius: 6px; }
QFrame#panel[selected="true"] { border: 2px solid #66d9ef; }
QLabel#section { color: #8fc6de; font-size: 14px; font-weight: 600; padding: 5px 2px; }
QLabel#muted { color: #91a3b5; }
QLabel#value { color: #f4f8fb; font-weight: 600; }
QPushButton, QToolButton { background: #2b3b4b; border: 1px solid #465b6d; border-radius: 4px; padding: 6px 8px; }
QPushButton:hover, QToolButton:hover { background: #385267; }
QPushButton:pressed { background: #1f7c92; }
QPushButton#danger { background: #8e3940; border-color: #d2686e; font-weight: 700; }
QPushButton#primary { background: #176d82; border-color: #4db6ce; font-weight: 700; }
QPushButton:disabled { color:#708292; background:#1b2732; border-color:#30404e; }
QPushButton#command { text-align: left; padding: 5px 7px; }
QPushButton#command[danger="true"] { color: #ffb3b3; border-color: #9f4c54; }
QPushButton#commandAction { padding:5px 7px; color:#8fe3ef; }
QPushButton#commandAction[danger="true"] { color:#ffb3b3; border-color:#9f4c54; }
QToolButton#commandGroup { text-align:left; background:#253746; color:#a9d8ea; font-weight:700; padding:7px; }
QToolButton#commandGroup:hover { background:#30485b; }
QToolButton#panelToggle { padding:5px 12px; background:#243544; }
QToolButton#panelToggle:checked { color:#7fe4f2; background:#176d82; border-color:#4db6ce; }
QFrame#topToolbar { background:#202c38; border:1px solid #334454; border-radius:4px; }
QLabel#topStatus { background:#18242e; border:1px solid #35495a; border-radius:4px; padding:0 12px; font-weight:600; }
QLineEdit, QPlainTextEdit, QComboBox, QSpinBox, QDoubleSpinBox { background: #111a22; border: 1px solid #42576a; border-radius: 4px; padding: 5px; }
QComboBox:disabled { color: #6f8090; background: #19232d; border-color: #2e3c49; }
QSlider::groove:horizontal { height: 5px; background: #405365; }
QSlider::handle:horizontal { width: 14px; margin: -5px 0; background: #66d9ef; border-radius: 7px; }
QScrollArea { border: none; }
QSplitter::handle { background: #0f171e; }
QToolTip { background: #0f171e; color: #e8f1f7; border: 1px solid #4b6578; }
"""


def add_label(text: str, object_name: str = "muted") -> QLabel:
    label = QLabel(text)
    label.setObjectName(object_name)
    return label


class HoldButton(QPushButton):
    held = Signal()

    def __init__(
        self,
        text: str,
        hold_ms: int = 800,
        parent: QWidget | None = None,
        progress_text: str = "按住中",
    ) -> None:
        super().__init__(text, parent)
        self._hold_ms = hold_ms
        self._progress_text = progress_text
        self._hold_timer = QTimer(self)
        self._hold_timer.setSingleShot(True)
        self._hold_timer.setInterval(hold_ms)
        self._progress_timer = QTimer(self)
        self._progress_timer.setInterval(35)
        self._progress_timer.timeout.connect(self._update_progress)
        self._press_started = 0.0
        self._press_text = text
        self._press_style = ""
        self._held_fired = False
        self._hold_timer.timeout.connect(self._fire_held)

    def _fire_held(self) -> None:
        self._progress_timer.stop()
        self._held_fired = True
        self._set_progress(100)
        self.setText("已触发")
        self.held.emit()

    def _update_progress(self) -> None:
        elapsed_ms = (time.monotonic() - self._press_started) * 1000.0
        self._set_progress(min(99, int(elapsed_ms * 100.0 / self._hold_ms)))

    def _set_progress(self, percent: int) -> None:
        progress = max(0.0, min(1.0, percent / 100.0))
        after = min(1.0, progress + 0.001)
        self.setText(f"{self._progress_text} {percent}%")
        self.setStyleSheet(
            "QPushButton {"
            "color:#f4f8fb; font-weight:700; border:1px solid #4db6ce;"
            "background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            f"stop:0 #176d82, stop:{progress:.3f} #176d82,"
            f"stop:{after:.3f} #2b3b4b, stop:1 #2b3b4b);"
            "}"
        )

    def consume_hold(self) -> bool:
        fired = self._held_fired
        self._held_fired = False
        return fired

    def begin_hold(self) -> None:
        if not self.isEnabled():
            return
        self._held_fired = False
        self._press_text = self.text()
        self._press_style = self.styleSheet()
        self._press_started = time.monotonic()
        self._hold_timer.start()
        self._progress_timer.start()
        self._set_progress(0)

    def cancel_hold(self) -> bool:
        self._hold_timer.stop()
        self._progress_timer.stop()
        fired = self._held_fired
        if not fired:
            self.setText(self._press_text)
            self.setStyleSheet(self._press_style)
        return fired

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self.begin_hold()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        self.cancel_hold()
        super().mouseReleaseEvent(event)


class ParameterSlider(QWidget):
    value_changed = Signal(int)

    def __init__(self, title: str, value: int = 0, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        top = QVBoxLayout()
        top.setSpacing(1)
        top.addWidget(add_label(title))
        self.value_label = add_label(f"{value}%", "value")
        self.value_label.setAlignment(Qt.AlignLeft)
        top.addWidget(self.value_label)
        layout.addLayout(top)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setValue(value)
        self.slider.valueChanged.connect(self._changed)
        layout.addWidget(self.slider)

    def _changed(self, value: int) -> None:
        self.value_label.setText(f"{value}%")
        self.value_changed.emit(value)

    def value(self) -> int:
        return self.slider.value()

    def setValue(self, value: int) -> None:
        self.slider.setValue(max(0, min(100, value)))

    def setEnabled(self, enabled: bool) -> None:
        super().setEnabled(enabled)
        self.slider.setEnabled(enabled)


class TentaclePanel(QFrame):
    selected = Signal(int)
    control_changed = Signal(int, float, int, int)
    return_center = Signal(int)
    lock_changed = Signal(int, bool)

    def __init__(self, index: int, model_path: Path, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.index = index
        self.locked = True
        self.active = False
        self.online: bool | None = None
        self.angle = 0.0
        self.bend = 0
        self.stiffness = 0
        self.feedback = "--"
        self.actual_angle = 0.0
        self.actual_bend = 0.0
        self.actual_dq = (0, 0, 0)
        self._updating_values = False
        self.setObjectName("panel")
        self.setMinimumSize(420, 280)
        # 四个面板由外层网格强制等分；忽略内部动态文字的 sizeHint，避免实时反馈撑大卡片。
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self._build_ui(model_path)

    def _build_ui(self, model_path: Path) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(7)
        header = QHBoxLayout()
        title = QLabel(f"T{self.index + 1}")
        title.setStyleSheet("font-size:15px; font-weight:700; color:#eef6fb;")
        header.addWidget(title)
        self.online_label = add_label("●在线")
        self.active_label = add_label("●待机")
        self.state_label = add_label("状态：正常")
        self.feedback_label = add_label("反馈：--")
        self.feedback_label.setMinimumWidth(110)
        self.feedback_label.setMaximumWidth(210)
        self.feedback_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        header.addWidget(self.online_label)
        header.addWidget(self.active_label)
        header.addWidget(self.state_label)
        header.addWidget(self.feedback_label)
        header.addStretch(1)
        self.lock_button = HoldButton("🔒 参数已锁定", 800, progress_text="按住解锁")
        self.lock_button.setFixedWidth(138)
        self.lock_button.setToolTip("点击锁定；解锁需要长按")
        self.lock_button.clicked.connect(self._lock_click)
        self.lock_button.held.connect(self._unlock_hold)
        header.addWidget(self.lock_button)
        root.addLayout(header)

        content = QHBoxLayout()
        content.setSpacing(6)
        params_widget = QWidget()
        params_widget.setMinimumWidth(85)
        params_widget.setMaximumWidth(115)
        params = QVBoxLayout(params_widget)
        params.setContentsMargins(0, 0, 0, 0)
        params.setSpacing(6)
        params.addWidget(add_label(f"操作对象：T{self.index + 1}", "value"))
        self.angle_label = add_label("控制角度\n0°", "value")
        params.addWidget(self.angle_label)
        self.bend_slider = ParameterSlider("目标 Bend", 0)
        self.stiff_slider = ParameterSlider("目标刚度", 0)
        self.bend_slider.value_changed.connect(self._bend_changed)
        self.stiff_slider.value_changed.connect(self._stiff_changed)
        params.addWidget(self.bend_slider)
        params.addWidget(self.stiff_slider)
        self.link_label = add_label("联动：无")
        params.addWidget(self.link_label)
        params.addStretch(1)
        content.addWidget(params_widget, 0)

        self.viewport = TentacleViewport(model_path, tentacle_index=self.index)
        self.viewport.selected.connect(lambda: self.selected.emit(self.index))
        content.addWidget(self.viewport, 4)

        dial_container = QFrame()
        dial_container.setObjectName("dialArea")
        dial_container.setMinimumWidth(130)
        dial_container.setMaximumWidth(175)
        dial_container.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        dial_container.setStyleSheet("QFrame#dialArea { background:#202c38; border:none; }")
        dial_box = QVBoxLayout(dial_container)
        dial_box.setContentsMargins(6, 0, 6, 0)
        dial_title = add_label("方向圆盘", "section")
        dial_title.setStyleSheet("background:transparent;")
        dial_box.addWidget(dial_title)
        self.dial = DirectionDial()
        self.dial.setStyleSheet("background:transparent;")
        self.dial.angle_changed.connect(self._angle_changed)
        self.dial.bend_wheel.connect(self._bend_wheel)
        dial_box.addWidget(self.dial, 1)
        self.angle_readout = add_label("0°", "value")
        self.angle_readout.setAlignment(Qt.AlignCenter)
        self.angle_readout.setStyleSheet("background:#17212b; color:#f4f8fb; font-weight:600; padding:3px;")
        dial_box.addWidget(self.angle_readout)
        self.center_button = QPushButton("回正")
        self.center_button.setObjectName("primary")
        self.center_button.clicked.connect(lambda: self.return_center.emit(self.index))
        dial_box.addWidget(self.center_button)
        content.addWidget(dial_container, 0)
        root.addLayout(content, 1)
        self._apply_lock()

    def _lock_click(self) -> None:
        if self.lock_button.consume_hold():
            return
        if not self.locked:
            self.set_locked(True)

    def _unlock_hold(self) -> None:
        if self.locked:
            self.set_locked(False)

    def set_locked(self, locked: bool) -> None:
        self.locked = locked
        self._apply_lock()
        self.lock_changed.emit(self.index, locked)

    def _apply_lock(self) -> None:
        self.bend_slider.setEnabled(not self.locked and self.active)
        self.stiff_slider.setEnabled(not self.locked and self.active)
        self.dial.locked = self.locked or not self.active
        self.center_button.setEnabled(not self.locked and self.active)
        self.lock_button.setText("🔒 参数已锁定" if self.locked else "🔓 参数可编辑")
        self.lock_button.setStyleSheet("color:#91a3b5;" if self.locked else "color:#66d9ef;")

    def set_status(self, online: bool | None, active: bool, selected: bool, feedback: str = "--") -> None:
        self.online = online
        self.active = active
        self.feedback = feedback
        if online is None:
            self.online_label.setText("●未同步")
            self.online_label.setStyleSheet("color:#91a3b5;")
            self.state_label.setText("状态：未同步")
            self.state_label.setStyleSheet("color:#91a3b5;")
        else:
            self.online_label.setText("●在线" if online else "●离线")
            self.online_label.setStyleSheet(f"color:{'#65d39a' if online else '#eb7474'};")
        self.active_label.setText("●活动" if active else "●待机")
        self.active_label.setStyleSheet(f"color:{'#66d9ef' if active else '#91a3b5'};")
        self.feedback_label.setText(f"反馈：{feedback}")
        self.feedback_label.setToolTip(f"反馈：{feedback}")
        self.viewport.set_state(self.actual_angle, self.actual_bend, bool(online), active)
        self._apply_lock()
        self.setProperty("selected", selected)
        self.style().unpolish(self)
        self.style().polish(self)

    def set_device_summary(self, online: bool, ready: bool, fault: bool) -> None:
        if fault:
            self.state_label.setText("状态：故障")
            self.state_label.setStyleSheet("color:#eb7474;")
        elif not online:
            self.state_label.setText("状态：离线")
            self.state_label.setStyleSheet("color:#91a3b5;")
        elif ready:
            self.state_label.setText("状态：就绪")
            self.state_label.setStyleSheet("color:#65d39a;")
        else:
            self.state_label.setText("状态：未就绪")
            self.state_label.setStyleSheet("color:#f2cf72;")

    def set_actual_pose(
        self,
        status: int,
        dq: tuple[int, int, int],
        angle: float,
        bend: float,
    ) -> None:
        online = bool(status & 0x01)
        ready = bool(status & 0x04)
        fault = bool(status & 0x08)
        changed = dq != self.actual_dq or abs(angle - self.actual_angle) > 0.05 or abs(bend - self.actual_bend) > 0.05
        self.actual_dq = dq
        if ready:
            self.actual_angle = angle % 360.0
            self.actual_bend = max(0.0, min(100.0, bend))
        freshness = "实时" if online else "最后值"
        trust = "可信" if ready else "未就绪"
        self.feedback = f"{freshness} {trust} {dq[0]},{dq[1]},{dq[2]}"
        self.feedback_label.setText(f"反馈：{self.feedback}")
        self.feedback_label.setToolTip(f"反馈：{self.feedback}")
        self.set_device_summary(online, ready, fault)
        if changed and ready:
            self.viewport.set_state(self.actual_angle, self.actual_bend, online, self.active)

    def set_link(self, text: str) -> None:
        self.link_label.setText(f"联动：{text}")

    def set_values(self, angle: float, bend: int, stiffness: int) -> None:
        self._updating_values = True
        try:
            self.angle = angle % 360
            self.bend = max(0, min(100, bend))
            self.stiffness = max(0, min(100, stiffness))
            self.angle_label.setText(f"控制角度\n{self.angle:.0f}°")
            self.angle_readout.setText(f"{self.angle:.0f}°")
            self.dial.set_angle(self.angle)
            self.bend_slider.setValue(self.bend)
            self.stiff_slider.setValue(self.stiffness)
            self.viewport.set_state(self.actual_angle, self.actual_bend, bool(self.online), self.active)
        finally:
            self._updating_values = False

    def _emit_control(self) -> None:
        if not self._updating_values and not self.locked and self.active:
            self.control_changed.emit(self.index, self.angle, self.bend, self.stiffness)

    def _angle_changed(self, angle: float) -> None:
        self.angle = angle
        self.angle_label.setText(f"控制角度\n{angle:.0f}°")
        self.angle_readout.setText(f"{angle:.0f}°")
        self._emit_control()

    def _bend_changed(self, value: int) -> None:
        self.bend = value
        self._emit_control()

    def _stiff_changed(self, value: int) -> None:
        self.stiffness = value
        self._emit_control()

    def _bend_wheel(self, steps: int, fast: bool) -> None:
        if self.locked or not self.active:
            return
        self.set_values(self.angle, self.bend + steps * (5 if fast else 1), self.stiffness)
        self._emit_control()


class SerialSidebar(QFrame):
    send_requested = Signal(str, str, str)
    command_selected = Signal(str)
    open_requested = Signal(object)
    close_requested = Signal()

    def __init__(self, store: CommandStore, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.store = store
        self.current_id: str | None = None
        self._adhoc_text = ""
        self._adhoc_encoding = "ASCII"
        self._adhoc_ending = "CRLF"
        self.setObjectName("sidebar")
        self.setMinimumWidth(260)
        self.setMaximumWidth(380)
        self._build_ui()
        self._populate_commands()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setAlignment(Qt.AlignTop)

        tabs = QTabWidget()
        self.tabs = tabs
        serial_page = QWidget()
        serial_root = QVBoxLayout(serial_page)
        serial_root.setContentsMargins(2, 6, 2, 2)
        serial_root.setAlignment(Qt.AlignTop)
        self.port_combo = QComboBox()
        self.refresh_ports()
        port_row = QHBoxLayout(); port_row.addWidget(add_label("端口")); port_row.addWidget(self.port_combo); serial_root.addLayout(port_row)
        self.baud_combo = QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.addItems([
            "1200", "2400", "4800", "9600", "19200", "38400", "57600",
            "115200", "230400", "460800", "921600", "1000000", "1500000",
            "2000000", "3000000",
        ])
        self.baud_combo.lineEdit().setValidator(QIntValidator(300, 12000000, self))
        self.baud_combo.setCurrentText("921600")
        self.data_combo = QComboBox(); self.data_combo.addItems(["8", "7", "6", "5"])
        self.parity_combo = QComboBox(); self.parity_combo.addItems(["None", "Even", "Odd", "Mark", "Space"])
        self.stop_combo = QComboBox(); self.stop_combo.addItems(["1", "1.5", "2"])
        self.flow_combo = QComboBox(); self.flow_combo.addItems(["None", "RTS/CTS", "XON/XOFF"])
        for label, widget in (("波特率", self.baud_combo), ("数据位", self.data_combo), ("校验位", self.parity_combo), ("停止位", self.stop_combo), ("流控", self.flow_combo)):
            row = QHBoxLayout(); row.addWidget(add_label(label)); row.addWidget(widget); serial_root.addLayout(row)
        buttons = QHBoxLayout()
        self.refresh_button = QPushButton("刷新"); self.open_button = QPushButton("打开串口"); self.open_button.setObjectName("primary")
        self.open_button.setEnabled(bool(self.port_combo.count()))
        buttons.addWidget(self.refresh_button); buttons.addWidget(self.open_button); serial_root.addLayout(buttons)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.open_button.clicked.connect(self._toggle_requested)
        serial_root.addStretch(1)

        command_page = QWidget()
        command_root = QVBoxLayout(command_page)
        command_root.setContentsMargins(2, 6, 2, 2)
        command_root.addWidget(add_label("自定义命令", "section"))
        self.command_scroll = QScrollArea(); self.command_scroll.setWidgetResizable(True); self.command_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.command_container = QWidget(); self.command_layout = QVBoxLayout(self.command_container); self.command_layout.setAlignment(Qt.AlignTop)
        self.command_scroll.setWidget(self.command_container); command_root.addWidget(self.command_scroll, 3)
        editor_row = QHBoxLayout()
        self.editor_title = add_label("临时命令", "section")
        self.adhoc_button = QToolButton()
        self.adhoc_button.setObjectName("panelToggle")
        self.adhoc_button.setText("临时命令")
        self.adhoc_button.setCheckable(True)
        self.adhoc_button.setChecked(True)
        editor_row.addWidget(self.editor_title)
        editor_row.addStretch(1)
        editor_row.addWidget(self.adhoc_button)
        command_root.addLayout(editor_row)
        self.input_edit = QPlainTextEdit(); self.input_edit.setPlaceholderText("选择参数命令后，在这里修改内容")
        self.input_edit.setMaximumHeight(75); command_root.addWidget(self.input_edit)
        row = QHBoxLayout()
        self.encoding_combo = QComboBox(); self.encoding_combo.addItems(["ASCII", "HEX"])
        self.ending_combo = QComboBox(); self.ending_combo.addItems(["NONE", "CR", "LF", "CRLF"]); self.ending_combo.setCurrentText("CRLF")
        row.addWidget(self.encoding_combo); row.addWidget(self.ending_combo); command_root.addLayout(row)
        send_row = QHBoxLayout(); self.send_button = QPushButton("发送"); self.send_button.setObjectName("primary"); self.clear_button = QPushButton("清空")
        send_row.addWidget(self.send_button); send_row.addWidget(self.clear_button); command_root.addLayout(send_row)
        self.send_button.clicked.connect(self._send_input)
        self.clear_button.clicked.connect(self.input_edit.clear)
        self.input_edit.textChanged.connect(self._save_current_draft)
        self.encoding_combo.currentTextChanged.connect(self._save_current_draft)
        self.ending_combo.currentTextChanged.connect(self._save_current_draft)
        self.adhoc_button.clicked.connect(self._activate_adhoc)

        tabs.addTab(serial_page, "串口配置")
        tabs.addTab(command_page, "命令控制")
        tabs.currentChanged.connect(self._update_tab_height)
        root.addWidget(tabs, 1)
        self.rx_label = add_label("RX：0 B   TX：0 B")
        self.status_label = add_label("状态：未连接")
        self.rx_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.status_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        root.addWidget(self.rx_label); root.addWidget(self.status_label)
        self._update_tab_height(0)

    def _update_tab_height(self, index: int) -> None:
        # Configuration is a compact form; only the command list needs full height.
        if index == 0:
            self.tabs.setMinimumHeight(250)
            self.tabs.setMaximumHeight(250)
            self.tabs.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        else:
            self.tabs.setMinimumHeight(0)
            self.tabs.setMaximumHeight(16777215)
            self.tabs.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = SerialService.available_ports()
        self.port_combo.addItems(ports)
        if current and self.port_combo.findText(current) >= 0:
            self.port_combo.setCurrentText(current)
        if hasattr(self, "open_button"):
            self.open_button.setEnabled(bool(ports))

    def _toggle_requested(self) -> None:
        if self.open_button.text() == "关闭串口":
            self.close_requested.emit()
            return
        if not self.port_combo.currentText():
            self.status_label.setText("状态：没有可用串口")
            return
        self.status_label.setText("状态：请求连接")
        self.open_requested.emit(self.serial_settings())

    def serial_settings(self) -> SerialSettings:
        return SerialSettings(self.port_combo.currentText(), int(self.baud_combo.currentText()), int(self.data_combo.currentText()), self.parity_combo.currentText(), self.stop_combo.currentText(), self.flow_combo.currentText())

    def _populate_commands(self) -> None:
        while self.command_layout.count():
            item = self.command_layout.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        self.command_sections: list[tuple[QToolButton, QWidget]] = []
        for group in self.store.groups:
            header = QToolButton()
            header.setObjectName("commandGroup")
            header.setText(group.name)
            header.setToolTip(group.description)
            header.setCheckable(True)
            header.setChecked(False)
            header.setArrowType(Qt.RightArrow)
            header.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
            header.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.command_layout.addWidget(header)
            content = QWidget()
            grid = QGridLayout(content); grid.setContentsMargins(6, 4, 2, 8); grid.setHorizontalSpacing(5); grid.setVerticalSpacing(5)
            grid.setColumnStretch(0, 1)
            for pos, definition in enumerate(group.commands):
                button = QPushButton(definition.name)
                button.setObjectName("command")
                button.setProperty("danger", "true" if definition.dangerous else "false")
                button.setToolTip(definition.description + "\n" + definition.template)
                button.clicked.connect(lambda checked=False, command_id=definition.command_id: self._select_command(command_id))
                grid.addWidget(button, pos, 0)
                action = QPushButton("执行")
                action.setObjectName("commandAction")
                action.setProperty("danger", "true" if definition.dangerous else "false")
                action.setFixedWidth(52)
                action.setToolTip(f"单次发送：{definition.template}")
                action.clicked.connect(
                    lambda checked=False, command_id=definition.command_id: self._execute_command(command_id)
                )
                grid.addWidget(action, pos, 1)
            content.setVisible(False)
            self.command_layout.addWidget(content)
            self.command_sections.append((header, content))
            header.toggled.connect(
                lambda checked, section_header=header, section_content=content:
                    self._toggle_command_group(section_header, section_content, checked)
            )
        self.command_layout.addStretch(1)

    def _toggle_command_group(self, header: QToolButton, content: QWidget, checked: bool) -> None:
        if checked:
            for other_header, other_content in self.command_sections:
                if other_header is header:
                    continue
                other_header.blockSignals(True)
                other_header.setChecked(False)
                other_header.setArrowType(Qt.RightArrow)
                other_header.blockSignals(False)
                other_content.setVisible(False)
        header.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)
        content.setVisible(checked)

    def _select_command(self, command_id: str) -> None:
        definition = self.store.definitions[command_id]
        draft = self.store.drafts[command_id]
        self._save_current_draft()
        self.current_id = command_id
        self.adhoc_button.blockSignals(True)
        self.adhoc_button.setChecked(False)
        self.adhoc_button.blockSignals(False)
        text = draft.text if definition.parameterized else definition.template
        self.input_edit.blockSignals(True); self.input_edit.setPlainText(text); self.input_edit.blockSignals(False)
        self.encoding_combo.setCurrentText(draft.encoding)
        self.ending_combo.setCurrentText(draft.line_ending)
        self.input_edit.setReadOnly(not definition.parameterized)
        self.clear_button.setEnabled(definition.parameterized)
        self.encoding_combo.setEnabled(False)
        self.ending_combo.setEnabled(False)
        self.editor_title.setText(
            f"{definition.name} · {'参数可编辑' if definition.parameterized else '固定命令'}"
        )
        self.command_selected.emit(command_id)
        self.status_label.setText(f"已选择：{definition.name}")

    def _save_current_draft(self) -> None:
        if self.current_id is None:
            self._adhoc_text = self.input_edit.toPlainText()
            self._adhoc_encoding = self.encoding_combo.currentText()
            self._adhoc_ending = self.ending_combo.currentText()
            return
        definition = self.store.definitions[self.current_id]
        if not definition.parameterized:
            return
        self.store.update_draft(
            self.current_id,
            text=self.input_edit.toPlainText(),
            encoding=definition.encoding,
            line_ending=definition.line_ending,
        )
        self.store.save()

    def _activate_adhoc(self) -> None:
        self._save_current_draft()
        self.current_id = None
        self.adhoc_button.blockSignals(True)
        self.adhoc_button.setChecked(True)
        self.adhoc_button.blockSignals(False)
        self.input_edit.blockSignals(True)
        self.input_edit.setPlainText(self._adhoc_text)
        self.input_edit.blockSignals(False)
        self.encoding_combo.setCurrentText(self._adhoc_encoding)
        self.ending_combo.setCurrentText(self._adhoc_ending)
        self.input_edit.setReadOnly(False)
        self.clear_button.setEnabled(True)
        self.encoding_combo.setEnabled(True)
        self.ending_combo.setEnabled(True)
        self.editor_title.setText("临时命令 · 不修改命令库")
        self.status_label.setText("已切换：临时命令")

    def _execute_command(self, command_id: str) -> None:
        definition = self.store.definitions[command_id]
        self._select_command(command_id)
        if definition.dangerous:
            answer = QMessageBox.question(
                self,
                "确认执行",
                f"确定单次发送“{definition.name}”？\n\n{definition.template}",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                self.status_label.setText("已取消危险命令")
                return
        self._send_input()

    @staticmethod
    def _validate_command(definition: CommandDefinition, text: str) -> str | None:
        normalized = text.strip()
        if not normalized:
            return "发送内容不能为空"
        if not definition.parameterized:
            return None if normalized == definition.template.strip() else "固定命令不可修改"
        tokens = normalized.split()
        expected = definition.template.strip().split()[0].upper()
        if not tokens or tokens[0].upper() != expected:
            return f"命令必须以 {expected} 开头"
        try:
            values = [int(value) for value in tokens[1:]]
        except ValueError:
            return "参数必须是整数"
        if expected in ("CANPOS", "CANREL", "CANABS"):
            return None if len(values) == 3 else "需要三个位置参数"
        if expected == "MVIRT":
            if len(values) != 4:
                return "需要 X、Y、Bend、刚度四个参数"
            x, y, bend, stiffness = values
            if not (-100 <= x <= 100 and -100 <= y <= 100):
                return "X、Y 必须在 -100~100"
            if not (0 <= bend <= 100 and 0 <= stiffness <= 100):
                return "Bend、刚度必须在 0~100"
            return None
        if expected == "SETREC":
            if len(values) != 2:
                return "需要采样数量和间隔两个参数"
            return None if values[0] > 0 and values[1] > 0 else "采样数量和间隔必须大于 0"
        if expected == "BALCFG":
            if len(values) != 5:
                return "需要起点、终点、步长、每档数量、间隔五个参数"
            start, end, step, target, interval = values
            if step <= 0 or target <= 0 or interval <= 0 or start > end:
                return "范围或采样参数无效"
            if (end - start) % step != 0:
                return "终点与起点之差必须能被步长整除"
            if end > 65535 or step > 65535:
                return "距离和步长不能超过 65535"
            return None
        return None

    def _send_input(self) -> None:
        self._save_current_draft()
        text = self.input_edit.toPlainText()
        if self.current_id:
            definition = self.store.definitions[self.current_id]
            error = self._validate_command(definition, text)
            if error:
                self.status_label.setText(f"未发送：{error}")
                return
            self.store.update_draft(self.current_id, last_sent=text)
            self.store.save()
        elif not text.strip():
            self.status_label.setText("未发送：临时命令为空")
            return
        self.send_requested.emit(text, self.encoding_combo.currentText(), self.ending_combo.currentText())

    def update_counters(self, rx: int, tx: int) -> None:
        self.rx_label.setText(f"RX：{rx:,} B   TX：{tx:,} B")

    def set_connected(self, connected: bool, message: str) -> None:
        self.status_label.setText(f"状态：{message}")
        self.open_button.setText("关闭串口" if connected else "打开串口")
        self.open_button.setEnabled(connected or self.port_combo.count() > 0)


class ModeSidebar(QFrame):
    mode_requested = Signal(str)
    source_requested = Signal(str)
    start_requested = Signal()
    stop_requested = Signal()
    emergency_requested = Signal()
    mode_draft_changed = Signal(bool)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._mode_editor_dirty = False
        self._mode_request_pending = False
        self._act_mode_locked = False
        self.setObjectName("modebar")
        self.setMinimumWidth(220); self.setMaximumWidth(300)
        root = QVBoxLayout(self); root.setContentsMargins(10, 8, 10, 8)
        root.addWidget(add_label("系统与模式", "section"))
        root.addWidget(add_label("控制来源"))
        source_row = QHBoxLayout()
        self.ai_source_button = QToolButton()
        self.ai_source_button.setText("AI")
        self.ai_source_button.setObjectName("panelToggle")
        self.ai_source_button.setCheckable(True)
        self.pc_source_button = QToolButton()
        self.pc_source_button.setText("PC")
        self.pc_source_button.setObjectName("panelToggle")
        self.pc_source_button.setCheckable(True)
        self.act_source_button = QToolButton()
        self.act_source_button.setText("ACT")
        self.act_source_button.setObjectName("panelToggle")
        self.act_source_button.setCheckable(True)
        source_row.addWidget(self.ai_source_button)
        source_row.addWidget(self.pc_source_button)
        source_row.addWidget(self.act_source_button)
        root.addLayout(source_row)
        self.ai_source_button.clicked.connect(lambda: self.source_requested.emit("AI"))
        self.pc_source_button.clicked.connect(lambda: self.source_requested.emit("PC"))
        self.act_source_button.clicked.connect(lambda: self.source_requested.emit("ACT"))
        self.set_control_source(None)
        root.addWidget(add_label("工作模式"))
        self.mode_combo = QComboBox(); self.mode_combo.addItems(["单触手", "双触手", "四触手"]); root.addWidget(self.mode_combo)
        root.addWidget(add_label("触手对象"))
        self.group_combo = QComboBox(); root.addWidget(self.group_combo)
        root.addWidget(add_label("联动关系"))
        self.relation_combo = QComboBox(); root.addWidget(self.relation_combo)
        self.apply_button = QPushButton("确认并应用模式"); self.apply_button.setObjectName("primary"); root.addWidget(self.apply_button)
        self.apply_button.setToolTip("仅向H7应用工作组合，不开启推理，也不启动电机")
        self.apply_button.clicked.connect(self._apply)
        self.mode_apply_state = add_label("○ 等待H7同步", "muted")
        self.mode_apply_state.setWordWrap(True)
        root.addWidget(self.mode_apply_state)
        self.mode_combo.currentTextChanged.connect(self._update_mode_controls)
        self.mode_combo.activated.connect(self._mark_mode_editor_dirty)
        self.group_combo.activated.connect(self._mark_mode_editor_dirty)
        self.relation_combo.activated.connect(self._mark_mode_editor_dirty)
        self._update_mode_controls(self.mode_combo.currentText())
        root.addWidget(add_label("当前运动模式", "section"))
        self.motion_mode_label = add_label("单触手 · T1", "value")
        self.motion_mode_label.setWordWrap(True)
        root.addWidget(self.motion_mode_label)
        root.addWidget(add_label("可用性", "section"))
        self.availability = [QLabel() for _ in range(4)]
        for label in self.availability: root.addWidget(label)
        root.addStretch(1)
        self.start_button = HoldButton("长按启动", progress_text="正在启动")
        self.start_button.setObjectName("primary")
        self.stop_button = QPushButton("停止")
        self.emergency_button = QPushButton("紧急停止"); self.emergency_button.setObjectName("danger")
        root.addWidget(self.start_button); root.addWidget(self.stop_button); root.addWidget(self.emergency_button)
        self.start_button.held.connect(self.start_requested.emit)
        self.stop_button.clicked.connect(self.stop_requested.emit)
        self.emergency_button.clicked.connect(self.emergency_requested.emit)
        self.set_runtime_state("UNKNOWN", set(), [None] * 4, False)

    def set_control_source(self, source: str | None, pending: bool = False) -> None:
        normalized = source.upper() if source else None
        for button, value in (
            (self.ai_source_button, "AI"),
            (self.pc_source_button, "PC"),
            (self.act_source_button, "ACT"),
        ):
            button.blockSignals(True)
            button.setChecked(normalized == value)
            button.setEnabled(not pending and normalized is not None)
            button.blockSignals(False)

    def set_source_available(self, available: bool) -> None:
        self.ai_source_button.setEnabled(available)
        self.pc_source_button.setEnabled(available)
        self.act_source_button.setEnabled(available)

    def set_act_mode(self, active: bool) -> None:
        """ACT来源固定使用SOLO T3，并禁止误改PC工作组合。"""
        self._act_mode_locked = bool(active)
        if active:
            self._mode_editor_dirty = False
            self._mode_request_pending = False
            self.set_mode_selection("SOLO 3", force=True)
            self.apply_button.setText("ACT固定模式")
            self.mode_apply_state.setText("● ACT固定：单触手 · T3 · 不适用")
            self.mode_apply_state.setStyleSheet("color:#66d9ef; font-weight:700;")
        else:
            self.apply_button.setText("确认并应用模式")
            self.mode_apply_state.setText("✓ 当前模式已应用")
            self.mode_apply_state.setStyleSheet("color:#65d39a; font-weight:700;")
        self._refresh_mode_editor_enabled()
        self.mode_draft_changed.emit(self.has_unapplied_mode())

    def _apply(self) -> None:
        mode = self.mode_combo.currentText()
        if mode == "单触手":
            self.mode_requested.emit(f"SOLO {self.group_combo.currentIndex() + 1}")
        elif mode == "双触手":
            group = "12" if self.group_combo.currentIndex() == 0 else "34"
            relation = "MIRROR" if self.relation_combo.currentText() == "镜像" else "SAME"
            self.mode_requested.emit(f"DUAL {group} {relation}")
        else:
            relation = "CENTER" if self.relation_combo.currentText() == "中心对称" else "SAME"
            self.mode_requested.emit(f"QUAD {relation}")

    def _mark_mode_editor_dirty(self, _index: int = -1) -> None:
        """标记用户正在编辑模式，避免周期状态帧覆盖尚未应用的选择。"""
        if not self._mode_request_pending and not self._act_mode_locked:
            if not self._mode_editor_dirty:
                self._mode_editor_dirty = True
                self.apply_button.setText("确认并应用模式")
                self.mode_apply_state.setText("● 模式已修改，尚未应用")
                self.mode_apply_state.setStyleSheet("color:#f2cf72; font-weight:700;")
                self.mode_draft_changed.emit(True)

    def has_unapplied_mode(self) -> bool:
        """返回模式编辑器是否存在未应用草稿或等待H7确认的请求。"""
        return self._mode_editor_dirty or self._mode_request_pending

    def set_mode_request_pending(self, pending: bool) -> None:
        """切换模式请求等待状态；等待H7确认期间禁止重复修改和发送。"""
        self._mode_request_pending = pending
        self.apply_button.setText("等待H7确认…" if pending else "确认并应用模式")
        if pending:
            self.mode_apply_state.setText("● 模式命令已发送，等待SNAP确认")
            self.mode_apply_state.setStyleSheet("color:#f2cf72; font-weight:700;")
        self._refresh_mode_editor_enabled()
        self.mode_draft_changed.emit(self.has_unapplied_mode())

    def complete_mode_request(self, mode_text: str | None) -> None:
        """结束模式编辑，并按H7实际状态恢复或提交编辑器内容。"""
        self._mode_editor_dirty = False
        self._mode_request_pending = False
        if mode_text and not mode_text.upper().startswith("UNKNOWN"):
            self.set_mode_selection(mode_text, force=True)
        self.apply_button.setText("确认并应用模式")
        self.mode_apply_state.setText("✓ 当前模式已应用")
        self.mode_apply_state.setStyleSheet("color:#65d39a; font-weight:700;")
        self._refresh_mode_editor_enabled()
        self.mode_draft_changed.emit(False)

    def reset_mode_editor(self) -> None:
        """串口状态改变时清除未完成的模式草稿和等待状态。"""
        self._mode_editor_dirty = False
        self._mode_request_pending = False
        self.apply_button.setText("确认并应用模式")
        self.mode_apply_state.setText("○ 等待H7同步")
        self.mode_apply_state.setStyleSheet("color:#91a3b5;")
        self._refresh_mode_editor_enabled()
        self.mode_draft_changed.emit(False)

    def fail_mode_request(self, message: str) -> None:
        """显示模式应用失败，并允许用户修正选择后重试。"""
        self._mode_request_pending = False
        self._mode_editor_dirty = True
        self.apply_button.setText("重试应用模式")
        self.mode_apply_state.setText(f"● {message}")
        self.mode_apply_state.setStyleSheet("color:#eb7474; font-weight:700;")
        self._refresh_mode_editor_enabled()
        self.mode_draft_changed.emit(True)

    def _refresh_mode_editor_enabled(self) -> None:
        """按当前模式和等待状态统一刷新模式编辑控件可用性。"""
        enabled = not self._mode_request_pending and not self._act_mode_locked
        mode = self.mode_combo.currentText()
        self.mode_combo.setEnabled(enabled)
        self.group_combo.setEnabled(enabled and mode != "四触手")
        self.relation_combo.setEnabled(enabled and mode != "单触手")
        self.apply_button.setEnabled(enabled)

    def _update_mode_controls(self, mode: str) -> None:
        self.group_combo.clear()
        self.relation_combo.clear()
        if mode == "单触手":
            self.group_combo.addItems(["T1", "T2", "T3", "T4"])
            self.group_combo.setEnabled(True)
            self.relation_combo.addItem("不适用")
            self.relation_combo.setEnabled(False)
        elif mode == "双触手":
            self.group_combo.addItems(["T1 + T2", "T3 + T4"])
            self.group_combo.setEnabled(True)
            self.relation_combo.addItems(["同向", "镜像"])
            self.relation_combo.setEnabled(True)
        else:
            self.group_combo.addItem("T1 + T2 + T3 + T4")
            self.group_combo.setEnabled(False)
            self.relation_combo.addItems(["同向", "中心对称"])
            self.relation_combo.setEnabled(True)
        self._refresh_mode_editor_enabled()

    def set_mode_selection(self, mode_text: str, *, force: bool = False) -> None:
        """同步模式编辑器；用户正在编辑或等待确认时不接受周期状态覆盖。"""
        if (self._mode_editor_dirty or self._mode_request_pending) and not force:
            return
        text = mode_text.upper().split()
        if not text:
            return
        if text[0] == "SOLO":
            self.mode_combo.setCurrentText("单触手")
            self.group_combo.setCurrentIndex(max(0, min(3, int(text[1]) - 1 if len(text) > 1 else 0)))
        elif text[0] == "DUAL":
            self.mode_combo.setCurrentText("双触手")
            self.group_combo.setCurrentIndex(0 if len(text) < 2 or text[1] == "12" else 1)
            self.relation_combo.setCurrentText("镜像" if "MIRROR" in text else "同向")
        elif text[0] == "QUAD":
            self.mode_combo.setCurrentText("四触手")
            self.relation_combo.setCurrentText("中心对称" if "CENTER" in text else "同向")
        if not self._act_mode_locked and not self.has_unapplied_mode():
            self.apply_button.setText("确认并应用模式")
            self.mode_apply_state.setText("✓ 当前模式已应用")
            self.mode_apply_state.setStyleSheet("color:#65d39a; font-weight:700;")

    def set_runtime_state(self, mode_text: str, active: set[int], online: list[bool | None], running: bool) -> None:
        text = mode_text.upper().split()
        if not text or text[0] == "UNKNOWN":
            description = "未同步"
        elif text[0] == "SOLO":
            target = text[1] if len(text) > 1 else "1"
            description = f"单触手 · T{target}"
        elif text and text[0] == "DUAL":
            group = text[1] if len(text) > 1 else "12"
            relation = "镜像" if "MIRROR" in text else "同向"
            description = f"双触手 · T{group[0]}+T{group[-1]} · {relation}"
        else:
            relation = "中心对称" if "CENTER" in text else "同向"
            description = f"四触手 · {relation}"
        self.motion_mode_label.setText(description)
        for index, label in enumerate(self.availability):
            online_state = online[index] if index < len(online) else None
            available = online_state is True
            if online_state is None:
                connection = '<span style="color:#91a3b5">● 未同步</span>'
            elif available:
                connection = '<span style="color:#65d39a">● 可用</span>'
            else:
                connection = '<span style="color:#eb7474">● 离线</span>'
            if index in active and available:
                state = (
                    '<span style="color:#66d9ef">● 运行中</span>'
                    if running else '<span style="color:#8fc6de">● 已选中</span>'
                )
            else:
                state = '<span style="color:#91a3b5">○ 待机</span>'
            label.setText(f"<b>T{index + 1}</b>&nbsp;&nbsp;{connection}&nbsp;&nbsp;{state}")
        if running:
            self.start_button.setEnabled(False)
            self.start_button.setText("● 已启动")
            self.start_button.setStyleSheet(
                "QPushButton { background:#236b50; color:#effff8; "
                "border:1px solid #65d39a; font-weight:700; }"
            )
            self.stop_button.setEnabled(True)
        else:
            self.start_button.setEnabled(bool(active))
            self.start_button.setText("长按启动")
            self.start_button.setStyleSheet("")
            self.stop_button.setEnabled(False)


class ReceiveDrawer(QFrame):
    fullscreen_requested = Signal()
    gesture_requested = Signal(bool)

    def __init__(self, settings: QSettings, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.settings = settings
        self._detail_tentacle = 0
        self._detail_page = int(self.settings.value("receive_detail_page", 0))
        self._detail_page = max(0, min(2, self._detail_page))
        self._detail_width = int(self.settings.value("receive_detail_width", 400))
        self._detail_width = max(360, min(640, self._detail_width))
        self._detail_open = self.settings.value("receive_detail_open", False, type=bool)
        self._poses = [(0, (0, 0, 0), 0.0, 0.0) for _ in range(4)]
        self._pose_received = [False] * 4
        self._gesture_enabled: bool | None = None
        self._gesture_pending_target: bool | None = None
        self._act_source_selected = False
        self.setObjectName("receivePanel")
        self.setStyleSheet(
            "QFrame#receivePanel { background:#1b2a36; border:1px solid #3c5263; }"
            "QFrame#summaryStrip { background:#16242e; border:1px solid #344a5a; }"
            "QToolButton#tentacleSummary { text-align:left; background:#13212a; color:#91a3b5; "
            "border:0; border-right:1px solid #304351; padding:6px 10px; font-weight:600; }"
            "QToolButton#tentacleSummary:hover { background:#203543; color:#e8f2f7; }"
            "QToolButton#tentacleSummary[state='ready'] { color:#86e2b0; border-bottom:2px solid #65d39a; }"
            "QToolButton#tentacleSummary[state='waiting'] { color:#f2cf72; border-bottom:2px solid #d4ad52; }"
            "QToolButton#tentacleSummary[state='fault'] { color:#ff9696; border-bottom:2px solid #d2686e; }"
            "QFrame#detailDrawer { background:#172832; border-left:2px solid #4db6ce; }"
            "QFrame#detailPage { background:#172832; border:0; }"
            "QFrame#gestureBar { background:#13212a; border:1px solid #304857; border-radius:4px; }"
            "QLabel#liveTitle { background:transparent; color:#8fc6de; font-weight:700; padding:4px 2px; }"
            "QLabel#liveValue { background:transparent; color:#e4edf3; padding:2px 0; }"
            "QLabel#poseHeader { background:transparent; color:#9fb3c1; font-weight:600; padding:4px 2px; }"
            "QLabel#metricValue { background:#13212a; color:#f0f5f8; border:1px solid #293f4d; "
            "border-radius:3px; padding:4px 7px; }"
            "QToolButton#drawerAction { background:#243746; color:#e8f2f7; border:1px solid #496173; "
            "border-radius:3px; padding:0 8px; font-weight:600; }"
            "QToolButton#drawerAction:hover { background:#315064; border-color:#66b8ce; }"
            "QToolButton#detailRail { min-width:36px; min-height:52px; padding:4px; "
            "background:#20313d; color:#9ab0bf; border:1px solid #344a5a; }"
            "QToolButton#detailRail:checked { background:#176d82; color:white; border-color:#4db6ce; }"
            "QPlainTextEdit { background:#111e27; color:#d7e5ee; border:1px solid #40586a; "
            "padding:8px; font-family:Consolas, 'Microsoft YaHei UI'; font-size:13px; }"
        )
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 7, 8, 8)
        root.setSpacing(6)

        header_row = QHBoxLayout()
        header_row.setSpacing(5)
        self.header = QLabel()
        self.header.setObjectName("value")
        self.header.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        header_row.addWidget(self.header, 1)
        self.fullscreen_button = QToolButton()
        self.fullscreen_button.setObjectName("drawerAction")
        self.fullscreen_button.setText("全屏")
        self.fullscreen_button.setToolTip("全屏显示（F11）")
        self.fullscreen_button.setFixedSize(62, 30)
        self.fullscreen_button.clicked.connect(self.fullscreen_requested.emit)
        header_row.addWidget(self.fullscreen_button)
        self.clear_button = QToolButton()
        self.clear_button.setObjectName("drawerAction")
        self.clear_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_TrashIcon)
        )
        self.clear_button.setIconSize(QSize(18, 18))
        self.clear_button.setToolTip("清空调试记录")
        self.clear_button.setFixedSize(34, 30)
        header_row.addWidget(self.clear_button)
        root.addLayout(header_row)

        summary_strip = QFrame()
        summary_strip.setObjectName("summaryStrip")
        summary_strip.setFixedHeight(44)
        summary_row = QHBoxLayout(summary_strip)
        summary_row.setContentsMargins(0, 0, 0, 0)
        summary_row.setSpacing(0)
        self.tentacle_summary_buttons: list[QToolButton] = []
        for index in range(4):
            button = QToolButton()
            button.setObjectName("tentacleSummary")
            button.setProperty("state", "offline")
            button.setText(f"T{index + 1}  ○ 未同步   —°   B—")
            button.setToolTip(f"查看T{index + 1}实际姿态")
            button.setMinimumWidth(0)
            button.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
            button.clicked.connect(lambda _checked=False, i=index: self.show_pose_detail(i))
            summary_row.addWidget(button, 1)
            self.tentacle_summary_buttons.append(button)
        root.addWidget(summary_strip)

        body_row = QHBoxLayout()
        body_row.setContentsMargins(0, 0, 0, 0)
        body_row.setSpacing(4)
        self.detail_splitter = QSplitter(Qt.Horizontal)
        self.detail_splitter.setOpaqueResize(False)
        self.detail_splitter.setHandleWidth(6)
        self.detail_splitter.setChildrenCollapsible(False)

        log_frame = QFrame()
        log_layout = QVBoxLayout(log_frame)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_layout.setSpacing(4)
        log_layout.addWidget(self._live_label("事件 / 调试记录", "liveTitle"))
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.text.document().setMaximumBlockCount(1000)
        log_layout.addWidget(self.text, 1)
        self.clear_button.clicked.connect(self.text.clear)
        self.detail_splitter.addWidget(log_frame)

        self.detail_drawer = QFrame()
        self.detail_drawer.setObjectName("detailDrawer")
        self.detail_drawer.setMinimumWidth(360)
        self.detail_drawer.setMaximumWidth(640)
        detail_layout = QVBoxLayout(self.detail_drawer)
        detail_layout.setContentsMargins(10, 7, 8, 8)
        detail_layout.setSpacing(6)
        detail_header = QHBoxLayout()
        self.detail_title = self._live_label("触手姿态", "liveTitle")
        detail_header.addWidget(self.detail_title, 1)
        close_detail = QToolButton()
        close_detail.setObjectName("drawerAction")
        close_detail.setText("×")
        close_detail.setFont(QFont("Segoe UI", 14, QFont.Weight.DemiBold))
        close_detail.setToolTip("收起侧栏")
        close_detail.setFixedSize(30, 28)
        close_detail.clicked.connect(self.close_detail)
        detail_header.addWidget(close_detail)
        detail_layout.addLayout(detail_header)
        self.detail_stack = QStackedWidget()
        self.detail_stack.addWidget(self._build_pose_page())
        self.detail_stack.addWidget(self._build_ai_page())
        self.detail_stack.addWidget(self._build_system_page())
        detail_layout.addWidget(self.detail_stack, 1)
        self.detail_splitter.addWidget(self.detail_drawer)
        self.detail_splitter.setStretchFactor(0, 1)
        self.detail_splitter.setStretchFactor(1, 0)
        self.detail_splitter.splitterMoved.connect(self._remember_detail_width)
        body_row.addWidget(self.detail_splitter, 1)

        rail = QFrame()
        rail_layout = QVBoxLayout(rail)
        rail_layout.setContentsMargins(0, 0, 0, 0)
        rail_layout.setSpacing(4)
        self.detail_buttons: list[QToolButton] = []
        for page, title in enumerate(("姿态", "AI", "系统")):
            button = QToolButton()
            button.setObjectName("detailRail")
            button.setText(title)
            button.setCheckable(True)
            button.setFixedWidth(42)
            button.setToolTip(f"展开{title}详情")
            button.clicked.connect(lambda _checked=False, p=page: self.show_detail_page(p))
            rail_layout.addWidget(button)
            self.detail_buttons.append(button)
        rail_layout.addStretch(1)
        body_row.addWidget(rail)
        root.addLayout(body_row, 1)

        self.pose_values: list[list[QLabel]] = []
        for index in range(4):
            self.pose_values.append([
                self._live_label(f"T{index + 1}"),
                self._live_label("未同步"),
                self._live_label("—"),
                self._live_label("—"),
                self._live_label("—"),
                self._live_label("—° / —"),
            ])

        self._rx = 0
        self._tx = 0
        self._last = "--"
        self.set_summary(0, 0)
        self.detail_stack.setCurrentIndex(self._detail_page)
        self.detail_drawer.setVisible(self._detail_open)
        self._sync_detail_buttons()
        self._refresh_pose_detail()

    def _build_pose_page(self) -> QWidget:
        page = QFrame()
        page.setObjectName("detailPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        selector = QHBoxLayout()
        selector.setSpacing(4)
        self.pose_select_buttons: list[QToolButton] = []
        for index in range(4):
            button = QToolButton()
            button.setText(f"T{index + 1}")
            button.setCheckable(True)
            button.clicked.connect(lambda _checked=False, i=index: self.select_pose_detail(i))
            selector.addWidget(button)
            self.pose_select_buttons.append(button)
        layout.addLayout(selector)
        self.detail_pose_view = TentacleViewport()
        self.detail_pose_view.setMinimumSize(180, 145)
        layout.addWidget(self.detail_pose_view, 1)
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        self.detail_pose_status = self._live_label("未同步", "metricValue")
        self.detail_pose_dq = [self._live_label("—", "metricValue") for _ in range(3)]
        self.detail_pose_angle = self._live_label("—°", "metricValue")
        self.detail_pose_bend = self._live_label("—", "metricValue")
        grid.addWidget(self._metric_name("状态"), 0, 0)
        grid.addWidget(self.detail_pose_status, 0, 1, 1, 3)
        grid.addWidget(self._metric_name("拉索1"), 1, 0)
        grid.addWidget(self.detail_pose_dq[0], 1, 1)
        grid.addWidget(self._metric_name("拉索2"), 1, 2)
        grid.addWidget(self.detail_pose_dq[1], 1, 3)
        grid.addWidget(self._metric_name("拉索3"), 2, 0)
        grid.addWidget(self.detail_pose_dq[2], 2, 1)
        grid.addWidget(self._metric_name("方向"), 2, 2)
        grid.addWidget(self.detail_pose_angle, 2, 3)
        grid.addWidget(self._metric_name("Bend"), 3, 0)
        grid.addWidget(self.detail_pose_bend, 3, 1)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        layout.addLayout(grid)
        return page

    def _build_ai_page(self) -> QWidget:
        page = QFrame()
        page.setObjectName("detailPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        self.ai_state = self._live_label("AI 输入 · 等待推理帧", "liveTitle")
        layout.addWidget(self.ai_state)
        gesture_bar = QFrame()
        gesture_bar.setObjectName("gestureBar")
        gesture_layout = QHBoxLayout(gesture_bar)
        gesture_layout.setContentsMargins(8, 3, 6, 3)
        gesture_layout.setSpacing(8)
        self.gesture_state = self._live_label("手势推理 · 未同步")
        self.gesture_button = QPushButton("开启推理")
        self.gesture_button.setFixedWidth(96)
        self.gesture_button.setEnabled(False)
        self.gesture_button.setToolTip("只控制H7手势模型，不会自动启动触手运动")
        self.gesture_button.clicked.connect(self._request_gesture_toggle)
        gesture_layout.addWidget(self.gesture_state, 1)
        gesture_layout.addWidget(self.gesture_button)
        layout.addWidget(gesture_bar)
        act_bar = QFrame()
        act_bar.setObjectName("gestureBar")
        act_layout = QVBoxLayout(act_bar)
        act_layout.setContentsMargins(8, 3, 6, 3)
        act_layout.setSpacing(1)
        self.act_state = self._live_label("○ ACT自主 · 未同步")
        self.act_detail = self._live_label("历史 --/12 · 结果 --")
        self.act_detail.setStyleSheet("color:#91a3b5;")
        act_layout.addWidget(self.act_state)
        act_layout.addWidget(self.act_detail)
        layout.addWidget(act_bar)
        self.ai_dial = DirectionDial()
        self.ai_dial.locked = True
        # 圆盘与下方参数表使用独立的高度预算，避免小窗口中270°刻度压到X/Y行。
        self.ai_dial.setMinimumSize(180, 160)
        self.ai_dial.setMaximumHeight(178)
        self.ai_dial.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.ai_dial, 1)
        layout.addSpacing(4)
        value_grid = QGridLayout()
        value_grid.setContentsMargins(0, 0, 0, 0)
        value_grid.setHorizontalSpacing(8)
        value_grid.setVerticalSpacing(3)
        self.ai_values = {
            name: self._live_label(default, "metricValue")
            for name, default in (
                ("x", "—"), ("y", "—"), ("angle", "—°"),
                ("distance", "— mm"), ("bend", "—"),
                ("raw", "—"), ("stiffness", "—"),
            )
        }
        ai_entries = (
            ("X", "x", "Y", "y"),
            ("角度", "angle", "距离", "distance"),
            ("Bend", "bend", "Raw", "raw"),
            ("刚度", "stiffness", None, None),
        )
        for row, (left_title, left_key, right_title, right_key) in enumerate(ai_entries):
            value_grid.addWidget(self._metric_name(left_title), row, 0)
            value_grid.addWidget(self.ai_values[left_key], row, 1)
            if right_title and right_key:
                value_grid.addWidget(self._metric_name(right_title), row, 2)
                value_grid.addWidget(self.ai_values[right_key], row, 3)
        value_grid.setColumnStretch(1, 1)
        value_grid.setColumnStretch(3, 1)
        layout.addLayout(value_grid)
        # Preserve the original text fields as a stable internal interface.
        self.ai_direction = self._live_label("X --   Y --   角度 --°")
        self.ai_shape = self._live_label("距离 -- mm   Bend --   Raw --")
        self.ai_extra = self._live_label("刚度 --")
        for label in (self.ai_direction, self.ai_shape, self.ai_extra):
            label.setParent(page)
            label.hide()
        layout.addStretch(1)
        return page

    def _build_system_page(self) -> QWidget:
        page = QFrame()
        page.setObjectName("detailPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        self.system_values = {
            name: self._live_label(default, "metricValue")
            for name, default in (
                ("source", "—"), ("control", "—"), ("mode", "—"),
                ("object", "—"), ("online", "—/4"), ("ready", "—/4"),
                ("fault", "—"), ("x", "—"), ("y", "—"),
                ("bend", "—"), ("stiffness", "—"), ("tx", "—"),
            )
        }
        sections = (
            ("控制状态", (("来源", "source", "控制", "control"), ("模式", "mode", None, None))),
            ("设备状态", (("对象", "object", "在线", "online"), ("就绪", "ready", "故障", "fault"))),
            ("输入状态", (("X", "x", "Y", "y"), ("Bend", "bend", "刚度", "stiffness"), ("TX", "tx", None, None))),
        )
        for section_title, rows in sections:
            layout.addWidget(self._live_label(section_title, "liveTitle"))
            grid = QGridLayout()
            grid.setContentsMargins(0, 0, 0, 0)
            grid.setHorizontalSpacing(8)
            grid.setVerticalSpacing(6)
            for row, (left_title, left_key, right_title, right_key) in enumerate(rows):
                grid.addWidget(self._metric_name(left_title), row, 0)
                if right_title is None:
                    grid.addWidget(self.system_values[left_key], row, 1, 1, 3)
                else:
                    grid.addWidget(self.system_values[left_key], row, 1)
                    grid.addWidget(self._metric_name(right_title), row, 2)
                    grid.addWidget(self.system_values[right_key], row, 3)
            grid.setColumnStretch(1, 1)
            grid.setColumnStretch(3, 1)
            layout.addLayout(grid)
        # Preserve the original text fields for existing tests and integrations.
        self.snapshot_title = self._live_label("系统快照")
        self.snapshot_mode = self._live_label("来源 --\n控制 --\n模式 --")
        self.snapshot_device = self._live_label("对象 --\n在线 --/4\n就绪 --/4\n故障 --")
        self.snapshot_input = self._live_label("输入 X --  Y --\nBend --  刚度 --\nTX --")
        for label in (
            self.snapshot_title,
            self.snapshot_mode,
            self.snapshot_device,
            self.snapshot_input,
        ):
            label.setParent(page)
            label.hide()
        layout.addStretch(1)
        return page

    @staticmethod
    def _live_label(text: str, object_name: str = "liveValue") -> QLabel:
        label = QLabel(text)
        label.setObjectName(object_name)
        label.setTextFormat(Qt.PlainText)
        label.setWordWrap(object_name == "liveValue")
        label.setMinimumWidth(0)
        label.setMinimumHeight(26)
        label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        label.setSizePolicy(
            QSizePolicy.Preferred,
            QSizePolicy.Minimum if object_name == "liveValue" else QSizePolicy.Fixed,
        )
        return label

    @staticmethod
    def _metric_name(text: str) -> QLabel:
        label = ReceiveDrawer._live_label(text, "poseHeader")
        label.setMinimumWidth(38)
        label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        return label

    def set_ai_mapping(
        self,
        valid: bool,
        x: int | None = None,
        y: int | None = None,
        angle: int | None = None,
        distance: int | None = None,
        bend: int | None = None,
        raw: int | None = None,
        status: int | None = None,
    ) -> None:
        if not valid:
            suffix = f" · st={status}" if status is not None else ""
            self._clear_ai_mapping(f"AI 输入 · 无效{suffix}", "#eb7474")
            return
        self.ai_state.setText("AI 输入 · 有效")
        self.ai_state.setStyleSheet("color:#65d39a; font-weight:700;")
        if self._gesture_enabled is True and self._gesture_pending_target is None:
            self.gesture_state.setText("● 手势推理 · 输出中")
            self.gesture_state.setStyleSheet("color:#65d39a; font-weight:700;")
        self.ai_direction.setText(f"X {x}   Y {y}   角度 {angle}°")
        self.ai_shape.setText(f"距离 {distance} mm   Bend {bend}   Raw {raw}")
        self.ai_dial.set_angle(float(angle or 0))
        self.ai_values["x"].setText(str(x))
        self.ai_values["y"].setText(str(y))
        self.ai_values["angle"].setText(f"{angle}°")
        self.ai_values["distance"].setText(f"{distance} mm")
        self.ai_values["bend"].setText(str(bend))
        self.ai_values["raw"].setText(str(raw))

    def _request_gesture_toggle(self) -> None:
        """请求切换H7手势推理；真实状态仍等待UART7回复确认。"""

        if (
            self._act_source_selected
            or self._gesture_enabled is None
            or self._gesture_pending_target is not None
        ):
            return
        self.gesture_requested.emit(not self._gesture_enabled)

    def set_gesture_inference_state(
        self,
        enabled: bool | None,
        pending_target: bool | None = None,
        error: str | None = None,
    ) -> None:
        """显示H7已确认的推理状态、待确认目标和操作错误。"""

        self._gesture_enabled = enabled
        self._gesture_pending_target = pending_target
        if pending_target is not None:
            action = "开启" if pending_target else "关闭"
            self.gesture_state.setText(f"● 手势推理 · 正在{action}")
            self.gesture_state.setStyleSheet("color:#f2cf72; font-weight:700;")
            self.gesture_button.setText(f"正在{action}…")
            self.gesture_button.setEnabled(False)
            return
        if error:
            self.gesture_state.setText(f"● 手势推理 · {error}")
            self.gesture_state.setStyleSheet("color:#eb7474; font-weight:700;")
        elif enabled is None:
            self.gesture_state.setText("○ 手势推理 · 未同步")
            self.gesture_state.setStyleSheet("color:#91a3b5;")
        elif enabled:
            self.gesture_state.setText("● 手势推理 · 已允许，等待AI数据")
            self.gesture_state.setStyleSheet("color:#65d39a; font-weight:700;")
        else:
            self.gesture_state.setText("○ 手势推理 · 已关闭")
            self.gesture_state.setStyleSheet("color:#91a3b5;")
            self._clear_ai_mapping("AI 输入 · 推理已关闭", "#91a3b5")
        self.gesture_button.setText("关闭推理" if enabled else "开启推理")
        self.gesture_button.setEnabled(
            enabled is not None and not self._act_source_selected
        )

    def set_control_source(self, source: str | None) -> None:
        """ACT来源下禁止重新开启手势CNN，避免与ACT模型争用推理资源。"""
        self._act_source_selected = source == "ACT"
        if self._act_source_selected:
            self.gesture_button.setEnabled(False)
            self.gesture_button.setToolTip("ACT自主控制不使用手势CNN；切回AI后可重新设置")
        else:
            self.gesture_button.setEnabled(
                self._gesture_enabled is not None
                and self._gesture_pending_target is None
            )
            self.gesture_button.setToolTip("只控制H7手势模型，不会自动启动触手运动")

    def set_act_status(
        self,
        state: int | None,
        history: int | None = None,
        result: int | None = None,
        runs: int | None = None,
        inference_ms: int | None = None,
    ) -> None:
        """显示UART7 ACTSTAT返回，不依据本地按钮推测运行状态。"""
        names = {0: "OFF", 1: "WARMUP", 2: "RUNNING", 3: "FAULT"}
        colors = {0: "#91a3b5", 1: "#f2cf72", 2: "#65d39a", 3: "#eb7474"}
        result_names = {
            0: "OK", 1: "NOT_READY", 2: "CREATE_FAILED", 3: "RUN_FAILED",
            4: "OUTPUT_INVALID", 5: "BAD_ARGUMENT", 6: "BUSY",
            7: "SENSOR_TIMEOUT", 8: "MOTOR_NOT_READY", 9: "MOTOR_FAULT",
            10: "TX_BUSY", 11: "TX_FAILED",
        }
        if state not in names:
            self.act_state.setText("○ ACT自主 · 未同步")
            self.act_state.setStyleSheet("color:#91a3b5;")
            self.act_detail.setText("历史 --/12 · 结果 --")
            return
        self.act_state.setText(f"● ACT自主 · {names[state]}")
        self.act_state.setStyleSheet(f"color:{colors[state]}; font-weight:700;")
        history_text = "--" if history is None else str(history)
        result_text = (
            "--"
            if result is None
            else f"{result_names.get(result, 'UNKNOWN')}({result})"
        )
        runs_text = "--" if runs is None else str(runs)
        time_text = "--" if inference_ms is None else str(inference_ms)
        self.act_detail.setText(
            f"历史 {history_text}/12 · 结果 {result_text} · 次数 {runs_text} · {time_text}ms"
        )

    def _clear_ai_mapping(self, title: str, color: str) -> None:
        """清除旧推理结果，防止关闭推理后继续展示过期数据。"""

        self.ai_state.setText(title)
        self.ai_state.setStyleSheet(f"color:{color}; font-weight:700;")
        self.ai_direction.setText("X --   Y --   角度 --°")
        self.ai_shape.setText("距离 -- mm   Bend --   Raw --")
        self.ai_dial.set_angle(0.0)
        for key, value in (
            ("x", "—"), ("y", "—"), ("angle", "—°"),
            ("distance", "— mm"), ("bend", "—"), ("raw", "—"),
        ):
            self.ai_values[key].setText(value)

    def set_snapshot(
        self,
        source: str,
        enabled: bool,
        mode: str,
        selected: int,
        online_mask: int,
        ready_mask: int,
        fault_mask: int,
        x: int,
        y: int,
        bend: int,
        stiffness: int,
        last_tx_status: int,
    ) -> None:
        self.snapshot_mode.setText(
            f"来源 {source}\n控制 {'运行' if enabled else '停止'}\n模式 {mode}"
        )
        self.snapshot_device.setText(
            f"对象 T{selected}\n在线 {online_mask.bit_count()}/4\n"
            f"就绪 {ready_mask.bit_count()}/4\n故障 0x{fault_mask:X}"
        )
        self.snapshot_input.setText(
            f"输入 X {x}  Y {y}\nBend {bend}   刚度 {stiffness}\nTX {last_tx_status}"
        )
        self.ai_extra.setText(f"刚度 {stiffness}")
        values = {
            "source": source,
            "control": "运行" if enabled else "停止",
            "mode": mode,
            "object": f"T{selected}",
            "online": f"{online_mask.bit_count()}/4",
            "ready": f"{ready_mask.bit_count()}/4",
            "fault": f"0x{fault_mask:X}",
            "x": str(x),
            "y": str(y),
            "bend": str(bend),
            "stiffness": str(stiffness),
            "tx": str(last_tx_status),
        }
        for key, value in values.items():
            self.system_values[key].setText(value)
        self.ai_values["stiffness"].setText(str(stiffness))

    def set_poses(self, poses: list[tuple[int, tuple[int, int, int], float, float]]) -> None:
        for index, (status, dq, angle, bend) in enumerate(poses[:4]):
            self._poses[index] = (status, dq, angle, bend)
            self._pose_received[index] = True
            flags = []
            flags.append("在线" if status & 0x01 else "离线")
            if status & 0x02:
                flags.append("活动")
            if status & 0x04:
                flags.append("就绪")
            if status & 0x08:
                flags.append("故障")
            labels = self.pose_values[index]
            labels[1].setText(" / ".join(flags))
            labels[1].setStyleSheet(
                "color:#eb7474;" if status & 0x08 or not status & 0x01 else "color:#65d39a;"
            )
            labels[2].setText(str(dq[0]))
            labels[3].setText(str(dq[1]))
            labels[4].setText(str(dq[2]))
            labels[5].setText(f"{angle:.1f}° / {bend:.1f}")
            self._update_tentacle_summary(index)
        self._refresh_pose_detail()

    def _update_tentacle_summary(self, index: int) -> None:
        status, _dq, angle, bend = self._poses[index]
        button = self.tentacle_summary_buttons[index]
        if not self._pose_received[index]:
            state = "offline"
            text = f"T{index + 1}  ○ 未同步   —°   B—"
        elif status & 0x08:
            state = "fault"
            text = f"T{index + 1}  ● 故障   {angle:.0f}°   B{bend:.0f}"
        elif not status & 0x01:
            state = "offline"
            text = f"T{index + 1}  ○ 离线   —°   B—"
        elif not status & 0x04:
            state = "waiting"
            text = f"T{index + 1}  ● 等待   {angle:.0f}°   B{bend:.0f}"
        else:
            state = "ready"
            text = f"T{index + 1}  ● 就绪   {angle:.0f}°   B{bend:.0f}"
        button.setText(text)
        button.setProperty("state", state)
        button.style().unpolish(button)
        button.style().polish(button)

    def show_pose_detail(self, index: int) -> None:
        self._detail_tentacle = index
        self.show_detail_page(0)
        self._refresh_pose_detail()

    def select_pose_detail(self, index: int) -> None:
        self._detail_tentacle = index
        self._refresh_pose_detail()

    def _refresh_pose_detail(self) -> None:
        index = self._detail_tentacle
        for button_index, button in enumerate(getattr(self, "pose_select_buttons", [])):
            button.setChecked(button_index == index)
        if not hasattr(self, "detail_pose_view"):
            return
        status, dq, angle, bend = self._poses[index]
        received = self._pose_received[index]
        self.detail_title.setText(f"T{index + 1} 实际姿态")
        self.detail_pose_view.set_tentacle(index)
        self.detail_pose_view.set_state(
            angle,
            bend,
            bool(status & 0x01) if received else False,
            bool(status & 0x02) if received else False,
        )
        if not received:
            self.detail_pose_status.setText("未同步")
            for label in self.detail_pose_dq:
                label.setText("—")
            self.detail_pose_angle.setText("—°")
            self.detail_pose_bend.setText("—")
            return
        flags = ["在线" if status & 0x01 else "离线"]
        if status & 0x02:
            flags.append("活动")
        if status & 0x04:
            flags.append("就绪")
        if status & 0x08:
            flags.append("故障")
        self.detail_pose_status.setText(" / ".join(flags))
        for label, value in zip(self.detail_pose_dq, dq):
            label.setText(str(value))
        self.detail_pose_angle.setText(f"{angle:.1f}°")
        self.detail_pose_bend.setText(f"{bend:.1f}")

    def show_detail_page(self, page: int) -> None:
        page = max(0, min(2, page))
        self._detail_page = page
        self._detail_open = True
        self.detail_stack.setCurrentIndex(page)
        self.detail_drawer.show()
        self.settings.setValue("receive_detail_page", page)
        self.settings.setValue("receive_detail_open", True)
        self._sync_detail_buttons()
        QTimer.singleShot(0, self.restore_detail_width)

    def close_detail(self) -> None:
        self._remember_detail_width()
        self._detail_open = False
        self.detail_drawer.hide()
        self.settings.setValue("receive_detail_open", False)
        self._sync_detail_buttons()

    def _sync_detail_buttons(self) -> None:
        for index, button in enumerate(self.detail_buttons):
            button.blockSignals(True)
            button.setChecked(self._detail_open and index == self._detail_page)
            button.blockSignals(False)
        titles = ("触手姿态", "AI参数", "系统状态")
        if self._detail_page != 0:
            self.detail_title.setText(titles[self._detail_page])

    def restore_detail_width(self) -> None:
        if not self._detail_open or not self.detail_drawer.isVisible():
            return
        total = max(640, self.detail_splitter.width())
        width = min(self._detail_width, max(360, total - 360))
        self.detail_splitter.setSizes([max(360, total - width), width])

    def _remember_detail_width(self, _position: int = 0, _index: int = 0) -> None:
        if not self.detail_drawer.isVisible():
            return
        sizes = self.detail_splitter.sizes()
        if len(sizes) > 1 and sizes[1] >= 340:
            self._detail_width = max(360, min(640, sizes[1]))
            self.settings.setValue("receive_detail_width", self._detail_width)

    def set_fullscreen_state(self, fullscreen: bool) -> None:
        self.fullscreen_button.setText("退出" if fullscreen else "全屏")
        self.fullscreen_button.setToolTip("退出全屏（F11）" if fullscreen else "全屏显示（F11）")

    def append(self, direction: str, payload: bytes) -> None:
        text = payload.decode("utf-8", errors="replace").replace("\r", "")
        color = {
            "RX": QColor("#7ee2ad"),
            "TX": QColor("#68d7ef"),
            "ERR": QColor("#ff8b8b"),
            "SYS": QColor("#f2cf72"),
        }.get(direction, QColor("#d7e5ee"))
        text_format = QTextCharFormat()
        text_format.setForeground(color)
        text_format.setFontFamily("Consolas")
        text_format.setFontPointSize(10.0)
        cursor = self.text.textCursor()
        cursor.movePosition(QTextCursor.End)
        for line in text.split("\n"):
            if line:
                cursor.insertText(f"{time.strftime('%H:%M:%S')}  {direction:<3}  {line}\n", text_format)
        self.text.setTextCursor(cursor)
        self.text.ensureCursorVisible()

    def set_summary(self, rx: int, tx: int, last: str = "--") -> None:
        self._rx, self._tx, self._last = rx, tx, last
        self.header.setText(f"串口通信   RX {rx:,} B   TX {tx:,} B   最后响应：{last}")
        self.header.setToolTip(f"RX {rx:,} B / TX {tx:,} B / 最后响应：{last}")


class MainWindow(QMainWindow):
    def __init__(self, base_dir: Path) -> None:
        super().__init__()
        self.base_dir = base_dir
        self.setWindowTitle("四触手控制台 · 原型")
        self.resize(1800, 1050)
        self.setMinimumSize(1120, 700)
        self.store = CommandStore(base_dir / "assets" / "default_commands.json")
        self.serial = SerialService(self)
        self.settings = QSettings("TentacleController", "TentacleController")
        self.selected_index = 0
        self.running = False
        self.locked = True
        self.mode_text = "UNKNOWN"
        self.mode_relation = "NONE"
        self.active_indices: list[int] = []
        self.serial_connected = False
        self.h7_online = False
        self.control_source: str | None = None
        self.online_mask = 0
        self.ready_mask = 0
        self.fault_mask = 0
        self.hand_present = False
        self.last_tx_status = 0
        self.gesture_enabled: bool | None = None
        self._pending_gesture: bool | None = None
        self._gesture_disable_after_stop = False
        self.act_state: int | None = None
        self.act_history = 0
        self.act_result = 0
        self.act_runs = 0
        self.act_inference_ms = 0
        self.bend_gain_q = 35000
        self._pending_source: str | None = None
        self._pending_mode_command: str | None = None
        self._rx_line_buffer = ""
        self._rx_decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        self._last_response = "--"
        self._last_log_response = "--"
        self._background_replies: dict[str, float] = {}
        self._manual_replies: dict[str, float] = {}
        self._keep_protocol_response = False
        self._last_stop_reason: str | None = None
        self._last_stop_mask = 0
        self._space_hold_active = False
        self._pending_virtual_command: str | None = None
        self._last_virtual_command: str | None = None
        self._virtual_hold_until = 0.0
        self._virtual_send_timer = QTimer(self)
        self._virtual_send_timer.setInterval(40)
        self._virtual_send_timer.timeout.connect(self._flush_virtual_command)
        self._virtual_send_timer.start()
        self._snapshot_timer = QTimer(self)
        self._snapshot_timer.setInterval(500)
        self._snapshot_timer.timeout.connect(self._poll_snapshot)
        self._act_status_timer = QTimer(self)
        self._act_status_timer.setInterval(500)
        self._act_status_timer.timeout.connect(self._poll_act_status)
        self._pose_timer = QTimer(self)
        self._pose_timer.setInterval(200)
        self._pose_timer.timeout.connect(self._poll_pose)
        self._last_h7_rx_at = 0.0
        self._h7_timeout_s = 1.6
        self._h7_hard_timeout_s = 3.0
        self._h7_reconnect_requested = False
        self._h7_monitor_timer = QTimer(self)
        self._h7_monitor_timer.setInterval(250)
        self._h7_monitor_timer.timeout.connect(self._check_h7_timeout)
        self._mode_apply_timer = QTimer(self)
        self._mode_apply_timer.setSingleShot(True)
        self._mode_apply_timer.setInterval(1500)
        self._mode_apply_timer.timeout.connect(self._mode_request_timeout)
        self._source_request_timer = QTimer(self)
        self._source_request_timer.setSingleShot(True)
        self._source_request_timer.setInterval(1500)
        self._source_request_timer.timeout.connect(self._source_request_timeout)
        self._gesture_request_timer = QTimer(self)
        self._gesture_request_timer.setSingleShot(True)
        self._gesture_request_timer.setInterval(1800)
        self._gesture_request_timer.timeout.connect(self._gesture_request_timeout)
        self._build_ui()
        self._connect_signals()
        self._apply_disconnected_state()
        QApplication.instance().installEventFilter(self)

    def _build_ui(self) -> None:
        central = QWidget(); self.setCentralWidget(central)
        root = QVBoxLayout(central); root.setContentsMargins(8, 8, 8, 8); root.setSpacing(7)
        top_toolbar = QFrame()
        top_toolbar.setObjectName("topToolbar")
        top_toolbar.setFixedHeight(46)
        panel_toolbar = QHBoxLayout(top_toolbar)
        panel_toolbar.setContentsMargins(4, 4, 4, 4)
        panel_toolbar.setSpacing(6)
        self.top_status = QLabel()
        self.top_status.setObjectName("topStatus")
        self.top_status.setTextFormat(Qt.PlainText)
        self.top_status.setWordWrap(False)
        self.top_status.setFixedHeight(36)
        self.top_status.setMinimumWidth(0)
        self.top_status.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.top_status.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.serial_panel_button = QToolButton()
        self.serial_panel_button.setObjectName("panelToggle")
        self.serial_panel_button.setText("串口 / 命令")
        self.serial_panel_button.setCheckable(True)
        self.serial_panel_button.setChecked(True)
        self.mode_panel_button = QToolButton()
        self.mode_panel_button.setObjectName("panelToggle")
        self.mode_panel_button.setText("系统 / 模式")
        self.mode_panel_button.setCheckable(True)
        self.mode_panel_button.setChecked(True)
        self.receive_button = QToolButton()
        self.receive_button.setObjectName("panelToggle")
        self.receive_button.setText("串口接收")
        self.receive_button.setCheckable(True)
        for button in (self.serial_panel_button, self.mode_panel_button, self.receive_button):
            button.setFixedSize(124, 36)
            panel_toolbar.addWidget(button)
        panel_toolbar.addWidget(self.top_status, 1)
        root.addWidget(top_toolbar)
        self.body_splitter = QSplitter(Qt.Horizontal)
        self.body_splitter.setOpaqueResize(False)
        self.body_splitter.setHandleWidth(8)
        self.body_splitter.setChildrenCollapsible(True)
        self.serial_sidebar = SerialSidebar(self.store)
        self.mode_sidebar = ModeSidebar()
        self.touch_container = QWidget(); self.touch_grid = QGridLayout(self.touch_container); self.touch_grid.setSpacing(8)
        self.touch_grid.setRowStretch(0, 1)
        self.touch_grid.setRowStretch(1, 1)
        self.touch_grid.setColumnStretch(0, 1)
        self.touch_grid.setColumnStretch(1, 1)
        self.panels: list[TentaclePanel] = []
        model_path = self.base_dir / "assets" / "model" / "robot.xml"
        for index in range(4):
            panel = TentaclePanel(index, model_path)
            self.panels.append(panel)
            self.touch_grid.addWidget(panel, index // 2, index % 2)
        self.body_splitter.addWidget(self.serial_sidebar); self.body_splitter.addWidget(self.mode_sidebar); self.body_splitter.addWidget(self.touch_container)
        self.body_splitter.setCollapsible(0, True)
        self.body_splitter.setCollapsible(1, True)
        self.body_splitter.setCollapsible(2, False)
        self.body_splitter.setSizes([290, 240, 1270])
        self._panel_last_widths = [290, 240]
        self.serial_panel_button.toggled.connect(lambda visible: self._toggle_splitter_panel(0, visible))
        self.mode_panel_button.toggled.connect(lambda visible: self._toggle_splitter_panel(1, visible))
        self.body_splitter.splitterMoved.connect(lambda _position, _index: self._sync_panel_buttons())
        root.addWidget(self.body_splitter, 1)
        self.receive_button.clicked.connect(self.toggle_receive_window)

        self.receive_drawer = ReceiveDrawer(self.settings)
        self.receive_dock = QDockWidget("串口通信记录", self)
        self.receive_dock.setWidget(self.receive_drawer)
        self.receive_dock.setAllowedAreas(Qt.NoDockWidgetArea)
        self.receive_dock.setFeatures(
            QDockWidget.DockWidgetMovable
            | QDockWidget.DockWidgetFloatable
            | QDockWidget.DockWidgetClosable
        )
        self.receive_dock.setFloating(True)
        self.receive_dock.setMinimumSize(760, 420)
        self.receive_dock.resize(1100, 620)
        self._receive_fullscreen = False
        self._receive_normal_geometry = None
        saved_geometry = self.settings.value("receive_window_geometry")
        self._receive_positioned = bool(saved_geometry)
        if saved_geometry:
            self.receive_dock.restoreGeometry(saved_geometry)
        self.receive_dock.hide()
        self.receive_dock.visibilityChanged.connect(self._receive_visibility_changed)
        self.receive_drawer.fullscreen_requested.connect(self.toggle_receive_fullscreen)
        self.receive_drawer.gesture_requested.connect(self.request_gesture_inference)

    def _toggle_splitter_panel(self, index: int, visible: bool) -> None:
        sizes = self.body_splitter.sizes()
        if visible and sizes[index] == 0:
            restored = self._panel_last_widths[index]
            sizes[index] = restored
            sizes[2] = max(1, sizes[2] - restored)
        elif not visible and sizes[index] > 0:
            self._panel_last_widths[index] = sizes[index]
            sizes[2] += sizes[index]
            sizes[index] = 0
        self.body_splitter.setSizes(sizes)
        self._sync_panel_buttons()

    def _sync_panel_buttons(self) -> None:
        sizes = self.body_splitter.sizes()
        for index, button in enumerate((self.serial_panel_button, self.mode_panel_button)):
            if sizes[index] > 0:
                self._panel_last_widths[index] = sizes[index]
            button.blockSignals(True)
            button.setChecked(sizes[index] > 0)
            button.blockSignals(False)

    def toggle_receive_window(self) -> None:
        if self.receive_dock.isVisible():
            self.receive_dock.hide()
            return
        if not self._receive_positioned:
            width = min(1100, max(760, self.width() - 160))
            height = min(720, max(520, int(self.height() * 0.62)))
            self.receive_dock.resize(width, height)
            position = self.mapToGlobal(QPoint((self.width() - width) // 2, (self.height() - height) // 2))
            self.receive_dock.move(position)
            self._receive_positioned = True
        self.receive_dock.show()
        self.receive_dock.raise_()
        QTimer.singleShot(0, self.receive_drawer.restore_detail_width)

    def toggle_receive_fullscreen(self) -> None:
        if not self.receive_dock.isVisible():
            self.toggle_receive_window()
        if self._receive_fullscreen:
            self.receive_dock.showNormal()
            if self._receive_normal_geometry:
                self.receive_dock.restoreGeometry(self._receive_normal_geometry)
            self._receive_fullscreen = False
        else:
            self._receive_normal_geometry = self.receive_dock.saveGeometry()
            self.receive_dock.showFullScreen()
            self._receive_fullscreen = True
        self.receive_drawer.set_fullscreen_state(self._receive_fullscreen)
        self.receive_dock.raise_()
        QTimer.singleShot(0, self.receive_drawer.restore_detail_width)

    def _receive_visibility_changed(self, visible: bool) -> None:
        self.receive_button.blockSignals(True)
        self.receive_button.setChecked(visible)
        self.receive_button.blockSignals(False)
        if not visible and self._receive_positioned and not self._receive_fullscreen:
            self.settings.setValue("receive_window_geometry", self.receive_dock.saveGeometry())

    def _connect_signals(self) -> None:
        self.serial_sidebar.send_requested.connect(self.send_text)
        self.serial_sidebar.open_requested.connect(self.serial.open)
        self.serial_sidebar.close_requested.connect(self.serial.close)
        self.mode_sidebar.mode_requested.connect(self.apply_mode)
        self.mode_sidebar.source_requested.connect(self.request_control_source)
        self.mode_sidebar.start_requested.connect(self.start_control)
        self.mode_sidebar.stop_requested.connect(self.stop_control)
        self.mode_sidebar.emergency_requested.connect(self.emergency_stop)
        self.mode_sidebar.mode_draft_changed.connect(self._on_mode_draft_changed)
        for panel in self.panels:
            panel.selected.connect(self.select_tentacle)
            panel.control_changed.connect(self.panel_control_changed)
            panel.return_center.connect(self.return_center)
            panel.lock_changed.connect(self.panel_lock_changed)
        self.serial.received.connect(self._on_received)
        self.serial.status_changed.connect(self._on_serial_status_changed)
        self.serial.reconnecting.connect(self._on_serial_reconnecting)
        self.serial.error.connect(lambda message: self.receive_drawer.append("ERR", message.encode("utf-8")))

    def _apply_disconnected_state(self) -> None:
        self.serial_connected = False
        self.h7_online = False
        self.control_source = None
        self.online_mask = 0
        self.ready_mask = 0
        self.fault_mask = 0
        self.hand_present = False
        self.last_tx_status = 0
        self.gesture_enabled = None
        self._pending_gesture = None
        self._gesture_disable_after_stop = False
        self._gesture_request_timer.stop()
        self.receive_drawer.set_gesture_inference_state(None)
        self.receive_drawer.set_control_source(None)
        self.act_state = None
        self.act_history = 0
        self.act_result = 0
        self.act_runs = 0
        self.act_inference_ms = 0
        self._act_status_timer.stop()
        self.receive_drawer.set_act_status(None)
        self._pending_source = None
        self._source_request_timer.stop()
        self._pending_mode_command = None
        self._mode_apply_timer.stop()
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self._rx_line_buffer = ""
        self._rx_decoder.reset()
        self._background_replies.clear()
        self._manual_replies.clear()
        self._keep_protocol_response = False
        self._last_stop_reason = None
        self._last_stop_mask = 0
        self._last_log_response = "--"
        self.running = False
        self.mode_text = "UNKNOWN"
        self.mode_relation = "NONE"
        self.active_indices = []
        self._last_response = "--"
        self._snapshot_timer.stop()
        self._pose_timer.stop()
        self._h7_monitor_timer.stop()
        self._last_h7_rx_at = 0.0
        self._h7_reconnect_requested = False
        for index, panel in enumerate(self.panels):
            panel.set_locked(True)
            panel.lock_button.setEnabled(False)
            panel.set_status(None, False, index == self.selected_index, "未同步")
        self.mode_sidebar.set_control_source(None)
        self.mode_sidebar.set_source_available(False)
        self.mode_sidebar.set_act_mode(False)
        self.mode_sidebar.reset_mode_editor()
        self._refresh_mode_runtime()
        self._render_top_status()

    def _on_serial_status_changed(self, connected: bool, message: str) -> None:
        self.serial_sidebar.set_connected(connected, message)
        if not connected:
            self._apply_disconnected_state()
            return

        self.serial_connected = True
        self.h7_online = False
        self._last_h7_rx_at = time.monotonic()
        self._h7_reconnect_requested = False
        self.control_source = None
        self.online_mask = 0
        self.ready_mask = 0
        self.fault_mask = 0
        self.hand_present = False
        self.last_tx_status = 0
        self.gesture_enabled = None
        self._pending_gesture = None
        self._gesture_disable_after_stop = False
        self._gesture_request_timer.stop()
        self.receive_drawer.set_gesture_inference_state(None)
        self.receive_drawer.set_control_source(None)
        self.act_state = None
        self.act_history = 0
        self.act_result = 0
        self.act_runs = 0
        self.act_inference_ms = 0
        self._act_status_timer.stop()
        self.receive_drawer.set_act_status(None)
        self._pending_source = None
        self._source_request_timer.stop()
        self._pending_mode_command = None
        self._mode_apply_timer.stop()
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self._rx_line_buffer = ""
        self._rx_decoder.reset()
        self._background_replies.clear()
        self._manual_replies.clear()
        self._keep_protocol_response = False
        self._last_stop_reason = None
        self._last_stop_mask = 0
        self._last_log_response = "--"
        self.running = False
        self.mode_sidebar.set_control_source(None)
        self.mode_sidebar.set_source_available(False)
        self.mode_sidebar.set_act_mode(False)
        self.mode_sidebar.reset_mode_editor()
        for index, panel in enumerate(self.panels):
            panel.set_locked(True)
            panel.lock_button.setEnabled(False)
            panel.set_status(None, False, index == self.selected_index, "等待H7")
        self._last_response = "同步中"
        self._refresh_mode_runtime()
        self._render_top_status()
        self._snapshot_timer.start()
        self._pose_timer.stop()
        self._h7_monitor_timer.start()
        # 给USB串口桥和H7留出稳定时间，避免刚打开端口立即堆叠三条查询。
        QTimer.singleShot(200, self._request_device_state)

    def _request_device_state(self) -> None:
        if not self.serial.port.isOpen():
            return
        for command in ("SNAP?", "POS?", "MCFG", "GESTURE?"):
            self.send_text(command, "ASCII", "CRLF", background=True)

    def _poll_snapshot(self) -> None:
        if self.serial.port.isOpen():
            self.send_text("SNAP?", "ASCII", "CRLF", background=True)

    def _poll_pose(self) -> None:
        if self.h7_online and self.serial.port.isOpen():
            self.send_text("POS?", "ASCII", "CRLF", background=True)

    def _poll_act_status(self) -> None:
        """ACT来源下以2Hz读取轻量状态；其他来源不增加UART7负担。"""
        if (
            self.control_source == "ACT"
            and self.h7_online
            and self.serial.port.isOpen()
        ):
            self.send_text("ACTSTAT?", "ASCII", "CRLF", background=True)

    def request_gesture_inference(self, enable: bool) -> None:
        """请求H7切换手势推理；关闭运行中的AI控制时先等待停止确认。"""

        if self.control_source == "ACT":
            self._last_response = "ACT自主模式下不能开启手势推理"
            self.receive_drawer.set_control_source("ACT")
            self._render_top_status()
            return
        if not self.serial_connected or not self.serial.port.isOpen() or not self.h7_online:
            self.receive_drawer.set_gesture_inference_state(
                self.gesture_enabled,
                error="H7未连接",
            )
            self._last_response = "手势推理切换失败：H7未连接"
            self._render_top_status()
            return
        if self._pending_gesture is not None:
            return

        target = bool(enable)
        if self.gesture_enabled is target:
            self._query_gesture_once()
            return
        if not target and self.control_source == "AI" and self.running:
            self._pending_gesture = False
            self._gesture_disable_after_stop = True
            self.receive_drawer.set_gesture_inference_state(
                self.gesture_enabled,
                pending_target=False,
            )
            self._gesture_request_timer.start()
            self._last_response = "正在停止AI控制，随后关闭推理"
            self._render_top_status()
            self.stop_control()
            return
        self._send_gesture_command(target)

    def _send_gesture_command(self, enable: bool) -> None:
        """发送单次GESTURE设置命令，并等待H7确认后再改变已确认状态。"""

        if not self.serial.port.isOpen():
            self._fail_gesture_request("手势推理切换失败：串口已断开")
            return
        self._pending_gesture = bool(enable)
        self._gesture_disable_after_stop = False
        self.receive_drawer.set_gesture_inference_state(
            self.gesture_enabled,
            pending_target=self._pending_gesture,
        )
        self._gesture_request_timer.start()
        self._last_response = "手势推理开启确认中" if enable else "手势推理关闭确认中"
        self._render_top_status()
        self.send_text(f"GESTURE {1 if enable else 0}", "ASCII", "CRLF")

    def _query_gesture_once(self) -> None:
        """后台查询一次真实推理状态，不进行持续轮询。"""

        if self.serial.port.isOpen():
            self.send_text("GESTURE?", "ASCII", "CRLF", background=True)

    def _apply_gesture_state(self, enabled: bool) -> None:
        """应用H7已经确认的推理状态并结束当前请求。"""

        self.gesture_enabled = bool(enabled)
        self._pending_gesture = None
        self._gesture_disable_after_stop = False
        self._gesture_request_timer.stop()
        self.receive_drawer.set_gesture_inference_state(self.gesture_enabled)

    def _continue_gesture_disable_after_stop(self) -> None:
        """收到停止确认后继续发送GESTURE 0，避免运行中直接关闭模型。"""

        if not self._gesture_disable_after_stop or self._pending_gesture is not False:
            return
        self._gesture_disable_after_stop = False
        self._gesture_request_timer.stop()
        QTimer.singleShot(0, lambda: self._send_gesture_command(False))

    def _fail_gesture_request(self, message: str) -> None:
        """结束失败请求并恢复最后一次由H7确认的显示状态。"""

        self._pending_gesture = None
        self._gesture_disable_after_stop = False
        self._gesture_request_timer.stop()
        self.receive_drawer.set_gesture_inference_state(
            self.gesture_enabled,
            error=message.split("：", 1)[-1],
        )
        self._last_response = message
        self._render_top_status()

    def _gesture_request_timeout(self) -> None:
        """GESTURE或前置停止未按时确认时回退显示，并主动查询一次真实状态。"""

        if self._pending_gesture is None:
            return
        self._fail_gesture_request("手势推理切换失败：确认超时")
        self._query_gesture_once()

    def _mark_h7_response(self) -> None:
        """记录H7完整行回复；从超时状态恢复时重新同步全部运行状态。"""
        was_online = self.h7_online
        self._last_h7_rx_at = time.monotonic()
        self.h7_online = True
        self._h7_reconnect_requested = False
        if not was_online and self.serial_connected:
            self._pose_timer.start()
            self._last_response = "H7已响应，正在同步"
            QTimer.singleShot(0, self._request_device_state)
            self._refresh_parameter_interlocks()

    def _check_h7_timeout(self) -> None:
        """短时无回复只告警，持续无回复才销毁串口并后台重建。"""
        if not self.serial_connected or not self.serial.port.isOpen():
            return
        elapsed = time.monotonic() - self._last_h7_rx_at
        if elapsed <= self._h7_timeout_s:
            return
        if self.h7_online:
            self._enter_h7_no_response("H7无响应，等待自动恢复")
        if (
            elapsed >= self._h7_hard_timeout_s
            and not self._h7_reconnect_requested
            and getattr(self.serial, "desired_open", False)
        ):
            self._h7_reconnect_requested = True
            self.serial.force_reconnect("H7持续无响应，正在自动重连")

    def _enter_h7_no_response(self, message: str) -> None:
        """进入H7无响应状态，仅锁住PC输入，不发送控制或停止指令。"""
        self.h7_online = False
        self._pose_timer.stop()
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self._last_response = message
        self.mode_sidebar.set_source_available(False)
        for index, panel in enumerate(self.panels):
            panel.set_status(None, panel.active, index == self.selected_index, "H7无响应")
        self._refresh_parameter_interlocks()
        self._refresh_mode_runtime()
        self._render_top_status()

    def _on_serial_reconnecting(self, message: str) -> None:
        """内部重连不改变用户的开启意图，界面保持现有COM/H7两级显示。"""
        self._enter_h7_no_response(message)

    def _request_snapshot_once(self) -> None:
        if self.serial.port.isOpen():
            QTimer.singleShot(0, lambda: self.send_text("SNAP?", "ASCII", "CRLF", background=True))

    def _on_received(self, payload: bytes) -> None:
        self._rx_line_buffer += self._rx_decoder.decode(payload, final=False)
        normalized = self._rx_line_buffer.replace("\r\n", "\n").replace("\r", "\n")
        lines = normalized.split("\n")
        self._rx_line_buffer = lines.pop()
        if len(self._rx_line_buffer) > 4096:
            self._rx_line_buffer = self._rx_line_buffer[-4096:]

        last_visible_line = ""
        for raw_line in lines:
            line = raw_line.strip()
            if not line:
                continue
            background = self._consume_background_reply(line)
            previous_response = self._last_response
            self._keep_protocol_response = False
            live_status = self._is_live_status_line(line)
            self._handle_serial_line(line)
            parse_error = (
                self._last_response in (
                    "AI 映射格式错误",
                    "SNAP 格式错误",
                    "POS 格式错误",
                    "ACTSTAT 格式错误",
                )
                and self._last_response != previous_response
            )
            if (background or live_status) and not parse_error:
                if not self._keep_protocol_response:
                    self._last_response = previous_response
                self._render_top_status()
                continue
            self.receive_drawer.append("RX", (line + "\r\n").encode("utf-8"))
            last_visible_line = line
        self.serial_sidebar.update_counters(self.serial.rx_bytes, self.serial.tx_bytes)
        self._set_receive_summary(last_visible_line[:40] if last_visible_line else None)

    def _set_receive_summary(self, last: str | None = None) -> None:
        if last is not None:
            self._last_log_response = last
        rx, tx = self.serial.rx_bytes, self.serial.tx_bytes
        self.receive_drawer.set_summary(rx, tx, self._last_log_response)
        self.receive_button.setToolTip(
            f"RX {rx:,} B / TX {tx:,} B / 最后响应：{self._last_log_response}"
        )

    @staticmethod
    def _background_reply_key(command: str) -> str | None:
        return {
            "SNAP?": "SNAP",
            "POS?": "POS",
            "MCFG": "MCFG",
            "GESTURE?": "GESTURE",
            "ACTSTAT?": "ACTSTAT",
        }.get(command.strip().upper())

    @staticmethod
    def _reply_key(line: str) -> str | None:
        if line.startswith("SNAP,"):
            return "SNAP"
        if line.startswith("POS,"):
            return "POS"
        if line.startswith("MCFG ") or line.startswith("OK MCFG "):
            return "MCFG"
        if line.startswith("GESTURE,") or line.startswith("OK GESTURE ") or line == "ERR GESTURE":
            return "GESTURE"
        if line.startswith("ACTSTAT,"):
            return "ACTSTAT"
        return None

    @staticmethod
    def _is_live_status_line(line: str) -> bool:
        if line.startswith("SNAP,"):
            return len(line.split(",")) == 16
        if line.startswith("POS,"):
            return len(line.split(",")) == 18
        if line.startswith("ACTSTAT,"):
            return len(line.split(",")) == 6
        return line.startswith("当前映射")

    def _apply_ai_mapping(self, line: str) -> bool:
        invalid = re.fullmatch(r"当前映射\s*[：:]\s*无效\s+st=(\d+)", line)
        if invalid:
            self.receive_drawer.set_ai_mapping(False, status=int(invalid.group(1)))
            return True
        match = re.fullmatch(
            r"当前映射\s*X\s*[：:]\s*(\-?\d+)\s+Y\s*[：:]\s*(\-?\d+)\s+"
            r"角度\s*[：:]\s*(\-?\d+)deg\s+距离\s*[：:]\s*(\d+)mm\s+"
            r"手.*?程度\s*[：:]\s*(\d+)\s+raw=(\d+)",
            line,
        )
        if not match:
            return False
        x, y, angle, distance, bend, raw = (int(value) for value in match.groups())
        self.receive_drawer.set_ai_mapping(True, x, y, angle, distance, bend, raw)
        return True

    def _consume_background_reply(self, line: str) -> bool:
        now = time.monotonic()
        self._background_replies = {
            key: deadline for key, deadline in self._background_replies.items() if deadline >= now
        }
        self._manual_replies = {
            key: deadline for key, deadline in self._manual_replies.items() if deadline >= now
        }
        key = self._reply_key(line)
        if key is None:
            return False
        if key in self._background_replies:
            del self._background_replies[key]
            return True
        if key in self._manual_replies:
            del self._manual_replies[key]
        return False

    @staticmethod
    def _parse_fields(line: str) -> dict[str, str]:
        return {key: value for key, value in re.findall(r"([A-Za-z_]+)=([^\s]+)", line)}

    @staticmethod
    def _format_stop_notice(reason: str, mask: int) -> str:
        reason_text = {
            "CAN_TIMEOUT": "CAN状态超时",
            "FAULT": "触手故障",
            "RESTORE_LOST": "坐标状态丢失",
            "ESTOP": "紧急停止",
        }.get(reason, reason)
        tentacles = "/".join(
            f"T{index + 1}" for index in range(4) if mask & (1 << index)
        )
        return f"{reason_text}（{tentacles}）" if tentacles else reason_text

    def _handle_serial_line(self, line: str) -> None:
        self._mark_h7_response()

        gesture_state = re.fullmatch(r"GESTURE,([01])", line)
        if gesture_state:
            self._apply_gesture_state(bool(int(gesture_state.group(1))))
            self._last_response = "手势推理状态已同步"
            self._render_top_status()
            return

        gesture_ok = re.fullmatch(r"OK GESTURE ([01])", line)
        if gesture_ok:
            self._apply_gesture_state(bool(int(gesture_ok.group(1))))
            self._last_response = "手势推理已开启" if self.gesture_enabled else "手势推理已关闭"
            self._keep_protocol_response = True
            QTimer.singleShot(50, self._query_gesture_once)
            self._render_top_status()
            return

        if line == "ERR GESTURE":
            self._fail_gesture_request("手势推理操作失败：H7拒绝命令")
            self._keep_protocol_response = True
            return

        if line.startswith("当前映射"):
            if not self._apply_ai_mapping(line):
                self._last_response = "AI 映射格式错误"
            self._render_top_status()
            return

        if line.startswith("ACTSTAT,"):
            if not self._apply_act_status(line):
                self._last_response = "ACTSTAT 格式错误"
                self._keep_protocol_response = True
            self._render_top_status()
            return

        if line.startswith("SNAP,"):
            if not self._apply_snapshot(line):
                self._last_response = "SNAP 格式错误"
            self._render_top_status()
            return

        if line.startswith("POS,"):
            if not self._apply_pose(line):
                self._last_response = "POS 格式错误"
            self._render_top_status()
            return

        stop_notice = re.fullmatch(
            r"MSTOP reason=([A-Z_]+) mask=(0x[0-9A-Fa-f]+|\d+)",
            line,
        )
        if stop_notice:
            reason = stop_notice.group(1)
            mask = int(stop_notice.group(2), 0)
            self._pending_virtual_command = None
            self._last_virtual_command = None
            self._virtual_hold_until = 0.0
            self._last_stop_reason = reason
            self._last_stop_mask = mask
            self._apply_running_state(False, send_on_start=False)
            self._continue_gesture_disable_after_stop()
            self._last_response = f"安全停止：{self._format_stop_notice(reason, mask)}"
            self._keep_protocol_response = True
            self._request_snapshot_once()
            self._render_top_status()
            return

        act_control_error = re.fullmatch(r"ERR MCTRL ACT result=(\d+) state=(\d+)", line)
        if act_control_error:
            result, state = (int(value) for value in act_control_error.groups())
            self.act_result = result
            self.act_state = state if state in (0, 1, 2, 3) else None
            self.receive_drawer.set_act_status(
                self.act_state,
                self.act_history,
                self.act_result,
                self.act_runs,
                self.act_inference_ms,
            )
            self._apply_running_state(self.act_state in (1, 2), send_on_start=False)
            reasons = {
                1: "ACT模型尚未就绪",
                2: "ACT模型创建失败",
                3: "ACT模型推理失败",
                4: "ACT模型输出无效",
                5: "ACT启动参数错误",
                6: "ACT当前状态忙，请先停止",
                7: "ACT的VL53数据已超时",
                8: "ACT的T3尚未ONLINE/READY",
                9: "ACT检测到T3故障",
                10: "ACT的CAN发送暂时繁忙",
                11: "ACT目标或STOP发送失败",
            }
            self._last_response = reasons.get(result, f"ACT控制失败：result={result}")
            self._keep_protocol_response = True
            self._request_snapshot_once()
            self._render_top_status()
            return

        if line == "ERR MCTRL ACT_STREAM_BUSY":
            self._last_response = "ACT采集/示教正在占用控制，请先停止USART1任务"
            self._keep_protocol_response = True
            self._render_top_status()
            return

        mctrl_error = re.fullmatch(
            r"ERR MCTRL (NOT_READY|FAULT|TX_BUSY) online=(0x[0-9A-Fa-f]+) "
            r"ready=(0x[0-9A-Fa-f]+) fault=(0x[0-9A-Fa-f]+)",
            line,
        )
        if mctrl_error:
            online_mask, ready_mask, fault_mask = (
                int(value, 0) for value in mctrl_error.groups()[1:]
            )
            self._pending_virtual_command = None
            self._apply_running_state(False, send_on_start=False)
            if self._gesture_disable_after_stop:
                self._fail_gesture_request("关闭推理失败：H7未能确认停止控制")
            self._apply_device_masks(online_mask, ready_mask, fault_mask)
            reason = self._active_device_block_reason()
            if mctrl_error.group(1) == "FAULT" and fault_mask:
                fault_names = "/".join(
                    f"T{index + 1}" for index in range(4) if fault_mask & (1 << index)
                )
                reason = f"{fault_names} 存在故障"
            if reason is None:
                reason = {
                    "FAULT": "H7报告活动触手故障",
                    "TX_BUSY": "H7发送队列繁忙",
                }.get(mctrl_error.group(1), "H7拒绝启动")
            self._last_response = f"启动失败：{reason}"
            self._request_snapshot_once()
            self._render_top_status()
            return

        if line == "ERR MVIRT SOURCE":
            self._pending_virtual_command = None
            self._apply_running_state(False, send_on_start=False)
            self._last_response = "目标拒绝：控制来源已变化，正在同步"
            self._request_snapshot_once()
            self._render_top_status()
            return

        if line == "ERR MVIRT NOT_READY":
            self._last_response = "H7拒绝本次目标，正在同步运行状态"
            self._request_snapshot_once()
            self._render_top_status()
            return

        if line == "ERR MVIRT TX_BUSY":
            self._virtual_hold_until = max(
                self._virtual_hold_until,
                time.monotonic() + 0.08,
            )
            if self._pending_virtual_command is None:
                self._pending_virtual_command = self._last_virtual_command
            self._last_response = "CAN发送忙：已保留最新目标"
            self._render_top_status()
            return

        if line == "ERR MVIRT FAULT":
            self._pending_virtual_command = None
            self._last_virtual_command = None
            self._virtual_hold_until = 0.0
            self._apply_running_state(False, send_on_start=False)
            self._last_response = "H7报告FAULT，等待停止通知"
            self._request_snapshot_once()
            self._render_top_status()
            return

        if line == "ERR MVIRT DISABLED":
            self._pending_virtual_command = None
            self._apply_running_state(False, send_on_start=False)
            self._last_response = "目标拒绝：H7控制未启动"
            self._request_snapshot_once()
            self._render_top_status()
            return

        virtual_error = re.fullmatch(r"ERR MVIRT status=(\d+)", line)
        if virtual_error:
            status = int(virtual_error.group(1))
            if status == 2:
                self._virtual_hold_until = max(
                    self._virtual_hold_until,
                    time.monotonic() + 0.08,
                )
                if self._pending_virtual_command is None:
                    self._pending_virtual_command = self._last_virtual_command
                self._last_response = "CAN发送忙：已保留最新目标"
            else:
                self._last_response = f"目标发送失败：状态{status}，等待H7状态确认"
                self._request_snapshot_once()
            self._render_top_status()
            return

        if line.startswith("OK MVIRT"):
            self._virtual_hold_until = 0.0
            self._last_response = "OK MVIRT"
            self._render_top_status()
            return

        source_match = re.fullmatch(r"MSRC,(AI|PC|ACT),([01])", line)
        if source_match:
            self._apply_control_source(source_match.group(1), bool(int(source_match.group(2))))
            self._last_response = f"MSRC {source_match.group(1)}"

        source_ok = re.match(r"OK MSRC (AI|PC|ACT) enable=([01])", line)
        if source_ok:
            self._apply_control_source(source_ok.group(1), bool(int(source_ok.group(2))))
            self._request_snapshot_once()
        elif line.startswith("ERR MSRC"):
            self._pending_source = None
            self._source_request_timer.stop()
            self.mode_sidebar.set_control_source(self.control_source)
            self.mode_sidebar.set_source_available(self.serial_connected)

        control_match = re.fullmatch(r"MCTRL,([01])", line)
        if control_match:
            enabled = bool(int(control_match.group(1)))
            if enabled:
                self._last_stop_reason = None
                self._last_stop_mask = 0
            self._apply_running_state(enabled)
            self._apply_act_control_hint(enabled)
            if not enabled:
                self._continue_gesture_disable_after_stop()
            self._last_response = f"MCTRL {control_match.group(1)}"

        control_ok = re.match(r"OK MCTRL ([01])(?:\s|$)", line)
        if control_ok:
            enabled = bool(int(control_ok.group(1)))
            if enabled:
                self._last_stop_reason = None
                self._last_stop_mask = 0
            self._apply_running_state(enabled)
            self._apply_act_control_hint(enabled)
            if not enabled:
                self._continue_gesture_disable_after_stop()
            self._request_snapshot_once()

        if line.startswith("ERR MCTRL"):
            if self._gesture_disable_after_stop:
                self._fail_gesture_request("关闭推理失败：H7未能确认停止控制")
                return

        if line.startswith("MWORK ") or line.startswith("OK MWORK "):
            pending_before = self._pending_mode_command is not None
            applied_mode = self._apply_work_fields(self._parse_fields(line))
            if applied_mode is not None:
                self._resolve_mode_request(applied_mode)
            if not line.startswith("OK"):
                self._last_response = "MWORK 已同步"
            else:
                self._apply_running_state(False, send_on_start=False)
                if not pending_before:
                    self._last_response = "OK MWORK"
                elif self._pending_mode_command is not None:
                    self._last_response = "模式确认不匹配，等待状态同步"
                self._request_snapshot_once()
            self._render_top_status()
            return

        if line.startswith("ERR MWORK"):
            self._fail_mode_request("模式切换失败：H7拒绝请求")
            return

        tsel_match = re.match(r"OK TSEL tentacle=([1-4])", line)
        if tsel_match:
            self.selected_index = int(tsel_match.group(1)) - 1
            self._refresh_panel_selection()
            self.send_text("TSTAT", "ASCII", "CRLF")

        if line.startswith("TSTAT "):
            self._apply_tstat_fields(self._parse_fields(line))
            self._last_response = "TSTAT 已同步"

        if line.startswith("MCFG ") or line.startswith("OK MCFG "):
            fields = self._parse_fields(line)
            try:
                bend_gain_q = int(fields["bend"])
            except (KeyError, ValueError):
                pass
            else:
                if bend_gain_q > 0:
                    self.bend_gain_q = bend_gain_q

        if line.startswith("OK CANSTOP"):
            self._apply_running_state(False)
            self._request_snapshot_once()

        if line.startswith("OK CANHOME"):
            self._pending_virtual_command = None
            self._last_virtual_command = None
            self._reset_active_control_values()
            self._last_response = "HOME目标已归零，等待实际姿态返回"
            self._keep_protocol_response = True
            self._request_snapshot_once()
            QTimer.singleShot(
                0,
                lambda: self.send_text("POS?", "ASCII", "CRLF", background=True),
            )
            self._render_top_status()
            return

        if line.startswith("OK"):
            words = line.split()
            self._last_response = " ".join(words[:2])
        elif line.startswith("ERR"):
            words = line.split()
            self._last_response = " ".join(words[:3])
        self._render_top_status()

    def _apply_act_control_hint(self, enabled: bool) -> None:
        """用MCTRL确认立即更新ACT启停提示，精确状态随后由ACTSTAT覆盖。"""
        if self.control_source != "ACT":
            return
        if enabled:
            if self.act_state not in (1, 2):
                self.act_state = 1
        else:
            self.act_state = 0
        self.receive_drawer.set_act_status(
            self.act_state,
            self.act_history,
            self.act_result,
            self.act_runs,
            self.act_inference_ms,
        )
        self._refresh_mode_runtime()

    def _apply_act_status(self, line: str) -> bool:
        """解析ACTSTAT,state,history,result,runs,time_ms并刷新真实运行状态。"""
        parts = line.split(",")
        if len(parts) != 6 or parts[0] != "ACTSTAT":
            return False
        try:
            state, history, result, runs, inference_ms = (
                int(value) for value in parts[1:]
            )
        except ValueError:
            return False
        if (
            state not in (0, 1, 2, 3)
            or not 0 <= history <= 12
            or not 0 <= result <= 11
            or runs < 0
            or inference_ms < 0
        ):
            return False
        self.act_state = state
        self.act_history = history
        self.act_result = result
        self.act_runs = runs
        self.act_inference_ms = inference_ms
        self.receive_drawer.set_act_status(
            state, history, result, runs, inference_ms
        )
        if self.control_source == "ACT":
            self._apply_running_state(state in (1, 2), send_on_start=False)
        return True

    def _apply_snapshot(self, line: str) -> bool:
        parts = line.split(",")
        if len(parts) != 16 or parts[0] != "SNAP":
            return False
        try:
            version = int(parts[1])
            source_value = int(parts[2])
            enabled = int(parts[3])
            active_mask = int(parts[4])
            relation = int(parts[5])
            selected = int(parts[6])
            online_mask = int(parts[7])
            ready_mask = int(parts[8])
            fault_mask = int(parts[9])
            hand_present = int(parts[10])
            x = int(parts[11])
            y = int(parts[12])
            bend = int(parts[13])
            stiffness = int(parts[14])
            last_tx_status = int(parts[15])
        except ValueError:
            return False

        if (
            version != 1
            or source_value not in (0, 1, 2)
            or enabled not in (0, 1)
            or active_mask not in (1, 2, 4, 8, 3, 12, 15)
            or relation not in (0, 1, 2, 3)
            or not 1 <= selected <= 4
            or any(mask < 0 or mask > 0x0F for mask in (online_mask, ready_mask, fault_mask))
            or hand_present not in (0, 1)
            or not -100 <= x <= 100
            or not -100 <= y <= 100
            or not 0 <= bend <= 100
            or not 0 <= stiffness <= 100
        ):
            return False

        mode = self._mode_from_snapshot(active_mask, relation, selected)
        if mode is None:
            return False

        source_name = {0: "AI", 1: "PC", 2: "ACT"}[source_value]
        if source_name == "ACT":
            # ACT策略启动时由H7固定选择T3；启动前也按这一安全目标显示和校验。
            mode = "SOLO 3"
            selected = 3

        previous_source = self.control_source
        self.hand_present = bool(hand_present)
        self.last_tx_status = last_tx_status
        self.set_mode_state(mode)
        self._resolve_mode_request(mode)
        self.selected_index = selected - 1
        self.receive_drawer.set_snapshot(
            source_name,
            bool(enabled),
            self._display_mode(mode),
            selected,
            online_mask,
            ready_mask,
            fault_mask,
            x,
            y,
            bend,
            stiffness,
            last_tx_status,
        )

        self._apply_device_masks(online_mask, ready_mask, fault_mask)
        reported_enabled = bool(enabled)
        was_running = self.running

        if self._pending_source is not None and source_name != self._pending_source:
            # 串口中可能还残留切换前的SNAP；不能用旧帧取消正在等待的来源请求。
            self._apply_running_state(reported_enabled, send_on_start=False)
            self.mode_sidebar.set_control_source(self.control_source, pending=True)
        else:
            self._apply_control_source(
                source_name,
                reported_enabled,
                send_on_start=False,
            )
        if not reported_enabled:
            self._continue_gesture_disable_after_stop()
        if was_running and not reported_enabled and self._last_stop_reason is None:
            self._last_response = "H7已停止（未收到停止原因）"
            self._keep_protocol_response = True
        # AI输入由H7持续刷新；PC输入只在首次连接或切换来源时同步一次。
        # 后续SNAP仅更新状态，不能覆盖用户正在编辑的本地目标参数。
        if source_value == 0 or (source_value == 1 and previous_source != "PC"):
            angle = math.degrees(math.atan2(y, x)) % 360 if abs(x) + abs(y) > 0.1 else 0.0
            self._apply_linked_values(self.active_indices[0], angle, bend, stiffness)
        self._refresh_panel_selection()
        self._refresh_mode_runtime()
        return True

    def _apply_device_masks(
        self,
        online_mask: int,
        ready_mask: int,
        fault_mask: int,
    ) -> None:
        """Apply H7 device masks without turning a persistent state into an event."""
        self.online_mask = online_mask
        self.ready_mask = ready_mask
        self.fault_mask = fault_mask
        active = set(self.active_indices)
        for index, panel in enumerate(self.panels):
            bit = 1 << index
            online = bool(online_mask & bit)
            ready = bool(ready_mask & bit)
            fault = bool(fault_mask & bit)
            feedback = (
                "状态已同步"
                if panel.feedback in ("未同步", "等待H7", "--")
                else panel.feedback
            )
            panel.set_status(online, index in active, index == self.selected_index, feedback)
            panel.set_device_summary(online, ready, fault)
        self._refresh_mode_runtime()

    def _apply_pose(self, line: str) -> bool:
        parts = line.split(",")
        if len(parts) != 18 or parts[0] != "POS":
            return False
        try:
            version = int(parts[1])
            values = [int(value) for value in parts[2:]]
        except ValueError:
            return False
        if version != 1:
            return False

        poses: list[tuple[int, tuple[int, int, int], float, float]] = []
        for index in range(4):
            offset = index * 4
            status = values[offset]
            dq = (values[offset + 1], values[offset + 2], values[offset + 3])
            if status < 0 or status > 0x0F:
                return False
            angle, bend = self._pose_from_dq(dq)
            poses.append((status, dq, angle, bend))
            panel = self.panels[index]
            panel.active = bool(status & 0x02)
            panel.online = bool(status & 0x01)
            panel.set_actual_pose(status, dq, angle, bend)
            panel.set_status(panel.online, panel.active, index == self.selected_index, panel.feedback)
        self.receive_drawer.set_poses(poses)
        self._refresh_mode_runtime()
        return True

    @staticmethod
    def _display_mode(mode: str) -> str:
        parts = mode.split()
        if not parts:
            return "--"
        if parts[0] == "SOLO":
            return f"单触手 T{parts[1]}"
        if parts[0] == "DUAL":
            relation = "镜像" if "MIRROR" in parts else "同向"
            return f"双触手 T{parts[1]} {relation}"
        relation = "中心对称" if "CENTER" in parts else "同向"
        return f"四触手 {relation}"

    def _pose_from_dq(self, dq: tuple[int, int, int]) -> tuple[float, float]:
        # H7 pull mapping uses these normalized cable directions.
        cable_x = (-26, -71, 97)
        cable_y = (97, -71, -26)
        mean_q = sum(dq) / 3.0
        differential = tuple(value - mean_q for value in dq)
        vector_x = sum(value * axis for value, axis in zip(differential, cable_x))
        vector_y = sum(value * axis for value, axis in zip(differential, cable_y))
        if abs(vector_x) + abs(vector_y) < 1e-6:
            angle = 0.0
        else:
            angle = math.degrees(math.atan2(vector_y, vector_x)) % 360.0
        amplitude_q = math.hypot(vector_x, vector_y) / 15000.0
        bend = max(0.0, min(100.0, amplitude_q * 10000.0 / max(1.0, float(self.bend_gain_q))))
        return angle, bend

    @staticmethod
    def _mode_from_snapshot(active_mask: int, relation: int, selected: int) -> str | None:
        if active_mask in (1, 2, 4, 8):
            expected_selected = active_mask.bit_length()
            return f"SOLO {expected_selected}" if relation == 0 else None
        if active_mask == 0x03 and relation in (1, 2):
            return f"DUAL 12 {'MIRROR' if relation == 2 else 'SAME'}"
        if active_mask == 0x0C and relation in (1, 2):
            return f"DUAL 34 {'MIRROR' if relation == 2 else 'SAME'}"
        if active_mask == 0x0F and relation in (1, 3):
            return f"QUAD {'CENTER' if relation == 3 else 'SAME'}"
        return None

    def _apply_control_source(self, source: str, enabled: bool, send_on_start: bool = True) -> None:
        source_changed = self.control_source != source
        previous_source = self.control_source
        self.control_source = source
        if self._pending_source is None or self._pending_source == source:
            self._pending_source = None
            self._source_request_timer.stop()
        if source_changed or not enabled:
            self._pending_virtual_command = None
            self._last_virtual_command = None
        self.mode_sidebar.set_control_source(source)
        self.mode_sidebar.set_source_available(self.serial_connected and self.h7_online)
        self.receive_drawer.set_control_source(source)
        if source_changed:
            if source == "ACT" and self.mode_text != "SOLO 3":
                self.set_mode_state("SOLO 3")
                self.selected_index = 2
                self._refresh_panel_selection()
            self.mode_sidebar.set_act_mode(source == "ACT")
            if source == "ACT":
                self._act_status_timer.start()
                QTimer.singleShot(0, self._poll_act_status)
            else:
                self._act_status_timer.stop()
                if previous_source == "ACT":
                    self.act_state = 0
                    self.receive_drawer.set_act_status(
                        0,
                        self.act_history,
                        self.act_result,
                        self.act_runs,
                        self.act_inference_ms,
                    )
        self._refresh_parameter_interlocks()
        self._apply_running_state(enabled, send_on_start=send_on_start)

    def _apply_running_state(self, running: bool, send_on_start: bool = True) -> None:
        self.running = running
        if not running:
            self._pending_virtual_command = None
            self._last_virtual_command = None
            self._virtual_hold_until = 0.0
        self._refresh_mode_runtime()

    def _apply_work_fields(self, fields: dict[str, str]) -> str | None:
        """解析H7工作模式字段并返回规范化模式字符串，解析失败返回None。"""
        try:
            mask = int(fields["mask"], 0)
            mirror = int(fields.get("mirror", "0")) != 0
            selected = int(fields.get("TSEL", str(self.selected_index + 1)))
        except (KeyError, ValueError):
            return None

        if mask in (1, 2, 4, 8):
            selected = (mask.bit_length() - 1) + 1
            mode = f"SOLO {selected}"
        elif mask == 0x03:
            mode = f"DUAL 12 {'MIRROR' if mirror else 'SAME'}"
        elif mask == 0x0C:
            mode = f"DUAL 34 {'MIRROR' if mirror else 'SAME'}"
        elif mask == 0x0F:
            mode = f"QUAD {'CENTER' if mirror else 'SAME'}"
        else:
            return None
        self.set_mode_state(mode)
        self.selected_index = max(0, min(3, selected - 1))
        self._refresh_panel_selection()
        return mode

    def _apply_tstat_fields(self, fields: dict[str, str]) -> None:
        try:
            index = int(fields["tentacle"]) - 1
            online = int(fields["online"]) != 0
        except (KeyError, ValueError):
            return
        if index < 0 or index >= len(self.panels):
            return
        line_q = fields.get("line", "--")
        panel = self.panels[index]
        panel.set_status(online, panel.active, index == self.selected_index, line_q)
        self._refresh_mode_runtime()

    def _render_top_status(self) -> None:
        com = "COM已打开" if self.serial_connected else "COM未连接"
        h7 = "在线" if self.h7_online else ("无响应" if self.serial_connected else "--")
        online_states = [panel.online for panel in self.panels]
        can = f"{sum(state is True for state in online_states)}/4" if all(state is not None for state in online_states) else "--/4"
        source = self.control_source or "--"
        if source == "ACT":
            source = f"ACT/{ {0: 'OFF', 1: 'WARMUP', 2: 'RUNNING', 3: 'FAULT'}.get(self.act_state, '--') }"
        motion = "状态未知" if self.serial_connected and not self.h7_online else ("运行中" if self.running else "已停止")
        self.top_status.setText(f"{com}   H7：{h7}   CAN：{can}   来源：{source}   {motion}   {self._last_response}")

    def send_text(
        self,
        text: str,
        encoding: str,
        ending: str,
        *,
        background: bool = False,
        control_validated: bool = False,
    ) -> None:
        if text.strip().upper() == "MCTRL 1" and not control_validated:
            reason = self._start_block_reason()
            if reason:
                self._last_response = reason
                self.receive_drawer.append("ERR", (reason + "\r\n").encode("utf-8"))
                self._render_top_status()
                return
        payload = self._encode(text, encoding, ending)
        if not payload:
            return
        if not self.serial.port.isOpen():
            self.receive_drawer.append("ERR", "串口未连接\r\n".encode("utf-8"))
            self._last_response = "串口未连接"
            self._render_top_status()
            return
        reply_key = self._background_reply_key(text)
        now = time.monotonic()
        if background and reply_key is not None:
            if self._background_replies.get(reply_key, 0.0) >= now:
                return
            if self._manual_replies.get(reply_key, 0.0) >= now:
                return
        elif reply_key is not None:
            self._background_replies.pop(reply_key, None)
        if not self.serial.write(payload):
            return
        if background and reply_key is not None:
            self._background_replies[reply_key] = now + 1.0
        else:
            if reply_key is not None:
                self._manual_replies[reply_key] = now + 1.0
            self.receive_drawer.append("TX", payload)
        self.serial_sidebar.update_counters(self.serial.rx_bytes, self.serial.tx_bytes)
        self._set_receive_summary(None if background else "等待响应")

    @staticmethod
    def _encode(text: str, encoding: str, ending: str) -> bytes:
        if encoding == "HEX":
            try: payload = bytes.fromhex(text)
            except ValueError: return b""
        else:
            payload = text.encode("utf-8")
        suffix = {"NONE": b"", "CR": b"\r", "LF": b"\n", "CRLF": b"\r\n"}.get(ending, b"\r\n")
        if not payload.endswith(suffix):
            payload += suffix
        return payload


    def set_mode_state(self, mode_text: str) -> None:
        text = mode_text.upper()
        if text.startswith("SOLO"):
            selected = int(text.split()[-1]) - 1 if text.split()[-1].isdigit() else 0
            active = {selected}; link = "无"; relation = "NONE"
        elif text.startswith("DUAL"):
            group = text.split()[1] if len(text.split()) > 1 else "12"
            active = {0, 1} if group == "12" else {2, 3}
            relation = "MIRROR" if "MIRROR" in text else "SAME"
            link = "镜像" if relation == "MIRROR" else "同向"
        else:
            active = {0, 1, 2, 3}
            relation = "CENTER" if "CENTER" in text else "SAME"
            link = "中心对称" if relation == "CENTER" else "同向"
        self.mode_text = mode_text
        self.mode_relation = relation
        self.active_indices = sorted(active)
        self.mode_sidebar.set_mode_selection(mode_text)
        if self.selected_index not in active:
            self.selected_index = self.active_indices[0]
        for index, panel in enumerate(self.panels):
            panel.active = index in active
            panel.set_link(link if index in active else "无")
            panel.set_status(panel.online, index in active, index == self.selected_index, panel.feedback)
        self._refresh_mode_runtime()
        self._render_top_status()

    def _refresh_mode_runtime(self) -> None:
        self.mode_sidebar.set_runtime_state(
            self.mode_text,
            set(self.active_indices),
            [panel.online for panel in self.panels],
            self.running,
        )
        if self.control_source == "ACT":
            if self.act_state == 1:
                self.mode_sidebar.start_button.setEnabled(False)
                self.mode_sidebar.start_button.setText("● ACT预热中")
                self.mode_sidebar.stop_button.setEnabled(True)
                self.mode_sidebar.stop_button.setText("停止ACT")
            elif self.act_state == 2:
                self.mode_sidebar.start_button.setEnabled(False)
                self.mode_sidebar.start_button.setText("● ACT运行中")
                self.mode_sidebar.stop_button.setEnabled(True)
                self.mode_sidebar.stop_button.setText("停止ACT")
            elif self.act_state == 3:
                self.mode_sidebar.start_button.setEnabled(False)
                self.mode_sidebar.start_button.setText("ACT故障")
                self.mode_sidebar.stop_button.setEnabled(True)
                self.mode_sidebar.stop_button.setText("清除ACT状态")
            else:
                self.mode_sidebar.stop_button.setText("停止")
        if not self.running:
            reason = self._start_block_reason()
            self.mode_sidebar.start_button.setEnabled(reason is None)
            self.mode_sidebar.start_button.setToolTip(reason or "长按 800ms 启动当前工作组")
            if reason is None:
                self.mode_sidebar.start_button.setText("长按启动")
            elif "串口" in reason:
                self.mode_sidebar.start_button.setText("串口未连接")
            elif "控制来源" in reason:
                self.mode_sidebar.start_button.setText("等待来源同步")
            elif "CAN状态" in reason:
                self.mode_sidebar.start_button.setText("等待CAN在线")
            elif "RESTORE" in reason:
                self.mode_sidebar.start_button.setText("等待RESTORE")
            elif "故障" in reason:
                self.mode_sidebar.start_button.setText("存在故障")
            else:
                self.mode_sidebar.start_button.setText("暂不可启动")

    def _start_block_reason(self) -> str | None:
        if not self.serial_connected:
            return "无法启动：串口未连接"
        if not self.h7_online:
            return "无法启动：H7无响应"
        if self.control_source not in ("AI", "PC", "ACT"):
            return "无法启动：控制来源尚未同步"
        if self.control_source == "ACT":
            if self.act_state is None:
                return "无法启动：ACT状态尚未同步"
            if self.act_state == 3:
                return "无法启动：ACT故障，请先停止以清除状态"
            if self.act_state in (1, 2):
                return "ACT已经启动"
        if not self.active_indices:
            return "无法启动：未选择活动触手"
        if self._pending_source is not None or self._pending_mode_command is not None:
            return "无法启动：来源或模式尚未确认"
        if self.mode_sidebar.has_unapplied_mode():
            return "无法启动：请先应用当前模式选择"
        return None

    def _parameter_edit_block_reason(self) -> str | None:
        """返回参数解锁阻止原因，防止草稿模式与H7实际模式不一致。"""
        if not self.serial_connected:
            return "串口未连接"
        if not self.h7_online:
            return "H7无响应，参数保持锁定"
        if self.control_source != "PC":
            if self.control_source == "AI":
                return "AI控制中，参数不可解锁"
            if self.control_source == "ACT":
                return "ACT自主控制固定使用T3，参数不可解锁"
            return "控制来源未同步"
        if self.mode_sidebar.has_unapplied_mode() or self._pending_mode_command is not None:
            return "模式尚未应用确认，参数保持锁定"
        return None

    def _refresh_parameter_interlocks(self) -> None:
        """按通讯、来源和模式确认状态统一刷新参数锁，避免绕过模式确认。"""
        editable = self._parameter_edit_block_reason() is None
        for panel in self.panels:
            if not editable and not panel.locked:
                panel.set_locked(True)
            panel.lock_button.setEnabled(editable)

    def _on_mode_draft_changed(self, _dirty: bool) -> None:
        """模式草稿变化后丢弃未发送目标并立即刷新启动与参数互锁。"""
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self._refresh_parameter_interlocks()
        self._refresh_mode_runtime()

    def _active_device_block_reason(self) -> str | None:
        """Return the current active-group blocker without causing control actions."""
        faults = [index + 1 for index in self.active_indices if self.fault_mask & (1 << index)]
        if faults:
            return "/".join(f"T{index}" for index in faults) + " 存在故障"
        offline = [index + 1 for index in self.active_indices if not self.online_mask & (1 << index)]
        if offline:
            return "/".join(f"T{index}" for index in offline) + " CAN状态超时"
        unready = [index + 1 for index in self.active_indices if not self.ready_mask & (1 << index)]
        if unready:
            return "/".join(f"T{index}" for index in unready) + " 尚未完成RESTORE"
        return None

    def _refresh_panel_selection(self) -> None:
        for index, panel in enumerate(self.panels):
            panel.set_status(panel.online, panel.active, index == self.selected_index, panel.feedback)
        self._render_top_status()

    def request_control_source(self, source: str) -> None:
        if not self.serial_connected or not self.serial.port.isOpen():
            self._last_response = "串口未连接"
            self._render_top_status()
            return
        normalized = source.upper()
        if normalized not in ("AI", "PC", "ACT") or normalized == self.control_source:
            self.mode_sidebar.set_control_source(self.control_source)
            return
        self._pending_source = normalized
        self._source_request_timer.start()
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self.mode_sidebar.set_control_source(self.control_source, pending=True)
        self._last_response = "来源切换中"
        self._render_top_status()
        self.send_text(f"MSRC {normalized}", "ASCII", "CRLF")

    def _source_request_timeout(self) -> None:
        """来源切换未确认时回到H7最后状态，禁止界面假切换。"""
        if self._pending_source is None:
            return
        requested = self._pending_source
        self._pending_source = None
        self.mode_sidebar.set_control_source(self.control_source)
        self.mode_sidebar.set_source_available(self.serial_connected and self.h7_online)
        self._last_response = f"来源切换到{requested}失败：H7确认超时"
        self._request_snapshot_once()
        self._refresh_mode_runtime()
        self._render_top_status()

    def apply_mode(self, mode: str) -> None:
        if self.control_source == "ACT":
            self._last_response = "ACT固定使用单触手T3，无需应用工作组合"
            self._render_top_status()
            return
        if self.running:
            self.receive_drawer.append("ERR", "请先停止控制再切换模式\r\n".encode("utf-8"))
            return
        if not self.serial_connected or not self.serial.port.isOpen():
            self._last_response = "模式切换失败：串口未连接"
            self._render_top_status()
            return
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self._pending_mode_command = " ".join(mode.upper().split())
        self.mode_sidebar.set_mode_request_pending(True)
        self._refresh_parameter_interlocks()
        self._mode_apply_timer.start()
        self._last_response = "模式切换中"
        self._render_top_status()
        self.send_text(f"MWORK {mode}", "ASCII", "CRLF")

    def _resolve_mode_request(self, actual_mode: str) -> None:
        """仅在H7实际模式与待应用模式一致时确认模式切换。"""
        if self._pending_mode_command is None:
            return
        normalized = " ".join(actual_mode.upper().split())
        if normalized != self._pending_mode_command:
            return
        self._pending_mode_command = None
        self._mode_apply_timer.stop()
        self.mode_sidebar.complete_mode_request(actual_mode)
        self._refresh_parameter_interlocks()
        self._last_response = "模式切换成功"
        self._render_top_status()

    def _fail_mode_request(self, message: str) -> None:
        """结束失败请求并保留用户草稿，使失败状态和重试入口保持可见。"""
        if self._pending_mode_command is None:
            return
        self._pending_mode_command = None
        self._mode_apply_timer.stop()
        self.mode_sidebar.fail_mode_request(message)
        self._refresh_parameter_interlocks()
        self._last_response = message
        self._render_top_status()

    def _mode_request_timeout(self) -> None:
        """H7未在限定时间确认模式时恢复真实状态，避免界面假切换。"""
        self._fail_mode_request("模式切换失败：H7未在1.5秒内确认")

    def select_tentacle(self, index: int) -> None:
        if self.control_source == "ACT":
            self._last_response = "ACT自主控制固定使用T3"
            self._render_top_status()
            return
        if not self.serial_connected:
            return
        self._last_response = f"选择T{index + 1}中"
        self._render_top_status()
        self.send_text(f"TSEL {index + 1}", "ASCII", "CRLF")

    def panel_control_changed(self, index: int, angle: float, bend: int, stiffness: int) -> None:
        self._apply_linked_values(index, angle, bend, stiffness)
        if self.control_source != "PC" or not self.running:
            return
        base_angle = self._base_angle_for_source(index, angle)
        x = int(round(math.cos(math.radians(base_angle)) * 100))
        y = int(round(math.sin(math.radians(base_angle)) * 100))
        self._pending_virtual_command = f"MVIRT {x} {y} {bend} {stiffness}"

    def _base_angle_for_source(self, source_index: int, angle: float) -> float:
        active = self.active_indices or [source_index]
        if self.mode_relation == "MIRROR" and len(active) == 2 and source_index == active[1]:
            return (180.0 - angle) % 360.0
        if self.mode_relation == "CENTER" and len(active) == 4:
            offsets = {active[0]: 0.0, active[1]: 180.0, active[2]: 0.0, active[3]: 180.0}
            return (angle - offsets.get(source_index, 0.0)) % 360.0
        return angle % 360.0

    def _flush_virtual_command(self) -> None:
        if self._pending_virtual_command is None:
            return
        if self.control_source != "PC" or not self.running or not self.serial.port.isOpen():
            self._pending_virtual_command = None
            return
        if time.monotonic() < self._virtual_hold_until:
            return
        command = self._pending_virtual_command
        self._pending_virtual_command = None
        self._last_virtual_command = command
        self.send_text(command, "ASCII", "CRLF")

    def _set_values_all(self, x: float, y: float, bend: int, stiffness: int) -> None:
        if not self.active_indices:
            return
        angle = math.degrees(math.atan2(y, x)) % 360 if abs(x) + abs(y) > 0.1 else 0.0
        source = self.selected_index if self.selected_index in self.active_indices else self.active_indices[0]
        self._apply_linked_values(source, angle, bend, stiffness)

    def _apply_linked_values(self, source_index: int, angle: float, bend: int, stiffness: int) -> None:
        """Apply the selected control angle according to the active link mode."""
        active = self.active_indices or [source_index]
        if self.mode_relation == "MIRROR" and len(active) == 2:
            other_index = active[0] if active[1] == source_index else active[1]
            angles = {source_index: angle, other_index: (180.0 - angle) % 360.0}
        elif self.mode_relation == "CENTER" and len(active) == 4:
            # Physical layout is two columns: T1/T3 on the left, T2/T4 on the right.
            offsets = {active[0]: 0.0, active[1]: 180.0, active[2]: 0.0, active[3]: 180.0}
            source_offset = offsets.get(source_index, 0.0)
            base = (angle - source_offset) % 360.0
            angles = {index: (base + offsets[index]) % 360.0 for index in active}
        else:
            angles = {index: angle for index in active}
        for index in active:
            self.panels[index].set_values(angles.get(index, angle), bend, stiffness)

    def _reset_active_control_values(self) -> None:
        """Reset local target controls without changing the POS actual-pose view."""
        active = self.active_indices or [self.selected_index]
        for index in active:
            self.panels[index].set_values(0.0, 0, 0)

    def return_center(self, index: int) -> None:
        if self.panels[index].locked or self.control_source != "PC":
            return
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._reset_active_control_values()
        if self.running:
            self._pending_virtual_command = "MVIRT 0 0 0 0"

    def panel_lock_changed(self, index: int, locked: bool) -> None:
        if not locked:
            reason = self._parameter_edit_block_reason()
            if reason:
                self.panels[index].set_locked(True)
                self._last_response = reason
                self._render_top_status()
                return
        panel = self.panels[index]
        panel.set_status(panel.online, panel.active, index == self.selected_index, panel.feedback)

    def start_control(self) -> None:
        reason = self._start_block_reason()
        if reason:
            self.receive_drawer.append("ERR", (reason + "\r\n").encode("utf-8"))
            self._last_response = reason
            self._render_top_status()
            return
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self._last_response = "启动确认中"
        self._render_top_status()
        self.send_text("MCTRL 1", "ASCII", "CRLF", control_validated=True)

    def stop_control(self) -> None:
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self._last_response = (
            "ACT停止/状态清除确认中"
            if self.control_source == "ACT"
            else "停止确认中"
        )
        self._render_top_status()
        self.send_text("MCTRL 0", "ASCII", "CRLF")

    def emergency_stop(self) -> None:
        self._pending_virtual_command = None
        self._last_virtual_command = None
        self._virtual_hold_until = 0.0
        self.running = False
        self.send_text("CANSTOP", "ASCII", "CRLF")
        for panel in self.panels: panel.set_locked(True)
        self.running = False
        self._refresh_mode_runtime()
        self._last_response = "紧急停止"
        self._render_top_status()

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if event.type() in (QEvent.KeyPress, QEvent.KeyRelease) and isinstance(event, QKeyEvent):
            focus = QApplication.focusWidget()
            if event.key() == Qt.Key_F11:
                if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
                    self.toggle_receive_fullscreen()
                event.accept()
                return True
            if event.key() == Qt.Key_Space:
                if isinstance(focus, (QLineEdit, QPlainTextEdit)) and not focus.isReadOnly():
                    return super().eventFilter(watched, event)
                # Windows key repeat can emit synthetic release/press pairs.
                # Ignore both so only the physical key release cancels a hold.
                if event.isAutoRepeat():
                    event.accept()
                    return True
                if event.type() == QEvent.KeyRelease:
                    if self._space_hold_active:
                        fired = self.mode_sidebar.start_button.cancel_hold()
                        self._space_hold_active = False
                        if not fired and not self.running:
                            self.top_status.setText("启动已取消：空格长按时间不足800ms")
                    event.accept()
                    return True
                if self.control_source == "ACT" and self.act_state == 3:
                    self.stop_control()
                elif self.running:
                    self.stop_control()
                else:
                    reason = self._start_block_reason()
                    if reason:
                        self._space_hold_active = False
                        self._last_response = reason
                        self._render_top_status()
                        event.accept()
                        return True
                    self._space_hold_active = True
                    self.mode_sidebar.start_button.begin_hold()
                    self.top_status.setText("长按空格启动：请保持按下直到进度达到100%")
                event.accept()
                return True
            if event.key() == Qt.Key_L:
                ctrl_pressed = bool(event.modifiers() & Qt.ControlModifier)
                if (
                    isinstance(focus, (QLineEdit, QPlainTextEdit))
                    and not focus.isReadOnly()
                    and not ctrl_pressed
                ):
                    return super().eventFilter(watched, event)
                if event.type() == QEvent.KeyRelease or event.isAutoRepeat():
                    event.accept()
                    return True
                block_reason = self._parameter_edit_block_reason()
                if block_reason:
                    self._last_response = block_reason
                    self._render_top_status()
                    event.accept()
                    return True
                if ctrl_pressed:
                    lock_all = not all(panel.locked for panel in self.panels)
                    for panel in self.panels:
                        panel.set_locked(lock_all)
                    state = "全部锁定" if lock_all else "全部解锁"
                    self.top_status.setText(f"参数状态：T1 / T2 / T3 / T4 已{state}")
                else:
                    panel = self.panels[self.selected_index]
                    panel.set_locked(not panel.locked)
                    state = "锁定" if panel.locked else "解锁"
                    self.top_status.setText(
                        f"参数状态：当前对象 T{self.selected_index + 1} 已{state}"
                    )
                event.accept()
                return True
        return super().eventFilter(watched, event)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.isAutoRepeat(): return
        if event.key() == Qt.Key_Escape:
            self.emergency_stop(); return
        if event.key() == Qt.Key_R:
            self.return_center(self.selected_index); return
        super().keyPressEvent(event)
