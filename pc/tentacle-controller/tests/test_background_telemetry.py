from __future__ import annotations

import os
from pathlib import Path
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtWidgets import QApplication
except ImportError as exc:
    import pytest

    pytest.skip(f"PySide6 runtime unavailable: {exc}", allow_module_level=True)

from app import MainWindow


SNAP_LINE = "SNAP,1,1,0,3,2,1,3,3,0,0,100,0,20,30,0"
POS_LINE = "POS,1,7,0,0,0,7,0,0,0,0,0,0,0,0,0,0,0"


class _OpenPort:
    @staticmethod
    def isOpen() -> bool:
        return True


class _FakeSerial:
    def __init__(self) -> None:
        self.port = _OpenPort()
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.writes: list[bytes] = []

    def write(self, payload: bytes) -> bool:
        self.writes.append(payload)
        self.tx_bytes += len(payload)
        return True


def _window() -> MainWindow:
    QApplication.instance() or QApplication([])
    return MainWindow(Path(__file__).resolve().parents[1])


def test_background_query_and_reply_are_hidden() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial

    window.send_text("SNAP?", "ASCII", "CRLF", background=True)
    assert fake_serial.writes == [b"SNAP?\r\n"]
    assert window.receive_drawer.text.toPlainText() == ""

    fake_serial.rx_bytes += len(SNAP_LINE) + 2
    window._on_received((SNAP_LINE + "\r\n").encode())
    assert window.online_mask == 3
    assert window.receive_drawer.text.toPlainText() == ""
    assert window.receive_drawer._last == "--"
    assert "在线 2/4" in window.receive_drawer.snapshot_device.text()
    window.close()


def test_manual_query_is_visible_but_pose_moves_to_fixed_status() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial

    window.send_text("POS?", "ASCII", "CRLF")
    assert "TX   POS?" in window.receive_drawer.text.toPlainText()

    payload = (POS_LINE + "\r\nOK MCTRL 0\r\n").encode()
    fake_serial.rx_bytes += len(payload)
    window._on_received(payload)
    log = window.receive_drawer.text.toPlainText()
    assert "RX   POS,1" not in log
    assert "RX   OK MCTRL 0" in log
    assert window.receive_drawer.pose_values[0][1].text() == "在线 / 活动 / 就绪"
    window.close()


def test_split_background_pose_reply_is_parsed_and_hidden() -> None:
    window = _window()
    window._background_replies["POS"] = time.monotonic() + 1.0

    split_at = len(POS_LINE) // 2
    window._on_received(POS_LINE[:split_at].encode())
    assert window.receive_drawer.text.toPlainText() == ""
    window._on_received((POS_LINE[split_at:] + "\r\n").encode())
    assert window.receive_drawer.text.toPlainText() == ""
    assert window.panels[0].online is True
    window.close()


def test_ai_mapping_updates_fixed_status_without_scrolling_log() -> None:
    window = _window()
    line = "当前映射X：46 Y：34 角度：36deg 距离：423mm 手弯曲程度：8 raw=10"

    window._on_received((line + "\r\n").encode())

    assert window.receive_drawer.text.toPlainText() == ""
    assert window.receive_drawer.ai_direction.text() == "X 46   Y 34   角度 36°"
    assert window.receive_drawer.ai_shape.text() == "距离 423 mm   Bend 8   Raw 10"
    window.close()


def test_ai_mapping_utf8_split_at_every_byte_is_not_corrupted() -> None:
    window = _window()
    payload = "当前映射X：90 Y：81 角度：42deg 距离：436mm 手弯曲程度：100 raw=99\r\n".encode()

    for byte in payload:
        window._on_received(bytes((byte,)))

    assert window.receive_drawer.text.toPlainText() == ""
    assert window.receive_drawer.ai_direction.text() == "X 90   Y 81   角度 42°"
    assert window.receive_drawer.ai_shape.text() == "距离 436 mm   Bend 100   Raw 99"
    window.close()


def test_damaged_ai_label_is_filtered_and_values_are_kept() -> None:
    window = _window()
    line = "当前映射X：72 Y：100 角度：54deg 距离：442mm 手弯���程度：100 raw=99"

    window._on_received((line + "\r\n").encode())

    assert window.receive_drawer.text.toPlainText() == ""
    assert window.receive_drawer.ai_direction.text() == "X 72   Y 100   角度 54°"
    assert window.receive_drawer.ai_shape.text() == "距离 442 mm   Bend 100   Raw 99"
    window.close()


def test_malformed_live_status_remains_in_debug_log() -> None:
    window = _window()

    window._on_received("SNAP,1,bad,3\r\n".encode())

    assert "SNAP,1,bad,3" in window.receive_drawer.text.toPlainText()
    window.close()


def test_pc_control_start_virtual_input_and_pose_feedback() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True

    window._on_received(b"SNAP,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0\r\n")
    assert window.control_source == "PC"
    assert window.mode_sidebar.start_button.isEnabled()

    window.start_control()
    assert fake_serial.writes[-1] == b"MCTRL 1\r\n"
    window._on_received(b"OK MCTRL 1 mode=SOLO mask=0x01 mirror=0\r\n")
    assert window.running is True

    window.panel_control_changed(0, 90.0, 40, 50)
    window._flush_virtual_command()
    assert fake_serial.writes[-1] == b"MVIRT 0 100 40 50\r\n"

    window._on_received(b"POS,1,7,100,0,-100,0,0,0,0,0,0,0,0,0,0,0,0\r\n")
    assert window.panels[0].online is True
    assert window.panels[0].actual_dq == (100, 0, -100)
    window.close()


def test_pc_snapshot_does_not_overwrite_local_target_after_initial_sync() -> None:
    window = _window()
    window.serial = _FakeSerial()
    window.serial_connected = True
    pc_snapshot = b"SNAP,1,1,0,1,0,1,1,1,0,0,100,0,20,30,0\r\n"

    window._on_received(pc_snapshot)
    assert window.panels[0].bend == 20
    assert window.panels[0].stiffness == 30

    window.panels[0].set_values(135.0, 55, 66)
    window._on_received(pc_snapshot)

    assert window.panels[0].angle == 135.0
    assert window.panels[0].bend == 55
    assert window.panels[0].stiffness == 66
    window.close()


def test_mode_draft_is_not_overwritten_by_background_snapshot() -> None:
    window = _window()
    window.serial = _FakeSerial()
    window.serial_connected = True
    solo = b"SNAP,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0\r\n"
    window._on_received(solo)

    window.mode_sidebar.mode_combo.setCurrentText("双触手")
    window.mode_sidebar._mark_mode_editor_dirty()
    window._on_received(solo)

    assert window.mode_sidebar.mode_combo.currentText() == "双触手"
    assert window.mode_sidebar.motion_mode_label.text() == "单触手 · T1"
    window.close()


def test_mode_request_waits_for_matching_h7_confirmation() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    solo = b"SNAP,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0\r\n"
    window._on_received(solo)
    window.mode_sidebar.mode_combo.setCurrentText("双触手")
    window.mode_sidebar._mark_mode_editor_dirty()

    window.apply_mode("DUAL 12 SAME")
    assert fake_serial.writes[-1] == b"MWORK DUAL 12 SAME\r\n"
    assert window.mode_sidebar.apply_button.text() == "等待H7确认…"

    window._on_received(solo)
    assert window.mode_sidebar.mode_combo.currentText() == "双触手"
    assert window._pending_mode_command == "DUAL 12 SAME"

    window._on_received(b"OK MWORK mode=COOP mask=0x03 mirror=0 TSEL=1\r\n")
    assert window._pending_mode_command is None
    assert window.mode_text == "DUAL 12 SAME"
    assert window.mode_sidebar.mode_combo.currentText() == "双触手"
    assert window.mode_sidebar.apply_button.text() == "确认并应用模式"
    assert window._last_response == "模式切换成功"
    window.close()


def test_mode_request_timeout_keeps_draft_and_exposes_retry() -> None:
    window = _window()
    window.serial = _FakeSerial()
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0\r\n")
    window.mode_sidebar.mode_combo.setCurrentText("双触手")
    window.mode_sidebar._mark_mode_editor_dirty()

    window.apply_mode("DUAL 12 SAME")
    window._mode_request_timeout()

    assert window._pending_mode_command is None
    assert window.mode_sidebar.mode_combo.currentText() == "双触手"
    assert window.mode_sidebar.apply_button.text() == "重试应用模式"
    assert "未在1.5秒内确认" in window._last_response
    window.close()


def test_act_source_locks_t3_routes_mctrl_and_parses_status() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True

    window._on_received(
        b"SNAP,1,2,0,1,0,1,4,4,0,0,0,0,0,0,0\r\n"
        b"ACTSTAT,0,0,0,0,0\r\n"
    )

    assert window.control_source == "ACT"
    assert window.mode_text == "SOLO 3"
    assert window.selected_index == 2
    assert not window.mode_sidebar.mode_combo.isEnabled()
    assert not window.mode_sidebar.apply_button.isEnabled()
    assert not window.panels[2].lock_button.isEnabled()
    assert window.mode_sidebar.start_button.isEnabled()

    window.start_control()
    assert fake_serial.writes[-1] == b"MCTRL 1\r\n"
    window._on_received(b"OK MCTRL 1 mode=ACT mask=0x04\r\n")
    assert window.act_state == 1
    assert window.running is True

    window._on_received(b"ACTSTAT,2,12,0,7,3\r\n")
    assert window.act_state == 2
    assert "RUNNING" in window.receive_drawer.act_state.text()

    window.stop_control()
    assert fake_serial.writes[-1] == b"MCTRL 0\r\n"
    window._on_received(b"OK MCTRL 0 mode=ACT mask=0x04\r\n")
    assert window.act_state == 0
    assert window.running is False
    window.close()


def test_stale_snapshot_does_not_cancel_pending_act_source() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0\r\n")

    window.request_control_source("ACT")
    assert window._pending_source == "ACT"
    window._on_received(b"SNAP,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0\r\n")

    assert window._pending_source == "ACT"
    assert window.control_source == "PC"
    window._on_received(b"OK MSRC ACT enable=0\r\n")
    assert window._pending_source is None
    assert window.control_source == "ACT"
    window.close()


def test_ai_snapshot_remains_authoritative_for_read_only_target_display() -> None:
    window = _window()
    window.serial = _FakeSerial()
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,0,1,0,1,1,1,0,0,100,0,55,66,0\r\n")
    window.panels[0].set_values(135.0, 10, 20)

    window._on_received(b"SNAP,1,0,0,1,0,1,1,1,0,0,0,100,35,45,0\r\n")

    assert window.panels[0].angle == 90.0
    assert window.panels[0].bend == 35
    assert window.panels[0].stiffness == 45
    window.close()


def test_return_center_resets_all_local_targets_to_zero() -> None:
    window = _window()
    window.serial = _FakeSerial()
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,1,3,2,1,3,3,0,0,0,100,40,50,0\r\n")
    window.panels[0].set_locked(False)
    window.panels[0].set_values(90.0, 40, 50)
    window.panels[1].set_values(90.0, 40, 50)

    window.return_center(0)

    for index in (0, 1):
        assert window.panels[index].angle == 0.0
        assert window.panels[index].bend == 0
        assert window.panels[index].stiffness == 0
    assert window._pending_virtual_command == "MVIRT 0 0 0 0"
    window.close()


def test_canhome_success_resets_target_display_and_requests_actual_pose() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,0,1,0,1,1,1,0,0,100,0,40,50,0\r\n")
    window.panels[0].set_values(90.0, 40, 50)

    window._on_received(b"OK CANHOME mode=SOLO mask=0x01 status=0 tx=10\r\n")
    QApplication.processEvents()

    assert window.panels[0].angle == 0.0
    assert window.panels[0].bend == 0
    assert window.panels[0].stiffness == 0
    assert b"POS?\r\n" in fake_serial.writes
    assert "HOME目标已归零" in window._last_response
    window.close()


def test_mctrl_not_ready_updates_masks_and_stays_stopped() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,0,3,2,1,3,3,0,0,0,0,0,0,0\r\n")

    window._on_received(
        b"ERR MCTRL NOT_READY online=0x03 ready=0x01 fault=0x00\r\n"
    )

    assert window.running is False
    assert window.online_mask == 0x03
    assert window.ready_mask == 0x01
    assert window.fault_mask == 0
    assert "T2" in window._last_response
    assert "RESTORE" in window._last_response
    QApplication.processEvents()
    assert b"SNAP?\r\n" in fake_serial.writes
    window.close()


def test_mvirt_disabled_stops_local_sending() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,1,1,0,1,1,1,0,0,0,0,20,30,0\r\n")
    assert window.running is True
    window._pending_virtual_command = "MVIRT 100 0 20 30"

    window._on_received(b"ERR MVIRT DISABLED\r\n")

    assert window.running is False
    assert window._pending_virtual_command is None
    assert "未启动" in window._last_response
    window.close()


def test_mvirt_source_and_send_errors_clear_old_target_and_resync() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    safe = b"SNAP,1,1,1,1,0,1,1,1,0,0,0,0,20,30,0\r\n"
    window._on_received(safe)
    window._pending_virtual_command = "MVIRT 100 0 20 30"

    window._on_received(b"ERR MVIRT SOURCE\r\n")
    QApplication.processEvents()
    assert window.running is False
    assert window._pending_virtual_command is None
    assert b"SNAP?\r\n" in fake_serial.writes

    window._on_received(safe)
    assert window.running is True
    window._pending_virtual_command = "MVIRT 0 100 20 30"
    window._on_received(b"ERR MVIRT status=2\r\n")
    QApplication.processEvents()
    assert window.running is True
    assert window._pending_virtual_command == "MVIRT 0 100 20 30"
    assert "CAN发送忙" in window._last_response
    window.close()


def test_unready_enabled_snapshot_never_stops_pc_control() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    unsafe = b"SNAP,1,1,1,1,0,1,1,0,0,0,0,0,20,30,2\r\n"

    window._on_received(unsafe)
    window._on_received(unsafe)
    window._on_received(unsafe)

    assert window.running is True
    assert fake_serial.writes.count(b"MCTRL 0\r\n") == 0
    window._pending_virtual_command = "MVIRT 100 0 20 30"
    window._flush_virtual_command()
    assert fake_serial.writes[-1] == b"MVIRT 100 0 20 30\r\n"
    window.close()


def test_single_unready_snapshot_recovers_without_stopping() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True

    window._on_received(b"SNAP,1,1,1,1,0,1,1,0,0,0,0,0,20,30,2\r\n")
    window._on_received(b"SNAP,1,1,1,1,0,1,1,1,0,0,0,0,20,30,0\r\n")

    assert window.running is True
    assert fake_serial.writes.count(b"MCTRL 0\r\n") == 0
    window.close()


def test_fault_snapshot_is_display_only_until_h7_stops() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    fault = b"SNAP,1,1,1,1,0,1,1,1,1,0,0,0,20,30,0\r\n"

    window._on_received(fault)
    window._on_received(fault)

    assert window.running is True
    assert window.fault_mask == 1
    assert fake_serial.writes.count(b"MCTRL 0\r\n") == 0
    window.close()


def test_named_mvirt_not_ready_does_not_stop_or_send_mctrl_zero() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,1,1,0,1,1,1,0,0,0,0,20,30,0\r\n")
    window._pending_virtual_command = "MVIRT 100 0 20 30"
    window._flush_virtual_command()

    window._on_received(b"ERR MVIRT NOT_READY\r\n")

    assert window.running is True
    assert fake_serial.writes.count(b"MCTRL 0\r\n") == 0
    assert "同步运行状态" in window._last_response
    window.close()


def test_named_mvirt_tx_busy_retries_but_fault_stops() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,1,1,0,1,1,1,0,0,0,0,20,30,0\r\n")
    window._last_virtual_command = "MVIRT 0 100 20 30"

    window._on_received(b"ERR MVIRT TX_BUSY\r\n")
    assert window.running is True
    assert window._pending_virtual_command == "MVIRT 0 100 20 30"

    window._on_received(b"ERR MVIRT FAULT\r\n")
    assert window.running is False
    assert window._pending_virtual_command is None
    assert fake_serial.writes.count(b"MCTRL 0\r\n") == 0
    window.close()


def test_mstop_stops_once_and_keeps_h7_reason() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,1,3,2,1,3,3,0,0,0,0,20,30,0\r\n")
    window._pending_virtual_command = "MVIRT 100 0 20 30"

    window._on_received(b"MSTOP reason=CAN_TIMEOUT mask=0x02\r\n")

    assert window.running is False
    assert window._pending_virtual_command is None
    assert window._last_stop_reason == "CAN_TIMEOUT"
    assert window._last_stop_mask == 0x02
    assert "CAN状态超时" in window._last_response
    assert "T2" in window._last_response
    assert fake_serial.writes.count(b"MCTRL 0\r\n") == 0
    window.close()


def test_snapshot_enable_zero_is_stopped_fallback_without_invented_reason() -> None:
    window = _window()
    window.serial = _FakeSerial()
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,1,1,0,1,1,1,0,0,0,0,20,30,0\r\n")

    window._on_received(b"SNAP,1,1,0,1,0,1,0,0,0,0,0,0,20,30,0\r\n")

    assert window.running is False
    assert "未收到停止原因" in window._last_response
    window.close()


def test_mctrl_fault_reply_uses_new_h7_mask_format() -> None:
    window = _window()
    window.serial = _FakeSerial()
    window.serial_connected = True

    window._on_received(
        b"ERR MCTRL FAULT online=0x03 ready=0x03 fault=0x02\r\n"
    )

    assert window.running is False
    assert window.online_mask == 0x03
    assert window.ready_mask == 0x03
    assert window.fault_mask == 0x02
    assert "T2" in window._last_response
    assert "故障" in window._last_response
    window.close()


def test_start_request_is_not_blocked_by_stale_snapshot_masks() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,0,1,0,1,0,0,0,0,0,0,0,0,0\r\n")

    assert window.mode_sidebar.start_button.isEnabled()
    window.start_control()
    assert fake_serial.writes[-1] == b"MCTRL 1\r\n"
    window.close()


def test_receive_window_compact_summary_and_side_pages() -> None:
    window = _window()
    drawer = window.receive_drawer
    window.receive_dock.show()
    drawer.close_detail()
    window.receive_dock.resize(760, 460)
    QApplication.processEvents()

    assert window.receive_dock.width() == 760
    assert len(drawer.tentacle_summary_buttons) == 4
    assert drawer.detail_drawer.isHidden()

    drawer.show_pose_detail(2)
    QApplication.processEvents()
    assert drawer.detail_drawer.isVisible()
    assert drawer.detail_stack.currentIndex() == 0
    assert drawer._detail_tentacle == 2

    drawer.show_detail_page(1)
    QApplication.processEvents()
    assert drawer.detail_stack.currentIndex() == 1
    drawer.show_detail_page(2)
    QApplication.processEvents()
    assert drawer.detail_stack.currentIndex() == 2
    window.close()


def test_receive_window_fullscreen_restores_normal_state() -> None:
    window = _window()
    window.receive_dock.resize(900, 520)
    window.receive_dock.show()
    QApplication.processEvents()

    window.toggle_receive_fullscreen()
    QApplication.processEvents()
    assert window._receive_fullscreen is True
    assert window.receive_dock.isFullScreen()

    window.toggle_receive_fullscreen()
    QApplication.processEvents()
    assert window._receive_fullscreen is False
    assert not window.receive_dock.isFullScreen()
    assert window.receive_dock.width() >= 760
    assert window.receive_dock.height() >= 420
    window.close()


def test_gesture_query_is_background_and_updates_ai_switch() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial

    window.send_text("GESTURE?", "ASCII", "CRLF", background=True)
    assert fake_serial.writes == [b"GESTURE?\r\n"]
    assert window.receive_drawer.text.toPlainText() == ""

    window._on_received(b"GESTURE,0\r\n")

    assert window.gesture_enabled is False
    assert window.receive_drawer.gesture_button.isEnabled()
    assert window.receive_drawer.gesture_button.text() == "开启推理"
    assert window.receive_drawer.text.toPlainText() == ""
    window.close()


def test_gesture_enable_waits_for_h7_confirmation() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"GESTURE,0\r\n")

    window.request_gesture_inference(True)

    assert fake_serial.writes[-1] == b"GESTURE 1\r\n"
    assert window.gesture_enabled is False
    assert window._pending_gesture is True
    assert not window.receive_drawer.gesture_button.isEnabled()

    window._on_received(b"OK GESTURE 1\r\n")
    QApplication.processEvents()

    assert window.gesture_enabled is True
    assert window._pending_gesture is None
    assert window.receive_drawer.gesture_button.text() == "关闭推理"
    assert b"GESTURE?\r\n" in fake_serial.writes
    window.close()


def test_disabling_gesture_stops_running_ai_before_command() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,0,1,1,0,1,1,1,0,1,0,0,20,30,0\r\n")
    window._on_received(b"GESTURE,1\r\n")

    window.request_gesture_inference(False)

    assert fake_serial.writes[-1] == b"MCTRL 0\r\n"
    assert b"GESTURE 0\r\n" not in fake_serial.writes

    window._on_received(b"OK MCTRL 0 mode=SOLO mask=0x01 mirror=0\r\n")
    QApplication.processEvents()

    assert b"GESTURE 0\r\n" in fake_serial.writes
    window._on_received(b"OK GESTURE 0\r\n")
    assert window.gesture_enabled is False
    window.close()


def test_disabling_gesture_does_not_stop_running_pc_control() -> None:
    window = _window()
    fake_serial = _FakeSerial()
    window.serial = fake_serial
    window.serial_connected = True
    window._on_received(b"SNAP,1,1,1,1,0,1,1,1,0,0,0,0,20,30,0\r\n")
    window._on_received(b"GESTURE,1\r\n")

    window.request_gesture_inference(False)

    assert fake_serial.writes[-1] == b"GESTURE 0\r\n"
    assert fake_serial.writes.count(b"MCTRL 0\r\n") == 0
    window.close()
