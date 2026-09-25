from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtWidgets import QApplication, QMessageBox
except ImportError as exc:
    import pytest

    pytest.skip(f"PySide6 runtime unavailable: {exc}", allow_module_level=True)

from app import SerialSidebar
from command_store import CommandStore


def _sidebar(tmp_path: Path) -> SerialSidebar:
    QApplication.instance() or QApplication([])
    source = Path(__file__).parents[1] / "assets" / "default_commands.json"
    return SerialSidebar(CommandStore(source, tmp_path / "state.json"))


def _command_id(sidebar: SerialSidebar, name: str) -> str:
    return next(
        definition.command_id
        for definition in sidebar.store.iter_commands()
        if definition.name == name
    )


def test_fixed_command_is_read_only_and_executes_once(tmp_path: Path, monkeypatch) -> None:
    sidebar = _sidebar(tmp_path)
    sent: list[tuple[str, str, str]] = []
    sidebar.send_requested.connect(lambda text, encoding, ending: sent.append((text, encoding, ending)))
    command_id = _command_id(sidebar, "开启映射")
    monkeypatch.setattr(QMessageBox, "question", lambda *args, **kwargs: QMessageBox.Yes)

    sidebar._execute_command(command_id)

    assert sidebar.input_edit.isReadOnly()
    assert sidebar.input_edit.toPlainText() == "MCTRL 1"
    assert sent == [("MCTRL 1", "ASCII", "CRLF")]
    sidebar.close()


def test_adhoc_text_does_not_replace_parameter_draft(tmp_path: Path) -> None:
    sidebar = _sidebar(tmp_path)
    command_id = _command_id(sidebar, "虚拟映射输入模板")
    sidebar._select_command(command_id)
    sidebar.input_edit.setPlainText("MVIRT 10 20 30 40")

    sidebar._activate_adhoc()
    sidebar.input_edit.setPlainText("UNLISTED 123")
    sidebar._select_command(command_id)

    assert sidebar.input_edit.toPlainText() == "MVIRT 10 20 30 40"
    assert sidebar.store.drafts[command_id].text == "MVIRT 10 20 30 40"
    sidebar.close()


def test_invalid_parameter_command_is_not_sent(tmp_path: Path) -> None:
    sidebar = _sidebar(tmp_path)
    sent: list[str] = []
    sidebar.send_requested.connect(lambda text, _encoding, _ending: sent.append(text))
    command_id = _command_id(sidebar, "虚拟映射输入模板")
    sidebar._select_command(command_id)
    sidebar.input_edit.setPlainText("MVIRT 500 0 30 40")

    sidebar._send_input()

    assert sent == []
    assert "X、Y 必须在 -100~100" in sidebar.status_label.text()
    sidebar.close()
