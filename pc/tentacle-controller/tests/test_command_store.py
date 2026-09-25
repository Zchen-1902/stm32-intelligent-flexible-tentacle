from pathlib import Path
import json

from command_store import CommandStore


def test_vofa_catalog_and_per_command_drafts(tmp_path: Path) -> None:
    source = Path(__file__).parents[1] / "assets" / "default_commands.json"
    state = tmp_path / "state.json"
    store = CommandStore(source, state)
    assert len(list(store.iter_commands())) > 20
    canpos = next(command for command in store.iter_commands() if "相对位置参数模板" in command.name)
    store.update_draft(canpos.command_id, text="CANPOS 100 0 -100", encoding="HEX", line_ending="NONE")
    store.save()
    restored = CommandStore(source, state)
    assert restored.drafts[canpos.command_id].text == "CANPOS 100 0 -100"
    assert restored.drafts[canpos.command_id].encoding == "ASCII"
    assert restored.drafts[canpos.command_id].line_ending == "CRLF"


def test_fixed_commands_ignore_contaminated_saved_drafts(tmp_path: Path) -> None:
    source = Path(__file__).parents[1] / "assets" / "default_commands.json"
    state = tmp_path / "state.json"
    store = CommandStore(source, state)
    fixed = next(command for command in store.iter_commands() if command.name == "开启映射")
    store.update_draft(fixed.command_id, text="BROKEN COMMAND", encoding="HEX", line_ending="NONE")
    store.save()

    restored = CommandStore(source, state)

    assert restored.drafts[fixed.command_id].text == "MCTRL 1"
    assert restored.drafts[fixed.command_id].encoding == "ASCII"
    assert restored.drafts[fixed.command_id].line_ending == "CRLF"
    assert fixed.parameterized is False
    assert next(command for command in restored.iter_commands() if "虚拟映射输入模板" in command.name).parameterized
