from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import json
import os
from pathlib import Path
from typing import Iterable


@dataclass(slots=True)
class CommandDefinition:
    command_id: str
    name: str
    group: str
    description: str
    template: str
    raw_bytes: bytes
    encoding: str = "ASCII"
    line_ending: str = "CRLF"
    loop_enabled: bool = False
    loop_interval_ms: int = 500
    loop_count: int = 1
    dangerous: bool = False
    parameterized: bool = False


@dataclass(slots=True)
class CommandDraft:
    text: str
    encoding: str = "ASCII"
    line_ending: str = "CRLF"
    loop_enabled: bool = False
    loop_interval_ms: int = 500
    loop_count: int = 1
    last_sent: str = ""


@dataclass(slots=True)
class CommandGroup:
    name: str
    description: str
    commands: list[CommandDefinition] = field(default_factory=list)


class CommandStore:
    """Loads VOFA buttons and persists an independent draft for every command."""

    DANGEROUS_PREFIXES = (
        "CANZERO", "CANHOME", "W25TEST", "W25SAVE", "W25LOAD", "VLSCAN",
    )
    DANGEROUS_EXACT = ("MCTRL 1",)

    def __init__(self, catalog_path: Path, state_path: Path | None = None) -> None:
        self.catalog_path = Path(catalog_path)
        self.state_path = state_path or self.default_state_path()
        self.groups: list[CommandGroup] = []
        self.definitions: dict[str, CommandDefinition] = {}
        self.drafts: dict[str, CommandDraft] = {}
        self.load()

    @staticmethod
    def default_state_path() -> Path:
        appdata = os.environ.get("APPDATA")
        root = Path(appdata) if appdata else Path.home() / ".tentacle_controller"
        return root / "TentacleController" / "command_state.json"

    def load(self) -> None:
        payload = json.loads(self.catalog_path.read_text(encoding="utf-8"))
        root = payload.get("ctx", {})
        self.groups = []
        self.definitions = {}
        for group_node in root.get("subCmds", []):
            if not group_node.get("is_group", False):
                continue
            group = CommandGroup(
                name=str(group_node.get("name", "未分组")),
                description=str(group_node.get("intro", "")),
            )
            for item in group_node.get("subCmds", []):
                if item.get("is_group", False):
                    continue
                definition = self._parse_definition(group.name, item)
                group.commands.append(definition)
                self.definitions[definition.command_id] = definition
            self.groups.append(group)
        self._load_drafts()

    def _parse_definition(self, group_name: str, item: dict) -> CommandDefinition:
        raw = self._decode_hex(str(item.get("cmd_hex", "")))
        ending, content = self._split_line_ending(raw)
        try:
            template = content.decode("utf-8")
            encoding = "ASCII"
        except UnicodeDecodeError:
            template = content.hex(" ").upper()
            encoding = "HEX"
        identity = f"{group_name}\0{item.get('name', '')}\0{raw.hex()}"
        command_id = hashlib.sha1(identity.encode("utf-8")).hexdigest()[:16]
        normalized = template.strip().upper()
        name = str(item.get("name", "未命名命令"))
        parameterized = bool(item.get("parameterized", ("模板" in name) or ("参数" in name)))
        return CommandDefinition(
            command_id=command_id,
            name=name,
            group=group_name,
            description=str(item.get("intro", "")),
            template=template,
            raw_bytes=raw,
            encoding=encoding,
            line_ending=ending,
            loop_enabled=bool(item.get("loop_on", False)),
            loop_interval_ms=max(10, int(item.get("loop_ms", 500))),
            loop_count=max(1, int(item.get("loop_count", 1))),
            dangerous=(
                normalized.startswith(self.DANGEROUS_PREFIXES)
                or normalized in self.DANGEROUS_EXACT
            ),
            parameterized=parameterized,
        )

    @staticmethod
    def _decode_hex(value: str) -> bytes:
        try:
            return bytes.fromhex(value)
        except ValueError:
            return b""

    @staticmethod
    def _split_line_ending(raw: bytes) -> tuple[str, bytes]:
        if raw.endswith(b"\r\n"):
            return "CRLF", raw[:-2]
        if raw.endswith(b"\r"):
            return "CR", raw[:-1]
        if raw.endswith(b"\n"):
            return "LF", raw[:-1]
        return "NONE", raw

    def _load_drafts(self) -> None:
        saved: dict[str, dict] = {}
        try:
            saved = json.loads(self.state_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            pass
        self.drafts = {}
        for command_id, definition in self.definitions.items():
            values = saved.get(command_id, {})
            self.drafts[command_id] = CommandDraft(
                text=(str(values.get("text", definition.template))
                      if definition.parameterized else definition.template),
                encoding=definition.encoding,
                line_ending=definition.line_ending,
                loop_enabled=bool(values.get("loop_enabled", definition.loop_enabled)),
                loop_interval_ms=max(10, int(values.get("loop_interval_ms", definition.loop_interval_ms))),
                loop_count=max(1, int(values.get("loop_count", definition.loop_count))),
                last_sent=str(values.get("last_sent", "")),
            )

    def save(self) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {command_id: asdict(draft) for command_id, draft in self.drafts.items()}
        temporary = self.state_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(self.state_path)

    def update_draft(self, command_id: str, **changes: object) -> CommandDraft:
        draft = self.drafts[command_id]
        for key, value in changes.items():
            if hasattr(draft, key):
                setattr(draft, key, value)
        return draft

    def reset_draft(self, command_id: str) -> CommandDraft:
        definition = self.definitions[command_id]
        self.drafts[command_id] = CommandDraft(
            text=definition.template,
            encoding=definition.encoding,
            line_ending=definition.line_ending,
            loop_enabled=definition.loop_enabled,
            loop_interval_ms=definition.loop_interval_ms,
            loop_count=definition.loop_count,
        )
        self.save()
        return self.drafts[command_id]

    def iter_commands(self) -> Iterable[CommandDefinition]:
        for group in self.groups:
            yield from group.commands
