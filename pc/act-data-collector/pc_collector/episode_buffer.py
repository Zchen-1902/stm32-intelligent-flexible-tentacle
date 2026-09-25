"""ACT Episode内存缓存、质量统计和原子落盘。"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import time
from typing import Any
from uuid import uuid4

import numpy as np

try:
    from .protocol import ActFrame
except ImportError:  # PyInstaller从pc_collector目录启动时使用顶层模块名。
    from protocol import ActFrame


@dataclass(frozen=True, slots=True)
class EpisodeQuality:
    frame_count: int
    sequence_gap_count: int
    duplicate_count: int
    invalid_position_count: int
    stale_position_count: int
    valid_zone_ratio: float
    receive_hz: float


@dataclass(frozen=True, slots=True)
class EpisodeSnapshot:
    arrays: dict[str, np.ndarray]
    metadata: dict[str, Any]


@dataclass(frozen=True, slots=True)
class TrainingCompatibility:
    schema_valid: bool
    valid_window_count: int
    frame_count: int
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_valid": self.schema_valid,
            "valid_window_count": self.valid_window_count,
            "frame_count": self.frame_count,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
        }


class EpisodeBuffer:
    """保存一次尚未落盘的Episode；只接收已经通过协议校验的完整帧。"""

    FRESH_STATUS_LIMIT_MS = 250

    def __init__(self, tentacle: int, start_h7_sent: int = 0, start_h7_drop: int = 0) -> None:
        if not 1 <= tentacle <= 4:
            raise ValueError("tentacle必须位于1..4")
        self.tentacle = tentacle
        self.started_at = datetime.now(timezone.utc)
        self.started_monotonic_ns = time.monotonic_ns()
        self.start_h7_sent = int(start_h7_sent)
        self.start_h7_drop = int(start_h7_drop)
        self.end_h7_sent = int(start_h7_sent)
        self.end_h7_drop = int(start_h7_drop)
        self.result: str | None = None
        self.position_label = "中间"
        self.distance_label = "中等"
        self.tilt_label = "竖直"
        self.notes = ""
        self.interrupted_reason = ""
        self._frames: list[ActFrame] = []
        self._receive_ns: list[int] = []
        self._sequence_gap_count = 0
        self._duplicate_count = 0
        self._invalid_position_count = 0
        self._stale_position_count = 0
        self._valid_zone_count = 0

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def has_frames(self) -> bool:
        return bool(self._frames)

    @property
    def latest_frame(self) -> ActFrame | None:
        return self._frames[-1] if self._frames else None

    @property
    def elapsed_seconds(self) -> float:
        if not self._receive_ns:
            return max(0.0, (time.monotonic_ns() - self.started_monotonic_ns) / 1e9)
        return max(0.0, (self._receive_ns[-1] - self.started_monotonic_ns) / 1e9)

    def append(self, frame: ActFrame, receive_ns: int | None = None) -> bool:
        """追加一帧；重复序号不重复保存，返回是否实际追加。"""

        if frame.tentacle != self.tentacle:
            raise ValueError(
                f"ACT_FRAME属于T{frame.tentacle}，当前Episode为T{self.tentacle}"
            )
        if self._frames:
            previous = self._frames[-1].frame_seq
            if frame.frame_seq == previous:
                self._duplicate_count += 1
                return False
            delta = (frame.frame_seq - previous) & 0xFFFFFFFF
            if 1 < delta < 0x80000000:
                self._sequence_gap_count += delta - 1
        self._frames.append(frame)
        self._receive_ns.append(receive_ns if receive_ns is not None else time.monotonic_ns())
        if not frame.position_valid:
            self._invalid_position_count += 1
        if frame.status_age_ms > self.FRESH_STATUS_LIMIT_MS:
            self._stale_position_count += 1
        self._valid_zone_count += int(
            np.count_nonzero(
                np.isin(frame.target_status, (5, 9)) & (frame.distance_mm > 0)
            )
        )
        return True

    def set_h7_counters(self, sent: int, dropped: int) -> None:
        self.end_h7_sent = int(sent)
        self.end_h7_drop = int(dropped)

    def set_annotation(
        self,
        result: str,
        position: str,
        distance: str,
        tilt: str,
        notes: str,
    ) -> None:
        if result not in ("success", "failure"):
            raise ValueError("result必须是success或failure")
        self.result = result
        self.position_label = position
        self.distance_label = distance
        self.tilt_label = tilt
        self.notes = notes.strip()

    def mark_interrupted(self, reason: str) -> None:
        self.interrupted_reason = reason.strip()

    def quality(self) -> EpisodeQuality:
        frame_count = self.frame_count
        if frame_count == 0:
            return EpisodeQuality(0, self._sequence_gap_count, self._duplicate_count, 0, 0, 0.0, 0.0)
        if len(self._receive_ns) > 1:
            duration = (self._receive_ns[-1] - self._receive_ns[0]) / 1e9
            receive_hz = (frame_count - 1) / duration if duration > 0 else 0.0
        else:
            receive_hz = 0.0
        return EpisodeQuality(
            frame_count=frame_count,
            sequence_gap_count=self._sequence_gap_count,
            duplicate_count=self._duplicate_count,
            invalid_position_count=self._invalid_position_count,
            stale_position_count=self._stale_position_count,
            valid_zone_ratio=self._valid_zone_count / float(frame_count * 64),
            receive_hz=receive_hz,
        )

    def snapshot(self, extra_metadata: dict[str, Any] | None = None) -> EpisodeSnapshot:
        """复制成与活动缓存解耦的保存快照，可安全交给后台线程。"""

        if not self._frames:
            raise ValueError("Episode没有可保存帧")
        arrays: dict[str, np.ndarray] = {
            "frame_seq": np.asarray([f.frame_seq for f in self._frames], dtype="<u4"),
            "timestamp_ms": np.asarray([f.timestamp_ms for f in self._frames], dtype="<u4"),
            "pc_receive_ns": np.asarray(self._receive_ns, dtype="<u8"),
            "distance_mm": np.stack([f.distance_mm for f in self._frames]),
            "target_status": np.stack([f.target_status for f in self._frames]),
            "target_count": np.stack([f.target_count for f in self._frames]),
            "signal_per_spad": np.stack([f.signal_per_spad for f in self._frames]),
            "ambient_per_spad": np.stack([f.ambient_per_spad for f in self._frames]),
            "reflectance": np.stack([f.reflectance for f in self._frames]),
            "range_sigma_mm": np.stack([f.range_sigma_mm for f in self._frames]),
            "cnh_raw": np.stack([f.cnh_raw for f in self._frames]),
            "cnh_scaler": np.stack([f.cnh_scaler for f in self._frames]),
            "tentacle": np.asarray([f.tentacle for f in self._frames], dtype="u1"),
            "valid_flags": np.asarray([f.valid_flags for f in self._frames], dtype="u1"),
            "status_age_ms": np.asarray([f.status_age_ms for f in self._frames], dtype="<u2"),
            "actual_q": np.stack([f.actual_q for f in self._frames]),
            "action_q": np.stack([f.action_q for f in self._frames]),
        }
        quality = self.quality()
        metadata: dict[str, Any] = {
            "format": "FlexibleTentacleACT raw episode",
            "format_version": 1,
            "protocol_version": 1,
            "act_frame_data_size": 4198,
            "created_utc": self.started_at.isoformat(),
            "saved_utc": datetime.now(timezone.utc).isoformat(),
            "tentacle": self.tentacle,
            "result": self.result,
            "labels": {
                "position": self.position_label,
                "distance": self.distance_label,
                "tilt": self.tilt_label,
            },
            "notes": self.notes,
            "interrupted_reason": self.interrupted_reason,
            "frame_count": quality.frame_count,
            "sequence_gap_count": quality.sequence_gap_count,
            "duplicate_count": quality.duplicate_count,
            "invalid_position_count": quality.invalid_position_count,
            "stale_position_count": quality.stale_position_count,
            "valid_zone_ratio": quality.valid_zone_ratio,
            "receive_hz": quality.receive_hz,
            "h7_sent_start": self.start_h7_sent,
            "h7_sent_end": self.end_h7_sent,
            "h7_drop_start": self.start_h7_drop,
            "h7_drop_end": self.end_h7_drop,
            "array_shapes": {name: list(value.shape) for name, value in arrays.items()},
            "array_dtypes": {name: str(value.dtype) for name, value in arrays.items()},
        }
        if extra_metadata:
            metadata.update(extra_metadata)
        return EpisodeSnapshot(arrays=arrays, metadata=metadata)

    def clear(self) -> None:
        self._frames.clear()
        self._receive_ns.clear()
        self._sequence_gap_count = 0
        self._duplicate_count = 0
        self._invalid_position_count = 0
        self._stale_position_count = 0
        self._valid_zone_count = 0


def save_episode(snapshot: EpisodeSnapshot, raw_root: Path) -> Path:
    """先写临时目录并完成同步，再原子改名为新的Episode目录。"""

    raw_root.mkdir(parents=True, exist_ok=True)
    existing = []
    for path in raw_root.glob("episode_[0-9][0-9][0-9][0-9][0-9][0-9]"):
        try:
            existing.append(int(path.name.removeprefix("episode_")))
        except ValueError:
            continue
    next_index = max(existing, default=0) + 1
    final_path = raw_root / f"episode_{next_index:06d}"
    while final_path.exists():
        next_index += 1
        final_path = raw_root / f"episode_{next_index:06d}"

    temporary = raw_root / f".{final_path.name}.tmp-{uuid4().hex}"
    temporary.mkdir(parents=False, exist_ok=False)
    try:
        np.savez_compressed(temporary / "raw_data.npz", **snapshot.arrays)
        metadata_path = temporary / "meta.json"
        metadata_path.write_text(
            json.dumps(snapshot.metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        for path in (temporary / "raw_data.npz", metadata_path):
            with path.open("rb+") as stream:
                os.fsync(stream.fileno())
        temporary.replace(final_path)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return final_path


def validate_training_compatibility(snapshot: EpisodeSnapshot) -> TrainingCompatibility:
    """按当前Tiny ToF-ACT读取器的字段、尺寸和窗口规则检查Episode。"""

    arrays = snapshot.arrays
    required = (
        "frame_seq",
        "timestamp_ms",
        "distance_mm",
        "target_status",
        "target_count",
        "signal_per_spad",
        "ambient_per_spad",
        "reflectance",
        "range_sigma_mm",
        "cnh_raw",
        "cnh_scaler",
        "valid_flags",
        "actual_q",
        "action_q",
    )
    errors: list[str] = []
    warnings: list[str] = []
    missing = [name for name in required if name not in arrays]
    if missing:
        errors.append(f"缺少字段: {', '.join(missing)}")
        return TrainingCompatibility(False, 0, 0, tuple(errors), tuple(warnings))

    frame_seq = np.asarray(arrays["frame_seq"])
    frame_count = int(frame_seq.size) if frame_seq.ndim == 1 else 0
    if frame_count == 0:
        errors.append("frame_seq必须是一维非空数组")
        return TrainingCompatibility(False, 0, 0, tuple(errors), tuple(warnings))

    expected_shapes = {
        "timestamp_ms": (frame_count,),
        "distance_mm": (frame_count, 64),
        "target_status": (frame_count, 64),
        "target_count": (frame_count, 64),
        "signal_per_spad": (frame_count, 64),
        "ambient_per_spad": (frame_count, 64),
        "reflectance": (frame_count, 64),
        "range_sigma_mm": (frame_count, 64),
        "cnh_raw": (frame_count, 10, 64),
        "cnh_scaler": (frame_count, 10, 64),
        "valid_flags": (frame_count,),
        "actual_q": (frame_count, 3),
        "action_q": (frame_count, 3),
    }
    for name, expected in expected_shapes.items():
        actual = np.asarray(arrays[name]).shape
        if actual != expected:
            errors.append(f"{name}尺寸应为{expected}，实际为{actual}")

    sequence_i64 = frame_seq.astype(np.int64, copy=False)
    if frame_seq.ndim != 1 or np.any(np.diff(sequence_i64) <= 0):
        errors.append("frame_seq必须严格递增且不能重复")

    scaler = np.asarray(arrays["cnh_scaler"])
    if np.any((scaler < 0) | (scaler > 30)):
        errors.append("cnh_scaler超出训练读取器允许范围0..30")

    if errors:
        return TrainingCompatibility(False, 0, frame_count, tuple(errors), tuple(warnings))

    # 与training.dataset.EpisodeWindowDataset保持一致：12帧历史、最多10帧未来动作。
    history_frames = 12
    action_chunk_frames = 10
    position_valid_flag = 0x01
    action_valid_flag = 0x02
    valid_flags = np.asarray(arrays["valid_flags"], dtype=np.uint8)
    valid_windows = 0
    for current in range(history_frames - 1, frame_count - 1):
        history_start = current - history_frames + 1
        future_end = min(current + 1 + action_chunk_frames, frame_count)
        if np.any(np.diff(sequence_i64[history_start:future_end]) != 1):
            continue
        if np.any((valid_flags[history_start : current + 1] & position_valid_flag) == 0):
            continue
        if not np.any((valid_flags[current + 1 : future_end] & action_valid_flag) != 0):
            continue
        valid_windows += 1

    if frame_count < history_frames + 1:
        warnings.append("帧数少于13，无法形成12帧历史加未来动作")
    elif valid_windows == 0:
        warnings.append("没有有效训练窗口，请检查frame_seq和POSITION/ACTION有效位")
    return TrainingCompatibility(True, valid_windows, frame_count, tuple(errors), tuple(warnings))


def save_validated_episode(
    snapshot: EpisodeSnapshot,
    raw_root: Path,
) -> tuple[Path, TrainingCompatibility]:
    """在保存线程中完成训练兼容性检查，并把报告写入meta.json。"""

    report = validate_training_compatibility(snapshot)
    snapshot.metadata["training_compatibility"] = report.as_dict()
    path = save_episode(snapshot, raw_root)
    return path, report
