"""将H7保存的原始episode转换为Tiny ToF-ACT输入。"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import numpy as np

from training.config import (
    CNH_BINS,
    GRID_SIZE,
    MOTOR_COUNT,
    REQUIRED_EPISODE_FIELDS,
    VALID_TARGET_STATUS,
    ZONE_COUNT,
)


def load_episode(path: str | Path) -> dict[str, np.ndarray]:
    """读取一个NPZ episode，并复制为不依赖文件句柄的数组字典。

    Args:
        path: 上位机保存的单个``.npz`` episode路径。

    Returns:
        字段名到NumPy数组的映射。

    Raises:
        ValueError: 文件缺少字段、数组尺寸错误或帧序号不递增。
    """

    with np.load(Path(path), allow_pickle=False) as source:
        episode = {name: source[name].copy() for name in source.files}
    validate_episode(episode)
    return episode


def validate_episode(episode: Mapping[str, np.ndarray]) -> int:
    """检查一个episode能否用于后续预处理。

    Args:
        episode: 包含H7原始传感器和电机字段的数组映射。

    Returns:
        episode中的帧数。

    Note:
        允许``frame_seq``跳号，以保留串口丢帧信息；Dataset会跳过跨越
        缺帧位置的训练窗口，但这里要求序号严格递增且不允许重复。
    """

    missing = [name for name in REQUIRED_EPISODE_FIELDS if name not in episode]
    if missing:
        raise ValueError(f"episode缺少字段: {', '.join(missing)}")

    frame_seq = np.asarray(episode["frame_seq"])
    if frame_seq.ndim != 1 or frame_seq.size == 0:
        raise ValueError("frame_seq必须是一维非空数组")
    frame_count = int(frame_seq.size)

    expected_shapes = {
        "timestamp_ms": (frame_count,),
        "distance_mm": (frame_count, ZONE_COUNT),
        "target_status": (frame_count, ZONE_COUNT),
        "target_count": (frame_count, ZONE_COUNT),
        "signal_per_spad": (frame_count, ZONE_COUNT),
        "ambient_per_spad": (frame_count, ZONE_COUNT),
        "reflectance": (frame_count, ZONE_COUNT),
        "range_sigma_mm": (frame_count, ZONE_COUNT),
        "cnh_raw": (frame_count, CNH_BINS, ZONE_COUNT),
        "cnh_scaler": (frame_count, CNH_BINS, ZONE_COUNT),
        "valid_flags": (frame_count,),
        "actual_q": (frame_count, MOTOR_COUNT),
        "action_q": (frame_count, MOTOR_COUNT),
    }
    for name, expected in expected_shapes.items():
        actual = np.asarray(episode[name]).shape
        if actual != expected:
            raise ValueError(f"{name}尺寸应为{expected}，实际为{actual}")

    if np.any(np.diff(frame_seq.astype(np.int64)) <= 0):
        raise ValueError("frame_seq必须严格递增且不能重复")
    return frame_count


def build_tof_features(episode: Mapping[str, np.ndarray]) -> np.ndarray:
    """把原始VL53字段转换为18通道8x8浮点特征。

    Args:
        episode: 已通过``validate_episode``检查的episode。

    Returns:
        ``float32[N,18,8,8]``。这里只恢复物理数值，不做训练集
        mean/std归一化，避免验证集信息泄漏到训练统计量。

    Note:
        10个CNH通道按照ST示例还原为``raw / (2 << scaler)``。
    """

    frame_count = validate_episode(episode)
    features = np.empty((frame_count, 18, ZONE_COUNT), dtype=np.float32)

    distance = np.asarray(episode["distance_mm"], dtype=np.float32)
    status = np.asarray(episode["target_status"])
    target_count = np.asarray(episode["target_count"])
    valid = np.isin(status, VALID_TARGET_STATUS) & (target_count > 0) & (distance > 0)

    features[:, 0] = np.where(valid, distance, 0.0)
    features[:, 1] = valid.astype(np.float32)
    features[:, 2] = status.astype(np.float32)
    features[:, 3] = target_count.astype(np.float32)
    features[:, 4] = np.asarray(episode["signal_per_spad"], dtype=np.float32)
    features[:, 5] = np.asarray(episode["ambient_per_spad"], dtype=np.float32)
    features[:, 6] = np.asarray(episode["reflectance"], dtype=np.float32)
    features[:, 7] = np.asarray(episode["range_sigma_mm"], dtype=np.float32)

    cnh_raw = np.asarray(episode["cnh_raw"], dtype=np.float32)
    cnh_scaler = np.asarray(episode["cnh_scaler"], dtype=np.int16)
    if np.any((cnh_scaler < 0) | (cnh_scaler > 30)):
        raise ValueError("cnh_scaler超出允许范围0..30")
    features[:, 8:18] = np.ldexp(cnh_raw, -(cnh_scaler + 1))

    return features.reshape(frame_count, 18, GRID_SIZE, GRID_SIZE)
