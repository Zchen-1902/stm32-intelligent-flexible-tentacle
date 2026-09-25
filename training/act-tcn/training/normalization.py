"""只使用训练集episode计算并保存Tiny ToF-ACT归一化统计量。"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from training.config import (
    ACTION_VALID_FLAG,
    MOTOR_COUNT,
    NORMALIZATION_EPS,
    POSITION_VALID_FLAG,
    TOF_CHANNELS,
    TOF_CHANNEL_NAMES,
)
from training.preprocess import build_tof_features, load_episode


@dataclass(frozen=True, slots=True)
class NormalizationStats:
    """训练集ToF、实际位置和动作目标的mean/std。"""

    tof_mean: np.ndarray
    tof_std: np.ndarray
    state_mean: np.ndarray
    state_std: np.ndarray
    action_mean: np.ndarray
    action_std: np.ndarray

    def save(self, path: str | Path) -> None:
        """将统计量保存为可由训练代码和H7部署工具共同读取的JSON。"""

        payload = {
            "version": 1,
            "tof_channel_names": list(TOF_CHANNEL_NAMES),
            "tof_mean": self.tof_mean.tolist(),
            "tof_std": self.tof_std.tolist(),
            "state_mean": self.state_mean.tolist(),
            "state_std": self.state_std.tolist(),
            "action_mean": self.action_mean.tolist(),
            "action_std": self.action_std.tolist(),
        }
        Path(path).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    @classmethod
    def load(cls, path: str | Path) -> "NormalizationStats":
        """读取并检查一份已有的stats.json。"""

        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("version") != 1:
            raise ValueError("不支持的stats.json版本")
        stats = cls(
            tof_mean=np.asarray(payload["tof_mean"], dtype=np.float32),
            tof_std=np.asarray(payload["tof_std"], dtype=np.float32),
            state_mean=np.asarray(payload["state_mean"], dtype=np.float32),
            state_std=np.asarray(payload["state_std"], dtype=np.float32),
            action_mean=np.asarray(payload["action_mean"], dtype=np.float32),
            action_std=np.asarray(payload["action_std"], dtype=np.float32),
        )
        _validate_stats(stats)
        return stats


def _finish_stats(total: np.ndarray, square: np.ndarray, count: int) -> tuple[np.ndarray, np.ndarray]:
    """由流式sum/square-sum得到稳定的float32 mean/std。"""

    if count <= 0:
        raise ValueError("没有可用于归一化统计的有效数据")
    mean = total / float(count)
    variance = np.maximum(square / float(count) - mean * mean, 0.0)
    std = np.sqrt(variance)
    std = np.where(std < NORMALIZATION_EPS, 1.0, std)
    return mean.astype(np.float32), std.astype(np.float32)


def fit_normalization(episode_paths: list[str | Path]) -> NormalizationStats:
    """仅遍历训练集episode计算统计量，不将验证/测试数据泄漏进来。

    Args:
        episode_paths: 已完成训练集划分后的NPZ路径列表。

    Returns:
        可直接传给Dataset并保存到stats.json的统计量。
    """

    if not episode_paths:
        raise ValueError("训练集episode不能为空")

    tof_sum = np.zeros(TOF_CHANNELS, dtype=np.float64)
    tof_square = np.zeros(TOF_CHANNELS, dtype=np.float64)
    state_sum = np.zeros(MOTOR_COUNT, dtype=np.float64)
    state_square = np.zeros(MOTOR_COUNT, dtype=np.float64)
    action_sum = np.zeros(MOTOR_COUNT, dtype=np.float64)
    action_square = np.zeros(MOTOR_COUNT, dtype=np.float64)
    tof_count = 0
    state_count = 0
    action_count = 0

    for path in episode_paths:
        episode = load_episode(path)
        tof = build_tof_features(episode).astype(np.float64, copy=False)
        tof_sum += tof.sum(axis=(0, 2, 3))
        tof_square += np.square(tof).sum(axis=(0, 2, 3))
        tof_count += tof.shape[0] * tof.shape[2] * tof.shape[3]

        flags = episode["valid_flags"].astype(np.uint8, copy=False)
        state = episode["actual_q"][(flags & POSITION_VALID_FLAG) != 0].astype(np.float64)
        action = episode["action_q"][(flags & ACTION_VALID_FLAG) != 0].astype(np.float64)
        if state.size:
            state_sum += state.sum(axis=0)
            state_square += np.square(state).sum(axis=0)
            state_count += state.shape[0]
        if action.size:
            action_sum += action.sum(axis=0)
            action_square += np.square(action).sum(axis=0)
            action_count += action.shape[0]

    tof_mean, tof_std = _finish_stats(tof_sum, tof_square, tof_count)
    state_mean, state_std = _finish_stats(state_sum, state_square, state_count)
    action_mean, action_std = _finish_stats(action_sum, action_square, action_count)
    stats = NormalizationStats(
        tof_mean,
        tof_std,
        state_mean,
        state_std,
        action_mean,
        action_std,
    )
    _validate_stats(stats)
    return stats


def _validate_stats(stats: NormalizationStats) -> None:
    """检查统计量形状、有限性和标准差合法性。"""

    expected = (
        (stats.tof_mean, (TOF_CHANNELS,)),
        (stats.tof_std, (TOF_CHANNELS,)),
        (stats.state_mean, (MOTOR_COUNT,)),
        (stats.state_std, (MOTOR_COUNT,)),
        (stats.action_mean, (MOTOR_COUNT,)),
        (stats.action_std, (MOTOR_COUNT,)),
    )
    for values, shape in expected:
        if values.shape != shape or not np.isfinite(values).all():
            raise ValueError(f"归一化统计量尺寸或数值无效，应为{shape}")
    if np.any(stats.tof_std <= 0) or np.any(stats.state_std <= 0) or np.any(stats.action_std <= 0):
        raise ValueError("归一化标准差必须大于0")


def normalize_tof(values: np.ndarray, stats: NormalizationStats) -> np.ndarray:
    """按通道归一化``[...,18,8,8]`` ToF输入。"""

    return (
        (values.astype(np.float32) - stats.tof_mean[None, :, None, None])
        / stats.tof_std[None, :, None, None]
    ).astype(np.float32)


def normalize_state(values: np.ndarray, stats: NormalizationStats) -> np.ndarray:
    """归一化末维为3的电机实际位置。"""

    return ((values.astype(np.float32) - stats.state_mean) / stats.state_std).astype(np.float32)


def normalize_action(values: np.ndarray, stats: NormalizationStats) -> np.ndarray:
    """归一化末维为3的动作目标。"""

    return ((values.astype(np.float32) - stats.action_mean) / stats.action_std).astype(np.float32)


def denormalize_state(values: np.ndarray, stats: NormalizationStats) -> np.ndarray:
    """把归一化电机状态还原为0.01rad机械角。"""

    return values * stats.state_std + stats.state_mean


def denormalize_action(values: np.ndarray, stats: NormalizationStats) -> np.ndarray:
    """把归一化动作还原为0.01rad机械角。"""

    return values * stats.action_std + stats.action_mean
