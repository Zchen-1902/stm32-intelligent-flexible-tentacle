"""按完整episode生成Tiny ToF-ACT时序训练样本。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from training.config import (
    ACTION_CHUNK_FRAMES,
    ACTION_VALID_FLAG,
    HISTORY_FRAMES,
    POSITION_VALID_FLAG,
)
from training.preprocess import build_tof_features, load_episode
from training.normalization import (
    NormalizationStats,
    normalize_action,
    normalize_state,
    normalize_tof,
)


@dataclass(frozen=True, slots=True)
class WindowIndex:
    """一个训练窗口在某条episode中的位置。"""

    episode_index: int
    current_frame: int


class EpisodeWindowDataset:
    """将多条episode转换为12帧观测到10帧未来动作的样本集。

    该类只依赖NumPy，但实现了``__len__``和``__getitem__``，后续可直接
    交给PyTorch DataLoader。每次只缓存最近读取的一条episode，避免一次
    把完整数据集加载到内存。
    """

    def __init__(
        self,
        episode_paths: list[str | Path],
        stats: NormalizationStats | None = None,
    ) -> None:
        if not episode_paths:
            raise ValueError("episode_paths不能为空")

        self.episode_paths = [Path(path) for path in episode_paths]
        self.stats = stats
        self.windows: list[WindowIndex] = []
        self._cache_index = -1
        self._cache_episode: dict[str, np.ndarray] | None = None
        self._cache_tof: np.ndarray | None = None

        for episode_index, path in enumerate(self.episode_paths):
            episode = load_episode(path)
            self._append_valid_windows(episode_index, episode)

        if not self.windows:
            raise ValueError("没有可用训练窗口，请检查帧数、frame_seq和valid_flags")

    def _append_valid_windows(
        self,
        episode_index: int,
        episode: dict[str, np.ndarray],
    ) -> None:
        """建立一条episode的窗口索引，不复制大型ToF数组。"""

        frame_seq = episode["frame_seq"].astype(np.int64, copy=False)
        valid_flags = episode["valid_flags"].astype(np.uint8, copy=False)
        frame_count = frame_seq.size

        for current in range(HISTORY_FRAMES - 1, frame_count - 1):
            history_start = current - HISTORY_FRAMES + 1
            future_end = min(current + 1 + ACTION_CHUNK_FRAMES, frame_count)
            sequence = frame_seq[history_start:future_end]

            if np.any(np.diff(sequence) != 1):
                continue
            history_flags = valid_flags[history_start : current + 1]
            if np.any((history_flags & POSITION_VALID_FLAG) == 0):
                continue
            future_flags = valid_flags[current + 1 : future_end]
            if not np.any((future_flags & ACTION_VALID_FLAG) != 0):
                continue
            self.windows.append(WindowIndex(episode_index, current))

    def __len__(self) -> int:
        return len(self.windows)

    def __getitem__(self, item: int) -> dict[str, np.ndarray]:
        index = self.windows[item]
        episode, tof = self._get_episode(index.episode_index)
        current = index.current_frame
        history_start = current - HISTORY_FRAMES + 1
        future_start = current + 1
        future_end = min(future_start + ACTION_CHUNK_FRAMES, episode["frame_seq"].size)
        valid_future = future_end - future_start

        action_chunk = np.empty((ACTION_CHUNK_FRAMES, 3), dtype=np.float32)
        action_chunk[:valid_future] = episode["action_q"][future_start:future_end]
        action_chunk[valid_future:] = episode["action_q"][future_end - 1]

        padding_mask = np.ones(ACTION_CHUNK_FRAMES, dtype=np.bool_)
        future_flags = episode["valid_flags"][future_start:future_end]
        padding_mask[:valid_future] = (future_flags & ACTION_VALID_FLAG) == 0

        tof_history = tof[history_start : current + 1].copy()
        state_history = episode["actual_q"][history_start : current + 1].astype(
            np.float32, copy=True
        )
        if self.stats is not None:
            tof_history = normalize_tof(tof_history, self.stats)
            state_history = normalize_state(state_history, self.stats)
            action_chunk = normalize_action(action_chunk, self.stats)

        return {
            "tof_history": tof_history,
            "state_history": state_history,
            "action_chunk": action_chunk,
            "padding_mask": padding_mask,
            "sample_seq": np.int64(episode["frame_seq"][current]),
        }

    def _get_episode(self, episode_index: int) -> tuple[dict[str, np.ndarray], np.ndarray]:
        """读取并缓存最近访问的一条episode。"""

        if episode_index != self._cache_index:
            episode = load_episode(self.episode_paths[episode_index])
            self._cache_episode = episode
            self._cache_tof = build_tof_features(episode)
            self._cache_index = episode_index

        assert self._cache_episode is not None
        assert self._cache_tof is not None
        return self._cache_episode, self._cache_tof
