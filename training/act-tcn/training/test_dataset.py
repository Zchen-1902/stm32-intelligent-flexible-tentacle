"""数据接口层的最小回归测试。"""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from training.dataset import EpisodeWindowDataset
from training.preprocess import build_tof_features, load_episode


def make_episode(frame_count: int = 15) -> dict[str, np.ndarray]:
    """生成字段和尺寸均与H7协议一致的模拟episode。"""

    zones = 64
    bins = 10
    frame_seq = np.arange(100, 100 + frame_count, dtype=np.uint32)
    action = np.arange(frame_count * 3, dtype=np.int32).reshape(frame_count, 3)
    return {
        "frame_seq": frame_seq,
        "timestamp_ms": np.arange(frame_count, dtype=np.uint32) * 62,
        "distance_mm": np.full((frame_count, zones), 250, dtype=np.int16),
        "target_status": np.full((frame_count, zones), 5, dtype=np.uint8),
        "target_count": np.ones((frame_count, zones), dtype=np.uint8),
        "signal_per_spad": np.full((frame_count, zones), 1000, dtype=np.uint32),
        "ambient_per_spad": np.full((frame_count, zones), 100, dtype=np.uint32),
        "reflectance": np.full((frame_count, zones), 20, dtype=np.uint8),
        "range_sigma_mm": np.full((frame_count, zones), 5, dtype=np.uint16),
        "cnh_raw": np.full((frame_count, bins, zones), 8, dtype=np.int32),
        "cnh_scaler": np.full((frame_count, bins, zones), 2, dtype=np.int8),
        "valid_flags": np.full(frame_count, 0x03, dtype=np.uint8),
        "actual_q": action - 1,
        "action_q": action,
    }


class DataInterfaceTest(unittest.TestCase):
    def test_features_and_future_window(self) -> None:
        with TemporaryDirectory() as directory:
            path = Path(directory) / "episode_000001.npz"
            np.savez_compressed(path, **make_episode())

            episode = load_episode(path)
            features = build_tof_features(episode)
            self.assertEqual(features.shape, (15, 18, 8, 8))
            self.assertTrue(np.all(features[:, 1] == 1.0))
            self.assertTrue(np.all(features[:, 8:] == 1.0))

            dataset = EpisodeWindowDataset([path])
            sample = dataset[0]
            self.assertEqual(sample["tof_history"].shape, (12, 18, 8, 8))
            self.assertEqual(sample["state_history"].shape, (12, 3))
            self.assertEqual(sample["action_chunk"].shape, (10, 3))
            self.assertEqual(sample["sample_seq"], 111)

            expected = episode["action_q"][12:15]
            np.testing.assert_array_equal(sample["action_chunk"][:3], expected)
            np.testing.assert_array_equal(
                sample["action_chunk"][3:],
                np.repeat(expected[-1][None, :], 7, axis=0),
            )
            np.testing.assert_array_equal(
                sample["padding_mask"],
                np.array([False, False, False] + [True] * 7),
            )

    def test_gap_is_not_used_as_continuous_time(self) -> None:
        with TemporaryDirectory() as directory:
            episode = make_episode(36)
            episode["frame_seq"][12:] += 1
            path = Path(directory) / "episode_gap.npz"
            np.savez_compressed(path, **episode)

            dataset = EpisodeWindowDataset([path])
            for window in dataset.windows:
                start = window.current_frame - 11
                end = min(window.current_frame + 11, 36)
                seq = episode["frame_seq"][start:end]
                self.assertTrue(np.all(np.diff(seq.astype(np.int64)) == 1))


if __name__ == "__main__":
    unittest.main()
