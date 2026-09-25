from __future__ import annotations

import json
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np

from pc_collector.episode_buffer import (
    EpisodeBuffer,
    save_episode,
    save_validated_episode,
    validate_training_compatibility,
)
from pc_collector.protocol import (
    ACT_FRAME_DATA_SIZE,
    PacketParser,
    PacketType,
    decode_act_frame,
    encode_binary_packet,
)
from training.preprocess import load_episode


def make_frame_data(sequence: int = 123, tentacle: int = 2) -> bytes:
    data = bytearray(ACT_FRAME_DATA_SIZE)
    struct.pack_into("<HII", data, 0, 0, sequence, 4567)
    offset = 10

    def put(dtype: str, values) -> None:
        nonlocal offset
        array = np.asarray(values, dtype=np.dtype(dtype))
        data[offset : offset + array.nbytes] = array.tobytes()
        offset += array.nbytes

    put("<i2", np.arange(64) + 200)
    put("u1", np.full(64, 5))
    put("u1", np.ones(64))
    put("<u4", np.arange(64) + 1000)
    put("<u4", np.arange(64) + 2000)
    put("u1", np.arange(64))
    put("<u2", np.arange(64) + 10)
    put("<i4", np.arange(640) - 320)
    put("i1", np.tile(np.arange(0, 10), 64))
    assert offset == 4170
    struct.pack_into("<BBH", data, offset, tentacle, 0x03, 25)
    offset += 4
    put("<i4", (100, 200, 300))
    put("<i4", (110, 210, 310))
    assert offset == ACT_FRAME_DATA_SIZE
    return bytes(data)


class ActFrameProtocolTests(unittest.TestCase):
    def test_decode_current_h7_layout(self) -> None:
        frame = decode_act_frame(make_frame_data())
        self.assertEqual(frame.frame_seq, 123)
        self.assertEqual(frame.timestamp_ms, 4567)
        self.assertEqual(frame.tentacle, 2)
        self.assertTrue(frame.position_valid)
        self.assertTrue(frame.action_valid)
        self.assertEqual(frame.distance_mm.shape, (64,))
        self.assertEqual(frame.cnh_raw.shape, (10, 64))
        self.assertEqual(frame.cnh_scaler.shape, (10, 64))
        np.testing.assert_array_equal(frame.actual_q, (100, 200, 300))

    def test_large_frame_fragmentation_and_sticky_packet(self) -> None:
        encoded = encode_binary_packet(PacketType.FRAME, make_frame_data())
        for split in (0, 1, 2, 5, 6, 7, 31, 512, 2048, len(encoded) - 1):
            parser = PacketParser()
            packets = parser.feed(encoded[:split])
            packets += parser.feed(encoded[split:] + encoded)
            self.assertEqual(len(packets), 2)
            self.assertEqual(len(packets[0].data), ACT_FRAME_DATA_SIZE)


class EpisodeBufferTests(unittest.TestCase):
    def test_sequence_quality_and_atomic_save(self) -> None:
        episode = EpisodeBuffer(tentacle=2)
        self.assertTrue(episode.append(decode_act_frame(make_frame_data(100))))
        self.assertTrue(episode.append(decode_act_frame(make_frame_data(102))))
        self.assertFalse(episode.append(decode_act_frame(make_frame_data(102))))
        quality = episode.quality()
        self.assertEqual(quality.frame_count, 2)
        self.assertEqual(quality.sequence_gap_count, 1)
        self.assertEqual(quality.duplicate_count, 1)
        self.assertEqual(quality.invalid_position_count, 0)
        self.assertAlmostEqual(quality.valid_zone_ratio, 1.0)

        episode.set_annotation("success", "左侧", "近", "竖直", "unit test")
        with tempfile.TemporaryDirectory() as temporary:
            path = save_episode(episode.snapshot(), Path(temporary))
            self.assertEqual(path.name, "episode_000001")
            self.assertTrue((path / "raw_data.npz").is_file())
            metadata = json.loads((path / "meta.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["frame_count"], 2)
            self.assertEqual(metadata["sequence_gap_count"], 1)
            with np.load(path / "raw_data.npz") as arrays:
                self.assertIn("timestamp_ms", arrays.files)
                self.assertNotIn("h7_timestamp_ms", arrays.files)
                self.assertEqual(arrays["distance_mm"].shape, (2, 64))
                self.assertEqual(arrays["cnh_raw"].shape, (2, 10, 64))
                self.assertEqual(arrays["actual_q"].shape, (2, 3))

    def test_saved_episode_is_readable_by_training_pipeline(self) -> None:
        episode = EpisodeBuffer(tentacle=2)
        for sequence in range(100, 122):
            episode.append(decode_act_frame(make_frame_data(sequence, 2)))
        episode.set_annotation("success", "左侧", "近", "竖直", "training compatibility")
        snapshot = episode.snapshot()
        report = validate_training_compatibility(snapshot)
        self.assertTrue(report.schema_valid)
        self.assertEqual(report.valid_window_count, 10)

        with tempfile.TemporaryDirectory() as temporary:
            path, saved_report = save_validated_episode(snapshot, Path(temporary))
            loaded = load_episode(path / "raw_data.npz")
            self.assertEqual(loaded["frame_seq"].shape, (22,))
            self.assertEqual(saved_report.valid_window_count, 10)
            metadata = json.loads((path / "meta.json").read_text(encoding="utf-8"))
            self.assertTrue(metadata["training_compatibility"]["schema_valid"])
            self.assertEqual(
                metadata["training_compatibility"]["valid_window_count"], 10
            )

    def test_variable_episode_lengths(self) -> None:
        for frame_count, expected_windows in ((5, 0), (13, 1), (22, 10), (47, 35)):
            episode = EpisodeBuffer(tentacle=1)
            for sequence in range(1000, 1000 + frame_count):
                episode.append(decode_act_frame(make_frame_data(sequence, 1)))
            report = validate_training_compatibility(episode.snapshot())
            self.assertTrue(report.schema_valid)
            self.assertEqual(report.frame_count, frame_count)
            self.assertEqual(report.valid_window_count, expected_windows)


if __name__ == "__main__":
    unittest.main()
