from __future__ import annotations

import struct
import unittest

from pc_collector.protocol import (
    CommandId,
    MAGIC_BYTES,
    PacketParser,
    PacketType,
    ProtocolError,
    ResultCode,
    STATUS_STRUCT,
    ACT_MOTOR_STRUCT,
    crc32,
    decode_command_result,
    decode_status,
    encode_binary_packet,
    encode_capture,
    encode_enable,
    encode_home,
    encode_move,
    encode_ping,
    encode_select,
    encode_stop,
    encode_status_request,
    encode_text_command,
)


class CommandEncodingTests(unittest.TestCase):
    def test_current_commands(self) -> None:
        self.assertEqual(encode_ping(), b"ACT PING\r\n")
        self.assertEqual(encode_status_request(), b"ACT STATUS\r\n")
        self.assertEqual(encode_select(4), b"ACT SELECT 4\r\n")
        self.assertEqual(encode_enable(True), b"ACT ENABLE 1\r\n")
        self.assertEqual(encode_enable(False), b"ACT ENABLE 0\r\n")
        self.assertEqual(encode_move(-100, 100, 82, 35), b"ACT MOVE -100 100 82 35\r\n")
        self.assertEqual(encode_home(), b"ACT HOME\r\n")
        self.assertEqual(encode_stop(), b"ACT STOP\r\n")
        self.assertEqual(encode_capture(True), b"ACT CAPTURE 1\r\n")
        self.assertEqual(encode_capture(False), b"ACT CAPTURE 0\r\n")

    def test_h7_stage_two_ids_and_results(self) -> None:
        self.assertEqual(CommandId.ENABLE, 4)
        self.assertEqual(CommandId.MOVE, 5)
        self.assertEqual(CommandId.HOME, 6)
        self.assertEqual(CommandId.STOP, 7)
        self.assertEqual(CommandId.CAPTURE, 8)
        self.assertEqual(PacketType.FRAME, 4)
        self.assertEqual(ResultCode.BUSY, 3)
        self.assertEqual(ResultCode.NOT_READY, 4)
        self.assertEqual(ResultCode.FAULT, 5)
        self.assertEqual(ResultCode.DISABLED, 6)
        self.assertEqual(ResultCode.TX_FAILED, 7)

    def test_command_validation(self) -> None:
        with self.assertRaises(ProtocolError):
            encode_select(0)
        with self.assertRaises(ProtocolError):
            encode_text_command("ACT PING\nACT STATUS")
        with self.assertRaises(ProtocolError):
            encode_move(101, 0, 0, 0)

    def test_crc_reference(self) -> None:
        self.assertEqual(crc32(b"123456789"), 0xCBF43926)


class PacketParserTests(unittest.TestCase):
    def setUp(self) -> None:
        self.ack = encode_binary_packet(
            PacketType.ACK,
            bytes((CommandId.PING, ResultCode.OK)),
        )

    def test_header_matches_h7_layout(self) -> None:
        self.assertEqual(self.ack[:2], MAGIC_BYTES)
        self.assertEqual(self.ack[:6], bytes.fromhex("5A A5 01 01 02 00"))
        self.assertEqual(struct.unpack_from("<I", self.ack, 8)[0], crc32(self.ack[:8]))

    def test_every_split_point(self) -> None:
        for split in range(len(self.ack)):
            parser = PacketParser()
            self.assertEqual(parser.feed(self.ack[:split]), [])
            packets = parser.feed(self.ack[split:])
            self.assertEqual(len(packets), 1)
            self.assertEqual(packets[0].packet_type, PacketType.ACK)

    def test_1000_fragmented_ping_acks(self) -> None:
        stream = self.ack * 1000
        parser = PacketParser()
        packets = []
        cursor = 0
        sizes = (1, 2, 5, 13, 3, 31, 7)
        index = 0
        while cursor < len(stream):
            size = sizes[index % len(sizes)]
            packets.extend(parser.feed(stream[cursor : cursor + size]))
            cursor += size
            index += 1
        self.assertEqual(len(packets), 1000)
        self.assertEqual(parser.stats.crc_errors, 0)

    def test_sticky_packets(self) -> None:
        status_data = STATUS_STRUCT.pack(3, 0, 0, 0, 12, 2)
        status = encode_binary_packet(PacketType.STATUS, status_data)
        packets = PacketParser().feed(self.ack + status)
        self.assertEqual([packet.packet_type for packet in packets], [1, 2])

    def test_bad_crc_and_garbage_resynchronize(self) -> None:
        bad = bytearray(self.ack)
        bad[-1] ^= 0x40
        parser = PacketParser()
        packets = parser.feed(b"noise" + bad + self.ack)
        self.assertEqual(len(packets), 1)
        self.assertEqual(parser.stats.crc_errors, 1)

    def test_invalid_length_resynchronizes(self) -> None:
        invalid = struct.pack("<HBBH", 0xA55A, 1, 1, 9000)
        parser = PacketParser()
        packets = parser.feed(invalid + self.ack)
        self.assertEqual(len(packets), 1)
        self.assertEqual(parser.stats.length_errors, 1)


class PayloadTests(unittest.TestCase):
    def test_ack(self) -> None:
        result = decode_command_result(bytes((CommandId.SELECT, ResultCode.OK)))
        self.assertEqual(result.command_id, CommandId.SELECT)
        self.assertEqual(result.result, ResultCode.OK)

    def test_status(self) -> None:
        status = decode_status(STATUS_STRUCT.pack(2, 0, 0, 0, 100, 4))
        self.assertEqual(status.selected_tentacle, 2)
        self.assertEqual(status.sent_frame_count, 100)
        self.assertEqual(status.dropped_frame_count, 4)
        self.assertIsNone(status.motor)

    def test_extended_status_contains_live_motor_pose(self) -> None:
        motor = ACT_MOTOR_STRUCT.pack(2, 0x03, 17, -100, 250, 900, 10, 20, 30)
        status = decode_status(STATUS_STRUCT.pack(2, 1, 0, 0, 100, 4) + motor)
        self.assertIsNotNone(status.motor)
        assert status.motor is not None
        self.assertEqual(status.motor.tentacle, 2)
        self.assertTrue(status.motor.position_valid)
        self.assertEqual(status.motor.status_age_ms, 17)
        self.assertEqual(status.motor.actual_q.tolist(), [-100, 250, 900])


if __name__ == "__main__":
    unittest.main()
