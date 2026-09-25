"""ACT USART1 控制协议。

PC向H7发送以CRLF结束的ASCII命令；H7返回带长度和CRC32的二进制包。
本模块不依赖Qt，只负责命令编码、流式拆包和payload解码。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct
import zlib

import numpy as np


MAGIC = 0xA55A
MAGIC_BYTES = struct.pack("<H", MAGIC)
PROTOCOL_VERSION = 1
MAX_DATA_LENGTH = 8192

HEADER_STRUCT = struct.Struct("<HBBH")
CRC_STRUCT = struct.Struct("<I")
HEADER_SIZE = HEADER_STRUCT.size
CRC_SIZE = CRC_STRUCT.size
PACKET_OVERHEAD = HEADER_SIZE + CRC_SIZE

RESULT_STRUCT = struct.Struct("<BB")
STATUS_STRUCT = struct.Struct("<BBBBII")
ACT_MOTOR_STRUCT = struct.Struct("<BBHiiiiii")
ACT_FRAME_DATA_SIZE = 4198
ACT_FRAME_RESERVED_SIZE = 2
ACT_SENSOR_OFFSET = ACT_FRAME_RESERVED_SIZE
ACT_SENSOR_SIZE = 4168
ACT_MOTOR_OFFSET = ACT_SENSOR_OFFSET + ACT_SENSOR_SIZE
ACT_MOTOR_SIZE = 28
ACT_POSITION_VALID = 0x01
ACT_ACTION_VALID = 0x02
ACT_FAULT = 0x04


class PacketType(IntEnum):
    ACK = 1
    STATUS = 2
    ERROR = 3
    FRAME = 4


class CommandId(IntEnum):
    UNKNOWN = 0
    PING = 1
    SELECT = 2
    STATUS = 3
    ENABLE = 4
    MOVE = 5
    HOME = 6
    STOP = 7
    CAPTURE = 8


class ResultCode(IntEnum):
    OK = 0
    BAD_COMMAND = 1
    BAD_ARGUMENT = 2
    BUSY = 3
    NOT_READY = 4
    FAULT = 5
    DISABLED = 6
    TX_FAILED = 7


@dataclass(frozen=True, slots=True)
class Packet:
    packet_type: int
    version: int
    data: bytes


@dataclass(frozen=True, slots=True)
class CommandResult:
    command_id: int
    result: int


@dataclass(frozen=True, slots=True)
class ActMotorState:
    tentacle: int
    valid_flags: int
    status_age_ms: int
    actual_q: np.ndarray
    action_q: np.ndarray

    @property
    def position_valid(self) -> bool:
        return bool(self.valid_flags & ACT_POSITION_VALID)

    @property
    def action_valid(self) -> bool:
        return bool(self.valid_flags & ACT_ACTION_VALID)

    @property
    def fault(self) -> bool:
        return bool(self.valid_flags & ACT_FAULT)


@dataclass(frozen=True, slots=True)
class StatusPayload:
    selected_tentacle: int
    control_enabled: int
    capture_state: int
    sent_frame_count: int
    dropped_frame_count: int
    motor: ActMotorState | None = None


@dataclass(frozen=True, slots=True)
class ActFrame:
    """H7在一个ACT_FRAME中封装的同步ToF和电机数据。"""

    frame_seq: int
    timestamp_ms: int
    distance_mm: np.ndarray
    target_status: np.ndarray
    target_count: np.ndarray
    signal_per_spad: np.ndarray
    ambient_per_spad: np.ndarray
    reflectance: np.ndarray
    range_sigma_mm: np.ndarray
    cnh_raw: np.ndarray
    cnh_scaler: np.ndarray
    tentacle: int
    valid_flags: int
    status_age_ms: int
    actual_q: np.ndarray
    action_q: np.ndarray

    @property
    def position_valid(self) -> bool:
        return bool(self.valid_flags & ACT_POSITION_VALID)

    @property
    def action_valid(self) -> bool:
        return bool(self.valid_flags & ACT_ACTION_VALID)

    @property
    def fault(self) -> bool:
        return bool(self.valid_flags & ACT_FAULT)


@dataclass(slots=True)
class ParserStats:
    received_bytes: int = 0
    valid_packets: int = 0
    crc_errors: int = 0
    length_errors: int = 0
    version_errors: int = 0
    discarded_bytes: int = 0


class ProtocolError(ValueError):
    """命令或二进制payload不符合当前协议。"""


def crc32(data: bytes | bytearray | memoryview) -> int:
    """计算与H7 Act_Stream_Crc32一致的CRC-32/ISO-HDLC。"""

    return zlib.crc32(data) & 0xFFFFFFFF


def encode_text_command(command: str) -> bytes:
    """将一条不含换行的ACT命令编码为ASCII CRLF。"""

    normalized = command.strip()
    if not normalized:
        raise ProtocolError("命令不能为空")
    if "\r" in normalized or "\n" in normalized:
        raise ProtocolError("命令中不能包含换行")
    try:
        encoded = normalized.encode("ascii", errors="strict")
    except UnicodeEncodeError as exc:
        raise ProtocolError("ACT命令只能包含ASCII字符") from exc
    if len(encoded) > 46:
        raise ProtocolError("命令超过H7当前48字节行缓冲限制")
    return encoded + b"\r\n"


def encode_ping() -> bytes:
    return encode_text_command("ACT PING")


def encode_status_request() -> bytes:
    return encode_text_command("ACT STATUS")


def encode_select(tentacle_id: int) -> bytes:
    if not 1 <= tentacle_id <= 4:
        raise ProtocolError("tentacle_id必须位于1..4")
    return encode_text_command(f"ACT SELECT {tentacle_id}")


def encode_enable(enabled: bool) -> bytes:
    return encode_text_command(f"ACT ENABLE {1 if enabled else 0}")


def encode_move(x: int, y: int, bend: int, stiffness: int) -> bytes:
    if not -100 <= x <= 100 or not -100 <= y <= 100:
        raise ProtocolError("x和y必须位于-100..100")
    if not 0 <= bend <= 100 or not 0 <= stiffness <= 100:
        raise ProtocolError("bend和stiffness必须位于0..100")
    return encode_text_command(f"ACT MOVE {x} {y} {bend} {stiffness}")


def encode_home() -> bytes:
    return encode_text_command("ACT HOME")


def encode_stop() -> bytes:
    return encode_text_command("ACT STOP")


def encode_capture(enabled: bool) -> bytes:
    return encode_text_command(f"ACT CAPTURE {1 if enabled else 0}")


def encode_record(enabled: bool) -> bytes:
    """兼容旧调用名；发送的始终是H7当前CAPTURE命令。"""

    return encode_capture(enabled)


def decode_command_result(data: bytes) -> CommandResult:
    if len(data) != RESULT_STRUCT.size:
        raise ProtocolError(f"ACK/ERROR数据应为2字节，实际为{len(data)}字节")
    return CommandResult(*RESULT_STRUCT.unpack(data))


def decode_status(data: bytes) -> StatusPayload:
    """解码STATUS；兼容旧12字节包和带28字节实际姿态的新40字节包。

    后续H7扩展ONLINE/READY/FAULT时，应按data_length兼容解析，不能修改
    前12字节的现有顺序。
    """

    if len(data) < STATUS_STRUCT.size:
        raise ProtocolError(f"STATUS数据至少应为12字节，实际为{len(data)}字节")
    selected, control, capture, _reserved, sent, dropped = STATUS_STRUCT.unpack_from(data)
    motor = None
    if len(data) != STATUS_STRUCT.size:
        if len(data) < STATUS_STRUCT.size + ACT_MOTOR_SIZE:
            raise ProtocolError(f"STATUS实际姿态字段不完整：{len(data)}字节")
        motor = decode_act_motor(data[STATUS_STRUCT.size : STATUS_STRUCT.size + ACT_MOTOR_SIZE])
    return StatusPayload(selected, control, capture, sent, dropped, motor)


def decode_act_motor(data: bytes) -> ActMotorState:
    """解码H7固定28字节电机实际姿态摘要。"""

    if len(data) != ACT_MOTOR_SIZE:
        raise ProtocolError(f"ACT电机数据应为{ACT_MOTOR_SIZE}字节，实际为{len(data)}字节")
    tentacle, valid_flags, status_age_ms, *values = ACT_MOTOR_STRUCT.unpack(data)
    if not 1 <= tentacle <= 4:
        raise ProtocolError(f"ACT电机数据触手编号无效：{tentacle}")
    return ActMotorState(
        tentacle=tentacle,
        valid_flags=valid_flags,
        status_age_ms=status_age_ms,
        actual_q=np.asarray(values[:3], dtype=np.int32),
        action_q=np.asarray(values[3:], dtype=np.int32),
    )


def decode_act_frame(data: bytes) -> ActFrame:
    """按当前H7固定布局解码4198字节ACT_FRAME数据区。

    返回数组均为独立副本，串口解析缓存释放后仍可安全保存和显示。
    """

    if len(data) != ACT_FRAME_DATA_SIZE:
        raise ProtocolError(
            f"ACT_FRAME数据应为{ACT_FRAME_DATA_SIZE}字节，实际为{len(data)}字节"
        )
    reserved = struct.unpack_from("<H", data, 0)[0]
    if reserved != 0:
        raise ProtocolError(f"ACT_FRAME保留字段应为0，实际为{reserved}")

    offset = ACT_SENSOR_OFFSET
    frame_seq, timestamp_ms = struct.unpack_from("<II", data, offset)
    offset += 8

    def take(dtype: str, count: int) -> np.ndarray:
        nonlocal offset
        array = np.frombuffer(data, dtype=np.dtype(dtype), count=count, offset=offset).copy()
        offset += array.nbytes
        return array

    distance_mm = take("<i2", 64)
    target_status = take("u1", 64)
    target_count = take("u1", 64)
    signal_per_spad = take("<u4", 64)
    ambient_per_spad = take("<u4", 64)
    reflectance = take("u1", 64)
    range_sigma_mm = take("<u2", 64)
    cnh_raw = take("<i4", 10 * 64).reshape(10, 64)
    cnh_scaler = take("i1", 10 * 64).reshape(10, 64)
    if offset != ACT_MOTOR_OFFSET:
        raise ProtocolError(f"ACT_FRAME传感器字段结束偏移异常：{offset}")

    motor = decode_act_motor(data[offset : offset + ACT_MOTOR_SIZE])
    offset += ACT_MOTOR_SIZE
    if offset != ACT_FRAME_DATA_SIZE:
        raise ProtocolError(f"ACT_FRAME电机字段结束偏移异常：{offset}")
    return ActFrame(
        frame_seq=frame_seq,
        timestamp_ms=timestamp_ms,
        distance_mm=distance_mm,
        target_status=target_status,
        target_count=target_count,
        signal_per_spad=signal_per_spad,
        ambient_per_spad=ambient_per_spad,
        reflectance=reflectance,
        range_sigma_mm=range_sigma_mm,
        cnh_raw=cnh_raw,
        cnh_scaler=cnh_scaler,
        tentacle=motor.tentacle,
        valid_flags=motor.valid_flags,
        status_age_ms=motor.status_age_ms,
        actual_q=motor.actual_q,
        action_q=motor.action_q,
    )


def encode_binary_packet(packet_type: int, data: bytes = b"", version: int = 1) -> bytes:
    """生成H7回复格式，供单元测试和独立模拟器使用。"""

    if not 0 <= packet_type <= 0xFF:
        raise ProtocolError("packet_type必须位于0..255")
    if not 0 <= version <= 0xFF:
        raise ProtocolError("version必须位于0..255")
    if len(data) > MAX_DATA_LENGTH:
        raise ProtocolError("data超过协议最大长度")
    body = HEADER_STRUCT.pack(MAGIC, packet_type, version, len(data)) + data
    return body + CRC_STRUCT.pack(crc32(body))


class PacketParser:
    """H7二进制回复的增量流式解析器。"""

    def __init__(self, max_data_length: int = MAX_DATA_LENGTH) -> None:
        if not 0 <= max_data_length <= 0xFFFF:
            raise ValueError("max_data_length必须位于0..65535")
        self.max_data_length = max_data_length
        self._buffer = bytearray()
        self.stats = ParserStats()

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def clear(self) -> None:
        self._buffer.clear()

    def feed(self, chunk: bytes | bytearray | memoryview) -> list[Packet]:
        if chunk:
            self._buffer.extend(chunk)
            self.stats.received_bytes += len(chunk)

        packets: list[Packet] = []
        while True:
            magic_index = self._buffer.find(MAGIC_BYTES)
            if magic_index < 0:
                self._keep_possible_magic_prefix()
                break
            if magic_index > 0:
                del self._buffer[:magic_index]
                self.stats.discarded_bytes += magic_index

            if len(self._buffer) < HEADER_SIZE:
                break

            magic, packet_type, version, data_length = HEADER_STRUCT.unpack_from(self._buffer)
            if magic != MAGIC:
                self._discard_candidate_byte()
                continue
            if version != PROTOCOL_VERSION:
                self.stats.version_errors += 1
                self._discard_candidate_byte()
                continue
            if data_length > self.max_data_length:
                self.stats.length_errors += 1
                self._discard_candidate_byte()
                continue

            packet_size = PACKET_OVERHEAD + data_length
            if len(self._buffer) < packet_size:
                break

            body_size = HEADER_SIZE + data_length
            expected_crc = CRC_STRUCT.unpack_from(self._buffer, body_size)[0]
            actual_crc = crc32(memoryview(self._buffer)[:body_size])
            if expected_crc != actual_crc:
                self.stats.crc_errors += 1
                self._discard_candidate_byte()
                continue

            data = bytes(self._buffer[HEADER_SIZE:body_size])
            del self._buffer[:packet_size]
            packets.append(Packet(packet_type, version, data))
            self.stats.valid_packets += 1

        return packets

    def _discard_candidate_byte(self) -> None:
        del self._buffer[0]
        self.stats.discarded_bytes += 1

    def _keep_possible_magic_prefix(self) -> None:
        keep = 1 if self._buffer.endswith(MAGIC_BYTES[:1]) else 0
        discard = len(self._buffer) - keep
        if discard > 0:
            del self._buffer[:discard]
            self.stats.discarded_bytes += discard


assert HEADER_SIZE == 6
assert PACKET_OVERHEAD == 10
assert RESULT_STRUCT.size == 2
assert STATUS_STRUCT.size == 12
assert ACT_MOTOR_STRUCT.size == ACT_MOTOR_SIZE
assert ACT_MOTOR_OFFSET == 4170
assert ACT_MOTOR_OFFSET + ACT_MOTOR_SIZE == ACT_FRAME_DATA_SIZE
