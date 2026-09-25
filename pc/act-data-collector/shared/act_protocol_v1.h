#ifndef ACT_PROTOCOL_V1_H
#define ACT_PROTOCOL_V1_H

/**
 * @file act_protocol_v1.h
 * @brief ACT USART1第一阶段H7二进制回复格式。
 *
 * PC发送端为ASCII文本命令。本头文件只描述H7到PC的小型二进制回复，
 * 与H743VIT6_ACT/Core/act_stream/act_stream.c保持一致。
 */

#include <stdint.h>

#define ACT_PROTOCOL_MAGIC             0xA55AU
#define ACT_PROTOCOL_VERSION           1U
#define ACT_PROTOCOL_HEADER_SIZE       6U
#define ACT_PROTOCOL_CRC_SIZE          4U

typedef enum
{
  ACT_PROTOCOL_PACKET_ACK = 1U,
  ACT_PROTOCOL_PACKET_STATUS = 2U,
  ACT_PROTOCOL_PACKET_ERROR = 3U
} ActProtocolPacketType_t;

typedef enum
{
  ACT_PROTOCOL_COMMAND_UNKNOWN = 0U,
  ACT_PROTOCOL_COMMAND_PING = 1U,
  ACT_PROTOCOL_COMMAND_SELECT = 2U,
  ACT_PROTOCOL_COMMAND_STATUS = 3U
} ActProtocolCommandId_t;

typedef enum
{
  ACT_PROTOCOL_RESULT_OK = 0U,
  ACT_PROTOCOL_RESULT_BAD_COMMAND = 1U,
  ACT_PROTOCOL_RESULT_BAD_ARGUMENT = 2U
} ActProtocolResult_t;

#if defined(__GNUC__)
#define ACT_PROTOCOL_PACKED __attribute__((packed))
#else
#define ACT_PROTOCOL_PACKED
#pragma pack(push, 1)
#endif

typedef struct ACT_PROTOCOL_PACKED
{
  uint16_t magic_le;
  uint8_t packet_type;
  uint8_t version;
  uint16_t data_length_le;
} ActProtocolHeaderV1_t;

typedef struct ACT_PROTOCOL_PACKED
{
  uint8_t command_id;
  uint8_t result;
} ActProtocolResultV1_t;

typedef struct ACT_PROTOCOL_PACKED
{
  uint8_t selected_tentacle;
  uint8_t control_enabled;
  uint8_t capture_state;
  uint8_t reserved;
  uint32_t sent_frame_count_le;
  uint32_t dropped_frame_count_le;
} ActProtocolStatusV1_t;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(ActProtocolHeaderV1_t) == 6U, "ACT header size mismatch");
_Static_assert(sizeof(ActProtocolResultV1_t) == 2U, "ACT result size mismatch");
_Static_assert(sizeof(ActProtocolStatusV1_t) == 12U, "ACT status size mismatch");
#endif

#endif /* ACT_PROTOCOL_V1_H */

