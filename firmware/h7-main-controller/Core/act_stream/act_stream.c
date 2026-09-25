#include "act_stream.h"

#include "act_policy_app.h"
#include "motor_control_app.h"
#include "usart.h"
#include "vl53_app.h"

#include <stdio.h>
#include <string.h>

/* USART1接收使用循环DMA；命令很短，256字节足以吸收短时主循环延迟。 */
#define ACT_STREAM_RX_DMA_SIZE       256U
#define ACT_STREAM_LINE_SIZE         48U
#define ACT_STREAM_TX_BUFFER_SIZE    64U   /* ACK/ERROR/STATUS小包专用缓冲区。 */
#define ACT_STREAM_FRAME_SENSOR_SIZE 4168U /* VL53_App_ActFrame_t固定协议尺寸。 */
#define ACT_STREAM_FRAME_MOTOR_SIZE  28U   /* MotorControl_ActData_t固定协议尺寸。 */
#define ACT_STREAM_FRAME_DATA_SIZE   4198U /* 保留2B + 传感器4168B + 电机28B。 */
#define ACT_STREAM_FRAME_PACKET_SIZE 4208U /* 包头6B + 数据4198B + CRC32。 */
#define ACT_STREAM_FRAME_TX_SIZE     4224U /* 向上取整到32B，满足H7 DCache要求。 */
#define ACT_STREAM_TX_TIMEOUT_MS     100U
#define ACT_STREAM_RX_RETRY_MS       100U
#define ACT_STREAM_MOVE_PERIOD_MS    50U  /* MOVE最高20Hz提交，旧目标不排队。 */

/* H7发送给PC的二进制包头固定信息。多字节数据均按STM32的小端顺序发送。 */
#define ACT_STREAM_PACKET_MAGIC      0xA55AU
#define ACT_STREAM_PROTOCOL_VERSION  1U

typedef enum
{
  ACT_STREAM_PACKET_ACK = 1U,       /* 命令执行成功。 */
  ACT_STREAM_PACKET_STATUS = 2U,    /* ACT模块当前状态。 */
  ACT_STREAM_PACKET_ERROR = 3U,     /* 命令格式或参数错误。 */
  ACT_STREAM_PACKET_FRAME = 4U      /* 一帧VL53与所选触手电机数据。 */
} Act_StreamPacketType_t;

typedef enum
{
  ACT_STREAM_COMMAND_UNKNOWN = 0U,
  ACT_STREAM_COMMAND_PING = 1U,
  ACT_STREAM_COMMAND_SELECT = 2U,
  ACT_STREAM_COMMAND_STATUS = 3U,
  ACT_STREAM_COMMAND_ENABLE = 4U,
  ACT_STREAM_COMMAND_MOVE = 5U,
  ACT_STREAM_COMMAND_HOME = 6U,
  ACT_STREAM_COMMAND_STOP = 7U,
  ACT_STREAM_COMMAND_CAPTURE = 8U
} Act_StreamCommand_t;

typedef enum
{
  ACT_STREAM_RESULT_OK = 0U,
  ACT_STREAM_RESULT_BAD_COMMAND = 1U,
  ACT_STREAM_RESULT_BAD_ARGUMENT = 2U,
  ACT_STREAM_RESULT_BUSY = 3U,
  ACT_STREAM_RESULT_NOT_READY = 4U,
  ACT_STREAM_RESULT_FAULT = 5U,
  ACT_STREAM_RESULT_DISABLED = 6U,
  ACT_STREAM_RESULT_TX_FAILED = 7U
} Act_StreamResult_t;

/* 只保存最新上位机目标，避免串口输入速度高于CAN速度时积压旧动作。 */
typedef struct
{
  int16_t x;
  int16_t y;
  uint8_t bend;
  uint8_t stiffness;
  uint8_t pending;
  uint32_t last_submit_ms;
} Act_StreamMove_t;

/* 命令响应仅保留一条；协议采用请求-应答，上位机应收到响应后再发下一条。 */
typedef struct
{
  uint8_t pending;
  Act_StreamCommand_t command;
  Act_StreamResult_t result;
} Act_StreamPendingResult_t;

typedef char Act_StreamSensorSizeCheck[
  (sizeof(VL53_App_ActFrame_t) == ACT_STREAM_FRAME_SENSOR_SIZE) ? 1 : -1];
typedef char Act_StreamMotorSizeCheck[
  (sizeof(MotorControl_ActData_t) == ACT_STREAM_FRAME_MOTOR_SIZE) ? 1 : -1];

ALIGN_32BYTES(static uint8_t s_act_rx_dma[ACT_STREAM_RX_DMA_SIZE]);
ALIGN_32BYTES(static uint8_t s_act_tx_dma[ACT_STREAM_TX_BUFFER_SIZE]);
ALIGN_32BYTES(static uint8_t s_act_frame_tx_dma[ACT_STREAM_FRAME_TX_SIZE]);

static Act_StreamStatus_t s_act_status;
static char s_act_line[ACT_STREAM_LINE_SIZE];
static uint16_t s_act_rx_tail;
static uint8_t s_act_line_length;
static uint8_t s_act_line_overflow;
static volatile uint8_t s_act_tx_busy;
static volatile uint8_t s_act_tx_is_frame;
static volatile uint8_t s_act_uart_recover_pending;
static uint32_t s_act_tx_start_ms;
static uint32_t s_act_rx_last_retry_ms;
static Act_StreamMove_t s_act_move;
static Act_StreamPendingResult_t s_act_pending_result;
static uint8_t s_act_status_pending;
static uint32_t s_act_last_frame_seq;
/* STOP未成功入队时保持控制锁，禁止UART7运动命令提前接管。 */
static uint8_t s_act_stop_hold;

/**
 * @brief 清除尚未提交的ACT运动目标。
 */
static void Act_Stream_ClearMove(void)
{
  memset(&s_act_move, 0, sizeof(s_act_move));
}

/**
 * @brief 开启当前所选触手的ACT控制。
 * @return 命令结果码。
 * @note 先完成全部检查，再切换工作组；失败时不会启用电机。
 */
static Act_StreamResult_t Act_Stream_EnableControl(void)
{
  const MotorControl_Status_t *motor_status;
  MotorControl_Summary_t summary;
  uint8_t mask;
  uint8_t status;

  if (s_act_stop_hold != 0U)
  {
    return ACT_STREAM_RESULT_TX_FAILED;
  }
  if (ActPolicyApp_GetRunState() != ACT_POLICY_APP_STATE_OFF)
  {
    return ACT_STREAM_RESULT_BUSY;
  }
  if (s_act_status.control_enabled != 0U)
  {
    return ACT_STREAM_RESULT_OK;
  }

  motor_status = MotorControl_GetStatus();
  if (motor_status->enable != 0U)
  {
    return ACT_STREAM_RESULT_BUSY;
  }

  mask = (uint8_t)(1U << (s_act_status.selected_tentacle - 1U));
  MotorControl_GetSummary(&summary);
  if ((summary.fault_mask & mask) != 0U)
  {
    return ACT_STREAM_RESULT_FAULT;
  }
  if (((summary.online_mask & mask) == 0U) ||
      ((summary.ready_mask & mask) == 0U))
  {
    return ACT_STREAM_RESULT_NOT_READY;
  }

  status = MotorControl_SelectWork(mask, 0U);
  if (status == 1U)
  {
    return ACT_STREAM_RESULT_BAD_ARGUMENT;
  }
  if ((status == 2U) || (status == 3U))
  {
    return ACT_STREAM_RESULT_NOT_READY;
  }
  if (status != 0U)
  {
    return ACT_STREAM_RESULT_TX_FAILED;
  }

  status = MotorControl_SetControlSource(MOTOR_CONTROL_SOURCE_PC);
  if (status != 0U)
  {
    return ACT_STREAM_RESULT_TX_FAILED;
  }

  MotorControl_SetEnable(1U);
  motor_status = MotorControl_GetStatus();
  if (motor_status->enable == 0U)
  {
    MotorControl_GetSummary(&summary);
    return ((summary.fault_mask & mask) != 0U) ?
           ACT_STREAM_RESULT_FAULT : ACT_STREAM_RESULT_NOT_READY;
  }

  Act_Stream_ClearMove();
  s_act_move.last_submit_ms = HAL_GetTick();
  s_act_stop_hold = 0U;
  s_act_status.control_enabled = 1U;
  return ACT_STREAM_RESULT_OK;
}

/**
 * @brief 正常关闭ACT控制。
 * @return 命令结果码。
 * @note STOP发送失败时保持控制锁，等待PC再次关闭或发送ACT STOP。
 */
static Act_StreamResult_t Act_Stream_DisableControl(void)
{
  uint8_t status;

  if ((s_act_status.control_enabled == 0U) &&
      (s_act_stop_hold == 0U))
  {
    return ACT_STREAM_RESULT_OK;
  }

  status = MotorControl_StopActive();
  Act_Stream_ClearMove();
  if (status != 0U)
  {
    s_act_stop_hold = 1U;
    return ACT_STREAM_RESULT_TX_FAILED;
  }

  s_act_stop_hold = 0U;
  s_act_status.control_enabled = 0U;
  return ACT_STREAM_RESULT_OK;
}

/**
 * @brief 解析并缓存ACT MOVE命令。
 * @param line 完整命令行。
 * @return 命令结果码。
 */
static Act_StreamResult_t Act_Stream_SetLatestMove(const char *line)
{
  const MotorControl_Status_t *motor_status;
  long x;
  long y;
  long bend;
  long stiffness;
  char extra;

  if (sscanf(line, "ACT MOVE %ld %ld %ld %ld %c",
             &x, &y, &bend, &stiffness, &extra) != 4)
  {
    return ACT_STREAM_RESULT_BAD_ARGUMENT;
  }
  if ((x < -100L) || (x > 100L) ||
      (y < -100L) || (y > 100L) ||
      (bend < 0L) || (bend > 100L) ||
      (stiffness < 0L) || (stiffness > 100L))
  {
    return ACT_STREAM_RESULT_BAD_ARGUMENT;
  }

  if (s_act_stop_hold != 0U)
  {
    return ACT_STREAM_RESULT_TX_FAILED;
  }

  motor_status = MotorControl_GetStatus();
  if ((s_act_status.control_enabled == 0U) ||
      (motor_status->enable == 0U) ||
      (motor_status->control_source != MOTOR_CONTROL_SOURCE_PC))
  {
    s_act_status.control_enabled = 0U;
    s_act_stop_hold = 0U;
    Act_Stream_ClearMove();
    return ACT_STREAM_RESULT_DISABLED;
  }

  s_act_move.x = (int16_t)x;
  s_act_move.y = (int16_t)y;
  s_act_move.bend = (uint8_t)bend;
  s_act_move.stiffness = (uint8_t)stiffness;
  s_act_move.pending = 1U;
  return ACT_STREAM_RESULT_OK;
}

/**
 * @brief 按20Hz上限提交最新MOVE目标。
 * @param now_ms 当前HAL毫秒时间。
 */
static void Act_Stream_SubmitMove(uint32_t now_ms)
{
  if ((s_act_status.control_enabled == 0U) ||
      (s_act_move.pending == 0U) ||
      ((uint32_t)(now_ms - s_act_move.last_submit_ms) <
       ACT_STREAM_MOVE_PERIOD_MS))
  {
    return;
  }

  s_act_move.pending = 0U;
  s_act_move.last_submit_ms = now_ms;
  MotorControl_SetVirtualInput(s_act_move.x,
                               s_act_move.y,
                               s_act_move.bend,
                               s_act_move.stiffness);
}

/**
 * @brief 计算标准CRC32，供PC检查二进制包是否完整。
 * @param data 需要计算的数据。
 * @param length 数据字节数。
 * @return CRC32计算结果。
 */
static uint32_t Act_Stream_Crc32(const uint8_t *data, uint16_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 1UL) != 0UL) ?
            ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }

  return ~crc;
}

/**
 * @brief 清理指定TX缓冲区的DCache，使DMA读取到CPU刚写入的数据。
 * @param data 32字节对齐的DMA发送缓冲区。
 * @param length 本次需要发送的字节数。
 * @note 长度自动向上补齐到32字节；调用方必须保证缓冲区容量足够。
 */
static void Act_Stream_CleanTxDCache(uint8_t *data, uint16_t length)
{
  uint16_t cache_length = (uint16_t)((length + 31U) & ~31U);

  SCB_CleanDCache_by_Addr((uint32_t *)data, (int32_t)cache_length);
}

/**
 * @brief 失效整个RX DMA缓冲区的DCache，使CPU读取到DMA最新数据。
 * @note RX缓冲区专供DMA写入，且大小为32字节整数倍，不会影响其他变量。
 */
static void Act_Stream_InvalidateRxDCache(void)
{
  SCB_InvalidateDCache_by_Addr((uint32_t *)s_act_rx_dma,
                              (int32_t)ACT_STREAM_RX_DMA_SIZE);
}

/**
 * @brief 启动USART1循环接收DMA。
 * @return 0表示成功，非0表示启动失败。
 * @note 每次重启RX DMA后，旧缓冲数据全部丢弃并从位置0重新接收。
 */
static uint8_t Act_Stream_StartRxDma(void)
{
  memset(s_act_rx_dma, 0, sizeof(s_act_rx_dma));
  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)s_act_rx_dma,
                                   (int32_t)ACT_STREAM_RX_DMA_SIZE);
  s_act_rx_tail = 0U;

  if (HAL_UART_Receive_DMA(&huart1,
                           s_act_rx_dma,
                           ACT_STREAM_RX_DMA_SIZE) != HAL_OK)
  {
    return 1U;
  }

  /* 主循环通过DMA写入位置取数据，不需要半传输中断。 */
  if (huart1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }

  return 0U;
}

/**
 * @brief 组装一个二进制包并通过USART1 DMA发送。
 * @param type 数据包类型。
 * @param data 有效数据；data_length为0时允许传入NULL。
 * @param data_length 有效数据长度。
 * @return 0表示开始发送，非0表示串口忙或参数非法。
 * @note 第一阶段采用请求-应答，上位机收到响应后再发下一条命令。
 */
static uint8_t Act_Stream_SendPacket(Act_StreamPacketType_t type,
                                    const uint8_t *data,
                                    uint16_t data_length)
{
  uint8_t *cursor = s_act_tx_dma;
  uint16_t magic = ACT_STREAM_PACKET_MAGIC;
  uint16_t packet_without_crc;
  uint16_t total_length;
  uint32_t crc;

  total_length = (uint16_t)(2U + 1U + 1U + 2U + data_length + 4U);
  if ((s_act_tx_busy != 0U) ||
      (total_length > ACT_STREAM_TX_BUFFER_SIZE) ||
      ((data_length > 0U) && (data == NULL)))
  {
    return 1U;
  }

  /* 包格式：magic、类型、版本、数据长度、有效数据、CRC32。 */
  memcpy(cursor, &magic, sizeof(magic));
  cursor += sizeof(magic);
  *cursor++ = (uint8_t)type;
  *cursor++ = ACT_STREAM_PROTOCOL_VERSION;
  memcpy(cursor, &data_length, sizeof(data_length));
  cursor += sizeof(data_length);

  if (data_length > 0U)
  {
    memcpy(cursor, data, data_length);
    cursor += data_length;
  }

  packet_without_crc = (uint16_t)(cursor - s_act_tx_dma);
  crc = Act_Stream_Crc32(s_act_tx_dma, packet_without_crc);
  memcpy(cursor, &crc, sizeof(crc));

  Act_Stream_CleanTxDCache(s_act_tx_dma, total_length);
  s_act_tx_busy = 1U;
  s_act_tx_is_frame = 0U;
  s_act_tx_start_ms = HAL_GetTick();

  if (HAL_UART_Transmit_DMA(&huart1, s_act_tx_dma, total_length) != HAL_OK)
  {
    s_act_tx_busy = 0U;
    return 1U;
  }

  return 0U;
}

/**
 * @brief 组装并发送一帧ACT训练数据。
 * @param sensor 本次VL53完整帧。
 * @param motor 与该VL53帧配对的所选触手电机数据。
 * @return 0表示DMA已启动，非0表示串口忙或参数无效。
 * @note 数据区前2字节保留为0，使传感器结构从包偏移8开始并保持4字节对齐。
 */
static uint8_t Act_Stream_SendFrame(const VL53_App_ActFrame_t *sensor,
                                    const MotorControl_ActData_t *motor)
{
  uint8_t *cursor = s_act_frame_tx_dma;
  uint16_t magic = ACT_STREAM_PACKET_MAGIC;
  uint16_t data_length = ACT_STREAM_FRAME_DATA_SIZE;
  uint16_t reserved = 0U;
  uint32_t crc;

  if ((sensor == NULL) || (motor == NULL) || (s_act_tx_busy != 0U))
  {
    return 1U;
  }

  memcpy(cursor, &magic, sizeof(magic));
  cursor += sizeof(magic);
  *cursor++ = (uint8_t)ACT_STREAM_PACKET_FRAME;
  *cursor++ = ACT_STREAM_PROTOCOL_VERSION;
  memcpy(cursor, &data_length, sizeof(data_length));
  cursor += sizeof(data_length);
  memcpy(cursor, &reserved, sizeof(reserved));
  cursor += sizeof(reserved);
  memcpy(cursor, sensor, sizeof(*sensor));
  cursor += sizeof(*sensor);
  memcpy(cursor, motor, sizeof(*motor));
  cursor += sizeof(*motor);

  crc = Act_Stream_Crc32(s_act_frame_tx_dma,
                         (uint16_t)(cursor - s_act_frame_tx_dma));
  memcpy(cursor, &crc, sizeof(crc));
  cursor += sizeof(crc);
  if ((uint16_t)(cursor - s_act_frame_tx_dma) !=
      ACT_STREAM_FRAME_PACKET_SIZE)
  {
    return 2U;
  }

  Act_Stream_CleanTxDCache(s_act_frame_tx_dma,
                           ACT_STREAM_FRAME_PACKET_SIZE);
  s_act_tx_busy = 1U;
  s_act_tx_is_frame = 1U;
  s_act_tx_start_ms = HAL_GetTick();
  if (HAL_UART_Transmit_DMA(&huart1,
                            s_act_frame_tx_dma,
                            ACT_STREAM_FRAME_PACKET_SIZE) != HAL_OK)
  {
    s_act_tx_busy = 0U;
    s_act_tx_is_frame = 0U;
    return 3U;
  }

  return 0U;
}

/**
 * @brief 返回命令执行结果。
 * @param command 对应的命令编号。
 * @param result 0表示成功，其余值表示错误原因。
 */
static void Act_Stream_SendResult(Act_StreamCommand_t command,
                                  Act_StreamResult_t result)
{
  uint8_t data[2];

  data[0] = (uint8_t)command;
  data[1] = (uint8_t)result;
  if (Act_Stream_SendPacket((result == ACT_STREAM_RESULT_OK) ?
                            ACT_STREAM_PACKET_ACK :
                            ACT_STREAM_PACKET_ERROR,
                            data,
                            sizeof(data)) != 0U)
  {
    s_act_pending_result.pending = 1U;
    s_act_pending_result.command = command;
    s_act_pending_result.result = result;
  }
}

/**
 * @brief 返回ACT模块状态摘要。
 * @note 前12字节保持旧STATUS长度；第4字节由保留位改为ACT运行状态。
 *       后续仍追加28字节所选触手实际姿态，旧字段偏移不变。
 */
static void Act_Stream_SendStatus(void)
{
  uint8_t data[12U + sizeof(MotorControl_ActData_t)];
  uint8_t *cursor = data;
  MotorControl_ActData_t motor;

  *cursor++ = s_act_status.selected_tentacle;
  *cursor++ = s_act_status.control_enabled;
  *cursor++ = (uint8_t)s_act_status.capture_state;
  /* 复用原保留字节返回自主ACT状态，不改变旧STATUS长度和后续偏移。 */
  *cursor++ = (uint8_t)ActPolicyApp_GetRunState();
  memcpy(cursor, &s_act_status.sent_frame_count,
         sizeof(s_act_status.sent_frame_count));
  cursor += sizeof(s_act_status.sent_frame_count);
  memcpy(cursor, &s_act_status.dropped_frame_count,
         sizeof(s_act_status.dropped_frame_count));
  cursor += sizeof(s_act_status.dropped_frame_count);
  MotorControl_CopyActData(s_act_status.selected_tentacle, &motor);
  memcpy(cursor, &motor, sizeof(motor));

  if (Act_Stream_SendPacket(ACT_STREAM_PACKET_STATUS,
                            data,
                            sizeof(data)) != 0U)
  {
    s_act_status_pending = 1U;
  }
}

/**
 * @brief DMA空闲后补发一条此前因串口忙而延迟的命令结果。
 * @note 只保留最近一条结果；PC应遵守请求-应答顺序，避免连续堆命令。
 */
static void Act_Stream_TrySendPendingResult(void)
{
  uint8_t data[2];
  Act_StreamPacketType_t type;

  if ((s_act_pending_result.pending == 0U) || (s_act_tx_busy != 0U))
  {
    return;
  }

  data[0] = (uint8_t)s_act_pending_result.command;
  data[1] = (uint8_t)s_act_pending_result.result;
  type = (s_act_pending_result.result == ACT_STREAM_RESULT_OK) ?
         ACT_STREAM_PACKET_ACK : ACT_STREAM_PACKET_ERROR;
  if (Act_Stream_SendPacket(type, data, sizeof(data)) == 0U)
  {
    s_act_pending_result.pending = 0U;
  }
}

/**
 * @brief DMA空闲后补发一次ACT STATUS响应。
 * @note 状态值在真正发送时重新读取，因此不会返回已经过期的计数。
 */
static void Act_Stream_TrySendPendingStatus(void)
{
  if ((s_act_status_pending != 0U) &&
      (s_act_tx_busy == 0U) &&
      (s_act_pending_result.pending == 0U))
  {
    s_act_status_pending = 0U;
    Act_Stream_SendStatus();
  }
}

/**
 * @brief 开启或关闭ACT帧采集。
 * @param enable 非0开启，0关闭。
 * @return 命令执行结果；传感器未就绪时不能开启。
 * @note 重复开启不会清零计数，防止串口重发命令切断同一episode。
 */
static Act_StreamResult_t Act_Stream_SetCapture(uint8_t enable)
{
  const VL53_App_ActFrame_t *frame;

  if (enable == 0U)
  {
    s_act_status.capture_state = ACT_STREAM_CAPTURE_IDLE;
    return ACT_STREAM_RESULT_OK;
  }

  if (s_act_status.capture_state == ACT_STREAM_CAPTURE_RUNNING)
  {
    return ACT_STREAM_RESULT_OK;
  }
  if (ActPolicyApp_GetRunState() != ACT_POLICY_APP_STATE_OFF)
  {
    return ACT_STREAM_RESULT_BUSY;
  }
  if (VL53_App_IsReady() == 0U)
  {
    return ACT_STREAM_RESULT_NOT_READY;
  }

  frame = VL53_App_GetLatestActFrame();
  s_act_last_frame_seq = (frame != NULL) ? frame->frame_seq : 0U;
  s_act_status.sent_frame_count = 0U;
  s_act_status.dropped_frame_count = 0U;
  s_act_status.capture_state = ACT_STREAM_CAPTURE_RUNNING;
  return ACT_STREAM_RESULT_OK;
}

/**
 * @brief USART1空闲时发送最新一帧VL53和电机数据。
 * @note 不建立帧队列；若期间产生多帧，只发送最新帧并准确累计跳过数量。
 */
static void Act_Stream_TrySendCaptureFrame(void)
{
  const VL53_App_ActFrame_t *sensor;
  MotorControl_ActData_t motor;
  uint32_t sequence_delta;

  if ((s_act_status.capture_state != ACT_STREAM_CAPTURE_RUNNING) ||
      (s_act_tx_busy != 0U) ||
      (s_act_pending_result.pending != 0U) ||
      (s_act_status_pending != 0U))
  {
    return;
  }

  sensor = VL53_App_GetLatestActFrame();
  if ((sensor == NULL) ||
      (sensor->frame_seq == 0U) ||
      (sensor->frame_seq == s_act_last_frame_seq))
  {
    return;
  }

  MotorControl_CopyActData(s_act_status.selected_tentacle, &motor);
  sequence_delta = sensor->frame_seq - s_act_last_frame_seq;
  if (Act_Stream_SendFrame(sensor, &motor) == 0U)
  {
    if (sequence_delta > 1U)
    {
      s_act_status.dropped_frame_count += sequence_delta - 1U;
    }
    s_act_last_frame_seq = sensor->frame_seq;
  }
}

/**
 * @brief 解析一条以换行结束的ACT文本命令。
 * @param line 不包含回车换行符的命令字符串。
 */
static void Act_Stream_HandleLine(const char *line)
{
  Act_StreamResult_t result;

  if (strcmp(line, "ACT PING") == 0)
  {
    Act_Stream_SendResult(ACT_STREAM_COMMAND_PING, ACT_STREAM_RESULT_OK);
    return;
  }

  if (strcmp(line, "ACT STATUS") == 0)
  {
    Act_Stream_SendStatus();
    return;
  }

  if (strncmp(line, "ACT SELECT ", 11U) == 0)
  {
    if ((Act_Stream_HasControl() != 0U) ||
        (s_act_status.capture_state == ACT_STREAM_CAPTURE_RUNNING))
    {
      Act_Stream_SendResult(ACT_STREAM_COMMAND_SELECT,
                            ACT_STREAM_RESULT_BUSY);
    }
    else if ((line[11] >= '1') && (line[11] <= '4') &&
             (line[12] == '\0'))
    {
      s_act_status.selected_tentacle = (uint8_t)(line[11] - '0');
      Act_Stream_SendResult(ACT_STREAM_COMMAND_SELECT, ACT_STREAM_RESULT_OK);
    }
    else
    {
      Act_Stream_SendResult(ACT_STREAM_COMMAND_SELECT,
                            ACT_STREAM_RESULT_BAD_ARGUMENT);
    }
    return;
  }

  if (strcmp(line, "ACT CAPTURE 1") == 0)
  {
    result = Act_Stream_SetCapture(1U);
    Act_Stream_SendResult(ACT_STREAM_COMMAND_CAPTURE, result);
    return;
  }

  if (strcmp(line, "ACT CAPTURE 0") == 0)
  {
    result = Act_Stream_SetCapture(0U);
    Act_Stream_SendResult(ACT_STREAM_COMMAND_CAPTURE, result);
    return;
  }

  if (strcmp(line, "ACT ENABLE 1") == 0)
  {
    result = Act_Stream_EnableControl();
    Act_Stream_SendResult(ACT_STREAM_COMMAND_ENABLE, result);
    return;
  }

  if (strcmp(line, "ACT ENABLE 0") == 0)
  {
    result = Act_Stream_DisableControl();
    Act_Stream_SendResult(ACT_STREAM_COMMAND_ENABLE, result);
    return;
  }

  if (strncmp(line, "ACT MOVE ", 9U) == 0)
  {
    result = Act_Stream_SetLatestMove(line);
    /* 合法MOVE不逐帧ACK；参数或状态异常时才通知上位机。 */
    if (result != ACT_STREAM_RESULT_OK)
    {
      Act_Stream_SendResult(ACT_STREAM_COMMAND_MOVE, result);
    }
    return;
  }

  if (strcmp(line, "ACT HOME") == 0)
  {
    if (s_act_stop_hold != 0U)
    {
      result = ACT_STREAM_RESULT_TX_FAILED;
    }
    else if (s_act_status.control_enabled == 0U)
    {
      result = ACT_STREAM_RESULT_DISABLED;
    }
    else
    {
      uint8_t status = MotorControl_HomeActive();

      result = (status == 0U) ? ACT_STREAM_RESULT_OK :
               (((status == 2U) || (status == 3U)) ?
                ACT_STREAM_RESULT_NOT_READY :
                ACT_STREAM_RESULT_TX_FAILED);
      if (result == ACT_STREAM_RESULT_OK)
      {
        Act_Stream_ClearMove();
      }
    }
    Act_Stream_SendResult(ACT_STREAM_COMMAND_HOME, result);
    return;
  }

  if (strcmp(line, "ACT STOP") == 0)
  {
    uint8_t status;

    /* 先急停，再清ACT状态，避免同一命令连续发送两轮STOP。 */
    status = MotorControl_EmergencyStop();
    (void)ActPolicyApp_SetRun(0U, s_act_status.selected_tentacle);

    Act_Stream_ClearMove();
    if (status == 0U)
    {
      s_act_stop_hold = 0U;
      s_act_status.control_enabled = 0U;
      result = ACT_STREAM_RESULT_OK;
    }
    else
    {
      /* 首次STOP未入队时保持控制锁，PC可继续发送ACT STOP重试。 */
      s_act_stop_hold = 1U;
      result = ACT_STREAM_RESULT_TX_FAILED;
    }
    Act_Stream_SendResult(ACT_STREAM_COMMAND_STOP, result);
    return;
  }

  Act_Stream_SendResult(ACT_STREAM_COMMAND_UNKNOWN,
                        ACT_STREAM_RESULT_BAD_COMMAND);
}

/**
 * @brief 将一个USART1接收字节加入当前命令行。
 * @param byte 本次收到的字节。
 * @note 命令过长时丢弃整行，直到收到换行后只报告一次格式错误。
 */
static void Act_Stream_PushRxByte(uint8_t byte)
{
  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    if (s_act_line_overflow != 0U)
    {
      Act_Stream_SendResult(ACT_STREAM_COMMAND_UNKNOWN,
                            ACT_STREAM_RESULT_BAD_COMMAND);
    }
    else if (s_act_line_length > 0U)
    {
      s_act_line[s_act_line_length] = '\0';
      Act_Stream_HandleLine(s_act_line);
    }

    s_act_line_length = 0U;
    s_act_line_overflow = 0U;
    return;
  }

  if (s_act_line_overflow != 0U)
  {
    return;
  }

  if (s_act_line_length >= (ACT_STREAM_LINE_SIZE - 1U))
  {
    s_act_line_length = 0U;
    s_act_line_overflow = 1U;
    return;
  }

  s_act_line[s_act_line_length++] = (char)byte;
}

/**
 * @brief 消费USART1循环DMA中新到达的字节。
 * @note DMA写入位置瞬间等于缓冲区长度时统一换算为0，避免尾指针卡死。
 */
static void Act_Stream_ProcessRx(void)
{
  uint16_t rx_position;

  if (huart1.hdmarx == NULL)
  {
    s_act_uart_recover_pending = 1U;
    return;
  }

  rx_position = (uint16_t)(ACT_STREAM_RX_DMA_SIZE -
                           __HAL_DMA_GET_COUNTER(huart1.hdmarx));
  if (rx_position >= ACT_STREAM_RX_DMA_SIZE)
  {
    rx_position = 0U;
  }

  Act_Stream_InvalidateRxDCache();
  while (s_act_rx_tail != rx_position)
  {
    Act_Stream_PushRxByte(s_act_rx_dma[s_act_rx_tail]);
    s_act_rx_tail++;
    if (s_act_rx_tail >= ACT_STREAM_RX_DMA_SIZE)
    {
      s_act_rx_tail = 0U;
    }
  }
}

/**
 * @brief 在主循环中恢复发生错误或超时的USART1 DMA。
 * @note 恢复动作限制为每100ms最多一次，避免硬件异常时反复占用主循环。
 */
static void Act_Stream_RecoverUart(void)
{
  uint32_t now_ms = HAL_GetTick();

  if ((s_act_uart_recover_pending == 0U) ||
      ((uint32_t)(now_ms - s_act_rx_last_retry_ms) < ACT_STREAM_RX_RETRY_MS))
  {
    return;
  }

  s_act_rx_last_retry_ms = now_ms;
  (void)HAL_UART_AbortTransmit(&huart1);
  (void)HAL_UART_AbortReceive(&huart1);
  if (s_act_tx_is_frame != 0U)
  {
    s_act_status.dropped_frame_count++;
  }
  s_act_tx_busy = 0U;
  s_act_tx_is_frame = 0U;

  if (Act_Stream_StartRxDma() == 0U)
  {
    s_act_uart_recover_pending = 0U;
    s_act_line_length = 0U;
    s_act_line_overflow = 0U;
  }
}

/**
 * @brief 初始化ACT模块状态并启动USART1循环接收DMA。
 */
void Act_Stream_Init(void)
{
  memset(&s_act_status, 0, sizeof(s_act_status));
  /* 当前ACT模型只使用触手3示教数据，默认选中且仅允许其自主运行。 */
  s_act_status.selected_tentacle = ACT_POLICY_APP_TENTACLE;
  s_act_status.capture_state = ACT_STREAM_CAPTURE_IDLE;

  s_act_line_length = 0U;
  s_act_line_overflow = 0U;
  s_act_tx_busy = 0U;
  s_act_tx_is_frame = 0U;
  s_act_uart_recover_pending = 0U;
  s_act_rx_last_retry_ms = HAL_GetTick() - ACT_STREAM_RX_RETRY_MS;
  s_act_stop_hold = 0U;
  memset(&s_act_pending_result, 0, sizeof(s_act_pending_result));
  s_act_status_pending = 0U;
  s_act_last_frame_seq = 0U;
  Act_Stream_ClearMove();

  if (Act_Stream_StartRxDma() != 0U)
  {
    s_act_uart_recover_pending = 1U;
  }
}

/**
 * @brief 执行ACT非阻塞通信任务。
 * @note 应在主循环持续调用；本函数不等待DMA完成。
 */
void Act_Stream_Task(void)
{
  uint32_t now_ms = HAL_GetTick();
  const MotorControl_Status_t *motor_status;

  if ((s_act_tx_busy != 0U) &&
      ((uint32_t)(now_ms - s_act_tx_start_ms) >= ACT_STREAM_TX_TIMEOUT_MS))
  {
    s_act_uart_recover_pending = 1U;
  }

  Act_Stream_RecoverUart();
  if (s_act_uart_recover_pending == 0U)
  {
    Act_Stream_ProcessRx();
  }

  /* CAN故障或UART7 CANSTOP关闭底层控制后，同步释放ACT本地控制锁。 */
  motor_status = MotorControl_GetStatus();
  if ((s_act_status.control_enabled != 0U) &&
      (s_act_stop_hold == 0U) &&
      ((motor_status->enable == 0U) ||
       (motor_status->control_source != MOTOR_CONTROL_SOURCE_PC)))
  {
    s_act_status.control_enabled = 0U;
    Act_Stream_ClearMove();
  }

  Act_Stream_SubmitMove(now_ms);
  /* 命令响应优先于采样帧，保证控制命令不会被连续大包饿死。 */
  Act_Stream_TrySendPendingResult();
  Act_Stream_TrySendPendingStatus();
  Act_Stream_TrySendCaptureFrame();
}

/**
 * @brief 复制ACT模块状态，禁止向调用方暴露内部可写对象。
 * @param status 接收状态的结构体地址；NULL表示不读取。
 */
void Act_Stream_GetStatus(Act_StreamStatus_t *status)
{
  if (status != NULL)
  {
    *status = s_act_status;
  }
}

uint8_t Act_Stream_HasControl(void)
{
  return ((s_act_status.control_enabled != 0U) ||
          (s_act_stop_hold != 0U) ||
          (ActPolicyApp_GetRunState() != ACT_POLICY_APP_STATE_OFF)) ? 1U : 0U;
}

/**
 * @brief 处理USART1 DMA发送完成事件。
 * @param huart HAL回调传入的串口句柄。
 */
void Act_Stream_OnUartTxComplete(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    if (s_act_tx_is_frame != 0U)
    {
      s_act_status.sent_frame_count++;
    }
    s_act_tx_is_frame = 0U;
    s_act_tx_busy = 0U;
  }
}

/**
 * @brief 记录USART1错误，实际DMA恢复留给主循环执行。
 * @param huart HAL回调传入的串口句柄。
 * @note 中断回调中不执行Abort或重新启动DMA，避免阻塞其他实时任务。
 */
void Act_Stream_OnUartError(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    s_act_uart_recover_pending = 1U;
  }
}
