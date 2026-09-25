#include "can_app.h"

#include "fdcan.h"
#include <string.h>

static uint8_t g_can_ready;
static uint8_t g_can_seq;
static uint32_t g_can_tx_count;
static uint8_t g_can_last_tx_seq[CAN_APP_G4_BOARD_COUNT];
static Can_App_G4Status_t g_g4_status[CAN_APP_G4_BOARD_COUNT];
static volatile uint8_t g_can_bus_off_pending;
static volatile uint8_t g_can_bus_off_event;
static uint32_t g_can_last_recover_ms;

#define CAN_APP_POSITION_SIGN_BOARD_MAX 4U
#define CAN_APP_SYNC_MIN_SPEED_Q         500L
#define CAN_APP_SYNC_STATUS_TIMEOUT_MS   200U
#define CAN_APP_BUS_RECOVER_PERIOD_MS    500U
#define CAN_APP_STATUS_BUS_OFF_FLAG      0x20U

/*
 * 线坐标到G4位置坐标的方向表。
 * 统一约定：VOFA输入、MCTRL输出、CANSTAT显示均为正数收线、负数放线。
 * 若某块板某个电机显示正数却实际放线，只改这里对应的符号。
 */
static const int8_t s_can_position_cmd_sign[CAN_APP_POSITION_SIGN_BOARD_MAX][3] =
{
  {1, -1, -1},
  {1, -1, 1},
  {-1, 1, 1},
  {1, 1,  1},
};

void Can_App_PositionToLineQ(uint8_t board_id, const int32_t position_q[3], int32_t line_q[3])
{
  uint8_t i;

  if ((position_q == NULL) || (line_q == NULL))
  {
    return;
  }

  if (board_id >= CAN_APP_POSITION_SIGN_BOARD_MAX)
  {
    board_id = 0U;
  }

  for (i = 0U; i < 3U; i++)
  {
    line_q[i] = position_q[i] * (int32_t)s_can_position_cmd_sign[board_id][i];
  }
}

/**
  * @brief  写入 uint32 小端数据。
  * @param  dst 目标字节地址，至少 4 字节。
  * @param  value 待写入数据。
  */
static void Can_App_PutU32Le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value);
  dst[1] = (uint8_t)(value >> 8);
  dst[2] = (uint8_t)(value >> 16);
  dst[3] = (uint8_t)(value >> 24);
}

/**
  * @brief  写入 int32 小端数据。
  * @param  dst 目标字节地址，至少 4 字节。
  * @param  value 待写入数据。
  */
static void Can_App_PutI32Le(uint8_t *dst, int32_t value)
{
  Can_App_PutU32Le(dst, (uint32_t)value);
}

/**
  * @brief  读取 uint16 小端数据。
  * @param  src 源字节地址，至少 2 字节。
  * @return 读取到的 uint16。
  */
static uint16_t Can_App_GetU16Le(const uint8_t *src)
{
  return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

/**
  * @brief  读取 uint32 小端数据。
  * @param  src 源字节地址，至少 4 字节。
  * @return 读取到的 uint32。
  */
static uint32_t Can_App_GetU32Le(const uint8_t *src)
{
  return ((uint32_t)src[0]) |
         ((uint32_t)src[1] << 8) |
         ((uint32_t)src[2] << 16) |
         ((uint32_t)src[3] << 24);
}

/**
  * @brief  读取 int32 小端数据。
  * @param  src 源字节地址，至少 4 字节。
  * @return 读取到的 int32。
  */
static int32_t Can_App_GetI32Le(const uint8_t *src)
{
  return (int32_t)Can_App_GetU32Le(src);
}

/**
  * @brief  float rad 转协议定点位置。
  * @param  rad 电机目标角度，单位 rad。
  * @return 协议定点值，rad * 100。
  */
static int32_t Can_App_RadToQ(float rad)
{
  float scaled = rad * CAN_APP_POS_SCALE;

  if (scaled >= 0.0f)
  {
    scaled += 0.5f;
  }
  else
  {
    scaled -= 0.5f;
  }

  return (int32_t)scaled;
}

/**
  * @brief 原子复制G4状态，避免主循环读到CAN中断正在更新的半帧数据。
  */
uint8_t Can_App_CopyG4StatusById(uint8_t board_id,
                                 Can_App_G4Status_t *snapshot)
{
  uint32_t primask;

  if ((snapshot == NULL) || (board_id >= CAN_APP_G4_BOARD_COUNT))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *snapshot = g_g4_status[board_id];
  if ((primask & 1U) == 0U)
  {
    __enable_irq();
  }

  return snapshot->valid;
}

/**
  * @brief  根据三路本次位移计算同步巡航速度。
  * @param  board_id G4板号。
  * @param  flags 位置命令标志，用于区分相对和绝对位置。
  * @param  target_q 三路线坐标目标，单位0.01rad。
  * @param  cruise_speed_q 最大位移轴的巡航速度，单位rad/s*100。
  * @param  speed_q 输出三路巡航速度，单位rad/s*100。
  * @note   最大位移轴保持设定巡航速度，其余轴按位移比例降低；
  *         G4状态无效或过期时退回三路同速，避免使用错误位置。
  */
static void Can_App_BuildSyncSpeedQ(uint8_t board_id,
                                    uint8_t flags,
                                    const int32_t target_q[3],
                                    int32_t cruise_speed_q,
                                    int32_t speed_q[3])
{
  Can_App_G4Status_t status;
  int32_t actual_line_q[3];
  uint64_t distance_q[3];
  uint64_t max_distance_q = 0U;
  uint8_t i;

  for (i = 0U; i < 3U; i++)
  {
    speed_q[i] = cruise_speed_q;
  }

  if (cruise_speed_q <= 0)
  {
    return;
  }

  if ((flags & CAN_APP_FLAG_RELATIVE) != 0U)
  {
    actual_line_q[0] = 0;
    actual_line_q[1] = 0;
    actual_line_q[2] = 0;
  }
  else
  {
    if ((Can_App_CopyG4StatusById(board_id, &status) == 0U) ||
        ((HAL_GetTick() - status.last_rx_tick_ms) > CAN_APP_SYNC_STATUS_TIMEOUT_MS))
    {
      return;
    }

    Can_App_PositionToLineQ(board_id, status.actual_q, actual_line_q);
  }

  for (i = 0U; i < 3U; i++)
  {
    int64_t delta = (int64_t)target_q[i] - (int64_t)actual_line_q[i];

    distance_q[i] = (delta >= 0) ? (uint64_t)delta : (uint64_t)(-delta);
    if (distance_q[i] > max_distance_q)
    {
      max_distance_q = distance_q[i];
    }
  }

  if (max_distance_q == 0U)
  {
    return;
  }

  for (i = 0U; i < 3U; i++)
  {
    if (distance_q[i] == 0U)
    {
      continue;
    }

    speed_q[i] = (int32_t)(((uint64_t)cruise_speed_q * distance_q[i]) /
                           max_distance_q);
    if (speed_q[i] < CAN_APP_SYNC_MIN_SPEED_Q)
    {
      speed_q[i] = CAN_APP_SYNC_MIN_SPEED_Q;
    }
  }
}

/**
 * @brief  发送一帧指定G4板的控制帧。
 * @param  board_id G4板号，0对应0x120，1对应0x121。
 * @param  command 命令字。
 * @param  flags 控制标志。
 * @param  motor_mask 电机掩码。
  * @param  motor0_q 电机 0 目标定点值。
  * @param  motor1_q 电机 1 目标定点值。
  * @param  motor2_q 电机 2 目标定点值。
 * @param  speed_limit_q 位置运动速度上限，单位 rad/s*100；0表示G4使用默认值。
 * @return 0 成功，非 0 失败。
 */
static uint8_t Can_App_SendCommandToBoard(uint8_t board_id,
                                   uint8_t command,
                                   uint8_t flags,
                                   uint8_t motor_mask,
                                   int32_t motor0_q,
                                   int32_t motor1_q,
                                   int32_t motor2_q,
                                   int32_t speed_limit_q)
{
  FDCAN_TxHeaderTypeDef tx_header;
  uint8_t payload[32];
  uint8_t tx_seq;
  int32_t target_q[3];
  int32_t speed_q[3];

  if ((g_can_ready == 0U) || (board_id >= CAN_APP_G4_BOARD_COUNT))
  {
    return 1U;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    return 2U;
  }

  target_q[0] = motor0_q;
  target_q[1] = motor1_q;
  target_q[2] = motor2_q;

  if (command == CAN_APP_CMD_POSITION)
  {
    Can_App_BuildSyncSpeedQ(board_id, flags, target_q, speed_limit_q, speed_q);
    motor0_q *= (int32_t)s_can_position_cmd_sign[board_id][0];
    motor1_q *= (int32_t)s_can_position_cmd_sign[board_id][1];
    motor2_q *= (int32_t)s_can_position_cmd_sign[board_id][2];
  }
  else
  {
    speed_q[0] = speed_limit_q;
    speed_q[1] = speed_limit_q;
    speed_q[2] = speed_limit_q;
  }

  memset(&tx_header, 0, sizeof(tx_header));
  memset(payload, 0, sizeof(payload));

  payload[0] = CAN_APP_PROTOCOL_VER;
  payload[1] = command;
  payload[2] = flags;
  payload[3] = motor_mask;
  tx_seq = g_can_seq++;
  payload[4] = tx_seq;
  Can_App_PutI32Le(&payload[8], motor0_q);
  Can_App_PutI32Le(&payload[12], motor1_q);
  Can_App_PutI32Le(&payload[16], motor2_q);
  Can_App_PutI32Le(&payload[20], speed_q[0]);
  Can_App_PutI32Le(&payload[24], speed_q[1]);
  Can_App_PutI32Le(&payload[28], speed_q[2]);

  tx_header.Identifier = CAN_APP_H7_CMD_BASE_ID + board_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_32;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_ON;
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, payload) != HAL_OK)
  {
    return 3U;
  }

  g_can_tx_count++;
  g_can_last_tx_seq[board_id] = tx_seq;
  return 0U;
}

uint8_t Can_App_GetLastTxSeqById(uint8_t board_id)
{
  return (board_id < CAN_APP_G4_BOARD_COUNT) ?
         g_can_last_tx_seq[board_id] : 0U;
}

/**
  * @brief 恢复BUS-OFF后的H7 FDCAN，先取消旧目标，避免恢复后补发。
  */
static uint8_t Can_App_RecoverBus(void)
{
  if (hfdcan1.State == HAL_FDCAN_STATE_BUSY)
  {
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, 0xFFU);
    if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
    {
      return 0U;
    }
  }

  if ((hfdcan1.State != HAL_FDCAN_STATE_READY) ||
      (HAL_FDCAN_Start(&hfdcan1) != HAL_OK))
  {
    return 0U;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                      FDCAN_IT_BUS_OFF,
                                      0U) != HAL_OK)
  {
    (void)HAL_FDCAN_Stop(&hfdcan1);
    return 0U;
  }

  return 1U;
}

/**
  * @brief  发送一帧默认G4板0的控制帧，兼容现有单板调用。
  */
static uint8_t Can_App_SendCommand(uint8_t command,
                                   uint8_t flags,
                                   uint8_t motor_mask,
                                   int32_t motor0_q,
                                   int32_t motor1_q,
                                   int32_t motor2_q,
                                   int32_t speed_limit_q)
{
  return Can_App_SendCommandToBoard(0U,
                                    command,
                                    flags,
                                    motor_mask,
                                    motor0_q,
                                    motor1_q,
                                    motor2_q,
                                    speed_limit_q);
}

uint8_t Can_App_Init(void)
{
  FDCAN_FilterTypeDef filter;

  g_can_ready = 0U;
  g_can_seq = 0U;
  g_can_tx_count = 0U;
  memset(g_can_last_tx_seq, 0, sizeof(g_can_last_tx_seq));
  memset(g_g4_status, 0, sizeof(g_g4_status));
  g_can_bus_off_pending = 0U;
  g_can_bus_off_event = 0U;
  g_can_last_recover_ms = 0U;
  memset(&filter, 0, sizeof(filter));

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = CAN_APP_G4_STATUS_BASE_ID;
  filter.FilterID2 = 0x7FCU;  /* 接收0x180~0x183四块G4状态帧。 */
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    return 1U;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    return 2U;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return 3U;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                     FDCAN_IT_BUS_OFF,
                                     0U) != HAL_OK)
  {
    return 4U;
  }

  g_can_ready = 1U;
  return 0U;
}

/**
  * @brief H7 CAN后台恢复任务；只恢复通信，不恢复MCTRL使能、不重发旧目标。
  */
void Can_App_Task(void)
{
  uint32_t now_ms;

  if (g_can_bus_off_pending == 0U)
  {
    return;
  }

  now_ms = HAL_GetTick();
  if ((now_ms - g_can_last_recover_ms) < CAN_APP_BUS_RECOVER_PERIOD_MS)
  {
    return;
  }
  g_can_last_recover_ms = now_ms;

  if (Can_App_RecoverBus() != 0U)
  {
    g_can_bus_off_pending = 0U;
    g_can_ready = 1U;
  }
}

uint8_t Can_App_TakeBusOffEvent(void)
{
  uint32_t primask;
  uint8_t event;

  primask = __get_PRIMASK();
  __disable_irq();
  event = g_can_bus_off_event;
  g_can_bus_off_event = 0U;
  if ((primask & 1U) == 0U)
  {
    __enable_irq();
  }

  return event;
}

uint8_t Can_App_IsReady(void)
{
  return g_can_ready;
}

/**
  * @brief  检查FDCAN发送队列能否容纳一整组控制帧。
  * @param  frame_count 本次需要连续发送的帧数。
  * @return 1空间足够，0未初始化或空间不足。
  * @note   只读取队列状态，不发送数据。
  */
uint8_t Can_App_HasTxFifoSpace(uint8_t frame_count)
{
  if ((g_can_ready == 0U) || (frame_count == 0U))
  {
    return 0U;
  }

  return (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) >= frame_count) ? 1U : 0U;
}

/**
  * @brief  判断FDCAN发送FIFO是否已经完全清空。
  * @return 1为空，0仍有帧等待发送或CAN尚未初始化。
  * @note   STOP补发前使用，防止自动重传期间继续向FIFO堆积新帧。
  */
uint8_t Can_App_IsTxFifoEmpty(void)
{
  if (g_can_ready == 0U)
  {
    return 0U;
  }

  return (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) ==
          hfdcan1.Init.TxFifoQueueElmtsNbr) ? 1U : 0U;
}

/**
  * @brief 发送三路相对位置命令。
  * @param motor0_q/motor1_q/motor2_q 三路位置增量，单位0.01rad。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q)
{
  return Can_App_SendCommand(CAN_APP_CMD_POSITION,
                             CAN_APP_FLAG_POS_APPLY,
                             CAN_APP_MOTOR_MASK_ALL,
                             motor0_q,
                             motor1_q,
                             motor2_q,
                             0);
}

/**
  * @brief 发送三路相对位置命令，并要求G4重置位置/速度/电流环状态。
  * @param motor0_q/motor1_q/motor2_q 三路位置增量，单位0.01rad。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsResetQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q)
{
  return Can_App_SendMotorTargetsResetQToBoard(0U, motor0_q, motor1_q, motor2_q);
}

/**
  * @brief 发送指定G4板的三路相对位置命令，并要求G4重置位置/速度/电流环状态。
  * @param board_id G4板号。
  * @param motor0_q/motor1_q/motor2_q 三路位置增量，单位0.01rad。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsResetQToBoard(uint8_t board_id,
                                              int32_t motor0_q,
                                              int32_t motor1_q,
                                              int32_t motor2_q)
{
  return Can_App_SendCommandToBoard(board_id,
                                    CAN_APP_CMD_POSITION,
                                    CAN_APP_FLAG_POS_RESET_APPLY,
                                    CAN_APP_MOTOR_MASK_ALL,
                                    motor0_q,
                                    motor1_q,
                                    motor2_q,
                                    0);
}

/**
  * @brief 发送三路绝对位置命令。
  * @param motor0_q/motor1_q/motor2_q 三路绝对目标位置，单位0.01rad。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsAbsQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q)
{
  return Can_App_SendMotorTargetsAbsSpeedQ(motor0_q, motor1_q, motor2_q, 0);
}

/**
  * @brief 发送三路绝对位置命令，并附带G4位置环速度上限。
  * @param motor0_q/motor1_q/motor2_q 三路绝对目标位置，单位0.01rad。
  * @param speed_limit_q 位置运动速度上限，单位rad/s*100；0表示G4使用默认值。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsAbsSpeedQ(int32_t motor0_q,
                                          int32_t motor1_q,
                                          int32_t motor2_q,
                                          int32_t speed_limit_q)
{
  return Can_App_SendMotorTargetsAbsSpeedQToBoard(0U,
                                                  motor0_q,
                                                  motor1_q,
                                                  motor2_q,
                                                  speed_limit_q);
}

/**
  * @brief 发送指定G4板的三路绝对位置命令，并附带位置环巡航速度。
  * @param board_id G4板号，0对应0x120，1对应0x121。
  * @param motor0_q/motor1_q/motor2_q 三路绝对目标位置，单位0.01rad。
  * @param speed_limit_q 位置运动速度，单位rad/s*100；0表示G4使用默认值。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsAbsSpeedQToBoard(uint8_t board_id,
                                                 int32_t motor0_q,
                                                 int32_t motor1_q,
                                                 int32_t motor2_q,
                                                 int32_t speed_limit_q)
{
  return Can_App_SendCommandToBoard(board_id,
                                    CAN_APP_CMD_POSITION,
                                    CAN_APP_FLAG_ENABLE | CAN_APP_FLAG_APPLY,
                                    CAN_APP_MOTOR_MASK_ALL,
                                    motor0_q,
                                    motor1_q,
                                    motor2_q,
                                    speed_limit_q);
}

/**
  * @brief 发送三路绝对位置命令，并要求G4重置位置/速度/电流环状态。
  * @param motor0_q/motor1_q/motor2_q 三路绝对目标位置，单位0.01rad。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsAbsResetQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q)
{
  return Can_App_SendMotorTargetsAbsResetQToBoard(0U, motor0_q, motor1_q, motor2_q);
}

/**
  * @brief 发送指定G4板的三路绝对位置命令，并要求G4重置位置/速度/电流环状态。
  * @param board_id G4板号。
  * @param motor0_q/motor1_q/motor2_q 三路绝对目标位置，单位0.01rad。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsAbsResetQToBoard(uint8_t board_id,
                                                 int32_t motor0_q,
                                                 int32_t motor1_q,
                                                 int32_t motor2_q)
{
  return Can_App_SendCommandToBoard(board_id,
                                    CAN_APP_CMD_POSITION,
                                    CAN_APP_FLAG_ENABLE | CAN_APP_FLAG_APPLY | CAN_APP_FLAG_RESET,
                                    CAN_APP_MOTOR_MASK_ALL,
                                    motor0_q,
                                    motor1_q,
                                    motor2_q,
                                    0);
}

/**
  * @brief 发送三路相对位置命令，输入单位为rad。
  * @param motor0_rad/motor1_rad/motor2_rad 三路位置增量，单位rad。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendMotorTargetsRad(float motor0_rad, float motor1_rad, float motor2_rad)
{
  return Can_App_SendMotorTargetsQ(Can_App_RadToQ(motor0_rad),
                                   Can_App_RadToQ(motor1_rad),
                                   Can_App_RadToQ(motor2_rad));
}

/**
  * @brief 发送停止命令，G4收到后关闭三路控制输出。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendStop(void)
{
  return Can_App_SendStopToBoard(0U);
}

/**
  * @brief 发送指定G4板停止命令。
  * @param board_id G4板号。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendStopToBoard(uint8_t board_id)
{
  return Can_App_SendCommandToBoard(board_id,
                                    CAN_APP_CMD_STOP,
                                    CAN_APP_FLAG_STOP,
                                    CAN_APP_MOTOR_MASK_ALL,
                                    0,
                                    0,
                                    0,
                                    0);
}

/**
  * @brief 发送置零命令，G4将三路当前位置记录为编码器零点，不产生运动。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendSetZero(void)
{
  return Can_App_SendSetZeroToBoard(0U);
}

/**
  * @brief 指定G4板置零，当前位置作为该板三路逻辑零点，不产生运动。
  * @param board_id G4板号。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendSetZeroToBoard(uint8_t board_id)
{
  return Can_App_SendCommandToBoard(board_id,
                                    CAN_APP_CMD_ZERO,
                                    CAN_APP_FLAG_ENABLE | CAN_APP_FLAG_APPLY,
                                    CAN_APP_MOTOR_MASK_ALL,
                                    0,
                                    0,
                                    0,
                                    0);
}

/**
  * @brief 发送回零命令，G4按位置闭环让三路电机运动到绝对0位置。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendHome(void)
{
  return Can_App_SendHomeToBoard(0U);
}

/**
  * @brief 指定G4板回零，三路电机运动到绝对0位置。
  * @param board_id G4板号。
  * @return 0成功，非0失败。
  */
uint8_t Can_App_SendHomeToBoard(uint8_t board_id)
{
  return Can_App_SendCommandToBoard(board_id,
                                    CAN_APP_CMD_HOME,
                                    CAN_APP_FLAG_ENABLE | CAN_APP_FLAG_APPLY,
                                    CAN_APP_MOTOR_MASK_ALL,
                                    0,
                                    0,
                                    0,
                                    0);
}

/**
  * @brief 发送实际坐标恢复命令，G4只恢复多圈坐标，不启动PWM、不运动。
  * @param motor0_q/motor1_q/motor2_q 断电前G4实际位置，单位0.01rad。
  * @return 0成功放入发送FIFO，非0失败。
  */
uint8_t Can_App_SendRestoreActualQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q)
{
  return Can_App_SendRestoreActualQToBoard(0U, motor0_q, motor1_q, motor2_q);
}

/**
  * @brief 指定G4板恢复实际多圈坐标，不启动PWM、不运动。
  * @param board_id G4板号。
  * @param motor0_q/motor1_q/motor2_q 断电前G4实际位置，单位0.01rad。
  * @return 0成功放入发送FIFO，非0失败。
  */
uint8_t Can_App_SendRestoreActualQToBoard(uint8_t board_id,
                                          int32_t motor0_q,
                                          int32_t motor1_q,
                                          int32_t motor2_q)
{
  return Can_App_SendCommandToBoard(board_id,
                                    CAN_APP_CMD_RESTORE,
                                    CAN_APP_FLAG_APPLY,
                                    CAN_APP_MOTOR_MASK_ALL,
                                    motor0_q,
                                    motor1_q,
                                    motor2_q,
                                    0);
}

uint32_t Can_App_GetTxCount(void)
{
  return g_can_tx_count;
}

const Can_App_G4Status_t *Can_App_GetG4Status(void)
{
  return &g_g4_status[0];
}

/**
  * @brief  获取指定G4板的最近状态。
  * @param  board_id G4板号。
  * @return 状态指针；board_id非法时返回board0状态，避免调用方空指针。
  */
const Can_App_G4Status_t *Can_App_GetG4StatusById(uint8_t board_id)
{
  if (board_id >= CAN_APP_G4_BOARD_COUNT)
  {
    return &g_g4_status[0];
  }

  return &g_g4_status[board_id];
}

uint8_t Can_App_IsG4Online(uint32_t timeout_ms)
{
  return Can_App_IsG4OnlineById(0U, timeout_ms);
}

/**
  * @brief  判断指定G4板是否在线。
  * @param  board_id G4板号。
  * @param  timeout_ms 超过该时间未收到状态帧则认为离线。
  * @return 1表示G4状态帧新鲜，0表示未收到或已经超时。
  * @note   status->online仅表示G4最近是否收到H7命令，不能作为板卡连接判据。
  */
uint8_t Can_App_IsG4OnlineById(uint8_t board_id, uint32_t timeout_ms)
{
  Can_App_G4Status_t status;

  if (Can_App_CopyG4StatusById(board_id, &status) == 0U)
  {
    return 0U;
  }

  if ((HAL_GetTick() - status.last_rx_tick_ms) > timeout_ms)
  {
    return 0U;
  }

  return 1U;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t payload[32];
  uint8_t status_flags;
  uint8_t board_id;
  Can_App_G4Status_t *status;

  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1))
  {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    memset(&rx_header, 0, sizeof(rx_header));
    memset(payload, 0, sizeof(payload));

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, payload) != HAL_OK)
    {
      return;
    }

    if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
        (rx_header.Identifier < CAN_APP_G4_STATUS_BASE_ID) ||
        (rx_header.Identifier >= (CAN_APP_G4_STATUS_BASE_ID + CAN_APP_G4_BOARD_COUNT)) ||
        (rx_header.RxFrameType != FDCAN_DATA_FRAME) ||
        (rx_header.DataLength != FDCAN_DLC_BYTES_32))
    {
      continue;
    }

    board_id = (uint8_t)(rx_header.Identifier - CAN_APP_G4_STATUS_BASE_ID);
    status = &g_g4_status[board_id];

    status->valid = 1U;
    status->last_rx_tick_ms = HAL_GetTick();
    status->rx_frame_count++;
    status->protocol_version = payload[0];
    status->board_id = payload[1];
    status->online = payload[2];
    status->control_word = payload[3];
    status->g4_rx_count_low16 = Can_App_GetU16Le(&payload[4]);
    status->g4_tx_count_low16 = Can_App_GetU16Le(&payload[6]);
    status->error_count = Can_App_GetU32Le(&payload[8]);
    status->pwm_started_mask = payload[12];
    status->last_relative_seq = payload[13];
    status_flags = payload[14];
    status->last_relative_seq_valid = status_flags & 0x01U;
    status->position_restore_valid = ((status_flags & 0x02U) != 0U) ? 1U : 0U;
    status->stall_fault_mask = (uint8_t)((status_flags >> 2) & 0x07U);
    status->can_bus_off_latched =
      ((status_flags & CAN_APP_STATUS_BUS_OFF_FLAG) != 0U) ? 1U : 0U;
    status->position_pending = payload[15];
    status->last_position_seq = payload[16];
    status->last_position_flags = payload[17];
    status->last_position_motor_mask = payload[18];
    status->last_position_command = payload[19];
    status->actual_q[0] = Can_App_GetI32Le(&payload[20]);
    status->actual_q[1] = Can_App_GetI32Le(&payload[24]);
    status->actual_q[2] = Can_App_GetI32Le(&payload[28]);
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1) ||
      ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0U))
  {
    return;
  }

  if (g_can_bus_off_pending == 0U)
  {
    g_can_last_recover_ms = HAL_GetTick() - CAN_APP_BUS_RECOVER_PERIOD_MS;
  }
  g_can_ready = 0U;
  g_can_bus_off_event = 1U;
  g_can_bus_off_pending = 1U;
}
