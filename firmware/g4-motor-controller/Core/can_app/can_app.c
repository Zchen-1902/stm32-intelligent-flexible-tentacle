#include "can_app.h"

#include "../Inc/fdcan.h"
#include "../motor/motor_core.h"
#include "../spi_encoder/spi_encoder.h"

#define CAN_APP_RX_FILTER_MASK          (0x7FFU)
#define CAN_APP_STATUS_PERIOD_MS        (100U)
#define CAN_APP_ONLINE_TIMEOUT_MS       (500U)
#define CAN_APP_BUS_RECOVER_PERIOD_MS   (500U)
#define CAN_APP_STATUS_BUS_OFF_FLAG     (0x20U)
#define CAN_APP_TARGET_MOTOR_COUNT      (3U)
#define CAN_APP_POSITION_FD_LENGTH      (32U)
#define CAN_APP_POSITION_SCALE          (100.0f)       // CANPOS单位：1 count = 0.01 rad

#define CAN_APP_CMD_POSITION            (0x01U)
#define CAN_APP_CMD_STOP                (0x02U)
#define CAN_APP_CMD_ZERO                (0x03U)
#define CAN_APP_CMD_HOME                (0x04U)
#define CAN_APP_CMD_RESTORE             (0x05U)  // H7下发断电前ACTUAL，G4只恢复多圈坐标，不启动PWM、不运动

#define CAN_APP_FLAG_ENABLE             (0x01U)
#define CAN_APP_FLAG_STOP               (0x02U)
#define CAN_APP_FLAG_RELATIVE           (0x04U)
#define CAN_APP_FLAG_APPLY              (0x08U)
#define CAN_APP_FLAG_RESET              (0x10U)
#define CAN_APP_FLAG_ESTOP              (0x80U)

typedef struct
{
  uint8_t command;
  uint8_t flags;
  uint8_t motor_mask;
  uint8_t seq;
  int32_t target_raw[CAN_APP_TARGET_MOTOR_COUNT];
  int32_t speed_limit_q[CAN_APP_TARGET_MOTOR_COUNT]; // 三路位置巡航速度，单位rad/s*100
} CAN_App_PositionCmd_t;

typedef struct
{
  uint8_t initialized;
  uint8_t online;
  uint8_t control_word;
  uint8_t mode;
  uint8_t motor_enable_mask;
  uint32_t rx_count;
  uint32_t tx_count;
  uint32_t error_count;
  uint32_t last_rx_tick;
  uint32_t last_status_tx_tick;
  uint8_t pwm_started_mask;
  volatile uint8_t control_active_mask;
  uint8_t last_relative_seq;
  uint8_t last_relative_seq_valid;
  volatile uint8_t position_pending;
  CAN_App_PositionCmd_t position_cmd;
  int32_t target_raw[CAN_APP_TARGET_MOTOR_COUNT];
} CAN_App_State_t;

static CAN_App_State_t s_can_app;
static volatile uint8_t s_status_tx_busy;
static volatile uint8_t s_can_bus_off_pending;
static volatile uint8_t s_can_bus_off_latched;
static uint8_t s_can_bus_off_stop_done;
static uint32_t s_can_last_recover_tick;

/**
  * @brief 将FDCAN DLC编码转换为字节数，后续切换CAN FD时继续复用。
  * @param dlc HAL FDCAN头中的DataLength字段。
  * @retval 对应payload字节数，未知编码返回0。
  */
static uint8_t CAN_App_DlcToBytes(uint32_t dlc)
{
  static const uint8_t dlc_to_bytes[16] =
  {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
    8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };

  if (dlc > FDCAN_DLC_BYTES_64)
  {
    return 0U;
  }

  return dlc_to_bytes[dlc];
}

/**
  * @brief 从小端payload中读取int32，用于暂存H7下发的目标测试值。
  * @param data 指向至少4字节payload。
  * @retval 小端int32数值。
  */
static int32_t CAN_App_ReadI32LE(const uint8_t *data)
{
  uint32_t value;

  value = ((uint32_t)data[0])
        | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16)
        | ((uint32_t)data[3] << 24);

  return (int32_t)value;
}

/**
  * @brief 向payload写入int32小端数据。
  * @param data 指向至少4字节payload。
  * @param value 待写入的int32数值。
  */
static void CAN_App_WriteI32LE(uint8_t *data, int32_t value)
{
  uint32_t raw;

  raw = (uint32_t)value;
  data[0] = (uint8_t)(raw & 0xFFU);
  data[1] = (uint8_t)((raw >> 8) & 0xFFU);
  data[2] = (uint8_t)((raw >> 16) & 0xFFU);
  data[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

/**
  * @brief 将rad转换为CAN定点位置。
  * @param rad 机械角，单位rad。
  * @return 定点位置，单位0.01rad。
  */
static int32_t CAN_App_RadToQ(float rad)
{
  return (int32_t)((rad >= 0.0f) ? ((rad * CAN_APP_POSITION_SCALE) + 0.5f)
                                 : ((rad * CAN_APP_POSITION_SCALE) - 0.5f));
}

/**
  * @brief 处理H7发来的控制帧，只缓存命令，不直接触发电机控制。
  * @param data payload指针。
  * @param length payload字节数。
  * @note 当前约定：byte0 control_word，byte1 mode，byte2 motor_enable_mask。
  */
static void CAN_App_HandleControlFrame(const uint8_t *data, uint8_t length)
{
  if (length < 3U)
  {
    s_can_app.error_count++;
    return;
  }

  s_can_app.control_word = data[0];
  s_can_app.mode = data[1];
  s_can_app.motor_enable_mask = data[2];
}

/**
  * @brief 处理H7发来的三电机位置CAN FD帧，只缓存目标，不在中断里控制电机。
  * @param header FDCAN接收头。
  * @param data payload指针，固定32字节。
  * @param length payload字节数。
  * @note 当前约定：byte8/12/16分别为motor0/1/2目标，byte20为速度上限，int32小端。
  */
static void CAN_App_HandlePositionFdFrame(const FDCAN_RxHeaderTypeDef *header,
                                          const uint8_t *data,
                                          uint8_t length)
{
  CAN_App_PositionCmd_t cmd;

  if ((header->FDFormat != FDCAN_FD_CAN) ||
      (header->BitRateSwitch != FDCAN_BRS_ON) ||
      (length != CAN_APP_POSITION_FD_LENGTH))
  {
    s_can_app.error_count++;
    return;
  }

  if ((data[0] != CAN_APP_PROTOCOL_VERSION) ||
      ((data[1] != CAN_APP_CMD_POSITION) &&
       (data[1] != CAN_APP_CMD_STOP) &&
       (data[1] != CAN_APP_CMD_ZERO) &&
       (data[1] != CAN_APP_CMD_HOME) &&
       (data[1] != CAN_APP_CMD_RESTORE)))
  {
    s_can_app.error_count++;
    return;
  }

  cmd.command = data[1];
  cmd.flags = data[2];
  cmd.motor_mask = data[3] & 0x07U;
  cmd.seq = data[4];
  cmd.target_raw[0] = CAN_App_ReadI32LE(&data[8]);
  cmd.target_raw[1] = CAN_App_ReadI32LE(&data[12]);
  cmd.target_raw[2] = CAN_App_ReadI32LE(&data[16]);
  cmd.speed_limit_q[0] = CAN_App_ReadI32LE(&data[20]);
  cmd.speed_limit_q[1] = CAN_App_ReadI32LE(&data[24]);
  cmd.speed_limit_q[2] = CAN_App_ReadI32LE(&data[28]);

  s_can_app.position_cmd = cmd;
  s_can_app.position_pending = 1U;
}

/**
  * @brief 处理H7发来的目标测试帧，只缓存原始目标值，暂不接入motor_core。
  * @param identifier 标准帧ID。
  * @param data payload指针。
  * @param length payload字节数。
  * @note 当前约定：0x110/0x111/0x112分别对应motor0/1/2，byte0..3为int32小端目标值。
  */
static void CAN_App_HandleTargetFrame(uint32_t identifier, const uint8_t *data, uint8_t length)
{
  uint32_t motor_id;

  if ((identifier < CAN_APP_ID_H7_TARGET_BASE) ||
      (identifier >= (CAN_APP_ID_H7_TARGET_BASE + CAN_APP_TARGET_MOTOR_COUNT)))
  {
    return;
  }

  if (length < 4U)
  {
    s_can_app.error_count++;
    return;
  }

  motor_id = identifier - CAN_APP_ID_H7_TARGET_BASE;
  s_can_app.target_raw[motor_id] = CAN_App_ReadI32LE(data);
}

/**
  * @brief 处理一帧已从RX FIFO取出的CAN消息。
  * @param header FDCAN接收头。
  * @param data payload缓冲区。
  * @note 回调路径只做解析和缓存，避免把控制逻辑放进FDCAN中断。
  */
static void CAN_App_ProcessRxFrame(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{
  uint8_t length;

  if ((header->IdType != FDCAN_STANDARD_ID) ||
      (header->RxFrameType != FDCAN_DATA_FRAME))
  {
    s_can_app.error_count++;
    return;
  }

  length = CAN_App_DlcToBytes(header->DataLength);
  if (length == 0U)
  {
    s_can_app.error_count++;
    return;
  }

  s_can_app.rx_count++;
  s_can_app.last_rx_tick = HAL_GetTick();
  s_can_app.online = 1U;

  if (header->Identifier == CAN_APP_ID_H7_POSITION_FD)
  {
    CAN_App_HandlePositionFdFrame(header, data, length);
  }
  else if (header->Identifier == CAN_APP_ID_H7_CONTROL)
  {
    CAN_App_HandleControlFrame(data, length);
  }
  else
  {
    CAN_App_HandleTargetFrame(header->Identifier, data, length);
  }
}

/**
  * @brief 尝试占用状态帧发送入口，避免主循环和位置环同时发状态帧。
  * @retval 1表示成功占用，0表示已有发送正在进行。
  */
static uint8_t CAN_App_TryLockStatusTx(void)
{
  uint32_t primask;
  uint8_t locked = 0U;

  primask = __get_PRIMASK();
  __disable_irq();

  if (s_status_tx_busy == 0U)
  {
    s_status_tx_busy = 1U;
    locked = 1U;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  return locked;
}

/**
  * @brief 释放状态帧发送入口。
  * @retval None
  */
static void CAN_App_UnlockStatusTx(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  s_status_tx_busy = 0U;

  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
  * @brief 发送G4板卡状态帧，用于H7确认从板在线和计数递增。
  * @retval HAL状态。
  * @note byte20~31携带三路实际多圈位置，单位0.01rad，与CAN位置目标坐标系一致。
  */
static HAL_StatusTypeDef CAN_App_SendStatusFrame(void)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[32] = {0};
  HAL_StatusTypeDef ret;
  uint8_t motor_id;
  motor_t *motor;
  uint8_t restore_valid;

  if (CAN_App_TryLockStatusTx() == 0U)
  {
    return HAL_BUSY;
  }

  if (s_can_app.initialized == 0U)
  {
    CAN_App_UnlockStatusTx();
    return HAL_ERROR;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    CAN_App_UnlockStatusTx();
    return HAL_BUSY;
  }

  tx_header.Identifier = CAN_APP_ID_G4_STATUS;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_32;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_ON;
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  restore_valid = Motor_Core_IsPositionRestoreValid();

  tx_data[0] = CAN_APP_PROTOCOL_VERSION;
  tx_data[1] = CAN_APP_BOARD_ID;
  tx_data[2] = s_can_app.online;
  tx_data[3] = s_can_app.control_word;
  tx_data[4] = (uint8_t)(s_can_app.rx_count & 0xFFU);
  tx_data[5] = (uint8_t)((s_can_app.rx_count >> 8) & 0xFFU);
  tx_data[6] = (uint8_t)(s_can_app.tx_count & 0xFFU);
  tx_data[7] = (uint8_t)((s_can_app.tx_count >> 8) & 0xFFU);
  tx_data[8] = (uint8_t)(s_can_app.error_count & 0xFFU);
  tx_data[9] = (uint8_t)((s_can_app.error_count >> 8) & 0xFFU);
  tx_data[10] = (uint8_t)((s_can_app.error_count >> 16) & 0xFFU);
  tx_data[11] = (uint8_t)((s_can_app.error_count >> 24) & 0xFFU);
  tx_data[12] = s_can_app.pwm_started_mask;
  tx_data[13] = s_can_app.last_relative_seq;
  /* byte14: bit0=relative有效，bit1=坐标可信，bit2~4=堵转，bit5=BUS-OFF锁存。 */
  tx_data[14] = (uint8_t)(s_can_app.last_relative_seq_valid & 0x01U);
  if (restore_valid != 0U)
  {
    tx_data[14] |= 0x02U;
  }
  for (motor_id = 0U; motor_id < CAN_APP_TARGET_MOTOR_COUNT; motor_id++)
  {
    motor = Motor_Core_GetMotor(motor_id);
    if ((motor != 0) && (motor->protect.stall_fault != 0U))
    {
      tx_data[14] |= (uint8_t)(1U << (2U + motor_id));
    }
  }
  if (s_can_bus_off_latched != 0U)
  {
    tx_data[14] |= CAN_APP_STATUS_BUS_OFF_FLAG;
  }
  tx_data[15] = s_can_app.position_pending;
  tx_data[16] = s_can_app.position_cmd.seq;
  tx_data[17] = s_can_app.position_cmd.flags;
  tx_data[18] = s_can_app.position_cmd.motor_mask;
  tx_data[19] = s_can_app.position_cmd.command;

  for (motor_id = 0U; motor_id < CAN_APP_TARGET_MOTOR_COUNT; motor_id++)
  {
    if (restore_valid != 0U)
    {
      CAN_App_WriteI32LE(&tx_data[20U + (motor_id * 4U)],
                         CAN_App_RadToQ(Motor_Core_GetPositionFeedbackRad(motor_id)));
    }
    else
    {
      CAN_App_WriteI32LE(&tx_data[20U + (motor_id * 4U)], 0);
    }
  }

  ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
  if (ret == HAL_OK)
  {
    s_can_app.tx_count++;
    s_can_app.last_status_tx_tick = HAL_GetTick();
  }
  else
  {
    s_can_app.error_count++;
  }

  CAN_App_UnlockStatusTx();
  return ret;
}

/**
  * @brief 停止指定mask内的电机输出。
  * @param motor_mask 电机位图，bit0/1/2对应motor0/1/2。
  * @retval None
  */
static void CAN_App_StopMotors(uint8_t motor_mask)
{
  uint8_t motor_id;

  s_can_app.control_active_mask &= (uint8_t)~motor_mask;

  for (motor_id = 0U; motor_id < CAN_APP_TARGET_MOTOR_COUNT; motor_id++)
  {
    if ((motor_mask & (1U << motor_id)) == 0U)
    {
      continue;
    }

    Motor_Core_SetCurrentRef(motor_id, 0.0f, 0.0f);
    Motor_Core_SetSpeedRef(motor_id, 0.0f);
    Motor_Core_SetMode(motor_id, MOTOR_MODE_IDLE);
    Motor_Core_ClearRuntimeFault(motor_id);
    (void)Motor_Core_StopPwmOutput(motor_id);
    s_can_app.pwm_started_mask &= (uint8_t)~(1U << motor_id);
  }
}

/**
  * @brief 只在CAN位置控制使能时启动尚未启动的PWM。
  * @param motor_mask 参与控制的电机位图，bit0/1/2对应motor0/1/2。
  * @retval 1表示全部就绪，0表示至少一路启动失败。
  * @note   该函数只在主循环调用，不在FDCAN中断中调用。
  */
static uint8_t CAN_App_EnsurePwmStarted(uint8_t motor_mask)
{
  uint8_t motor_id;

  for (motor_id = 0U; motor_id < CAN_APP_TARGET_MOTOR_COUNT; motor_id++)
  {
    if ((motor_mask & (1U << motor_id)) == 0U)
    {
      continue;
    }

    if ((s_can_app.pwm_started_mask & (1U << motor_id)) != 0U)
    {
      continue;
    }

    (void)Motor_Core_OutputNeutralPwm(motor_id);
    if (Motor_Core_IsPwmStartAllowed(motor_id) == 0U)
    {
      return 0U;
    }

    if (Motor_Core_StartPwmOutput(motor_id) != HAL_OK)
    {
      return 0U;
    }

    s_can_app.pwm_started_mask |= (uint8_t)(1U << motor_id);
  }

  return 1U;
}

/**
  * @brief 从中断缓存中取出一条待处理位置命令。
  * @param cmd 输出命令缓存。
  * @retval 1表示取到命令，0表示无待处理命令。
  * @note   临界区很短，只复制结构体并清pending，避免主循环读到半包。
  */
static uint8_t CAN_App_TakePendingPositionCommand(CAN_App_PositionCmd_t *cmd)
{
  uint8_t has_pending;

  if (cmd == 0)
  {
    return 0U;
  }

  __disable_irq();
  has_pending = s_can_app.position_pending;
  if (has_pending != 0U)
  {
    *cmd = s_can_app.position_cmd;
    s_can_app.position_pending = 0U;
  }
  __enable_irq();

  return has_pending;
}

/**
  * @brief 在主循环中应用最近收到的CAN FD三电机位置命令。
  * @retval None
  */
static void CAN_App_ApplyPendingPositionCommand(void)
{
  CAN_App_PositionCmd_t cmd;
  float target_rad[CAN_APP_TARGET_MOTOR_COUNT];
  uint8_t motor_mask;
  uint8_t motor_id;

  if (CAN_App_TakePendingPositionCommand(&cmd) == 0U)
  {
    return;
  }

  motor_mask = (cmd.motor_mask == 0U) ? 0x07U : cmd.motor_mask;

  if (((cmd.flags & (CAN_APP_FLAG_STOP | CAN_APP_FLAG_ESTOP)) != 0U) ||
      (cmd.command == CAN_APP_CMD_STOP))
  {
    CAN_App_StopMotors(motor_mask);
    /* 收到H7的新STOP后再清故障锁存，保证H7一定看见本次BUS-OFF。 */
    s_can_bus_off_latched = 0U;
    return;
  }

  /* 置零：当前物理姿态定义为H7坐标0点，不修改编码器零点和FOC电角度。 */
  if (cmd.command == CAN_APP_CMD_ZERO)
  {
    int32_t zero_q[CAN_APP_TARGET_MOTOR_COUNT] = {0};

    for (motor_id = 0U; motor_id < CAN_APP_TARGET_MOTOR_COUNT; motor_id++)
    {
      zero_q[motor_id] = 0;
    }

    if (Motor_Core_RestorePositionFeedbackQ(zero_q) == 0U)
    {
      s_can_app.error_count++;
      return;
    }

    s_can_app.last_relative_seq_valid = 0U;
    return;
  }

  /* 恢复：记录H7保存位置和G4当前原始角之间的关系，不启动PWM、不产生运动。 */
  if (cmd.command == CAN_APP_CMD_RESTORE)
  {
    if ((s_can_app.pwm_started_mask != 0U) || (s_can_app.control_active_mask != 0U))
    {
      s_can_app.error_count++;
      return;
    }

    if (Motor_Core_RestorePositionFeedbackQ(cmd.target_raw) == 0U)
    {
      s_can_app.error_count++;
      return;
    }

    s_can_app.last_relative_seq_valid = 0U;
    return;
  }

  if (((cmd.flags & (CAN_APP_FLAG_ENABLE | CAN_APP_FLAG_APPLY)) !=
       (CAN_APP_FLAG_ENABLE | CAN_APP_FLAG_APPLY)) ||
      (motor_mask != 0x07U))
  {
    s_can_app.error_count++;
    return;
  }

  if (((cmd.flags & CAN_APP_FLAG_RELATIVE) != 0U) &&
      (s_can_app.last_relative_seq_valid != 0U) &&
      (cmd.seq == s_can_app.last_relative_seq))
  {
    return;
  }

  /*
   * 配置目标和重置状态期间先禁止控制环运行，避免TIM7/ADC中断看到部分电机已启动后提前进入闭环。
   * 目标、模式和状态全部准备完毕后，再一次性置位control_active_mask。
   */
  s_can_app.control_active_mask &= (uint8_t)~motor_mask;

  if (CAN_App_EnsurePwmStarted(motor_mask) == 0U)
  {
    s_can_app.error_count++;
    return;
  }

  for (motor_id = 0U; motor_id < CAN_APP_TARGET_MOTOR_COUNT; motor_id++)
  {
    /* 回零：复用位置闭环执行链路，目标绝对位置固定为0。 */
    target_rad[motor_id] = (cmd.command == CAN_APP_CMD_HOME) ?
                           0.0f :
                           ((float)cmd.target_raw[motor_id] / CAN_APP_POSITION_SCALE);
  }

  for (motor_id = 0U; motor_id < CAN_APP_TARGET_MOTOR_COUNT; motor_id++)
  {
    Motor_Core_SetPositionSpeedLimit(
        motor_id,
        (float)cmd.speed_limit_q[motor_id] / CAN_APP_POSITION_SCALE);
  }

  if ((cmd.flags & CAN_APP_FLAG_RESET) != 0U)
  {
    for (motor_id = 0U; motor_id < CAN_APP_TARGET_MOTOR_COUNT; motor_id++)
    {
      Motor_Core_SetCurrentRef(motor_id, 0.0f, 0.0f);
      Motor_Core_SetSpeedRef(motor_id, 0.0f);
      Motor_Core_SetMode(motor_id, MOTOR_MODE_IDLE);
      (void)Motor_Core_OutputNeutralPwm(motor_id);
    }
  }

  if ((cmd.command == CAN_APP_CMD_POSITION) && ((cmd.flags & CAN_APP_FLAG_RELATIVE) != 0U))
  {
    Motor_Core_SetPositionRelativeAll(target_rad);
    s_can_app.last_relative_seq = cmd.seq;
    s_can_app.last_relative_seq_valid = 1U;
  }
  else
  {
    Motor_Core_SetPositionAbsoluteAll(target_rad);
  }

  s_can_app.control_active_mask |= motor_mask;
}

/**
  * @brief 恢复BUS-OFF后的FDCAN，只在主循环调用。
  * @return 1恢复成功，0本轮失败，后续继续按500ms重试。
  */
static uint8_t CAN_App_RecoverBus(void)
{
  if (hfdcan1.State == HAL_FDCAN_STATE_BUSY)
  {
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan1,
                                    FDCAN_TX_BUFFER0 |
                                    FDCAN_TX_BUFFER1 |
                                    FDCAN_TX_BUFFER2);
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
  * @brief 初始化CAN应用层：配置标准帧过滤器、启动FDCAN并开启RX FIFO0通知。
  * @retval HAL状态。
  * @note 每块G4只接收自己的0x120+BOARD_ID命令，其他触手不再刷新本板在线状态。
  */
HAL_StatusTypeDef CAN_App_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = CAN_APP_ID_H7_POSITION_FD;
  filter.FilterID2 = CAN_APP_RX_FILTER_MASK;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                      FDCAN_IT_BUS_OFF,
                                      0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_can_app.initialized = 1U;
  s_status_tx_busy = 0U;
  s_can_bus_off_pending = 0U;
  s_can_bus_off_latched = 0U;
  s_can_bus_off_stop_done = 0U;
  s_can_last_recover_tick = 0U;
  s_can_app.last_status_tx_tick = HAL_GetTick();

  return HAL_OK;
}

/**
  * @brief TIM7固定分频后请求发送G4状态帧。
  * @retval None
  * @note 调用周期由TIM7保证为20Hz；Tx FIFO忙时直接跳过，不阻塞控制环。
  */
void CAN_App_RequestStatusTx(void)
{
  if (s_can_app.initialized == 0U)
  {
    return;
  }

  (void)CAN_App_SendStatusFrame();
}

/**
  * @brief CAN应用层轮询任务，当前负责在线超时判断和低频状态上报。
  * @note 放在main循环中调用即可，不要求固定周期；内部用HAL_GetTick限速。
  */
void CAN_App_Task(void)
{
  uint32_t now_tick;

  if (s_can_app.initialized == 0U)
  {
    return;
  }

  now_tick = HAL_GetTick();

  if (s_can_bus_off_pending != 0U)
  {
    /* BUS-OFF先在本板关PWM；保留HOME和多圈坐标。 */
    if (s_can_bus_off_stop_done == 0U)
    {
      CAN_App_StopMotors(0x07U);
      s_can_app.position_pending = 0U;
      s_can_bus_off_stop_done = 1U;
    }

    if ((now_tick - s_can_last_recover_tick) >= CAN_APP_BUS_RECOVER_PERIOD_MS)
    {
      s_can_last_recover_tick = now_tick;
      if (CAN_App_RecoverBus() != 0U)
      {
        s_can_bus_off_pending = 0U;
        s_can_bus_off_stop_done = 0U;
        s_can_app.last_status_tx_tick = now_tick;
      }
      else
      {
        s_can_app.error_count++;
      }
    }
    return;
  }

  CAN_App_ApplyPendingPositionCommand();

  if ((now_tick - s_can_app.last_rx_tick) > CAN_APP_ONLINE_TIMEOUT_MS)
  {
    s_can_app.online = 0U;
  }

  if ((now_tick - s_can_app.last_status_tx_tick) >= CAN_APP_STATUS_PERIOD_MS)
  {
    (void)CAN_App_SendStatusFrame();
  }
}

/**
  * @brief 获取CAN路径已经启动PWM并需要运行控制环的电机mask。
  * @retval bit0/1/2分别表示motor0/1/2是否由CAN位置命令启动。
  */
uint8_t CAN_App_GetActiveMotorMask(void)
{
  return s_can_app.control_active_mask;
}

uint8_t CAN_App_IsOnline(void)
{
  return s_can_app.online;
}

uint32_t CAN_App_GetRxCount(void)
{
  return s_can_app.rx_count;
}

uint32_t CAN_App_GetTxCount(void)
{
  return s_can_app.tx_count;
}

uint32_t CAN_App_GetErrorCount(void)
{
  return s_can_app.error_count;
}

/**
  * @brief FDCAN RX FIFO0中断回调，取出所有已到达消息并交给can_app缓存。
  * @param hfdcan FDCAN句柄。
  * @param RxFifo0ITs RX FIFO0中断标志。
  * @note 中断中不执行电机控制、不发送VOFA、不做耗时处理。
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[64];

  if ((hfdcan != &hfdcan1) ||
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      s_can_app.error_count++;
      break;
    }

    CAN_App_ProcessRxFrame(&rx_header, rx_data);
  }
}

/**
  * @brief BUS-OFF中断只置位，停止电机和外设恢复全部放到主循环。
  */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
  if ((hfdcan != &hfdcan1) ||
      ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0U))
  {
    return;
  }

  if (s_can_bus_off_pending == 0U)
  {
    s_can_last_recover_tick = HAL_GetTick() - CAN_APP_BUS_RECOVER_PERIOD_MS;
  }
  s_can_app.online = 0U;
  s_can_app.error_count++;
  s_can_bus_off_latched = 1U;
  s_can_bus_off_pending = 1U;
}
