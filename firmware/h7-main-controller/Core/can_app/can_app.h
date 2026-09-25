#ifndef CAN_APP_H
#define CAN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CAN_APP_G4_BOARD_COUNT      4U
#define CAN_APP_H7_CMD_BASE_ID      0x120U
#define CAN_APP_G4_STATUS_BASE_ID   0x180U
#define CAN_APP_H7_CMD_ID           CAN_APP_H7_CMD_BASE_ID
#define CAN_APP_G4_STATUS_ID        CAN_APP_G4_STATUS_BASE_ID

#define CAN_APP_PROTOCOL_VER    2U
#define CAN_APP_CMD_POSITION    0x01U
#define CAN_APP_CMD_STOP        0x02U
#define CAN_APP_CMD_ZERO        0x03U
#define CAN_APP_CMD_HOME        0x04U
#define CAN_APP_CMD_RESTORE     0x05U

#define CAN_APP_FLAG_ENABLE     0x01U
#define CAN_APP_FLAG_STOP       0x02U
#define CAN_APP_FLAG_RELATIVE   0x04U
#define CAN_APP_FLAG_APPLY      0x08U
#define CAN_APP_FLAG_RESET      0x10U
/* CANPOS 默认走连续轨迹更新，不每帧清控制状态；需要重新开启动作段时再显式带 RESET。 */
#define CAN_APP_FLAG_POS_APPLY  (CAN_APP_FLAG_ENABLE | CAN_APP_FLAG_RELATIVE | CAN_APP_FLAG_APPLY)
#define CAN_APP_FLAG_POS_RESET_APPLY  (CAN_APP_FLAG_POS_APPLY | CAN_APP_FLAG_RESET)

#define CAN_APP_MOTOR_MASK_ALL  0x07U
#define CAN_APP_POS_SCALE       100.0f       // CANPOS单位：1 count = 0.01 rad

typedef struct
{
  uint8_t valid;
  uint32_t last_rx_tick_ms;
  uint32_t rx_frame_count;

  uint8_t protocol_version;
  uint8_t board_id;
  uint8_t online;
  uint8_t control_word;
  uint16_t g4_rx_count_low16;
  uint16_t g4_tx_count_low16;
  uint32_t error_count;
  uint8_t pwm_started_mask;
  uint8_t last_relative_seq;
  uint8_t last_relative_seq_valid;
  uint8_t position_restore_valid; /* 1表示G4已收到H7 ACTUAL，actual_q可信。 */
  uint8_t stall_fault_mask;       /* bit0/1/2分别表示G4电机1/2/3触发堵转保护。 */
  uint8_t can_bus_off_latched;    /* 1表示G4发生过BUS-OFF，等待H7下发STOP确认。 */
  uint8_t position_pending;
  uint8_t last_position_seq;
  uint8_t last_position_flags;
  uint8_t last_position_motor_mask;
  uint8_t last_position_command;
  int32_t actual_q[3];          /* G4三路实际多圈位置，单位0.01rad；显示前统一转换为线坐标。 */
} Can_App_G4Status_t;

uint8_t Can_App_Init(void);
void Can_App_Task(void);
uint8_t Can_App_IsReady(void);
uint8_t Can_App_TakeBusOffEvent(void);
uint8_t Can_App_CopyG4StatusById(uint8_t board_id, Can_App_G4Status_t *snapshot);
uint8_t Can_App_GetLastTxSeqById(uint8_t board_id);
uint8_t Can_App_HasTxFifoSpace(uint8_t frame_count);
uint8_t Can_App_IsTxFifoEmpty(void);
uint8_t Can_App_SendMotorTargetsQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q);    /* 线坐标相对位置：正数收线，负数放线。 */
uint8_t Can_App_SendMotorTargetsResetQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q); /* 线坐标相对位置：带RESET，正数收线。 */
uint8_t Can_App_SendMotorTargetsResetQToBoard(uint8_t board_id, int32_t motor0_q, int32_t motor1_q, int32_t motor2_q); /* 指定G4板的线坐标相对位置RESET命令。 */
uint8_t Can_App_SendMotorTargetsAbsQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q); /* 线坐标绝对位置：正数收线。 */
uint8_t Can_App_SendMotorTargetsAbsSpeedQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q, int32_t speed_limit_q); /* 线坐标绝对位置：附带速度上限，正数收线。 */
uint8_t Can_App_SendMotorTargetsAbsSpeedQToBoard(uint8_t board_id, int32_t motor0_q, int32_t motor1_q, int32_t motor2_q, int32_t speed_limit_q); /* 指定G4板的线坐标绝对位置命令。 */
uint8_t Can_App_SendMotorTargetsAbsResetQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q); /* 线坐标绝对位置：带RESET，正数收线。 */
uint8_t Can_App_SendMotorTargetsAbsResetQToBoard(uint8_t board_id, int32_t motor0_q, int32_t motor1_q, int32_t motor2_q); /* 指定G4板的线坐标绝对位置RESET命令。 */
uint8_t Can_App_SendMotorTargetsRad(float motor0_rad, float motor1_rad, float motor2_rad);
uint8_t Can_App_SendStop(void);
uint8_t Can_App_SendStopToBoard(uint8_t board_id);
uint8_t Can_App_SendSetZero(void); /* 置零：当前位置作为G4三路编码器零点，不运动。 */
uint8_t Can_App_SendSetZeroToBoard(uint8_t board_id);
uint8_t Can_App_SendHome(void);    /* 回零：三路电机运动到绝对0位置。 */
uint8_t Can_App_SendHomeToBoard(uint8_t board_id);
uint8_t Can_App_SendRestoreActualQ(int32_t motor0_q, int32_t motor1_q, int32_t motor2_q); /* 恢复G4实际多圈坐标，不运动。 */
uint8_t Can_App_SendRestoreActualQToBoard(uint8_t board_id, int32_t motor0_q, int32_t motor1_q, int32_t motor2_q);
uint32_t Can_App_GetTxCount(void);
const Can_App_G4Status_t *Can_App_GetG4Status(void);
const Can_App_G4Status_t *Can_App_GetG4StatusById(uint8_t board_id);
uint8_t Can_App_IsG4Online(uint32_t timeout_ms);
uint8_t Can_App_IsG4OnlineById(uint8_t board_id, uint32_t timeout_ms);
void Can_App_PositionToLineQ(uint8_t board_id, const int32_t position_q[3], int32_t line_q[3]);

#ifdef __cplusplus
}
#endif

#endif
