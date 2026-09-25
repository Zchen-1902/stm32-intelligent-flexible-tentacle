#ifndef MOTOR_CONTROL_APP_H
#define MOTOR_CONTROL_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  int32_t bend_gain_q;          /* 最大弯曲幅度，单位0.01rad。 */
  int32_t stiff_gain_q;         /* 三绳共同预紧量，单位0.01rad。 */
  int32_t input_deadband;       /* 手中心死区，范围0~100。 */
  int32_t auto_dir_full;        /* 手偏移达到该值后方向响应拉满，范围1~100。 */
  int32_t motor_speed_base_q;   /* 基础速度，单位rad/s*100；radius<=auto_dir_full时使用。 */
  int32_t motor_speed_max_q;    /* 最大速度，单位rad/s*100；radius=100时使用。 */
} MotorControl_Config_t;

typedef struct
{
  uint8_t enable;
  uint8_t control_source;       /* 控制来源：AI传感器或PC上位机。 */
  uint8_t work_mode;            /* 0单触手，1双触手协同。 */
  uint8_t active_mask;          /* bit0~bit3分别表示触手1~4是否参与运动。 */
  uint8_t mirror_enable;        /* 双触手或四触手中心模式是否交换对应触手的M1和M3。 */
  uint8_t selected_tentacle;    /* 当前H7输出选择的触手编号，1~N。 */
  uint8_t selected_board_id;    /* 当前触手对应的G4 board_id。 */
  int16_t input_x;             /* 手中心X方向，范围 -100~100。 */
  int16_t input_y;             /* 手中心Y方向，范围 -100~100。 */
  uint8_t bend_0_100;          /* 弯曲程度，0=不弯，100=最大弯曲。 */
  uint8_t stiffness_0_100;     /* 刚度/预紧程度，0=最软，100=最硬。 */
  uint8_t coop_stage;          /* COOP离散抓取档位：0无，1初始化上翘，2 bend0，3 bend50，4 bend100。 */
  uint8_t coop_open_0_100;     /* COOP判断用张开程度，0=握拳，100=张开。 */
  int32_t motor_speed_limit_q;  /* 下发给G4的位置运动速度上限，单位rad/s*100。 */
  int32_t neutral_q[3];        /* HOME初始点，来自设置初始状态时的G4 actual，单位 0.01rad。 */
  int32_t target_q[3];         /* HOME + 映射偏移后的G4绝对目标，单位 0.01rad。 */
  int32_t pull_q[3];           /* 相对HOME的拉紧偏移，正数表示收绳/拉紧，单位 0.01rad。 */
  int32_t output_q[3];         /* 最近一次发送给G4的绝对目标，单位 0.01rad。 */
  uint32_t tx_count;
  uint8_t last_tx_status;
  uint8_t block_reason;        /* 当前不能运动的原因，使用MOTOR_CONTROL_BLOCK_xxx。 */
  uint8_t hand_present;        /* 当前是否检测到有效手部。 */
  uint8_t distance_zone;       /* 0无效，1近距离，2远距离。 */
  uint8_t select_armed;        /* 中心死区稳定后置1，允许抓握虚拟按键。 */
  uint8_t select_event;        /* 0无，1近单击，2近双击，3远单击，4远双击。 */
  uint32_t select_count;       /* 选择事件累计计数。 */
} MotorControl_Status_t;

#define MOTOR_CONTROL_WORK_SOLO       0U
#define MOTOR_CONTROL_WORK_COOP       1U

#define MOTOR_CONTROL_SOURCE_AI       0U
#define MOTOR_CONTROL_SOURCE_PC       1U
#define MOTOR_CONTROL_SOURCE_ACT      2U

/* 当前运动被阻止的原因。 */
#define MOTOR_CONTROL_BLOCK_NONE       0U
#define MOTOR_CONTROL_BLOCK_NOT_READY  1U
#define MOTOR_CONTROL_BLOCK_TX_BUSY    2U
#define MOTOR_CONTROL_BLOCK_FAULT      3U

/* 运行中真正停止的原因，与命令阻止原因分开。 */
#define MOTOR_CONTROL_STOP_NONE          0U
#define MOTOR_CONTROL_STOP_CAN_TIMEOUT   1U
#define MOTOR_CONTROL_STOP_FAULT         2U
#define MOTOR_CONTROL_STOP_RESTORE_LOST  3U
#define MOTOR_CONTROL_STOP_ESTOP         4U
#define MOTOR_CONTROL_STOP_GROUP_SYNC    5U
#define MOTOR_CONTROL_STOP_CAN_BUS_OFF   6U

typedef struct
{
  uint8_t reason; /* 使用MOTOR_CONTROL_STOP_xxx。 */
  uint8_t mask;   /* bit0~bit3分别表示触手1~4。 */
} MotorControl_StopNotice_t;

#define MOTOR_CONTROL_T1_MASK         0x01U
#define MOTOR_CONTROL_T2_MASK         0x02U
#define MOTOR_CONTROL_T3_MASK         0x04U
#define MOTOR_CONTROL_T4_MASK         0x08U
#define MOTOR_CONTROL_DUAL12_MASK     0x03U
#define MOTOR_CONTROL_DUAL34_MASK     0x0CU
#define MOTOR_CONTROL_QUAD_MASK       0x0FU

#define MOTOR_CONTROL_RELATION_NONE    0U /* 单触手，无触手间联动关系。 */
#define MOTOR_CONTROL_RELATION_SAME    1U /* 多触手使用相同三路目标。 */
#define MOTOR_CONTROL_RELATION_MIRROR  2U /* 双触手交换M1/M3，实现镜像运动。 */
#define MOTOR_CONTROL_RELATION_CENTER  3U /* 四触手按中心对称关系运动。 */

typedef struct
{
  uint8_t relation;    /* 当前工作组关系，使用MOTOR_CONTROL_RELATION_xxx。 */
  uint8_t online_mask; /* bit0~bit3表示触手1~4当前是否在线。 */
  uint8_t ready_mask;  /* bit0~bit3表示触手坐标恢复完成，可以安全控制。 */
  uint8_t fault_mask;  /* bit0~bit3表示对应触手是否存在堵转故障。 */
} MotorControl_Summary_t;

#define MOTOR_CONTROL_POSE_COUNT          4U
#define MOTOR_CONTROL_POSE_STATUS_ONLINE  0x01U /* G4状态帧新鲜且在线。 */
#define MOTOR_CONTROL_POSE_STATUS_ACTIVE  0x02U /* 属于当前工作组。 */
#define MOTOR_CONTROL_POSE_STATUS_READY   0x04U /* 坐标恢复完成，位置可信。 */
#define MOTOR_CONTROL_POSE_STATUS_FAULT   0x08U /* 至少一台电机堵转。 */

typedef struct
{
  uint8_t status; /* 使用MOTOR_CONTROL_POSE_STATUS_xxx组合。 */
  int32_t dq[3];  /* 相对各自HOME的位置；1q=0.01rad，正数表示收线。 */
} MotorControl_Pose_t;

#define MOTOR_CONTROL_ACT_POSITION_VALID 0x01U /* actual_q坐标已经恢复可信。 */
#define MOTOR_CONTROL_ACT_ACTION_VALID   0x02U /* action_q是最近成功入CAN队列的目标。 */
#define MOTOR_CONTROL_ACT_FAULT          0x04U /* 所选G4至少一台电机堵转。 */

/**
 * @brief ACT一帧需要的所选触手电机数据。
 *
 * actual_q和action_q都使用H7统一线坐标并减去该触手HOME，
 * 1q=0.01rad，正数表示收线，负数表示放线。
 */
typedef struct
{
  uint8_t tentacle;       /* 触手编号，范围1~4。 */
  uint8_t valid_flags;    /* MOTOR_CONTROL_ACT_xxx组合。 */
  uint16_t status_age_ms; /* 最近CAN状态年龄，最大65535ms。 */
  int32_t actual_q[3];    /* 三台电机当前实际位置。 */
  int32_t action_q[3];    /* 三台电机最近成功下发目标。 */
} MotorControl_ActData_t;

#define MOTOR_CONTROL_RUN_CONTROL      0U
#define MOTOR_CONTROL_RUN_KEY_MODE     1U

#define MOTOR_CONTROL_KEY_STATE_IDLE   0U
#define MOTOR_CONTROL_KEY_STATE_LOCKED 1U
#define MOTOR_CONTROL_KEY_STATE_MODE   2U
#define MOTOR_CONTROL_KEY_STATE_INIT   3U
#define MOTOR_CONTROL_KEY_STATE_READY  4U

#define MOTOR_CONTROL_MODE_NONE        0U
#define MOTOR_CONTROL_MODE_COOP        1U
#define MOTOR_CONTROL_MODE_SOLO        2U
#define MOTOR_CONTROL_MODE_WRAP        3U

#define MOTOR_CONTROL_KEY_NONE         0U
#define MOTOR_CONTROL_KEY_1            1U
#define MOTOR_CONTROL_KEY_2            2U
#define MOTOR_CONTROL_KEY_3            3U
#define MOTOR_CONTROL_KEY_4            4U

typedef struct
{
  uint8_t run_state;              /* CONTROL 或 KEY_MODE。 */
  uint8_t key_state;              /* 锁定、已选模式、已初始化、准备开始。 */
  uint8_t selected_mode;          /* NONE/COOP/SOLO/WRAP。 */
  uint8_t selected_tentacle;      /* 单触手模式下当前触手，0/1。 */
  uint8_t init_done;              /* 1表示已经完成初始化选择。 */
  uint8_t last_key;               /* 最近一次识别到的虚拟按键。 */
  uint8_t pulse_state;            /* 张握识别阶段：0等张开，1等握拳，2等释放。 */
  uint8_t pulse_stable;           /* 当前阶段已连续满足的帧数。 */
  uint8_t raw_bend;               /* 最近一次用于按键识别的未滤波弯曲值。 */
  uint8_t pending_zone;           /* 等待单击确认的距离区：0无，1近，2远。 */
  uint16_t pending_wait_ms;        /* 单击确认剩余等待时间，单位ms。 */
  int16_t coop_init_angle_deg;    /* 协同初始化时由x换算出的预览角度。 */
  uint32_t notice_count;          /* 状态变化计数，用于VOFA只打印一次。 */
} MotorControl_KeyInfo_t;

void MotorControl_Init(void);
uint8_t MotorControl_SelectWork(uint8_t active_mask, uint8_t mirror_enable);
uint8_t MotorControl_SetWorkMode(uint8_t mode);
uint8_t MotorControl_GetWorkMode(void);
uint8_t MotorControl_SetSelectedTentacle(uint8_t tentacle);
uint8_t MotorControl_GetSelectedTentacle(void);
uint8_t MotorControl_GetSelectedBoardId(void);
uint8_t MotorControl_StopActive(void);
uint8_t MotorControl_EmergencyStop(void);
uint8_t MotorControl_TakeStopNotice(MotorControl_StopNotice_t *notice);
uint8_t MotorControl_HomeActive(void);
uint8_t MotorControl_SendRelativeActive(int32_t q0, int32_t q1, int32_t q2);
/**
 * @brief 向指定单触手发送ACT绝对目标。
 * @param tentacle 触手编号，当前模型固定使用触手3。
 * @param q0/q1/q2 相对该触手HOME的线坐标目标，1q=0.01rad，正数收线。
 * @return 0成功，1状态或参数错误，2发送FIFO暂忙，3发送失败。
 * @note 本函数只接受ACT控制来源，不进行相对位置累加。
 */
uint8_t MotorControl_SendActTarget(uint8_t tentacle,
                                   int32_t q0,
                                   int32_t q1,
                                   int32_t q2);
void MotorControl_SetEnable(uint8_t enable);
uint8_t MotorControl_SetControlSource(uint8_t source);
uint8_t MotorControl_GetControlSource(void);
void MotorControl_GetSummary(MotorControl_Summary_t *summary);
void MotorControl_GetPoses(MotorControl_Pose_t poses[MOTOR_CONTROL_POSE_COUNT]);
/**
 * @brief 复制当前所选触手的ACT电机数据。
 * @param tentacle 需要读取的触手编号，范围1~4。
 * @param data 接收数据的结构体地址；NULL时不执行。
 * @note 只读取RAM和CAN快照，不发送CAN、不访问W25、不会阻塞等待。
 */
void MotorControl_CopyActData(uint8_t tentacle, MotorControl_ActData_t *data);
uint8_t MotorControl_ToggleKeyMode(void);
void MotorControl_ClearKeySelection(void);
uint8_t MotorControl_RunCoopInit(int16_t x);
uint8_t MotorControl_RunCoopBend(uint8_t bend, int16_t x);
void MotorControl_SetNeutralQ(int32_t q0, int32_t q1, int32_t q2);
void MotorControl_SetVirtualInput(int16_t x, int16_t y, uint8_t bend, uint8_t stiffness);
void MotorControl_ClearRestorePending(void);
uint8_t MotorControl_IsRestorePending(void);
void MotorControl_StartRestoreConfirmQ(int32_t q0, int32_t q1, int32_t q2);
void MotorControl_UpdateRestoreActualQ(int32_t q0, int32_t q1, int32_t q2);
void MotorControl_Task(void);
const MotorControl_Status_t *MotorControl_GetStatus(void);
const MotorControl_Config_t *MotorControl_GetConfig(void);
const MotorControl_KeyInfo_t *MotorControl_GetKeyInfo(void);
uint8_t MotorControl_TakeKeyNotice(MotorControl_KeyInfo_t *info);
uint8_t MotorControl_TakeKeyDebug(MotorControl_KeyInfo_t *info);
uint8_t MotorControl_SetConfig(const MotorControl_Config_t *config, uint8_t save_to_flash);

#ifdef __cplusplus
}
#endif

#endif
