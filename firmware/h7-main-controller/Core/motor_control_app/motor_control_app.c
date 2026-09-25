#include "motor_control_app.h"

#include "can_app.h"
#include "vl53_app.h"
#include "w25q64_app.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define MCTRL_INPUT_LIMIT              100
#define MCTRL_INPUT_DEADBAND_DEFAULT   12L    /* 默认中心死区，手在中心附近时不改变弯曲方向。 */
#define MCTRL_AUTO_DIR_FULL_DEFAULT    40L    /* 默认方向拉满半径，超过该半径后方向快速跟随。 */
#define MCTRL_BEND_GAIN_Q_DEFAULT      27000L /* 默认最大方向弯曲量，单位0.01rad，即270rad。 */
#define MCTRL_STIFF_GAIN_Q_DEFAULT     2000L   /* 默认最大共同拉紧量，单位0.01rad，即20rad。 */
#define MCTRL_SPEED_BASE_Q_DEFAULT     3500L  /* 默认基础速度，单位rad/s*100；radius<=auto_dir_full时为25rad/s。 */
#define MCTRL_SPEED_MAX_Q_DEFAULT      5000L  /* 默认最大速度，单位rad/s*100；radius=100时为45rad/s。 */
#define MCTRL_RELEASE_LIMIT_Q          30000L /* 最大放绳量，单位0.01rad；高于默认弯曲量，避免反向控制被缩小。 */
#define MCTRL_PULL_SCALE_Q             10000L /* 弯曲分量统一缩放系数，10000表示不缩放。 */
#define MCTRL_COOP_STAGE_NONE          0U
#define MCTRL_COOP_STAGE_INIT          1U     /* 张开85~100：双触手到初始化张开姿态。 */
#define MCTRL_COOP_STAGE_BEND0         2U     /* 张开50~85：bend=0中立弯曲状态，方向仍由x控制。 */
#define MCTRL_COOP_STAGE_BEND50        3U     /* 张开15~50：双触手半抓取。 */
#define MCTRL_COOP_STAGE_BEND100       4U     /* 张开0~15：双触手深抓取。 */
#define MCTRL_COOP_STAGE_STABLE_FRAMES 3U     /* 连续稳定帧数，防止阈值附近抖动反复发目标。 */
#define MCTRL_COOP_X_UPDATE_STEP       8      /* 同一档位内x变化超过该值才更新目标，避免CAN目标抖动。 */
#define MCTRL_COOP_INIT_X_DEFAULT      (-71)  /* 初始化方向：电机2收缩方向，用于双触手预张开。 */
#define MCTRL_COOP_INIT_Y_DEFAULT      (-71)
#define MCTRL_COOP_INIT_BEND_DEFAULT   80U    /* 初始化上翘幅度：张开手时固定为bend=80。 */
#define MCTRL_COOP_WORK_M1_X           (-26)  /* 包裹方向左端：触手1向电机1方向弯。 */
#define MCTRL_COOP_WORK_M1_Y           97
#define MCTRL_COOP_WORK_M3_X           97     /* 包裹方向右端：触手1向电机3方向弯。 */
#define MCTRL_COOP_WORK_M3_Y           (-26)
#define MCTRL_COOP_GRIP_STIFF_DEFAULT  0U     /* 第一版抓取不额外共同预紧，减少发热和不确定性。 */
#define MCTRL_TENTACLE_MAX_COUNT       4U
#define MCTRL_TENTACLE_ACTIVE_COUNT    CAN_APP_G4_BOARD_COUNT

#define MCTRL_AUTO_DIR_ALPHA_SLOW_NUM   1L
#define MCTRL_AUTO_DIR_ALPHA_FAST_NUM   4L
#define MCTRL_AUTO_DIR_ALPHA_DEN        10L
#define MCTRL_AUTO_BEND_ALPHA_NUM       1L
#define MCTRL_AUTO_BEND_ALPHA_DEN       4L
#define MCTRL_AUTO_STIFF_ALPHA_NUM      1L
#define MCTRL_AUTO_STIFF_ALPHA_DEN      5L
#define MCTRL_SELECT_DISTANCE_SPLIT_MM  320U
#define MCTRL_CENTER_SELECT_HOLD_MS     600U
#define MCTRL_GRIP_LOW_TH               20U
#define MCTRL_GRIP_HIGH_TH              80U
#define MCTRL_GRIP_RELEASE_TH           25U
#define MCTRL_GRIP_PULSE_TIMEOUT_MS     1000U
#define MCTRL_GRIP_DOUBLE_WINDOW_MS     1200U
#define MCTRL_KEY_NEAR_MIN_MM           220U
#define MCTRL_KEY_NEAR_MAX_MM           320U
#define MCTRL_KEY_FAR_MIN_MM            350U
#define MCTRL_KEY_FAR_MAX_MM            450U
#define MCTRL_VIRTUAL_KEY_TRIGGER_ENABLE 0U     /* 0：关闭手势自动进入按键模式；1：恢复自动触发。 */
#define MCTRL_KEY_OPEN_TH               35U    /* 虚拟按键张开阈值，bend低表示手张开。 */
#define MCTRL_KEY_FIST_TH               65U    /* 虚拟按键握拳阈值，bend高表示握拳。 */
#define MCTRL_KEY_RELEASE_TH            40U    /* 释放阈值略高于张开阈值，便于完成张-握-张。 */
#define MCTRL_KEY_STABLE_FRAMES         1U     /* 临时使用1帧触发，先验证虚拟按键链路是否能进入。 */
#define MCTRL_KEY_PULSE_TIMEOUT_MS      1000U
#define MCTRL_KEY_DOUBLE_WINDOW_MS      1500U
#define MCTRL_KEY_SINGLE_LEAVE_MS       300U   /* 单击pending后离开该距离区超过300ms，提前确认单击。 */
#define MCTRL_KEY_DBG_PERIOD_MS         200U   /* 张握识别调试输出周期，只在一次有效动作期间打开。 */
#define MCTRL_KEY_MODE_X_LEFT           (-33)
#define MCTRL_KEY_MODE_X_RIGHT          33
#define MCTRL_KEY_COOP_ANGLE_MAX_DEG    45L
#define MCTRL_KEY_GRIP_WAIT_OPEN        0U
#define MCTRL_KEY_GRIP_WAIT_FIST        1U
#define MCTRL_KEY_GRIP_WAIT_RELEASE     2U
#define MCTRL_G4_STATUS_MAX_AGE_MS      1000U   /* 允许短时状态帧延迟，避免瞬时CAN抖动误判离线。 */
#define MCTRL_G4_STOP_AGE_MS            3000U   /* 运行中连续超时达到3秒才真正停止。 */
#define MCTRL_CAN_TIMEOUT_STOP_ENABLE   0U      /* 临时关闭CAN状态超时自动停机，仅保留告警。 */
#define MCTRL_POSITION_SAVE_PERIOD_MS   750U
#define MCTRL_RESTORE_RETRY_MS          100U    /* 自动恢复G4实际坐标的重发间隔，避免连续发送RESTORE。 */
#define MCTRL_RESTORE_TOL_Q             20L     /* 判断G4坐标恢复完成的容差，单位0.01rad；20表示0.20rad。 */
#define MCTRL_RESTORE_INVALID_LIMIT     10U     /* 只累计新的无效状态帧。 */
#define MCTRL_RESTORE_INVALID_MIN_MS    500U    /* 10帧同时至少持续500ms，避免50Hz状态快速误判。 */
#define MCTRL_SAFETY_CHECK_PERIOD_MS    20U     /* 50Hz检查运行健康，只读取RAM状态。 */
#define MCTRL_SAFETY_STOP_PERIOD_MS     100U    /* STOP补发间隔。 */
#define MCTRL_SAFETY_STOP_MAX_RETRY     5U      /* 每次安全事件最多入队5轮。 */

#define MCTRL_DISTANCE_ZONE_NONE        0U
#define MCTRL_DISTANCE_ZONE_NEAR        1U
#define MCTRL_DISTANCE_ZONE_FAR         2U
#define MCTRL_SELECT_EVENT_NONE         0U
#define MCTRL_GRIP_STATE_IDLE           0U
#define MCTRL_GRIP_STATE_WAIT_RELEASE   1U

typedef struct
{
  uint8_t board_id;             /* 当前触手对应的G4板号，后续扩展四触手只改映射表和启用数量。 */
} MotorControl_TentacleRoute_t;

static MotorControl_Status_t g_mctrl;
static MotorControl_Config_t g_mctrl_config;
static const MotorControl_TentacleRoute_t s_mctrl_tentacle_route[MCTRL_TENTACLE_MAX_COUNT] =
{
  {0U},
  {1U},
  {2U},
  {3U},
};
static uint8_t s_mctrl_selected_tentacle = 0U;
static uint32_t s_mctrl_last_ai_frame = 0U;
static int16_t s_mctrl_auto_dir_x = 0;
static int16_t s_mctrl_auto_dir_y = 0;
static uint8_t s_mctrl_auto_dir_valid = 0U;
static uint8_t s_mctrl_auto_filter_valid = 0U;
static uint8_t s_mctrl_auto_bend = 0U;
static uint8_t s_mctrl_auto_stiff = 0U;
static uint8_t s_mctrl_center_enter_valid = 0U;
static uint32_t s_mctrl_center_enter_ms = 0U;
static uint8_t s_mctrl_grip_state = MCTRL_GRIP_STATE_IDLE;
static uint8_t s_mctrl_grip_seen_low = 0U;
static uint32_t s_mctrl_grip_high_ms = 0U;
static uint8_t s_mctrl_select_pending = 0U;
static uint8_t s_mctrl_select_pending_zone = MCTRL_DISTANCE_ZONE_NONE;
static uint32_t s_mctrl_select_pending_ms = 0U;
static uint32_t s_mctrl_last_position_save_ms = 0U;                           /* ACTUAL写入节流：所有触手共用，避免连续写W25。 */
static uint8_t s_mctrl_position_save_cursor = 0U;                             /* ACTUAL保存轮询游标，每次最多保存一个触手。 */
static int32_t s_mctrl_home_q[MCTRL_TENTACLE_MAX_COUNT][3] = {{0}};           /* 每个触手独立HOME，单位0.01rad。 */
static uint8_t s_mctrl_restore_pending[MCTRL_TENTACLE_MAX_COUNT] = {0};       /* 每个触手是否等待G4恢复ACTUAL。 */
static uint8_t s_mctrl_restore_done[MCTRL_TENTACLE_MAX_COUNT] = {0};          /* 每个触手G4坐标是否已恢复可信。 */
static uint8_t s_mctrl_actual_save_enabled[MCTRL_TENTACLE_MAX_COUNT] = {0};   /* 每个触手是否允许自动保存ACTUAL。 */
static uint8_t s_mctrl_actual_save_pending[MCTRL_TENTACLE_MAX_COUNT] = {0};   /* 每个触手是否有待写入的最新ACTUAL。 */
static int32_t s_mctrl_pending_actual_q[MCTRL_TENTACLE_MAX_COUNT][3] = {{0}}; /* 待写入W25的最新ACTUAL，单位0.01rad。 */
static uint32_t s_mctrl_restore_last_try_ms[MCTRL_TENTACLE_MAX_COUNT] = {0};  /* 每个触手RESTORE重发节流。 */
static int32_t s_mctrl_restore_actual_q[MCTRL_TENTACLE_MAX_COUNT][3] = {{0}}; /* 每个触手断电前ACTUAL，单位0.01rad。 */
static uint8_t s_mctrl_restore_invalid_count[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* 每个触手连续收到恢复无效新帧的次数。 */
static uint32_t s_mctrl_restore_last_rx_frame_count[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* 防止同一状态帧被重复计数。 */
static uint32_t s_mctrl_restore_invalid_since_ms[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* 第一帧恢复无效的时间。 */
static uint32_t s_mctrl_position_last_rx_frame_count[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* ACTUAL只采集一次新状态帧。 */
static uint8_t s_mctrl_restore_is_startup[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* 1表示整机上电恢复，可优先信任仍有效的G4坐标。 */
static uint8_t s_mctrl_ever_synced[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* H7本次上电后是否完成过一次G4坐标同步。 */
static uint8_t s_mctrl_link_lost[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* 已同步后连续3秒收不到状态帧。 */
static uint8_t s_mctrl_position_unknown[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* G4连续报告坐标无效。 */
static uint8_t s_mctrl_reconnect_stop_sent[MCTRL_TENTACLE_MAX_COUNT] = {0}; /* 每次重连只启动一轮STOP补发。 */
static uint8_t s_mctrl_link_warning_mask = 0U; /* 状态年龄超过1秒的触手，仅记录不停止。 */
static uint8_t s_mctrl_safety_stop_pending = 0U; /* 运行中异常，等待STOP确认。 */
static uint8_t s_mctrl_safety_stop_retry_count = 0U;
static uint32_t s_mctrl_safety_last_check_ms = 0U;
static uint32_t s_mctrl_safety_last_stop_ms = 0U;
static uint8_t s_mctrl_stop_reason = MOTOR_CONTROL_STOP_NONE; /* 锁存到下一次成功启动。 */
static uint8_t s_mctrl_stop_mask = 0U;
static uint8_t s_mctrl_stop_notice_pending = 0U;
static uint8_t s_mctrl_output_valid = 0U; /* 最近绝对目标是否确实进入CAN发送队列。 */
static MotorControl_KeyInfo_t s_mctrl_key_info;
static uint8_t s_mctrl_key_notice_pending = 0U;
static uint8_t s_mctrl_key_grip_state = MCTRL_KEY_GRIP_WAIT_OPEN;
static uint8_t s_mctrl_key_stable_count = 0U;
static uint32_t s_mctrl_key_grip_ms = 0U;
static uint8_t s_mctrl_key_pending_zone = MCTRL_DISTANCE_ZONE_NONE;
static uint32_t s_mctrl_key_pending_ms = 0U;
static uint32_t s_mctrl_key_pending_leave_ms = 0U;
static int16_t s_mctrl_key_pending_x = 0;
static int16_t s_mctrl_key_pending_y = 0;
static uint8_t s_mctrl_key_entry_pending = 0U;  /* 普通控制模式下，进入按键模式需要双击确认。 */
static uint32_t s_mctrl_key_entry_ms = 0U;
static uint8_t s_mctrl_key_dbg_active = 0U;
static uint32_t s_mctrl_key_dbg_last_ms = 0U;
static uint8_t s_mctrl_coop_active_stage = MCTRL_COOP_STAGE_NONE;
static uint8_t s_mctrl_coop_candidate_stage = MCTRL_COOP_STAGE_NONE;
static uint8_t s_mctrl_coop_candidate_count = 0U;
static int16_t s_mctrl_coop_active_x = 0;

static uint8_t MotorControl_IsG4StatusFresh(const Can_App_G4Status_t *status, uint32_t now_ms);
static uint8_t MotorControl_GetSelectedBoardIdInternal(void);
static void MotorControl_UpdateRouteStatus(void);
static void MotorControl_RebuildTarget(void);
static uint8_t MotorControl_HasRestorePendingForIndex(uint8_t index);
static uint8_t MotorControl_IsCoopReady(void);
static uint8_t MotorControl_CheckActiveState(uint8_t *all_stopped);
static uint8_t MotorControl_GetActiveBlockReason(void);
static void MotorControl_UpdateSafetyStop(uint32_t now_ms);
static void MotorControl_ResetAutoState(void);
static void MotorControl_SendTargetOnce(void);

/**
  * @brief  原子复制一块G4的最新状态。
  * @param  board_id G4板号。
  * @param  snapshot 输出状态快照。
  * @note   临界区只复制RAM，不计算、不打印，避免明显影响CAN中断。
  */
static void MotorControl_CopyG4Status(uint8_t board_id,
                                      Can_App_G4Status_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }
  (void)Can_App_CopyG4StatusById(board_id, snapshot);
}

/**
  * @brief  将int64差值安全限制到int32范围。
  * @param  value 待转换差值。
  * @return 饱和后的int32数值。
  */
static int32_t MotorControl_SaturateI64ToI32(int64_t value)
{
  if (value > 2147483647LL)
  {
    return 2147483647L;
  }
  if (value < (-2147483647LL - 1LL))
  {
    return (-2147483647L - 1L);
  }
  return (int32_t)value;
}

/**
  * @brief  装载映射控制默认配置。
  * @param  无。
  */
static void MotorControl_LoadDefaultConfig(void)
{
  g_mctrl_config.bend_gain_q = MCTRL_BEND_GAIN_Q_DEFAULT;
  g_mctrl_config.stiff_gain_q = MCTRL_STIFF_GAIN_Q_DEFAULT;
  g_mctrl_config.input_deadband = MCTRL_INPUT_DEADBAND_DEFAULT;
  g_mctrl_config.auto_dir_full = MCTRL_AUTO_DIR_FULL_DEFAULT;
  g_mctrl_config.motor_speed_base_q = MCTRL_SPEED_BASE_Q_DEFAULT;
  g_mctrl_config.motor_speed_max_q = MCTRL_SPEED_MAX_Q_DEFAULT;
}

/**
  * @brief  获取当前选中触手对应的G4板号。
  * @return G4 board_id；越界时回退到0，避免错误命令访问无效状态数组。
  */
static uint8_t MotorControl_GetSelectedBoardIdInternal(void)
{
  if ((s_mctrl_selected_tentacle >= MCTRL_TENTACLE_ACTIVE_COUNT) ||
      (s_mctrl_selected_tentacle >= MCTRL_TENTACLE_MAX_COUNT))
  {
    return 0U;
  }

  return s_mctrl_tentacle_route[s_mctrl_selected_tentacle].board_id;
}

/**
  * @brief  同步当前触手路由到公开状态，便于VOFA查看。
  */
static void MotorControl_UpdateRouteStatus(void)
{
  g_mctrl.selected_tentacle = (uint8_t)(s_mctrl_selected_tentacle + 1U);
  g_mctrl.selected_board_id = MotorControl_GetSelectedBoardIdInternal();
}

/**
  * @brief  判断触手数组下标是否在当前启用范围内。
  * @param  index 触手数组下标，从0开始。
  * @return 1有效，0无效。
  */
static uint8_t MotorControl_IsValidTentacleIndex(uint8_t index)
{
  return ((index < MCTRL_TENTACLE_ACTIVE_COUNT) &&
          (index < MCTRL_TENTACLE_MAX_COUNT)) ? 1U : 0U;
}

/**
  * @brief  判断指定触手是否属于当前工作组。
  * @param  index 触手数组下标，从0开始。
  * @return 1表示参与当前运动，0表示不参与。
  */
static uint8_t MotorControl_IsActive(uint8_t index)
{
  return ((MotorControl_IsValidTentacleIndex(index) != 0U) &&
          ((g_mctrl.active_mask & (uint8_t)(1U << index)) != 0U)) ? 1U : 0U;
}

/**
  * @brief  统计当前工作组需要发送的CAN控制帧数量。
  * @return 当前参与运动的触手数量。
  */
static uint8_t MotorControl_GetActiveCount(void)
{
  uint8_t count = 0U;
  uint8_t i;

  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    if (MotorControl_IsActive(i) != 0U)
    {
      count++;
    }
  }

  return count;
}

/**
  * @brief  获取当前工作组中编号最小的触手。
  * @return 触手数组下标；工作组异常时回退到TSEL选择。
  */
static uint8_t MotorControl_GetFirstActiveIndex(void)
{
  uint8_t i;

  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    if (MotorControl_IsActive(i) != 0U)
    {
      return i;
    }
  }

  return s_mctrl_selected_tentacle;
}

/**
  * @brief  根据当前工作关系生成指定触手的三路目标。
  * @note   双触手镜像时交换第二条触手的M1/M3；
  *         四触手中心模式时交换触手2和触手4的M1/M3，M2保持不变。
  */
static void MotorControl_MapActiveTarget(uint8_t index,
                                         const int32_t src[3],
                                         int32_t dst[3])
{
  int64_t sum_q;
  int32_t common_q;
  uint8_t same_second;
  uint8_t mirror_second;

  if ((src == NULL) || (dst == NULL))
  {
    return;
  }

  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];

  /*
   * 双触手同向运动时，第二条触手因安装方向相差180度，
   * 只反转弯曲差分量，三绳共同预紧量保持不变。
   */
  same_second = (((g_mctrl.active_mask == MOTOR_CONTROL_DUAL12_MASK) &&
                  (index == 1U)) ||
                 ((g_mctrl.active_mask == MOTOR_CONTROL_DUAL34_MASK) &&
                  (index == 3U))) &&
                (g_mctrl.mirror_enable == 0U);
  if (same_second != 0U)
  {
    sum_q = (int64_t)src[0] + (int64_t)src[1] + (int64_t)src[2];
    common_q = (int32_t)(sum_q / 3);
    dst[0] = (int32_t)(((int64_t)common_q * 2) - src[0]);
    dst[1] = (int32_t)(((int64_t)common_q * 2) - src[1]);
    dst[2] = (int32_t)(((int64_t)common_q * 2) - src[2]);
  }

  mirror_second = ((((g_mctrl.active_mask == MOTOR_CONTROL_DUAL12_MASK) && (index == 1U)) ||
                    ((g_mctrl.active_mask == MOTOR_CONTROL_DUAL34_MASK) && (index == 3U)) ||
                    ((g_mctrl.active_mask == MOTOR_CONTROL_QUAD_MASK) &&
                     ((index == 1U) || (index == 3U)))) &&
                   (g_mctrl.mirror_enable != 0U)) ? 1U : 0U;
  if (mirror_second != 0U)
  {
    dst[0] = src[2];
    dst[2] = src[0];
  }
}

/**
  * @brief  判断指定触手是否具备运动条件。
  * @param  index 触手数组下标，从0开始。
  * @param  status G4状态快照。
  * @param  now_ms 当前HAL时钟。
  * @return 1可运动，0尚未完成坐标恢复或状态已过期。
  */
static uint8_t MotorControl_IsTentacleReady(
  uint8_t index,
  const Can_App_G4Status_t *status,
  uint32_t now_ms)
{
  return ((MotorControl_IsValidTentacleIndex(index) != 0U) &&
          (status != NULL) &&
          (MotorControl_IsG4StatusFresh(status, now_ms) != 0U) &&
          (s_mctrl_ever_synced[index] != 0U) &&
          (s_mctrl_link_lost[index] == 0U) &&
          (s_mctrl_position_unknown[index] == 0U) &&
          (s_mctrl_restore_done[index] != 0U) &&
          (s_mctrl_restore_pending[index] == 0U) &&
          (status->can_bus_off_latched == 0U) &&
          (status->position_restore_valid != 0U)) ? 1U : 0U;
}

/**
  * @brief  接受G4回传的可信ACTUAL，并恢复H7侧保存资格。
  * @param  index 触手下标。
  * @param  status G4新鲜状态快照。
  * @note   正常及重连时G4始终是位置权威，H7不会用旧ACTUAL反向覆盖。
  */
static void MotorControl_AcceptG4Actual(
  uint8_t index,
  const Can_App_G4Status_t *status)
{
  uint8_t j;

  if ((MotorControl_IsValidTentacleIndex(index) == 0U) ||
      (status == NULL) || (status->valid == 0U) ||
      (status->position_restore_valid == 0U))
  {
    return;
  }

  for (j = 0U; j < 3U; j++)
  {
    s_mctrl_restore_actual_q[index][j] = status->actual_q[j];
    s_mctrl_pending_actual_q[index][j] = status->actual_q[j];
  }

  s_mctrl_restore_pending[index] = 0U;
  s_mctrl_restore_done[index] = 1U;
  s_mctrl_restore_is_startup[index] = 0U;
  s_mctrl_actual_save_enabled[index] = 1U;
  s_mctrl_actual_save_pending[index] = 1U;
  s_mctrl_restore_invalid_count[index] = 0U;
  s_mctrl_restore_invalid_since_ms[index] = 0U;
  s_mctrl_restore_last_rx_frame_count[index] = status->rx_frame_count;
  s_mctrl_position_last_rx_frame_count[index] = status->rx_frame_count;
  s_mctrl_ever_synced[index] = 1U;
  s_mctrl_link_lost[index] = 0U;
  s_mctrl_position_unknown[index] = 0U;
  s_mctrl_reconnect_stop_sent[index] = 0U;
}

/**
  * @brief  启动一轮有限STOP补发。
  */
static void MotorControl_StartStopBurst(uint32_t now_ms)
{
  s_mctrl_safety_stop_pending = 1U;
  s_mctrl_safety_stop_retry_count = 0U;
  s_mctrl_safety_last_stop_ms =
    now_ms - MCTRL_SAFETY_STOP_PERIOD_MS;
}

/**
  * @brief  锁存一次安全停止；同一原因只合并触手mask，不重复重启STOP流程。
  */
static void MotorControl_LatchSafetyStop(
  uint8_t reason,
  uint8_t mask,
  uint32_t now_ms)
{
  if (s_mctrl_stop_reason == reason)
  {
    s_mctrl_stop_mask |= mask;
    return;
  }

  if (s_mctrl_stop_reason != MOTOR_CONTROL_STOP_NONE)
  {
    return;
  }

  s_mctrl_stop_reason = reason;
  s_mctrl_stop_mask = mask;
  s_mctrl_stop_notice_pending = 1U;
  g_mctrl.enable = 0U;
  g_mctrl.block_reason =
    (reason == MOTOR_CONTROL_STOP_FAULT) ?
    MOTOR_CONTROL_BLOCK_FAULT : MOTOR_CONTROL_BLOCK_NOT_READY;
  MotorControl_ResetAutoState();
  MotorControl_StartStopBurst(now_ms);
}

/**
  * @brief  检查活动触手的运动许可和停止状态。
  * @param  all_stopped 可选输出；1表示活动G4均在线且PWM全部停止。
  * @return NONE、NOT_READY或FAULT。
  * @note   一次遍历同时获得两种状态，避免重复复制CAN状态。
  */
static uint8_t MotorControl_CheckActiveState(uint8_t *all_stopped)
{
  uint32_t now_ms = HAL_GetTick();
  uint8_t active_found = 0U;
  uint8_t not_ready = 0U;
  uint8_t fault = 0U;
  uint8_t stopped = 1U;
  uint8_t i;

  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    Can_App_G4Status_t status;

    if (MotorControl_IsActive(i) == 0U)
    {
      continue;
    }

    active_found = 1U;
    MotorControl_CopyG4Status(s_mctrl_tentacle_route[i].board_id, &status);

    if (MotorControl_IsG4StatusFresh(&status, now_ms) == 0U)
    {
      not_ready = 1U;
      stopped = 0U;
      continue;
    }

    if (status.pwm_started_mask != 0U)
    {
      stopped = 0U;
    }

    if (status.stall_fault_mask != 0U)
    {
      fault = 1U;
    }

    if (MotorControl_IsTentacleReady(i, &status, now_ms) == 0U)
    {
      not_ready = 1U;
    }
  }

  if (all_stopped != NULL)
  {
    *all_stopped = ((active_found != 0U) && (stopped != 0U)) ? 1U : 0U;
  }

  if (fault != 0U)
  {
    return MOTOR_CONTROL_BLOCK_FAULT;
  }

  if ((active_found == 0U) || (not_ready != 0U))
  {
    return MOTOR_CONTROL_BLOCK_NOT_READY;
  }

  return MOTOR_CONTROL_BLOCK_NONE;
}

/**
  * @brief  获取当前工作组不能运动的原因。
  */
static uint8_t MotorControl_GetActiveBlockReason(void)
{
  /* 旧运动的安全STOP尚未结束时，禁止启动下一轮控制。 */
  if (s_mctrl_safety_stop_pending != 0U)
  {
    return MOTOR_CONTROL_BLOCK_NOT_READY;
  }

  return MotorControl_CheckActiveState(NULL);
}

/**
  * @brief  检查运行健康；启动READY只用于MCTRL 1，不参与短时运行波动。
  */
static void MotorControl_UpdateRuntimeHealth(uint32_t now_ms)
{
  uint8_t warning_mask = 0U;
  uint8_t timeout_mask = 0U;
  uint8_t fault_mask = 0U;
  uint8_t bus_off_mask = 0U;
  uint8_t restore_lost_mask = 0U;
  uint8_t i;

  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    Can_App_G4Status_t status;
    uint8_t bit = (uint8_t)(1U << i);
    uint32_t age_ms;

    /* 从未同步过表示仍在上电等待，不是运行中断连。 */
    if (s_mctrl_ever_synced[i] == 0U)
    {
      continue;
    }

    MotorControl_CopyG4Status(s_mctrl_tentacle_route[i].board_id,
                              &status);
    age_ms = (status.valid != 0U) ?
             (now_ms - status.last_rx_tick_ms) : 0xFFFFFFFFUL;

    if (age_ms > MCTRL_G4_STOP_AGE_MS)
    {
      warning_mask |= bit;

      /* 比赛联调：状态断流不停止控制，也不进入重连STOP流程。 */
#if (MCTRL_CAN_TIMEOUT_STOP_ENABLE != 0U)
      if (s_mctrl_link_lost[i] == 0U)
      {
        s_mctrl_link_lost[i] = 1U;
        s_mctrl_actual_save_enabled[i] = 0U;
        s_mctrl_reconnect_stop_sent[i] = 0U;
        if (MotorControl_IsActive(i) != 0U)
        {
          timeout_mask |= bit;
        }
      }
#endif
      continue;
    }

    if (age_ms > MCTRL_G4_STATUS_MAX_AGE_MS)
    {
      warning_mask |= bit; /* 1~3秒只记录，不停止也不阻止MVIRT。 */
      continue;
    }

    if ((MotorControl_IsActive(i) != 0U) &&
        (status.stall_fault_mask != 0U))
    {
      fault_mask |= bit;
    }

    if ((MotorControl_IsActive(i) != 0U) &&
        (status.can_bus_off_latched != 0U))
    {
      bus_off_mask |= bit;
      continue;
    }

    if (s_mctrl_link_lost[i] != 0U)
    {
      /* 3秒断连后的首次重连先清除G4可能残留的旧运动。 */
      if (s_mctrl_reconnect_stop_sent[i] == 0U)
      {
        s_mctrl_reconnect_stop_sent[i] = 1U;
        MotorControl_StartStopBurst(now_ms);
      }

      if ((status.pwm_started_mask == 0U) &&
          (status.position_restore_valid != 0U))
      {
        MotorControl_AcceptG4Actual(i, &status);
      }
    }

    if (status.position_restore_valid != 0U)
    {
      s_mctrl_restore_invalid_count[i] = 0U;
      s_mctrl_restore_invalid_since_ms[i] = 0U;
      if (s_mctrl_position_unknown[i] != 0U)
      {
        MotorControl_AcceptG4Actual(i, &status);
      }
      continue;
    }

    if (status.rx_frame_count ==
        s_mctrl_restore_last_rx_frame_count[i])
    {
      continue;
    }
    s_mctrl_restore_last_rx_frame_count[i] = status.rx_frame_count;

    if (s_mctrl_restore_invalid_count[i] == 0U)
    {
      s_mctrl_restore_invalid_since_ms[i] = now_ms;
    }
    if (s_mctrl_restore_invalid_count[i] <
        MCTRL_RESTORE_INVALID_LIMIT)
    {
      s_mctrl_restore_invalid_count[i]++;
    }

    if ((s_mctrl_restore_invalid_count[i] >=
         MCTRL_RESTORE_INVALID_LIMIT) &&
        ((now_ms - s_mctrl_restore_invalid_since_ms[i]) >=
         MCTRL_RESTORE_INVALID_MIN_MS))
    {
      s_mctrl_position_unknown[i] = 1U;
      s_mctrl_restore_done[i] = 0U;
      s_mctrl_restore_pending[i] = 0U; /* 运行中禁止自动反写旧ACTUAL。 */
      s_mctrl_actual_save_enabled[i] = 0U;
      s_mctrl_actual_save_pending[i] = 0U;
      if (MotorControl_IsActive(i) != 0U)
      {
        restore_lost_mask |= bit;
      }
    }
  }

  /* 仅在告警边沿变化时更新，持续抖动不重复处理。 */
  if (s_mctrl_link_warning_mask != warning_mask)
  {
    s_mctrl_link_warning_mask = warning_mask;
  }

  if (fault_mask != 0U)
  {
    if ((g_mctrl.enable != 0U) ||
        (s_mctrl_stop_reason == MOTOR_CONTROL_STOP_FAULT))
    {
      MotorControl_LatchSafetyStop(
        MOTOR_CONTROL_STOP_FAULT, fault_mask, now_ms);
    }
  }
  else if (bus_off_mask != 0U)
  {
    MotorControl_LatchSafetyStop(MOTOR_CONTROL_STOP_CAN_BUS_OFF,
                                 bus_off_mask,
                                 now_ms);
  }
  else if (restore_lost_mask != 0U)
  {
    if ((g_mctrl.enable != 0U) ||
        (s_mctrl_stop_reason == MOTOR_CONTROL_STOP_RESTORE_LOST))
    {
      MotorControl_LatchSafetyStop(
        MOTOR_CONTROL_STOP_RESTORE_LOST,
        restore_lost_mask,
        now_ms);
    }
  }
  else if (timeout_mask != 0U)
  {
    if ((g_mctrl.enable != 0U) ||
        (s_mctrl_stop_reason == MOTOR_CONTROL_STOP_CAN_TIMEOUT))
    {
      MotorControl_LatchSafetyStop(
        MOTOR_CONTROL_STOP_CAN_TIMEOUT,
        timeout_mask,
        now_ms);
    }
  }
}

/**
  * @brief  20ms检查运行健康，并按100ms间隔有限补发STOP。
  */
static void MotorControl_UpdateSafetyStop(uint32_t now_ms)
{
  uint8_t all_stopped = 0U;
  uint8_t active_count;

  if (Can_App_TakeBusOffEvent() != 0U)
  {
    MotorControl_LatchSafetyStop(MOTOR_CONTROL_STOP_CAN_BUS_OFF,
                                 g_mctrl.active_mask,
                                 now_ms);
  }

  if ((now_ms - s_mctrl_safety_last_check_ms) >=
      MCTRL_SAFETY_CHECK_PERIOD_MS)
  {
    s_mctrl_safety_last_check_ms = now_ms;
    MotorControl_UpdateRuntimeHealth(now_ms);
  }

  if (s_mctrl_safety_stop_pending == 0U)
  {
    return;
  }

  (void)MotorControl_CheckActiveState(&all_stopped);
  if (all_stopped != 0U)
  {
    s_mctrl_safety_stop_pending = 0U;
    return;
  }

  if (s_mctrl_safety_stop_retry_count >=
      MCTRL_SAFETY_STOP_MAX_RETRY)
  {
    s_mctrl_safety_stop_pending = 0U;
    return;
  }

  if ((now_ms - s_mctrl_safety_last_stop_ms) <
      MCTRL_SAFETY_STOP_PERIOD_MS)
  {
    return;
  }

  active_count = MotorControl_GetActiveCount();
  if (s_mctrl_safety_stop_retry_count == 0U)
  {
    if (Can_App_HasTxFifoSpace(active_count) == 0U)
    {
      g_mctrl.last_tx_status = 2U;
      return;
    }
  }
  else if (Can_App_IsTxFifoEmpty() == 0U)
  {
    return;
  }

  s_mctrl_safety_last_stop_ms = now_ms;
  s_mctrl_safety_stop_retry_count++;
  g_mctrl.last_tx_status = MotorControl_StopActive();
}

/**
  * @brief  查询指定触手是否还在等待G4坐标恢复。
  * @param  index 触手数组下标，从0开始。
  * @return 1表示等待恢复，0表示可控制。
  */
static uint8_t MotorControl_HasRestorePendingForIndex(uint8_t index)
{
  if (MotorControl_IsValidTentacleIndex(index) == 0U)
  {
    return 0U;
  }

  return s_mctrl_restore_pending[index];
}

/**
  * @brief  计算指定触手的绝对目标。
  * @param  index 触手数组下标。
  * @param  pull_q 三绳拉紧偏移，正数收绳，单位0.01rad。
  * @param  target_q 输出绝对目标，单位0.01rad。
  * @note   H7统一使用线坐标：正数收线、负数放线，所以目标=HOME+pull。
  */
static void MotorControl_BuildTargetForTentacle(uint8_t index,
                                                const int32_t pull_q[3],
                                                int32_t target_q[3])
{
  uint8_t i;

  if ((pull_q == 0) || (target_q == 0) ||
      (MotorControl_IsValidTentacleIndex(index) == 0U))
  {
    return;
  }

  for (i = 0U; i < 3U; i++)
  {
    target_q[i] = s_mctrl_home_q[index][i] + pull_q[i];
  }
}

/**
  * @brief  将当前选中触手的HOME装载到公开状态。
  * @param  无。
  * @note   g_mctrl.neutral_q始终表示当前TSEL选中的触手HOME。
  */
static void MotorControl_LoadSelectedHomeToStatus(void)
{
  uint8_t i;

  if (MotorControl_IsValidTentacleIndex(s_mctrl_selected_tentacle) == 0U)
  {
    return;
  }

  for (i = 0U; i < 3U; i++)
  {
    g_mctrl.neutral_q[i] = s_mctrl_home_q[s_mctrl_selected_tentacle][i];
  }

  MotorControl_RebuildTarget();
}

/**
  * @brief  判断G4上报的实际位置是否已经恢复到断电前ACTUAL附近。
  * @param  tentacle_index 触手数组下标，从0开始。
  * @param  status G4状态指针。
  * @return 1表示恢复完成，0表示尚未完成。
  */
static uint8_t MotorControl_IsRestoreReached(uint8_t tentacle_index, const Can_App_G4Status_t *status)
{
  uint8_t i;

  if ((MotorControl_IsValidTentacleIndex(tentacle_index) == 0U) ||
      (status == NULL) || (status->valid == 0U))
  {
    return 0U;
  }

  if (status->position_restore_valid == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < 3U; i++)
  {
    int32_t err = status->actual_q[i] - s_mctrl_restore_actual_q[tentacle_index][i];
    if (err < 0)
    {
      err = -err;
    }
    if (err > MCTRL_RESTORE_TOL_Q)
    {
      return 0U;
    }
  }

  return 1U;
}

/**
  * @brief  上电后自动把H7保存的ACTUAL下发给G4，恢复G4多圈实际坐标。
  * @param  now_ms 当前HAL tick。
  * @note   该流程只改G4坐标基准，不让电机运动；恢复完成前暂停ACTUAL自动保存。
  */
static void MotorControl_RestoreG4Task(uint32_t now_ms)
{
  const Can_App_G4Status_t *status;
  uint8_t i;

  for (i = 0U; (i < MCTRL_TENTACLE_ACTIVE_COUNT) && (i < MCTRL_TENTACLE_MAX_COUNT); i++)
  {
    uint8_t board_id = s_mctrl_tentacle_route[i].board_id;

    status = Can_App_GetG4StatusById(board_id);

    if (s_mctrl_restore_pending[i] == 0U)
    {
      continue;
    }

    if (MotorControl_IsG4StatusFresh(status, now_ms) == 0U)
    {
      continue;
    }

    /* H7重启但G4仍有可信坐标时，必须采用G4 ACTUAL，不能用W25旧值覆盖。 */
    if ((s_mctrl_restore_is_startup[i] != 0U) &&
        (status->position_restore_valid != 0U))
    {
      MotorControl_AcceptG4Actual(i, status);
      continue;
    }

    if (MotorControl_IsRestoreReached(i, status) != 0U)
    {
      MotorControl_AcceptG4Actual(i, status);
      continue;
    }

    /* 控制运行中禁止改坐标；人工恢复也必须先停机。 */
    if (g_mctrl.enable != 0U)
    {
      continue;
    }

    if ((now_ms - s_mctrl_restore_last_try_ms[i]) < MCTRL_RESTORE_RETRY_MS)
    {
      continue;
    }
    s_mctrl_restore_last_try_ms[i] = now_ms;

    /* 上电初期先清除G4可能残留的旧运动，再恢复坐标。 */
    if (status->pwm_started_mask != 0U)
    {
      (void)Can_App_SendStopToBoard(board_id);
      continue;
    }

    (void)Can_App_SendRestoreActualQToBoard(board_id,
                                            s_mctrl_restore_actual_q[i][0],
                                            s_mctrl_restore_actual_q[i][1],
                                            s_mctrl_restore_actual_q[i][2]);
  }
}

/**
  * @brief  对int32数值做上下限保护。
  * @param  value 输入值。
  * @param  min_v 最小允许值。
  * @param  max_v 最大允许值。
  * @return 限幅后的数值。
  */
static int32_t MotorControl_ClampI32(int32_t value, int32_t min_v, int32_t max_v)
{
  if (value < min_v)
  {
    return min_v;
  }

  if (value > max_v)
  {
    return max_v;
  }

  return value;
}

/**
  * @brief  限制虚拟手部输入范围。
  * @param  value 原始方向输入。
  * @return 限制到 -100~100 后的输入。
  */
static int16_t MotorControl_ClampInput(int16_t value)
{
  if (value < -MCTRL_INPUT_LIMIT)
  {
    return -MCTRL_INPUT_LIMIT;
  }

  if (value > MCTRL_INPUT_LIMIT)
  {
    return MCTRL_INPUT_LIMIT;
  }

  return value;
}

/**
  * @brief  计算uint32整数平方根，用于把x/y归一化为方向向量。
  * @param  value 输入无符号整数。
  * @return floor(sqrt(value))。
  * @note   这里只在H7低频映射任务中使用，不在电机快环中使用。
  */
static uint32_t MotorControl_IsqrtU32(uint32_t value)
{
  uint32_t root = 0U;
  uint32_t bit = 1UL << 30;

  while (bit > value)
  {
    bit >>= 2;
  }

  while (bit != 0U)
  {
    if (value >= (root + bit))
    {
      value -= root + bit;
      root = (root >> 1) + bit;
    }
    else
    {
      root >>= 1;
    }
    bit >>= 2;
  }

  return root;
}

/**
  * @brief  计算手部x/y输入的半径平方。
  * @param  x/y 输入控制量，范围会先限制到-100~100。
  * @return 半径平方。
  */
static uint32_t MotorControl_InputRadiusSq(int16_t x, int16_t y)
{
  int32_t cx = (int32_t)MotorControl_ClampInput(x);
  int32_t cy = (int32_t)MotorControl_ClampInput(y);

  return (uint32_t)((cx * cx) + (cy * cy));
}

/**
  * @brief  根据手中心偏离半径计算电机运动速度上限。
  * @param  radius 手平面偏离中心的幅值，范围0~100。
  * @return 速度上限，单位rad/s*100。
  * @note   radius<=auto_dir_full时保持基础速度；继续远离中心时线性增加到最大速度。
  */
static int32_t MotorControl_CalcSpeedLimitQ(uint32_t radius)
{
  int32_t full = g_mctrl_config.auto_dir_full;
  int32_t base_q = g_mctrl_config.motor_speed_base_q;
  int32_t max_q = g_mctrl_config.motor_speed_max_q;
  int32_t radius_i32;

  if (radius > (uint32_t)MCTRL_INPUT_LIMIT)
  {
    radius = MCTRL_INPUT_LIMIT;
  }

  if ((max_q <= base_q) || (full >= MCTRL_INPUT_LIMIT) || ((int32_t)radius <= full))
  {
    return base_q;
  }

  radius_i32 = (int32_t)radius;
  return base_q + (((max_q - base_q) * (radius_i32 - full)) / (MCTRL_INPUT_LIMIT - full));
}

/**
  * @brief  将手部x/y输入归一化为方向向量。
  * @param  raw_x/raw_y 原始平面输入。
  * @param  dir_x/dir_y 输出方向，范围-100~100。
  * @note   x/y只表达方向，不表达弯曲幅度；弯曲幅度由bend单独决定。
  */
static void MotorControl_NormalizeInput(int16_t raw_x, int16_t raw_y, int16_t *dir_x, int16_t *dir_y)
{
  int32_t x = (int32_t)MotorControl_ClampInput(raw_x);
  int32_t y = (int32_t)MotorControl_ClampInput(raw_y);
  uint32_t radius = MotorControl_IsqrtU32((uint32_t)((x * x) + (y * y)));

  if ((dir_x == NULL) || (dir_y == NULL))
  {
    return;
  }

  if (radius < (uint32_t)g_mctrl_config.input_deadband)
  {
    *dir_x = 0;
    *dir_y = 0;
    return;
  }

  *dir_x = (int16_t)MotorControl_ClampI32((x * 100L) / (int32_t)radius,
                                          -MCTRL_INPUT_LIMIT,
                                          MCTRL_INPUT_LIMIT);
  *dir_y = (int16_t)MotorControl_ClampI32((y * 100L) / (int32_t)radius,
                                          -MCTRL_INPUT_LIMIT,
                                          MCTRL_INPUT_LIMIT);
}

/**
  * @brief  一阶整数滤波，带最小1步更新，避免小误差被整数除法吃掉。
  * @param  prev 上一次滤波值。
  * @param  raw 当前原始值。
  * @param  alpha_num/alpha_den 滤波系数分子/分母。
  * @return 滤波后的值。
  */
static int32_t MotorControl_FilterI32(int32_t prev, int32_t raw, int32_t alpha_num, int32_t alpha_den)
{
  int32_t delta;
  int32_t step;

  if ((alpha_num <= 0L) || (alpha_den <= 0L))
  {
    return raw;
  }

  delta = raw - prev;
  step = (delta * alpha_num) / alpha_den;
  if ((step == 0L) && (delta != 0L))
  {
    step = (delta > 0L) ? 1L : -1L;
  }

  return prev + step;
}

/**
  * @brief  判断G4状态帧是否足够新，可用于掉电位置保存。
  * @param  status G4状态指针。
  * @param  now_ms 当前HAL tick。
  * @return 1表示状态有效且未超时。
  */
static uint8_t MotorControl_IsG4StatusFresh(const Can_App_G4Status_t *status, uint32_t now_ms)
{
  if ((status == NULL) || (status->valid == 0U))
  {
    return 0U;
  }

  return ((now_ms - status->last_rx_tick_ms) <= MCTRL_G4_STATUS_MAX_AGE_MS) ? 1U : 0U;
}

/**
  * @brief  周期保存G4实际多圈位置，用于掉电后恢复当前位置参考。
  * @param  now_ms 当前HAL tick。
  * @note   只保存ACTUAL记录，不改HOME初始零点；HOME只由CANZERO/MSETZERO写入。
  */
static void MotorControl_PositionSaveTask(uint32_t now_ms)
{
  const Can_App_G4Status_t *status;
  uint8_t i;
  uint8_t try_count;

  /*
   * 先只采集最新ACTUAL到pending，不立刻写W25。
   * 这样触手1/2同时在线时，不会在同一轮主循环连续写两次Flash。
   */
  for (i = 0U; (i < MCTRL_TENTACLE_ACTIVE_COUNT) && (i < MCTRL_TENTACLE_MAX_COUNT); i++)
  {
    uint8_t board_id = s_mctrl_tentacle_route[i].board_id;

    if ((s_mctrl_actual_save_enabled[i] == 0U) ||
        (s_mctrl_ever_synced[i] == 0U) ||
        (s_mctrl_link_lost[i] != 0U) ||
        (s_mctrl_position_unknown[i] != 0U))
    {
      continue;
    }

    status = Can_App_GetG4StatusById(board_id);
    if (MotorControl_IsG4StatusFresh(status, now_ms) == 0U)
    {
      continue;
    }

    if (status->position_restore_valid == 0U)
    {
      /* 单帧无效只等待健康检查确认，不能永久关闭ACTUAL保存。 */
      continue;
    }

    if (status->rx_frame_count ==
        s_mctrl_position_last_rx_frame_count[i])
    {
      continue;
    }
    s_mctrl_position_last_rx_frame_count[i] =
      status->rx_frame_count;

    s_mctrl_pending_actual_q[i][0] = status->actual_q[0];
    s_mctrl_pending_actual_q[i][1] = status->actual_q[1];
    s_mctrl_pending_actual_q[i][2] = status->actual_q[2];
    s_mctrl_actual_save_pending[i] = 1U;
  }

  if ((now_ms - s_mctrl_last_position_save_ms) < MCTRL_POSITION_SAVE_PERIOD_MS)
  {
    return;
  }
  s_mctrl_last_position_save_ms = now_ms;

  /*
   * 每750ms最多消费一个pending，并按触手轮询。
   * 写失败时pending不清除，下一轮继续尝试，避免丢失最近ACTUAL。
   */
  for (try_count = 0U;
       (try_count < MCTRL_TENTACLE_ACTIVE_COUNT) && (try_count < MCTRL_TENTACLE_MAX_COUNT);
       try_count++)
  {
    uint8_t index = s_mctrl_position_save_cursor;
    uint8_t board_id;

    s_mctrl_position_save_cursor++;
    if ((s_mctrl_position_save_cursor >= MCTRL_TENTACLE_ACTIVE_COUNT) ||
        (s_mctrl_position_save_cursor >= MCTRL_TENTACLE_MAX_COUNT))
    {
      s_mctrl_position_save_cursor = 0U;
    }

    if ((MotorControl_IsValidTentacleIndex(index) == 0U) ||
        (s_mctrl_actual_save_pending[index] == 0U))
    {
      continue;
    }

    board_id = s_mctrl_tentacle_route[index].board_id;
    if (W25Q64_App_SavePositionRecordForTentacle((uint8_t)(index + 1U),
                                                 board_id,
                                                 W25Q64_APP_RECORD_TYPE_ACTUAL,
                                                 s_mctrl_pending_actual_q[index],
                                                 NULL) == W25Q64_APP_OK)
    {
      s_mctrl_restore_actual_q[index][0] = s_mctrl_pending_actual_q[index][0];
      s_mctrl_restore_actual_q[index][1] = s_mctrl_pending_actual_q[index][1];
      s_mctrl_restore_actual_q[index][2] = s_mctrl_pending_actual_q[index][2];
      s_mctrl_actual_save_pending[index] = 0U;
    }

    break;
  }
}

/**
  * @brief  清空自动映射滤波状态。
  * @param  无。
  * @note   重新使能自动控制或回零时调用，避免沿用旧方向。
  */
static void MotorControl_ResetAutoState(void)
{
  s_mctrl_last_ai_frame = 0U;
  s_mctrl_auto_dir_x = 0;
  s_mctrl_auto_dir_y = 0;
  s_mctrl_auto_dir_valid = 0U;
  s_mctrl_auto_filter_valid = 0U;
  s_mctrl_auto_bend = 0U;
  s_mctrl_auto_stiff = 0U;
  s_mctrl_center_enter_valid = 0U;
  s_mctrl_grip_state = MCTRL_GRIP_STATE_IDLE;
  s_mctrl_grip_seen_low = 0U;
  s_mctrl_select_pending = 0U;
  s_mctrl_select_pending_zone = MCTRL_DISTANCE_ZONE_NONE;
  g_mctrl.motor_speed_limit_q = g_mctrl_config.motor_speed_base_q;
  g_mctrl.hand_present = 0U;
  g_mctrl.distance_zone = MCTRL_DISTANCE_ZONE_NONE;
  g_mctrl.select_armed = 0U;
  g_mctrl.select_event = MCTRL_SELECT_EVENT_NONE;
  g_mctrl.select_count = 0U;
  g_mctrl.coop_stage = MCTRL_COOP_STAGE_NONE;
  g_mctrl.coop_open_0_100 = 0U;
  s_mctrl_coop_active_stage = MCTRL_COOP_STAGE_NONE;
  s_mctrl_coop_candidate_stage = MCTRL_COOP_STAGE_NONE;
  s_mctrl_coop_candidate_count = 0U;
  s_mctrl_coop_active_x = 0;
}

/**
  * @brief  按张开程度划分COOP抓取离散档位。
  * @param  open_0_100 张开程度，0=握拳，100=张开。
  * @return COOP离散档位。
  * @note   这里按用户定义的四个区间执行，不做连续卷曲映射。
  */
static uint8_t MotorControl_CoopStageFromOpen(uint8_t open_0_100)
{
  if (open_0_100 >= 85U)
  {
    return MCTRL_COOP_STAGE_INIT;
  }

  if (open_0_100 >= 50U)
  {
    return MCTRL_COOP_STAGE_BEND0;
  }

  if (open_0_100 >= 15U)
  {
    return MCTRL_COOP_STAGE_BEND50;
  }

  return MCTRL_COOP_STAGE_BEND100;
}

/**
  * @brief  手动COOP命令的bend值换算成显示档位。
  * @param  bend_0_100 手动弯曲程度，0~100。
  * @return COOP显示档位。
  */
static uint8_t MotorControl_CoopStageFromManualBend(uint8_t bend_0_100)
{
  if (bend_0_100 == 0U)
  {
    return MCTRL_COOP_STAGE_BEND0;
  }

  if (bend_0_100 <= 50U)
  {
    return MCTRL_COOP_STAGE_BEND50;
  }

  return MCTRL_COOP_STAGE_BEND100;
}

/**
  * @brief  判断COOP档位是否需要使用手部x控制包裹方向。
  * @param  stage COOP离散档位。
  * @return 1表示需要使用x更新方向，0表示固定方向。
  */
static uint8_t MotorControl_CoopStageUsesWorkX(uint8_t stage)
{
  return ((stage == MCTRL_COOP_STAGE_BEND0) ||
          (stage == MCTRL_COOP_STAGE_BEND50) ||
          (stage == MCTRL_COOP_STAGE_BEND100)) ? 1U : 0U;
}

/**
  * @brief  根据手部x计算COOP包裹方向。
  * @param  raw_x 手部X位置，范围会限制到-100~100。
  * @param  x/y 输出方向输入。
  *
  * 说明：
  * 包裹方向只允许在电机1方向到电机3方向之间变化，不经过电机2。
  * x=-100对应电机1方向，x=100对应电机3方向，x=0为两者中间方向。
  */
static void MotorControl_CoopWorkDirFromX(int16_t raw_x, int16_t *x, int16_t *y)
{
  int32_t t;

  if ((x == 0) || (y == 0))
  {
    return;
  }

  t = (int32_t)MotorControl_ClampInput(raw_x) + 100L;
  *x = (int16_t)((((200L - t) * MCTRL_COOP_WORK_M1_X) +
                  (t * MCTRL_COOP_WORK_M3_X)) / 200L);
  *y = (int16_t)((((200L - t) * MCTRL_COOP_WORK_M1_Y) +
                  (t * MCTRL_COOP_WORK_M3_Y)) / 200L);
}

/**
  * @brief  将COOP离散档位换成虚拟输入点。
  * @param  stage COOP离散档位。
  * @param  work_x bend0/bend50/bend100使用的手部X位置。
  * @param  x/y 输出方向输入，范围-100~100。
  * @param  bend/stiff 输出弯曲和预紧，范围0~100。
  *
  * 说明：
  * - INIT：电机2方向初始化上翘，便于形成包裹前姿态。
  * - BEND0/BEND50/BEND100：沿电机1到电机3之间的包裹方向变化。
  */
static void MotorControl_CoopStageToInput(uint8_t stage,
                                          int16_t work_x,
                                          int16_t *x,
                                          int16_t *y,
                                          uint8_t *bend,
                                          uint8_t *stiff)
{
  if ((x == 0) || (y == 0) || (bend == 0) || (stiff == 0))
  {
    return;
  }

  *x = 0;
  *y = 0;
  *bend = 0U;
  *stiff = 0U;

  if (stage == MCTRL_COOP_STAGE_INIT)
  {
    *x = MCTRL_COOP_INIT_X_DEFAULT;
    *y = MCTRL_COOP_INIT_Y_DEFAULT;
    *bend = MCTRL_COOP_INIT_BEND_DEFAULT;
  }
  else if (stage == MCTRL_COOP_STAGE_BEND0)
  {
    MotorControl_CoopWorkDirFromX(work_x, x, y);
    *bend = 0U;
    *stiff = MCTRL_COOP_GRIP_STIFF_DEFAULT;
  }
  else if (stage == MCTRL_COOP_STAGE_BEND50)
  {
    MotorControl_CoopWorkDirFromX(work_x, x, y);
    *bend = 50U;
    *stiff = MCTRL_COOP_GRIP_STIFF_DEFAULT;
  }
  else if (stage == MCTRL_COOP_STAGE_BEND100)
  {
    MotorControl_CoopWorkDirFromX(work_x, x, y);
    *bend = 100U;
    *stiff = MCTRL_COOP_GRIP_STIFF_DEFAULT;
  }
}

/**
  * @brief  根据手部距离划分近/远选择区。
  * @param  z_mm 手部中心距离，单位mm。
  * @return 1近距离，2远距离。
  */
static uint8_t MotorControl_GetDistanceZone(uint16_t z_mm)
{
  return (z_mm <= MCTRL_SELECT_DISTANCE_SPLIT_MM) ? MCTRL_DISTANCE_ZONE_NEAR : MCTRL_DISTANCE_ZONE_FAR;
}

/**
  * @brief  标记按键状态发生变化，供VOFA低频打印一次。
  * @param  无。
  */
static void MotorControl_KeyNotice(void)
{
  s_mctrl_key_info.notice_count++;
  s_mctrl_key_notice_pending = 1U;
}

/**
  * @brief  根据距离判断虚拟按键区域。
  * @param  z_mm 手部中心距离，单位mm。
  * @return 1近距离，2远距离，0表示处于空白区。
  */
static uint8_t MotorControl_GetKeyDistanceZone(uint16_t z_mm)
{
  if ((z_mm >= MCTRL_KEY_NEAR_MIN_MM) && (z_mm <= MCTRL_KEY_NEAR_MAX_MM))
  {
    return MCTRL_DISTANCE_ZONE_NEAR;
  }

  if ((z_mm >= MCTRL_KEY_FAR_MIN_MM) && (z_mm <= MCTRL_KEY_FAR_MAX_MM))
  {
    return MCTRL_DISTANCE_ZONE_FAR;
  }

  return MCTRL_DISTANCE_ZONE_NONE;
}

/**
  * @brief  复位“张开-握拳-张开”的单次脉冲识别状态。
  * @param  无。
  */
static void MotorControl_ResetKeyPulse(void)
{
  s_mctrl_key_grip_state = MCTRL_KEY_GRIP_WAIT_OPEN;
  s_mctrl_key_stable_count = 0U;
  s_mctrl_key_grip_ms = 0U;
}

/**
  * @brief  打开虚拟按键调试输出。
  * @param  now_ms 当前HAL tick。
  */
static void MotorControl_KeyDebugStart(uint32_t now_ms)
{
  s_mctrl_key_dbg_active = 1U;
  s_mctrl_key_dbg_last_ms = now_ms - MCTRL_KEY_DBG_PERIOD_MS;
}

/**
  * @brief  关闭虚拟按键调试输出。
  * @param  无。
  */
static void MotorControl_KeyDebugStop(void)
{
  s_mctrl_key_dbg_active = 0U;
  s_mctrl_key_dbg_last_ms = 0U;
}

/**
  * @brief  刷新VOFA可查询的虚拟按键调试状态。
  * @param  raw_bend 最近一次未滤波弯曲值。
  * @param  now_ms 当前HAL tick。
  */
static void MotorControl_UpdateKeyDebug(uint8_t raw_bend, uint32_t now_ms)
{
  uint32_t elapsed_ms;

  s_mctrl_key_info.pulse_state = s_mctrl_key_grip_state;
  s_mctrl_key_info.pulse_stable = s_mctrl_key_stable_count;
  s_mctrl_key_info.raw_bend = raw_bend;
  s_mctrl_key_info.pending_zone = s_mctrl_key_pending_zone;

  if (s_mctrl_key_pending_zone == MCTRL_DISTANCE_ZONE_NONE)
  {
    s_mctrl_key_info.pending_wait_ms = 0U;
    return;
  }

  elapsed_ms = now_ms - s_mctrl_key_pending_ms;
  s_mctrl_key_info.pending_wait_ms =
    (elapsed_ms >= MCTRL_KEY_DOUBLE_WINDOW_MS) ? 0U :
    (uint16_t)(MCTRL_KEY_DOUBLE_WINDOW_MS - elapsed_ms);
}

/**
  * @brief  识别一次“张开-握拳-张开”手势脉冲。
  * @param  ai 当前AI估计结果。
  * @param  now_ms 当前HAL tick。
  * @return 1表示识别到完整脉冲，0表示尚未完成。
  * @note   bend_0_100=100-openness，因此张开是低值，握拳是高值。
  */
static uint8_t MotorControl_UpdateKeyPulse(const VL53_App_AiEstimate_t *ai, uint32_t now_ms)
{
  uint8_t bend;
  uint8_t matched = 0U;

  if (ai == NULL)
  {
    return 0U;
  }

  /* 虚拟按键需要响应快速张握，使用未滤波bend；映射控制仍使用滤波后的bend_0_100。 */
  bend = ai->bend_raw_0_100;

  if ((s_mctrl_key_grip_state != MCTRL_KEY_GRIP_WAIT_OPEN) &&
      ((now_ms - s_mctrl_key_grip_ms) > MCTRL_KEY_PULSE_TIMEOUT_MS))
  {
    MotorControl_ResetKeyPulse();
    MotorControl_UpdateKeyDebug(bend, now_ms);
    MotorControl_KeyDebugStop();
    return 0U;
  }

  if (s_mctrl_key_grip_state == MCTRL_KEY_GRIP_WAIT_OPEN)
  {
    matched = (bend <= MCTRL_KEY_OPEN_TH) ? 1U : 0U;
    if (matched != 0U)
    {
      if (s_mctrl_key_stable_count < MCTRL_KEY_STABLE_FRAMES)
      {
        s_mctrl_key_stable_count++;
      }

      if (s_mctrl_key_stable_count >= MCTRL_KEY_STABLE_FRAMES)
      {
        s_mctrl_key_grip_state = MCTRL_KEY_GRIP_WAIT_FIST;
        s_mctrl_key_stable_count = 0U;
        s_mctrl_key_grip_ms = now_ms;
      }
    }
    else
    {
      s_mctrl_key_stable_count = 0U;
    }
  }
  else if (s_mctrl_key_grip_state == MCTRL_KEY_GRIP_WAIT_FIST)
  {
    matched = (bend >= MCTRL_KEY_FIST_TH) ? 1U : 0U;
    if (matched != 0U)
    {
      if (s_mctrl_key_stable_count < MCTRL_KEY_STABLE_FRAMES)
      {
        s_mctrl_key_stable_count++;
      }

      if (s_mctrl_key_stable_count >= MCTRL_KEY_STABLE_FRAMES)
      {
        s_mctrl_key_grip_state = MCTRL_KEY_GRIP_WAIT_RELEASE;
        s_mctrl_key_stable_count = 0U;
        s_mctrl_key_grip_ms = now_ms;
        if (s_mctrl_key_info.run_state == MOTOR_CONTROL_RUN_KEY_MODE)
        {
          MotorControl_KeyDebugStart(now_ms);
        }
      }
    }
    else
    {
      s_mctrl_key_stable_count = 0U;
    }
  }
  else
  {
    matched = (bend <= MCTRL_KEY_RELEASE_TH) ? 1U : 0U;
    if (matched != 0U)
    {
      if (s_mctrl_key_stable_count < MCTRL_KEY_STABLE_FRAMES)
      {
        s_mctrl_key_stable_count++;
      }

      if (s_mctrl_key_stable_count >= MCTRL_KEY_STABLE_FRAMES)
      {
        MotorControl_ResetKeyPulse();
        MotorControl_UpdateKeyDebug(bend, now_ms);
        return 1U;
      }
    }
    else
    {
      s_mctrl_key_stable_count = 0U;
    }
  }

  MotorControl_UpdateKeyDebug(bend, now_ms);
  return 0U;
}

/**
  * @brief  根据KEY1触发时的x位置选择演示模式。
  * @param  x 手部X位置，范围-100~100。
  * @return 模式编号。
  */
static uint8_t MotorControl_KeyModeFromX(int16_t x)
{
  if (x < MCTRL_KEY_MODE_X_LEFT)
  {
    return MOTOR_CONTROL_MODE_COOP;
  }

  if (x > MCTRL_KEY_MODE_X_RIGHT)
  {
    return MOTOR_CONTROL_MODE_WRAP;
  }

  return MOTOR_CONTROL_MODE_SOLO;
}

/**
  * @brief  取消当前初始化预览，回到按键锁定状态。
  * @param  无。
  * @note   第一版只改状态；后续接真实动作时再回到pre-init目标。
  */
static void MotorControl_KeyCancelInit(void)
{
  s_mctrl_key_info.key_state = MOTOR_CONTROL_KEY_STATE_LOCKED;
  s_mctrl_key_info.selected_mode = MOTOR_CONTROL_MODE_NONE;
  s_mctrl_key_info.selected_tentacle = 0U;
  s_mctrl_key_info.init_done = 0U;
  s_mctrl_key_info.coop_init_angle_deg = 0;
}

/**
  * @brief  清除虚拟按键一次动作中的临时等待状态。
  * @param  无。
  * @note   只清按键识别缓存，不改变电机HOME和G4坐标恢复状态。
  */
static void MotorControl_ClearKeyRuntime(void)
{
  MotorControl_ResetKeyPulse();
  MotorControl_KeyDebugStop();
  s_mctrl_key_pending_zone = MCTRL_DISTANCE_ZONE_NONE;
  s_mctrl_key_pending_leave_ms = 0U;
  s_mctrl_key_entry_pending = 0U;
  s_mctrl_key_entry_ms = 0U;
}

/**
  * @brief  处理一个虚拟按键事件，只更新状态，不发送电机目标。
  * @param  key 按键编号1~4。
  * @param  x/y 按键触发时的手部位置。
  */
static void MotorControl_HandleKeyEvent(uint8_t key, int16_t x, int16_t y)
{
  s_mctrl_key_info.last_key = key;

  if (key == MOTOR_CONTROL_KEY_1)
  {
    s_mctrl_key_info.selected_mode = MotorControl_KeyModeFromX(x);
    s_mctrl_key_info.key_state = MOTOR_CONTROL_KEY_STATE_MODE;
    s_mctrl_key_info.init_done = 0U;
  }
  else if ((key == MOTOR_CONTROL_KEY_2) &&
           (s_mctrl_key_info.selected_mode != MOTOR_CONTROL_MODE_NONE))
  {
    s_mctrl_key_info.key_state = MOTOR_CONTROL_KEY_STATE_INIT;
    s_mctrl_key_info.init_done = 1U;
    if (s_mctrl_key_info.selected_mode == MOTOR_CONTROL_MODE_COOP)
    {
      s_mctrl_key_info.coop_init_angle_deg =
        (int16_t)(((int32_t)MotorControl_ClampInput(x) * MCTRL_KEY_COOP_ANGLE_MAX_DEG) / 100L);
    }
    else if (s_mctrl_key_info.selected_mode == MOTOR_CONTROL_MODE_SOLO)
    {
      s_mctrl_key_info.selected_tentacle = (x < 0) ? 0U : 1U;
      (void)y;
    }
  }
  else if ((key == MOTOR_CONTROL_KEY_3) && (s_mctrl_key_info.init_done != 0U))
  {
    s_mctrl_key_info.key_state = MOTOR_CONTROL_KEY_STATE_READY;
  }
  else if (key == MOTOR_CONTROL_KEY_4)
  {
    MotorControl_KeyCancelInit();
  }

  MotorControl_KeyNotice();
}

/**
  * @brief  单次按键等待双击超时后，确认KEY1或KEY2。
  * @param  now_ms 当前HAL tick。
  */
static void MotorControl_UpdateKeySingleTimeout(uint32_t now_ms)
{
  uint8_t key;

  if ((s_mctrl_key_pending_zone == MCTRL_DISTANCE_ZONE_NONE) ||
      ((now_ms - s_mctrl_key_pending_ms) <= MCTRL_KEY_DOUBLE_WINDOW_MS))
  {
    MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
    return;
  }

  key = (s_mctrl_key_pending_zone == MCTRL_DISTANCE_ZONE_NEAR) ?
        MOTOR_CONTROL_KEY_1 : MOTOR_CONTROL_KEY_2;
  s_mctrl_key_pending_zone = MCTRL_DISTANCE_ZONE_NONE;
  s_mctrl_key_pending_leave_ms = 0U;
  MotorControl_HandleKeyEvent(key, s_mctrl_key_pending_x, s_mctrl_key_pending_y);
  MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
  MotorControl_KeyDebugStop();
}

/**
  * @brief  单击pending后，检测手是否已经离开对应按键距离区。
  * @param  zone 当前距离区。
  * @param  now_ms 当前HAL tick。
  * @return 1表示已经提前确认单击。
  *
  * 说明：如果已经进入pulse=2等释放，表示第二次张握已经开始，
  * 此时不提前确认单击，避免误伤双击KEY3/KEY4。
  */
static uint8_t MotorControl_UpdateKeyPendingLeave(uint8_t zone, uint32_t now_ms)
{
  uint8_t key;

  if (s_mctrl_key_pending_zone == MCTRL_DISTANCE_ZONE_NONE)
  {
    s_mctrl_key_pending_leave_ms = 0U;
    return 0U;
  }

  if (s_mctrl_key_grip_state == MCTRL_KEY_GRIP_WAIT_RELEASE)
  {
    s_mctrl_key_pending_leave_ms = 0U;
    MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
    return 0U;
  }

  if (zone == s_mctrl_key_pending_zone)
  {
    s_mctrl_key_pending_leave_ms = 0U;
    MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
    return 0U;
  }

  if (s_mctrl_key_pending_leave_ms == 0U)
  {
    s_mctrl_key_pending_leave_ms = now_ms;
    MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
    return 0U;
  }

  if ((now_ms - s_mctrl_key_pending_leave_ms) < MCTRL_KEY_SINGLE_LEAVE_MS)
  {
    MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
    return 0U;
  }

  key = (s_mctrl_key_pending_zone == MCTRL_DISTANCE_ZONE_NEAR) ?
        MOTOR_CONTROL_KEY_1 : MOTOR_CONTROL_KEY_2;
  s_mctrl_key_pending_zone = MCTRL_DISTANCE_ZONE_NONE;
  s_mctrl_key_pending_leave_ms = 0U;
  MotorControl_HandleKeyEvent(key, s_mctrl_key_pending_x, s_mctrl_key_pending_y);
  MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
  MotorControl_KeyDebugStop();
  return 1U;
}

/**
  * @brief  高优先级虚拟按键捕获层。
  * @param  ai 当前AI估计结果。
  * @param  now_ms 当前HAL tick。
  * @return 1表示当前帧被按键层消费，映射控制应冻结。
  */
static uint8_t MotorControl_KeyProcess(const VL53_App_AiEstimate_t *ai, uint32_t now_ms)
{
  uint8_t zone;
  uint8_t key;

  if (s_mctrl_key_info.run_state == MOTOR_CONTROL_RUN_KEY_MODE)
  {
    zone = MotorControl_GetKeyDistanceZone(ai->center_z_mm);
    if (MotorControl_UpdateKeyPendingLeave(zone, now_ms) != 0U)
    {
      return 1U;
    }

    if (MotorControl_UpdateKeyPulse(ai, now_ms) != 0U)
    {
      if (zone != MCTRL_DISTANCE_ZONE_NONE)
      {
        if ((s_mctrl_key_pending_zone == zone) &&
            ((now_ms - s_mctrl_key_pending_ms) <= MCTRL_KEY_DOUBLE_WINDOW_MS))
        {
          key = (zone == MCTRL_DISTANCE_ZONE_NEAR) ? MOTOR_CONTROL_KEY_3 : MOTOR_CONTROL_KEY_4;
          s_mctrl_key_pending_zone = MCTRL_DISTANCE_ZONE_NONE;
          s_mctrl_key_pending_leave_ms = 0U;
          MotorControl_HandleKeyEvent(key, ai->x, ai->y);
          MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
          MotorControl_KeyDebugStop();
        }
        else
        {
          s_mctrl_key_pending_zone = zone;
          s_mctrl_key_pending_ms = now_ms;
          s_mctrl_key_pending_leave_ms = 0U;
          s_mctrl_key_pending_x = ai->x;
          s_mctrl_key_pending_y = ai->y;
          MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
        }
      }
      else if (s_mctrl_key_pending_zone == MCTRL_DISTANCE_ZONE_NONE)
      {
        MotorControl_KeyDebugStop();
      }
      else
      {
        MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
      }
    }
    return 1U;
  }

#if (MCTRL_VIRTUAL_KEY_TRIGGER_ENABLE == 0U)
  /* 暂停普通控制状态下的手势触发，并清除可能残留的半次按键动作。 */
  if ((s_mctrl_key_entry_pending != 0U) ||
      (s_mctrl_key_entry_ms != 0U) ||
      (s_mctrl_key_grip_state != MCTRL_KEY_GRIP_WAIT_OPEN))
  {
    MotorControl_ClearKeyRuntime();
  }
  return 0U;
#else
  if (MotorControl_UpdateKeyPulse(ai, now_ms) != 0U)
  {
    if ((s_mctrl_key_entry_pending != 0U) &&
        ((now_ms - s_mctrl_key_entry_ms) <= MCTRL_KEY_DOUBLE_WINDOW_MS))
    {
      s_mctrl_key_entry_pending = 0U;
      s_mctrl_key_entry_ms = 0U;

      s_mctrl_key_info.run_state = MOTOR_CONTROL_RUN_KEY_MODE;
      s_mctrl_key_info.key_state = MOTOR_CONTROL_KEY_STATE_LOCKED;
      s_mctrl_key_info.selected_mode = MOTOR_CONTROL_MODE_NONE;
      s_mctrl_key_info.init_done = 0U;
      s_mctrl_key_info.last_key = MOTOR_CONTROL_KEY_NONE;
      s_mctrl_key_pending_zone = MCTRL_DISTANCE_ZONE_NONE;
      s_mctrl_key_pending_leave_ms = 0U;
      MotorControl_KeyDebugStop();
      VL53_App_SetAiLogEnable(0U);
      MotorControl_KeyNotice();
      return 1U;
    }

    s_mctrl_key_entry_pending = 1U;
    s_mctrl_key_entry_ms = now_ms;
    MotorControl_KeyDebugStop();
    return 1U;
  }

  if ((s_mctrl_key_entry_pending != 0U) &&
      ((now_ms - s_mctrl_key_entry_ms) > MCTRL_KEY_DOUBLE_WINDOW_MS))
  {
    s_mctrl_key_entry_pending = 0U;
    s_mctrl_key_entry_ms = 0U;
  }

  return (s_mctrl_key_grip_state == MCTRL_KEY_GRIP_WAIT_RELEASE) ? 1U : 0U;
#endif
}

/**
  * @brief  清空中心选择区的临时状态，不清除已经等待确认的单击。
  * @param  无。
  */
static void MotorControl_ResetCenterSelectState(void)
{
  s_mctrl_center_enter_valid = 0U;
  s_mctrl_grip_state = MCTRL_GRIP_STATE_IDLE;
  s_mctrl_grip_seen_low = 0U;
  g_mctrl.select_armed = 0U;
}

/**
  * @brief  记录一次虚拟选择事件。
  * @param  zone 距离区，1近距离，2远距离。
  * @param  click_count 1单击，2双击。
  */
static void MotorControl_RecordSelectEvent(uint8_t zone, uint8_t click_count)
{
  if (zone == MCTRL_DISTANCE_ZONE_NEAR)
  {
    g_mctrl.select_event = (click_count >= 2U) ? 2U : 1U;
  }
  else if (zone == MCTRL_DISTANCE_ZONE_FAR)
  {
    g_mctrl.select_event = (click_count >= 2U) ? 4U : 3U;
  }
  else
  {
    return;
  }

  g_mctrl.select_count++;
}

/**
  * @brief  双击等待超时后，把等待中的第一次抓握确认为单击。
  * @param  now_ms 当前HAL tick。
  */
static void MotorControl_UpdateSelectTimeout(uint32_t now_ms)
{
  if ((s_mctrl_select_pending != 0U) &&
      ((now_ms - s_mctrl_select_pending_ms) > MCTRL_GRIP_DOUBLE_WINDOW_MS))
  {
    MotorControl_RecordSelectEvent(s_mctrl_select_pending_zone, 1U);
    s_mctrl_select_pending = 0U;
  }
}

/**
  * @brief  处理一次完整的抓握脉冲。
  * @param  zone 当前距离区。
  * @param  now_ms 当前HAL tick。
  */
static void MotorControl_OnGripPulse(uint8_t zone, uint32_t now_ms)
{
  if (zone == MCTRL_DISTANCE_ZONE_NONE)
  {
    return;
  }

  if ((s_mctrl_select_pending != 0U) &&
      (s_mctrl_select_pending_zone == zone) &&
      ((now_ms - s_mctrl_select_pending_ms) <= MCTRL_GRIP_DOUBLE_WINDOW_MS))
  {
    MotorControl_RecordSelectEvent(zone, 2U);
    s_mctrl_select_pending = 0U;
    return;
  }

  if (s_mctrl_select_pending != 0U)
  {
    MotorControl_RecordSelectEvent(s_mctrl_select_pending_zone, 1U);
  }

  s_mctrl_select_pending = 1U;
  s_mctrl_select_pending_zone = zone;
  s_mctrl_select_pending_ms = now_ms;
}

/**
  * @brief  在中心死区内识别“张开-握紧-张开”的虚拟按键。
  * @param  ai 当前AI估计结果。
  * @param  now_ms 当前HAL tick。
  * @note   这里只产生事件，不直接改变电机目标。
  */
static void MotorControl_UpdateCenterSelect(const VL53_App_AiEstimate_t *ai, uint32_t now_ms)
{
  uint8_t zone;

  if (ai == NULL)
  {
    return;
  }

  zone = MotorControl_GetDistanceZone(ai->center_z_mm);
  g_mctrl.hand_present = 1U;
  g_mctrl.distance_zone = zone;

  if (s_mctrl_center_enter_valid == 0U)
  {
    s_mctrl_center_enter_ms = now_ms;
    s_mctrl_center_enter_valid = 1U;
    g_mctrl.select_armed = 0U;
    return;
  }

  if ((now_ms - s_mctrl_center_enter_ms) < MCTRL_CENTER_SELECT_HOLD_MS)
  {
    return;
  }

  g_mctrl.select_armed = 1U;
  if (s_mctrl_grip_state == MCTRL_GRIP_STATE_IDLE)
  {
    if (ai->bend_0_100 <= MCTRL_GRIP_LOW_TH)
    {
      s_mctrl_grip_seen_low = 1U;
    }
    else if ((s_mctrl_grip_seen_low != 0U) && (ai->bend_0_100 >= MCTRL_GRIP_HIGH_TH))
    {
      s_mctrl_grip_state = MCTRL_GRIP_STATE_WAIT_RELEASE;
      s_mctrl_grip_high_ms = now_ms;
    }
  }
  else if ((now_ms - s_mctrl_grip_high_ms) > MCTRL_GRIP_PULSE_TIMEOUT_MS)
  {
    s_mctrl_grip_state = MCTRL_GRIP_STATE_IDLE;
    s_mctrl_grip_seen_low = 0U;
  }
  else if (ai->bend_0_100 <= MCTRL_GRIP_RELEASE_TH)
  {
    MotorControl_OnGripPulse(zone, now_ms);
    s_mctrl_grip_state = MCTRL_GRIP_STATE_IDLE;
    s_mctrl_grip_seen_low = 1U;
  }
}

/**
  * @brief  根据虚拟触手命令重算三电机绝对目标。
  * @param  无。
  * @note   三根拉索相隔约120度。这里用方向向量点积代替 atan2/cos，
  *         减少实时计算量。
  *
  *         当前实测方向：
  *         电机1收缩 -> 105deg，电机2收缩 -> -135deg，电机3收缩 -> -15deg。
  *
  *         工作区约定：
  *         前方工作区为 -45deg ~ 135deg，后方工作区为 135deg ~ -45deg。
  *         如果后续需要判断前后区，可直接使用 input_x + input_y：
  *         >=0 表示前方工作区，<0 表示后方工作区。
  */
static void MotorControl_RebuildTarget(void)
{
  static const int16_t cable_x[3] = {-26, -71, 97};
  static const int16_t cable_y[3] = {97, -71, -26};
  int32_t bend_pull_q[3];
  int32_t common_pull_q;
  int32_t bend_scale_q = MCTRL_PULL_SCALE_Q;
  uint8_t primary_index = MotorControl_GetFirstActiveIndex();
  uint8_t i;

  common_pull_q = (g_mctrl_config.stiff_gain_q * (int32_t)g_mctrl.stiffness_0_100) / 100L;

  /* 先计算三路完整弯曲分量，暂不单独截断任何一路。 */
  for (i = 0U; i < 3U; i++)
  {
    int32_t projection;

    if (((g_mctrl.input_x > -g_mctrl_config.input_deadband) && (g_mctrl.input_x < g_mctrl_config.input_deadband)) &&
        ((g_mctrl.input_y > -g_mctrl_config.input_deadband) && (g_mctrl.input_y < g_mctrl_config.input_deadband)))
    {
      projection = 0L;
    }
    else
    {
      projection = (((int32_t)g_mctrl.input_x * cable_x[i]) +
                    ((int32_t)g_mctrl.input_y * cable_y[i])) / 100L;
      projection = MotorControl_ClampI32(projection, -100L, 100L);
    }

    bend_pull_q[i] = (g_mctrl_config.bend_gain_q *
                      (int32_t)g_mctrl.bend_0_100 *
                      projection) / 10000L;

    /*
     * 某一路超过最大放线量时，只计算缩放比例，不在这里单独截断。
     * common_pull_q是共同预紧，可为负向弯曲分量提供额外放线余量。
     */
    if ((bend_pull_q[i] < 0L) &&
        ((common_pull_q + bend_pull_q[i]) < -MCTRL_RELEASE_LIMIT_Q))
    {
      int32_t scale_q = ((MCTRL_RELEASE_LIMIT_Q + common_pull_q) *
                         MCTRL_PULL_SCALE_Q) / (-bend_pull_q[i]);

      if (scale_q < bend_scale_q)
      {
        bend_scale_q = scale_q;
      }
    }
  }

  /* 三路弯曲分量统一缩放，保持原有方向和相对比例。 */
  for (i = 0U; i < 3U; i++)
  {
    g_mctrl.pull_q[i] = common_pull_q +
                        ((bend_pull_q[i] * bend_scale_q) / MCTRL_PULL_SCALE_Q);

    g_mctrl.target_q[i] = s_mctrl_home_q[primary_index][i] + g_mctrl.pull_q[i];
  }
}

/**
  * @brief  发送一次当前绝对目标。
  * @note   不等待逐板ACK；CAN暂忙时丢弃本帧，下一次输入继续发送最新目标。
  */
static void MotorControl_SendTargetOnce(void)
{
  int32_t mapped_pull[3];
  int32_t target_q[3];
  int32_t primary_target_q[3] = {0, 0, 0};
  uint8_t primary_index = MotorControl_GetFirstActiveIndex();
  uint8_t primary_target_ready = 0U;
  uint8_t active_count;
  uint8_t i;

  g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_NONE;
  if ((g_mctrl.enable == 0U) ||
      (s_mctrl_safety_stop_pending != 0U) ||
      (s_mctrl_stop_reason != MOTOR_CONTROL_STOP_NONE))
  {
    g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_NOT_READY;
    return;
  }

  /* 运行中不复用启动READY条件；1~3秒短时状态波动仍允许发送新目标。 */
  active_count = MotorControl_GetActiveCount();
  if (Can_App_HasTxFifoSpace(active_count) == 0U)
  {
    g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_TX_BUSY;
    g_mctrl.last_tx_status = 2U;
    return;
  }

  g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_NONE;
  g_mctrl.last_tx_status = 0U;
  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    uint8_t board_id;
    uint8_t tx_status;

    if (MotorControl_IsActive(i) == 0U)
    {
      continue;
    }

    board_id = s_mctrl_tentacle_route[i].board_id;
    MotorControl_MapActiveTarget(i, g_mctrl.pull_q, mapped_pull);
    MotorControl_BuildTargetForTentacle(i, mapped_pull, target_q);
    tx_status = Can_App_SendMotorTargetsAbsSpeedQToBoard(board_id,
                                                         target_q[0],
                                                         target_q[1],
                                                         target_q[2],
                                                         g_mctrl.motor_speed_limit_q);
    if (tx_status == 0U)
    {
      g_mctrl.tx_count++;
    }
    else
    {
      /* FIFO短暂繁忙不触发整组停止，后续输入会发送新的目标。 */
      g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_TX_BUSY;
      g_mctrl.last_tx_status = 2U;
      return;
    }

    if (i == primary_index)
    {
      primary_target_q[0] = target_q[0];
      primary_target_q[1] = target_q[1];
      primary_target_q[2] = target_q[2];
      primary_target_ready = 1U;
    }
  }

  /* 整个工作组成功入队后，才把目标作为ACT训练标签。 */
  if (primary_target_ready != 0U)
  {
    g_mctrl.output_q[0] = primary_target_q[0];
    g_mctrl.output_q[1] = primary_target_q[1];
    g_mctrl.output_q[2] = primary_target_q[2];
    s_mctrl_output_valid = 1U;
  }
}

/**
  * @brief  写入虚拟控制输入并按当前使能状态发送目标。
  * @param  x/y 方向输入，范围-100~100。
  * @param  bend/stiffness 弯曲和预紧，范围0~100。
  * @param  speed_limit_q 位置运动速度上限，单位rad/s*100。
  * @note   x/y只参与方向归一化；速度上限由调用者按原始半径单独计算。
  */
static void MotorControl_ApplyVirtualInput(int16_t x,
                                           int16_t y,
                                           uint8_t bend,
                                           uint8_t stiffness,
                                           int32_t speed_limit_q)
{
  int16_t dir_x = 0;
  int16_t dir_y = 0;

  MotorControl_NormalizeInput(x, y, &dir_x, &dir_y);
  g_mctrl.input_x = dir_x;
  g_mctrl.input_y = dir_y;
  g_mctrl.bend_0_100 = (bend > 100U) ? 100U : bend;
  g_mctrl.stiffness_0_100 = (stiffness > 100U) ? 100U : stiffness;
  g_mctrl.motor_speed_limit_q = speed_limit_q;
  MotorControl_RebuildTarget();
  if (g_mctrl.enable != 0U)
  {
    MotorControl_SendTargetOnce();
  }
}

/**
  * @brief  使用VL53/AI估计结果更新自动映射输入。
  * @param  ai VL53模块输出的AI估计结果。
  * @note   中心死区用于虚拟按键，期间不刷新电机目标；离开中心后才更新运动映射。
  */
static void MotorControl_UpdateFromAiEstimate(const VL53_App_AiEstimate_t *ai)
{
  uint32_t now_ms = HAL_GetTick();
  uint32_t radius_sq;
  uint32_t radius;
  uint32_t dead_sq = (uint32_t)(g_mctrl_config.input_deadband * g_mctrl_config.input_deadband);
  uint32_t full_sq = (uint32_t)(g_mctrl_config.auto_dir_full * g_mctrl_config.auto_dir_full);

  if (ai == NULL)
  {
    return;
  }

  g_mctrl.hand_present = 1U;
  g_mctrl.distance_zone = MotorControl_GetDistanceZone(ai->center_z_mm);

  radius_sq = MotorControl_InputRadiusSq(ai->x, ai->y);
  radius = MotorControl_IsqrtU32(radius_sq);
  if (g_mctrl.work_mode == MOTOR_CONTROL_WORK_COOP)
  {
    uint8_t next_stage = MotorControl_CoopStageFromOpen(ai->openness_score);
    int16_t work_x = MotorControl_ClampInput(ai->x);
    uint8_t need_send = 0U;

    g_mctrl.coop_open_0_100 = ai->openness_score;

    if (next_stage != s_mctrl_coop_candidate_stage)
    {
      s_mctrl_coop_candidate_stage = next_stage;
      s_mctrl_coop_candidate_count = 1U;
      return;
    }

    if (s_mctrl_coop_candidate_count < MCTRL_COOP_STAGE_STABLE_FRAMES)
    {
      s_mctrl_coop_candidate_count++;
      return;
    }

    if (next_stage != s_mctrl_coop_active_stage)
    {
      need_send = 1U;
    }
    else if (MotorControl_CoopStageUsesWorkX(next_stage) != 0U)
    {
      int16_t dx = work_x - s_mctrl_coop_active_x;

      if (dx < 0)
      {
        dx = -dx;
      }

      if (dx >= MCTRL_COOP_X_UPDATE_STEP)
      {
        need_send = 1U;
      }
    }

    if (need_send != 0U)
    {
      int16_t stage_x;
      int16_t stage_y;
      uint8_t stage_bend;
      uint8_t stage_stiff;

      MotorControl_CoopStageToInput(next_stage, work_x, &stage_x, &stage_y, &stage_bend, &stage_stiff);
      s_mctrl_coop_active_stage = next_stage;
      s_mctrl_coop_active_x = work_x;
      g_mctrl.coop_stage = next_stage;
      MotorControl_ApplyVirtualInput(stage_x,
                                     stage_y,
                                     stage_bend,
                                     stage_stiff,
                                     MotorControl_CalcSpeedLimitQ(radius));
    }
    return;
  }

  if (radius_sq < dead_sq)
  {
    MotorControl_UpdateCenterSelect(ai, now_ms);
    return;
  }

  MotorControl_ResetCenterSelectState();

  if (s_mctrl_auto_filter_valid == 0U)
  {
    s_mctrl_auto_bend = ai->bend_0_100;
    s_mctrl_auto_stiff = ai->stiffness_0_100;
    s_mctrl_auto_filter_valid = 1U;
  }
  else
  {
    s_mctrl_auto_bend = (uint8_t)MotorControl_ClampI32(
      MotorControl_FilterI32((int32_t)s_mctrl_auto_bend,
                             (int32_t)ai->bend_0_100,
                             MCTRL_AUTO_BEND_ALPHA_NUM,
                             MCTRL_AUTO_BEND_ALPHA_DEN),
      0L,
      100L);

    s_mctrl_auto_stiff = (uint8_t)MotorControl_ClampI32(
      MotorControl_FilterI32((int32_t)s_mctrl_auto_stiff,
                             (int32_t)ai->stiffness_0_100,
                             MCTRL_AUTO_STIFF_ALPHA_NUM,
                             MCTRL_AUTO_STIFF_ALPHA_DEN),
      0L,
      100L);
  }

  if (radius_sq >= dead_sq)
  {
    int16_t dir_x = 0;
    int16_t dir_y = 0;
    int32_t alpha_num = (radius_sq >= full_sq) ? MCTRL_AUTO_DIR_ALPHA_FAST_NUM :
                                                MCTRL_AUTO_DIR_ALPHA_SLOW_NUM;

    MotorControl_NormalizeInput(ai->x, ai->y, &dir_x, &dir_y);

    if (s_mctrl_auto_dir_valid == 0U)
    {
      s_mctrl_auto_dir_x = dir_x;
      s_mctrl_auto_dir_y = dir_y;
      s_mctrl_auto_dir_valid = 1U;
    }
    else
    {
      s_mctrl_auto_dir_x = (int16_t)MotorControl_ClampI32(
        MotorControl_FilterI32((int32_t)s_mctrl_auto_dir_x, (int32_t)dir_x, alpha_num, MCTRL_AUTO_DIR_ALPHA_DEN),
        -MCTRL_INPUT_LIMIT, MCTRL_INPUT_LIMIT);
      s_mctrl_auto_dir_y = (int16_t)MotorControl_ClampI32(
        MotorControl_FilterI32((int32_t)s_mctrl_auto_dir_y, (int32_t)dir_y, alpha_num, MCTRL_AUTO_DIR_ALPHA_DEN),
        -MCTRL_INPUT_LIMIT, MCTRL_INPUT_LIMIT);
    }
  }

  MotorControl_ApplyVirtualInput(s_mctrl_auto_dir_x,
                                 s_mctrl_auto_dir_y,
                                 s_mctrl_auto_bend,
                                 s_mctrl_auto_stiff,
                                 MotorControl_CalcSpeedLimitQ(radius));
}

void MotorControl_Init(void)
{
  W25Q64_App_MotorConfig_t flash_cfg;
  W25Q64_App_PositionRecord_t home_record;
  W25Q64_App_PositionRecord_t actual_record;
  MotorControl_Config_t loaded_cfg;
  uint8_t home_status;
  uint8_t actual_status;
  uint8_t i;

  memset(&g_mctrl, 0, sizeof(g_mctrl));
  g_mctrl.control_source = MOTOR_CONTROL_SOURCE_AI;
  g_mctrl.work_mode = MOTOR_CONTROL_WORK_SOLO;
  g_mctrl.active_mask = MOTOR_CONTROL_T1_MASK;
  g_mctrl.mirror_enable = 0U;
  s_mctrl_selected_tentacle = 0U;
  MotorControl_UpdateRouteStatus();
  memset(&s_mctrl_key_info, 0, sizeof(s_mctrl_key_info));
  s_mctrl_key_info.run_state = MOTOR_CONTROL_RUN_CONTROL;
  MotorControl_LoadDefaultConfig();
  if (W25Q64_App_LoadMotorConfig(&flash_cfg) == W25Q64_APP_OK)
  {
    loaded_cfg.bend_gain_q = flash_cfg.bend_gain_q;
    loaded_cfg.stiff_gain_q = flash_cfg.stiff_gain_q;
    loaded_cfg.input_deadband = flash_cfg.input_deadband;
    loaded_cfg.auto_dir_full = flash_cfg.auto_dir_full;
    loaded_cfg.motor_speed_base_q = flash_cfg.motor_speed_base_q;
    loaded_cfg.motor_speed_max_q = flash_cfg.motor_speed_max_q;
    (void)MotorControl_SetConfig(&loaded_cfg, 0U);
  }
  MotorControl_ResetAutoState();

  s_mctrl_last_position_save_ms = 0U;
  s_mctrl_position_save_cursor = 0U;
  memset(s_mctrl_home_q, 0, sizeof(s_mctrl_home_q));
  memset(s_mctrl_restore_pending, 0, sizeof(s_mctrl_restore_pending));
  memset(s_mctrl_restore_done, 0, sizeof(s_mctrl_restore_done));
  memset(s_mctrl_actual_save_enabled, 0, sizeof(s_mctrl_actual_save_enabled));
  memset(s_mctrl_actual_save_pending, 0, sizeof(s_mctrl_actual_save_pending));
  memset(s_mctrl_pending_actual_q, 0, sizeof(s_mctrl_pending_actual_q));
  memset(s_mctrl_restore_last_try_ms, 0, sizeof(s_mctrl_restore_last_try_ms));
  memset(s_mctrl_restore_actual_q, 0, sizeof(s_mctrl_restore_actual_q));
  memset(s_mctrl_restore_invalid_count, 0, sizeof(s_mctrl_restore_invalid_count));
  memset(s_mctrl_restore_last_rx_frame_count, 0, sizeof(s_mctrl_restore_last_rx_frame_count));
  memset(s_mctrl_restore_invalid_since_ms, 0, sizeof(s_mctrl_restore_invalid_since_ms));
  memset(s_mctrl_position_last_rx_frame_count, 0, sizeof(s_mctrl_position_last_rx_frame_count));
  memset(s_mctrl_restore_is_startup, 0, sizeof(s_mctrl_restore_is_startup));
  memset(s_mctrl_ever_synced, 0, sizeof(s_mctrl_ever_synced));
  memset(s_mctrl_link_lost, 0, sizeof(s_mctrl_link_lost));
  memset(s_mctrl_position_unknown, 0, sizeof(s_mctrl_position_unknown));
  memset(s_mctrl_reconnect_stop_sent, 0, sizeof(s_mctrl_reconnect_stop_sent));
  s_mctrl_link_warning_mask = 0U;
  s_mctrl_safety_stop_pending = 0U;
  s_mctrl_safety_stop_retry_count = 0U;
  s_mctrl_safety_last_check_ms = 0U;
  s_mctrl_safety_last_stop_ms = 0U;
  s_mctrl_stop_reason = MOTOR_CONTROL_STOP_NONE;
  s_mctrl_stop_mask = 0U;
  s_mctrl_stop_notice_pending = 0U;
  s_mctrl_output_valid = 0U;

  for (i = 0U; (i < MCTRL_TENTACLE_ACTIVE_COUNT) && (i < MCTRL_TENTACLE_MAX_COUNT); i++)
  {
    uint8_t tentacle_id = (uint8_t)(i + 1U);

    home_status = W25Q64_App_LoadLatestPositionRecordForTentacle(tentacle_id,
                                                                 W25Q64_APP_RECORD_TYPE_HOME,
                                                                 &home_record);
    actual_status = W25Q64_App_LoadLatestPositionRecordForTentacle(tentacle_id,
                                                                   W25Q64_APP_RECORD_TYPE_ACTUAL,
                                                                   &actual_record);

    if (home_status == W25Q64_APP_OK)
    {
      s_mctrl_home_q[i][0] = home_record.position_q[0];
      s_mctrl_home_q[i][1] = home_record.position_q[1];
      s_mctrl_home_q[i][2] = home_record.position_q[2];
    }

    if ((home_status == W25Q64_APP_OK) && (actual_status == W25Q64_APP_OK))
    {
      s_mctrl_restore_actual_q[i][0] = actual_record.position_q[0];
      s_mctrl_restore_actual_q[i][1] = actual_record.position_q[1];
      s_mctrl_restore_actual_q[i][2] = actual_record.position_q[2];
      s_mctrl_restore_pending[i] = 1U;
      s_mctrl_restore_is_startup[i] = 1U;
    }
  }

  MotorControl_LoadSelectedHomeToStatus();
}

/**
  * @brief  获取当前选中的触手编号。
  * @return 触手编号，1表示触手1。
  */
uint8_t MotorControl_GetSelectedTentacle(void)
{
  return (uint8_t)(s_mctrl_selected_tentacle + 1U);
}

/**
  * @brief  获取当前选中触手对应的G4板号。
  * @return G4 board_id。
  */
uint8_t MotorControl_GetSelectedBoardId(void)
{
  return MotorControl_GetSelectedBoardIdInternal();
}

/**
  * @brief  选择当前参与运动的触手，不立即发送运动目标。
  * @param  active_mask 触手位掩码，支持单触手、12/34双触手和四触手。
  * @param  mirror_enable 双触手或四触手中心模式是否交换M1/M3。
  * @return 0成功，1参数错误，2有触手RESTORE未完成，3有触手离线，4旧工作组停止失败。
  */
uint8_t MotorControl_SelectWork(uint8_t active_mask, uint8_t mirror_enable)
{
  uint8_t i;
  uint8_t stop_status;

  if ((active_mask != MOTOR_CONTROL_T1_MASK) &&
      (active_mask != MOTOR_CONTROL_T2_MASK) &&
      (active_mask != MOTOR_CONTROL_T3_MASK) &&
      (active_mask != MOTOR_CONTROL_T4_MASK) &&
      (active_mask != MOTOR_CONTROL_DUAL12_MASK) &&
      (active_mask != MOTOR_CONTROL_DUAL34_MASK) &&
      (active_mask != MOTOR_CONTROL_QUAD_MASK))
  {
    return 1U;
  }

  /* 先确认新工作组全部可用，避免切换后只能控制其中一部分。 */
  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    uint8_t board_id;
    const Can_App_G4Status_t *can_status;

    if ((active_mask & (uint8_t)(1U << i)) == 0U)
    {
      continue;
    }
    if (MotorControl_HasRestorePendingForIndex(i) != 0U)
    {
      return 2U;
    }

    board_id = s_mctrl_tentacle_route[i].board_id;
    if (Can_App_IsG4OnlineById(board_id, MCTRL_G4_STATUS_MAX_AGE_MS) == 0U)
    {
      return 3U;
    }
    can_status = Can_App_GetG4StatusById(board_id);
    if ((can_status == NULL) || (can_status->position_restore_valid == 0U))
    {
      return 2U;
    }
  }

  stop_status = MotorControl_StopActive();
  if (stop_status != 0U)
  {
    return 4U;
  }

  g_mctrl.active_mask = active_mask;
  g_mctrl.mirror_enable = (mirror_enable != 0U) ? 1U : 0U;
  g_mctrl.work_mode = ((active_mask & (uint8_t)(active_mask - 1U)) == 0U) ?
                      MOTOR_CONTROL_WORK_SOLO : MOTOR_CONTROL_WORK_COOP;

  /* 单触手工作组同时同步TSEL，保证CANSTAT、CANZERO和W25操作对象一致。 */
  if (g_mctrl.work_mode == MOTOR_CONTROL_WORK_SOLO)
  {
    for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
    {
      if ((active_mask & (uint8_t)(1U << i)) != 0U)
      {
        s_mctrl_selected_tentacle = i;
        break;
      }
    }
    MotorControl_UpdateRouteStatus();
    MotorControl_LoadSelectedHomeToStatus();
  }

  MotorControl_ResetAutoState();
  MotorControl_RebuildTarget();
  s_mctrl_output_valid = 0U;
  return 0U;
}

uint8_t MotorControl_SetWorkMode(uint8_t mode)
{
  if (mode == MOTOR_CONTROL_WORK_SOLO)
  {
    return MotorControl_SelectWork((uint8_t)(1U << s_mctrl_selected_tentacle), 0U);
  }

  if (mode == MOTOR_CONTROL_WORK_COOP)
  {
    /* 兼容旧TMODE COOP：默认选择触手1、2对称控制。 */
    return MotorControl_SelectWork(MOTOR_CONTROL_DUAL12_MASK, 1U);
  }

  return 1U;
}

uint8_t MotorControl_GetWorkMode(void)
{
  return g_mctrl.work_mode;
}

/**
  * @brief  切换当前H7输出触手。
  * @param  tentacle 触手编号，从1开始；当前启用数量由MCTRL_TENTACLE_ACTIVE_COUNT决定。
  * @return 0成功，非0表示编号非法。
  * @note   若映射控制正在运行，切换前会先停止旧触手，避免旧目标继续保持。
  */
uint8_t MotorControl_SetSelectedTentacle(uint8_t tentacle)
{
  if ((tentacle == 0U) || (tentacle > MCTRL_TENTACLE_ACTIVE_COUNT))
  {
    return 1U;
  }

  if (g_mctrl.enable != 0U)
  {
    (void)MotorControl_StopActive();
  }

  s_mctrl_selected_tentacle = (uint8_t)(tentacle - 1U);
  if (g_mctrl.work_mode == MOTOR_CONTROL_WORK_SOLO)
  {
    g_mctrl.active_mask = (uint8_t)(1U << s_mctrl_selected_tentacle);
    g_mctrl.mirror_enable = 0U;
  }
  MotorControl_UpdateRouteStatus();
  MotorControl_LoadSelectedHomeToStatus();
  MotorControl_ResetAutoState();
  s_mctrl_output_valid = 0U;
  return 0U;
}

uint8_t MotorControl_StopActive(void)
{
  uint8_t status = 0U;
  uint8_t i;

  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    uint8_t tx_status;

    if (MotorControl_IsActive(i) == 0U)
    {
      continue;
    }

    tx_status = Can_App_SendStopToBoard(s_mctrl_tentacle_route[i].board_id);
    if (tx_status != 0U)
    {
      status = tx_status;
    }
  }

  g_mctrl.enable = 0U;
  s_mctrl_output_valid = 0U;
  g_mctrl.last_tx_status = status;
  return status;
}

/**
  * @brief  执行全局急停并锁存ESTOP通知。
  * @return 首轮STOP发送状态。
  * @note   后续仍按100ms间隔补发，整轮最多5次。
  */
uint8_t MotorControl_EmergencyStop(void)
{
  uint32_t now_ms = HAL_GetTick();
  uint8_t status;

  s_mctrl_stop_reason = MOTOR_CONTROL_STOP_ESTOP;
  s_mctrl_stop_mask = 0U;
  s_mctrl_stop_notice_pending = 1U;
  g_mctrl.enable = 0U;
  g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_NOT_READY;
  MotorControl_ResetAutoState();

  status = MotorControl_StopActive();
  s_mctrl_safety_stop_pending = 1U;
  s_mctrl_safety_stop_retry_count = 1U;
  s_mctrl_safety_last_stop_ms = now_ms;
  return status;
}

/**
  * @brief  读取一次安全停止通知；停止原因本身仍锁存到下次成功启动。
  */
uint8_t MotorControl_TakeStopNotice(
  MotorControl_StopNotice_t *notice)
{
  if ((notice == NULL) ||
      (s_mctrl_stop_notice_pending == 0U))
  {
    return 0U;
  }

  notice->reason = s_mctrl_stop_reason;
  notice->mask = s_mctrl_stop_mask;
  s_mctrl_stop_notice_pending = 0U;
  return 1U;
}

uint8_t MotorControl_HomeActive(void)
{
  uint8_t status = 0U;
  uint8_t primary_index = MotorControl_GetFirstActiveIndex();
  uint8_t i;

  /* 先完整检查，避免已经让前几条触手运动后才发现后续触手不可用。 */
  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    uint8_t board_id;

    if (MotorControl_IsActive(i) == 0U)
    {
      continue;
    }
    if (MotorControl_HasRestorePendingForIndex(i) != 0U)
    {
      return 2U;
    }

    board_id = s_mctrl_tentacle_route[i].board_id;
    if (Can_App_IsG4OnlineById(board_id, MCTRL_G4_STATUS_MAX_AGE_MS) == 0U)
    {
      return 3U;
    }
  }

  /* 必须能一次容纳整个工作组，避免部分触手先执行HOME。 */
  if (Can_App_HasTxFifoSpace(MotorControl_GetActiveCount()) == 0U)
  {
    g_mctrl.last_tx_status = 2U;
    return 2U;
  }

  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    uint8_t tx_status;

    if (MotorControl_IsActive(i) == 0U)
    {
      continue;
    }

    tx_status = Can_App_SendMotorTargetsAbsResetQToBoard(s_mctrl_tentacle_route[i].board_id,
                                                         s_mctrl_home_q[i][0],
                                                         s_mctrl_home_q[i][1],
                                                         s_mctrl_home_q[i][2]);
    if (tx_status != 0U)
    {
      status = tx_status;
      MotorControl_LatchSafetyStop(MOTOR_CONTROL_STOP_GROUP_SYNC,
                                   g_mctrl.active_mask,
                                   HAL_GetTick());
      break;
    }
  }

  if (status == 0U)
  {
    /* HOME已成功入队：同步清除旧映射输入，避免SNAP继续上报旧Bend/刚度。 */
    g_mctrl.input_x = 0;
    g_mctrl.input_y = 0;
    g_mctrl.bend_0_100 = 0U;
    g_mctrl.stiffness_0_100 = 0U;
    MotorControl_RebuildTarget();
    g_mctrl.output_q[0] = s_mctrl_home_q[primary_index][0];
    g_mctrl.output_q[1] = s_mctrl_home_q[primary_index][1];
    g_mctrl.output_q[2] = s_mctrl_home_q[primary_index][2];
    s_mctrl_output_valid = 1U;
  }

  g_mctrl.last_tx_status = status;
  return status;
}

/**
  * @brief  向当前工作组发送三路相对位置命令。
  * @param  q0/q1/q2 线坐标增量，单位0.01rad，正数收线。
  * @return 0成功，2表示RESTORE未完成，3表示CAN状态过期，其他值为发送失败。
  */
uint8_t MotorControl_SendRelativeActive(int32_t q0, int32_t q1, int32_t q2)
{
  const int32_t input_q[3] = {q0, q1, q2};
  int32_t mapped_q[3];
  uint8_t status = 0U;
  uint8_t i;

  /* 发送前统一检查，避免只更新工作组中的一部分触手。 */
  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    uint8_t board_id;

    if (MotorControl_IsActive(i) == 0U)
    {
      continue;
    }
    if (MotorControl_HasRestorePendingForIndex(i) != 0U)
    {
      return 2U;
    }

    board_id = s_mctrl_tentacle_route[i].board_id;
    if (Can_App_IsG4OnlineById(board_id, MCTRL_G4_STATUS_MAX_AGE_MS) == 0U)
    {
      return 3U;
    }
  }

  /* 队列不足时整组不发送，避免多触手目标不同步。 */
  if (Can_App_HasTxFifoSpace(MotorControl_GetActiveCount()) == 0U)
  {
    g_mctrl.last_tx_status = 2U;
    return 2U;
  }

  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    uint8_t tx_status;

    if (MotorControl_IsActive(i) == 0U)
    {
      continue;
    }

    MotorControl_MapActiveTarget(i, input_q, mapped_q);
    tx_status = Can_App_SendMotorTargetsResetQToBoard(s_mctrl_tentacle_route[i].board_id,
                                                       mapped_q[0],
                                                       mapped_q[1],
                                                       mapped_q[2]);
    if (tx_status != 0U)
    {
      status = tx_status;
      MotorControl_LatchSafetyStop(MOTOR_CONTROL_STOP_GROUP_SYNC,
                                   g_mctrl.active_mask,
                                   HAL_GetTick());
      break;
    }
  }

  g_mctrl.last_tx_status = status;
  if (status == 0U)
  {
    /* 相对命令没有返回新的绝对目标，后续帧不能沿用旧action标签。 */
    s_mctrl_output_valid = 0U;
  }
  return status;
}

/**
  * @brief 发送ACT单触手绝对目标。
  * @param tentacle 触手编号。
  * @param q0/q1/q2 相对该触手HOME的统一线坐标目标，单位0.01rad。
  * @return 0成功，1状态或参数错误，2发送FIFO暂忙，3发送失败。
  * @note 目标直接换算为HOME加相对位置，不使用相对命令，避免累计漂移。
  */
uint8_t MotorControl_SendActTarget(uint8_t tentacle,
                                   int32_t q0,
                                   int32_t q1,
                                   int32_t q2)
{
  const int32_t action_q[3] = {q0, q1, q2};
  int32_t target_q[3];
  uint8_t index;
  uint8_t status;
  uint8_t i;

  if ((tentacle == 0U) ||
      (tentacle > MCTRL_TENTACLE_ACTIVE_COUNT))
  {
    return 1U;
  }

  index = (uint8_t)(tentacle - 1U);
  if ((g_mctrl.control_source != MOTOR_CONTROL_SOURCE_ACT) ||
      (g_mctrl.enable == 0U) ||
      (g_mctrl.work_mode != MOTOR_CONTROL_WORK_SOLO) ||
      (g_mctrl.active_mask != (uint8_t)(1U << index)))
  {
    return 1U;
  }

  if (Can_App_HasTxFifoSpace(1U) == 0U)
  {
    g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_TX_BUSY;
    g_mctrl.last_tx_status = 2U;
    return 2U;
  }

  for (i = 0U; i < 3U; i++)
  {
    target_q[i] = MotorControl_SaturateI64ToI32(
      (int64_t)s_mctrl_home_q[index][i] +
      (int64_t)action_q[i]);
  }

  status = Can_App_SendMotorTargetsAbsSpeedQToBoard(
    s_mctrl_tentacle_route[index].board_id,
    target_q[0],
    target_q[1],
    target_q[2],
    g_mctrl_config.motor_speed_base_q);
  if (status != 0U)
  {
    g_mctrl.last_tx_status = status;
    return 3U;
  }

  g_mctrl.motor_speed_limit_q = g_mctrl_config.motor_speed_base_q;
  g_mctrl.output_q[0] = target_q[0];
  g_mctrl.output_q[1] = target_q[1];
  g_mctrl.output_q[2] = target_q[2];
  g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_NONE;
  g_mctrl.last_tx_status = 0U;
  g_mctrl.tx_count++;
  s_mctrl_output_valid = 1U;
  return 0U;
}

/**
  * @brief  检查双触手协同命令是否可以发送。
  * @return 0可发送，2表示RESTORE未完成，3表示当前启用触手不足两个。
  */
static uint8_t MotorControl_IsCoopReady(void)
{
  uint8_t i;

  if ((g_mctrl.work_mode != MOTOR_CONTROL_WORK_COOP) ||
      ((g_mctrl.active_mask != MOTOR_CONTROL_DUAL12_MASK) &&
       (g_mctrl.active_mask != MOTOR_CONTROL_DUAL34_MASK)))
  {
    return 4U;
  }

  for (i = 0U; i < MCTRL_TENTACLE_ACTIVE_COUNT; i++)
  {
    if ((MotorControl_IsActive(i) != 0U) &&
        (MotorControl_HasRestorePendingForIndex(i) != 0U))
    {
      return 2U;
    }
  }

  return 0U;
}

void MotorControl_SetEnable(uint8_t enable)
{
  uint8_t next_enable = (enable != 0U) ? 1U : 0U;

  if (next_enable != 0U)
  {
    g_mctrl.block_reason = MotorControl_GetActiveBlockReason();
    if (g_mctrl.block_reason != MOTOR_CONTROL_BLOCK_NONE)
    {
      g_mctrl.enable = 0U;
      return;
    }
  }

  /* 重复设置保持幂等，不重复发送STOP，也不重置当前输入状态。 */
  if (next_enable == g_mctrl.enable)
  {
    return;
  }

  if (next_enable == 0U)
  {
    (void)MotorControl_StopActive();
    return;
  }

  g_mctrl.enable = 1U;
  s_mctrl_stop_reason = MOTOR_CONTROL_STOP_NONE;
  s_mctrl_stop_mask = 0U;
  s_mctrl_stop_notice_pending = 0U;
  s_mctrl_link_warning_mask = 0U;
  g_mctrl.block_reason = MOTOR_CONTROL_BLOCK_NONE;
  MotorControl_ResetAutoState();
  /* 等待下一帧AI数据或下一条MVIRT，禁止重新发送内存中的旧目标。 */
}

/**
  * @brief  切换AI或PC控制来源。
  * @param  source MOTOR_CONTROL_SOURCE_AI/PC。
  * @return 0成功，1来源参数错误，2当前工作组STOP发送失败。
  * @note   切换前必须可靠停止当前工作组，防止新旧来源连续下发目标。
  */
uint8_t MotorControl_SetControlSource(uint8_t source)
{
  uint8_t stop_status;

  if ((source != MOTOR_CONTROL_SOURCE_AI) &&
      (source != MOTOR_CONTROL_SOURCE_PC) &&
      (source != MOTOR_CONTROL_SOURCE_ACT))
  {
    return 1U;
  }

  if (source == g_mctrl.control_source)
  {
    return 0U;
  }

  stop_status = MotorControl_StopActive();
  if (stop_status != 0U)
  {
    return 2U;
  }

  MotorControl_ResetAutoState();
  g_mctrl.control_source = source;
  return 0U;
}

/**
  * @brief  获取当前控制来源。
  * @return MOTOR_CONTROL_SOURCE_AI、MOTOR_CONTROL_SOURCE_PC或
  *         MOTOR_CONTROL_SOURCE_ACT。
  */
uint8_t MotorControl_GetControlSource(void)
{
  return g_mctrl.control_source;
}

/**
  * @brief  获取上位机状态同步所需的触手摘要。
  * @param  summary 输出当前工作关系、在线、可控制和故障位图。
  * @note   只读取RAM和最新CAN状态，不读取W25，也不发送CAN命令。
  */
void MotorControl_GetSummary(MotorControl_Summary_t *summary)
{
  uint32_t now_ms;
  uint8_t i;

  if (summary == NULL)
  {
    return;
  }

  memset(summary, 0, sizeof(*summary));

  if ((g_mctrl.active_mask != 0U) &&
      ((g_mctrl.active_mask & (uint8_t)(g_mctrl.active_mask - 1U)) != 0U))
  {
    if (g_mctrl.mirror_enable == 0U)
    {
      summary->relation = MOTOR_CONTROL_RELATION_SAME;
    }
    else if (g_mctrl.active_mask == MOTOR_CONTROL_QUAD_MASK)
    {
      summary->relation = MOTOR_CONTROL_RELATION_CENTER;
    }
    else
    {
      summary->relation = MOTOR_CONTROL_RELATION_MIRROR;
    }
  }

  now_ms = HAL_GetTick();
  for (i = 0U;
       (i < MCTRL_TENTACLE_ACTIVE_COUNT) && (i < MCTRL_TENTACLE_MAX_COUNT);
       i++)
  {
    Can_App_G4Status_t status;
    uint8_t bit = (uint8_t)(1U << i);
    uint8_t board_id = s_mctrl_tentacle_route[i].board_id;

    MotorControl_CopyG4Status(board_id, &status);
    if (MotorControl_IsG4StatusFresh(&status, now_ms) == 0U)
    {
      continue;
    }

    summary->online_mask |= bit;

    if (status.stall_fault_mask != 0U)
    {
      summary->fault_mask |= bit;
    }

    if (MotorControl_IsTentacleReady(i, &status, now_ms) != 0U)
    {
      summary->ready_mask |= bit;
    }
  }
}

/**
  * @brief  获取四个触手相对各自HOME的实际位置。
  * @param  poses 固定四元素输出数组，依次对应触手1~4。
  * @note   HOME已经是H7统一线坐标；只转换G4回传的原始actual_q。
  */
void MotorControl_GetPoses(
  MotorControl_Pose_t poses[MOTOR_CONTROL_POSE_COUNT])
{
  uint32_t now_ms;
  uint8_t i;

  if (poses == NULL)
  {
    return;
  }

  memset(poses, 0,
         sizeof(MotorControl_Pose_t) * MOTOR_CONTROL_POSE_COUNT);
  now_ms = HAL_GetTick();

  for (i = 0U;
       (i < MCTRL_TENTACLE_ACTIVE_COUNT) &&
       (i < MCTRL_TENTACLE_MAX_COUNT) &&
       (i < MOTOR_CONTROL_POSE_COUNT);
       i++)
  {
    Can_App_G4Status_t status;
    int32_t actual_line_q[3] = {0, 0, 0};
    uint8_t board_id = s_mctrl_tentacle_route[i].board_id;
    uint8_t online;
    uint8_t j;

    MotorControl_CopyG4Status(board_id, &status);
    online = MotorControl_IsG4StatusFresh(&status, now_ms);

    if ((g_mctrl.active_mask & (uint8_t)(1U << i)) != 0U)
    {
      poses[i].status |= MOTOR_CONTROL_POSE_STATUS_ACTIVE;
    }

    if (online != 0U)
    {
      poses[i].status |= MOTOR_CONTROL_POSE_STATUS_ONLINE;

      if (status.stall_fault_mask != 0U)
      {
        poses[i].status |= MOTOR_CONTROL_POSE_STATUS_FAULT;
      }

      if (MotorControl_IsTentacleReady(i, &status, now_ms) != 0U)
      {
        poses[i].status |= MOTOR_CONTROL_POSE_STATUS_READY;
      }
    }

    /* 离线时保留最后位置；上位机必须通过ONLINE位判断是否仍为实时数据。 */
    if (status.valid != 0U)
    {
      Can_App_PositionToLineQ(board_id, status.actual_q, actual_line_q);
      for (j = 0U; j < 3U; j++)
      {
        int64_t delta = (int64_t)actual_line_q[j] -
                        (int64_t)s_mctrl_home_q[i][j];
        poses[i].dq[j] = MotorControl_SaturateI64ToI32(delta);
      }
    }
  }
}

/**
  * @brief 复制指定触手的ACT电机状态和最近有效动作目标。
  * @param tentacle 触手编号，范围1~4。
  * @param data 输出结构体；传入NULL时直接返回。
  * @note CAN状态在短临界区内复制，之后只做坐标换算，不访问Flash。
  */
void MotorControl_CopyActData(uint8_t tentacle, MotorControl_ActData_t *data)
{
  Can_App_G4Status_t status;
  int32_t actual_line_q[3] = {0, 0, 0};
  uint32_t now_ms;
  uint32_t age_ms;
  uint8_t index;
  uint8_t board_id;
  uint8_t i;

  if (data == NULL)
  {
    return;
  }

  memset(data, 0, sizeof(*data));
  data->status_age_ms = 0xFFFFU;
  if ((tentacle == 0U) || (tentacle > MCTRL_TENTACLE_MAX_COUNT))
  {
    return;
  }

  index = (uint8_t)(tentacle - 1U);
  data->tentacle = tentacle;
  if (MotorControl_IsValidTentacleIndex(index) == 0U)
  {
    return;
  }

  board_id = s_mctrl_tentacle_route[index].board_id;
  MotorControl_CopyG4Status(board_id, &status);
  if (status.valid == 0U)
  {
    return;
  }

  now_ms = HAL_GetTick();
  age_ms = now_ms - status.last_rx_tick_ms;
  data->status_age_ms = (age_ms > 0xFFFFUL) ? 0xFFFFU : (uint16_t)age_ms;

  Can_App_PositionToLineQ(board_id, status.actual_q, actual_line_q);
  for (i = 0U; i < 3U; i++)
  {
    data->actual_q[i] = MotorControl_SaturateI64ToI32(
      (int64_t)actual_line_q[i] - (int64_t)s_mctrl_home_q[index][i]);
  }

  if ((status.position_restore_valid != 0U) &&
      (s_mctrl_restore_done[index] != 0U) &&
      (s_mctrl_restore_pending[index] == 0U))
  {
    data->valid_flags |= MOTOR_CONTROL_ACT_POSITION_VALID;
  }
  if (status.stall_fault_mask != 0U)
  {
    data->valid_flags |= MOTOR_CONTROL_ACT_FAULT;
  }

  if ((s_mctrl_output_valid != 0U) &&
      (g_mctrl.enable != 0U) &&
      (g_mctrl.work_mode == MOTOR_CONTROL_WORK_SOLO) &&
      (g_mctrl.active_mask == (uint8_t)(1U << index)))
  {
    for (i = 0U; i < 3U; i++)
    {
      data->action_q[i] = MotorControl_SaturateI64ToI32(
        (int64_t)g_mctrl.output_q[i] - (int64_t)s_mctrl_home_q[index][i]);
    }
    data->valid_flags |= MOTOR_CONTROL_ACT_ACTION_VALID;
  }
}

/**
  * @brief  串口命令切换普通控制模式和虚拟按键模式。
  * @return 切换后的运行状态。
  * @note   进入按键模式时停止当前映射输出，避免按键选择和自动映射互相抢目标。
  */
uint8_t MotorControl_ToggleKeyMode(void)
{
  if (s_mctrl_key_info.run_state == MOTOR_CONTROL_RUN_KEY_MODE)
  {
    MotorControl_ClearKeyRuntime();
    s_mctrl_key_info.run_state = MOTOR_CONTROL_RUN_CONTROL;
    s_mctrl_key_info.key_state = MOTOR_CONTROL_KEY_STATE_IDLE;
    s_mctrl_key_info.selected_mode = MOTOR_CONTROL_MODE_NONE;
    s_mctrl_key_info.init_done = 0U;
    VL53_App_SetAiLogEnable(1U);
  }
  else
  {
    (void)MotorControl_StopActive();
    MotorControl_ClearKeyRuntime();
    s_mctrl_key_info.run_state = MOTOR_CONTROL_RUN_KEY_MODE;
    MotorControl_KeyCancelInit();
    VL53_App_SetAiLogEnable(0U);
  }

  MotorControl_KeyNotice();
  return s_mctrl_key_info.run_state;
}

/**
  * @brief  清除按键选择并保持在按键模式。
  * @param  无。
  */
void MotorControl_ClearKeySelection(void)
{
  MotorControl_ClearKeyRuntime();
  s_mctrl_key_info.run_state = MOTOR_CONTROL_RUN_KEY_MODE;
  MotorControl_KeyCancelInit();
  VL53_App_SetAiLogEnable(0U);
  MotorControl_KeyNotice();
}

/**
  * @brief  手动发送双触手初始化上翘目标。
  * @param  x 保留参数，当前INIT固定为电机2方向上翘；记录x便于后续观察流程。
  * @return 0成功，非0表示发送失败或RESTORE未完成。
  */
uint8_t MotorControl_RunCoopInit(int16_t x)
{
  int16_t stage_x;
  int16_t stage_y;
  uint8_t stage_bend;
  uint8_t stage_stiff;
  uint8_t status;

  status = MotorControl_IsCoopReady();
  if (status != 0U)
  {
    return status;
  }

  g_mctrl.enable = 0U; /* 手动COOP命令只发一次目标，不打开连续AI映射。 */

  MotorControl_CoopStageToInput(MCTRL_COOP_STAGE_INIT,
                                x,
                                &stage_x,
                                &stage_y,
                                &stage_bend,
                                &stage_stiff);
  s_mctrl_coop_active_stage = MCTRL_COOP_STAGE_INIT;
  s_mctrl_coop_active_x = MotorControl_ClampInput(x);
  g_mctrl.coop_stage = MCTRL_COOP_STAGE_INIT;
  g_mctrl.coop_open_0_100 = 100U;
  MotorControl_ApplyVirtualInput(stage_x,
                                 stage_y,
                                 stage_bend,
                                 stage_stiff,
                                 g_mctrl_config.motor_speed_base_q);
  MotorControl_SendTargetOnce();

  s_mctrl_key_info.selected_mode = MOTOR_CONTROL_MODE_COOP;
  s_mctrl_key_info.key_state = MOTOR_CONTROL_KEY_STATE_INIT;
  s_mctrl_key_info.init_done = 1U;
  MotorControl_KeyNotice();
  return g_mctrl.last_tx_status;
}

/**
  * @brief  手动发送双触手协同抓取目标。
  * @param  bend 弯曲程度，0~100。
  * @param  x 包裹方向，-100对应电机1方向，100对应电机3方向。
  * @return 0成功，非0表示参数错误、发送失败或RESTORE未完成。
  */
uint8_t MotorControl_RunCoopBend(uint8_t bend, int16_t x)
{
  int16_t stage_x;
  int16_t stage_y;
  uint8_t status;

  if (bend > 100U)
  {
    return 1U;
  }

  status = MotorControl_IsCoopReady();
  if (status != 0U)
  {
    return status;
  }

  g_mctrl.enable = 0U; /* 手动COOP命令只发一次目标，不打开连续AI映射。 */

  MotorControl_CoopWorkDirFromX(x, &stage_x, &stage_y);
  s_mctrl_coop_active_stage = MotorControl_CoopStageFromManualBend(bend);
  s_mctrl_coop_active_x = MotorControl_ClampInput(x);
  g_mctrl.coop_stage = s_mctrl_coop_active_stage;
  g_mctrl.coop_open_0_100 = (uint8_t)(100U - bend);
  MotorControl_ApplyVirtualInput(stage_x,
                                 stage_y,
                                 bend,
                                 MCTRL_COOP_GRIP_STIFF_DEFAULT,
                                 g_mctrl_config.motor_speed_base_q);
  MotorControl_SendTargetOnce();

  s_mctrl_key_info.selected_mode = MOTOR_CONTROL_MODE_COOP;
  s_mctrl_key_info.key_state = MOTOR_CONTROL_KEY_STATE_READY;
  s_mctrl_key_info.init_done = 1U;
  MotorControl_KeyNotice();
  return g_mctrl.last_tx_status;
}

void MotorControl_SetNeutralQ(int32_t q0, int32_t q1, int32_t q2)
{
  if (MotorControl_IsValidTentacleIndex(s_mctrl_selected_tentacle) != 0U)
  {
    s_mctrl_home_q[s_mctrl_selected_tentacle][0] = q0;
    s_mctrl_home_q[s_mctrl_selected_tentacle][1] = q1;
    s_mctrl_home_q[s_mctrl_selected_tentacle][2] = q2;
  }

  g_mctrl.neutral_q[0] = q0;
  g_mctrl.neutral_q[1] = q1;
  g_mctrl.neutral_q[2] = q2;
  MotorControl_RebuildTarget();
}

void MotorControl_SetVirtualInput(int16_t x, int16_t y, uint8_t bend, uint8_t stiffness)
{
  MotorControl_ApplyVirtualInput(x,
                                 y,
                                 bend,
                                 stiffness,
                                 MotorControl_CalcSpeedLimitQ(MotorControl_IsqrtU32(MotorControl_InputRadiusSq(x, y))));
}

/**
  * @brief  清除上电恢复等待状态，允许后续继续自动保存ACTUAL。
  * @param  无。
  * @note   CANZERO成功后，HOME和ACTUAL已经明确更新为当前G4位置，此时旧恢复流程必须结束。
  */
void MotorControl_ClearRestorePending(void)
{
  uint8_t i = s_mctrl_selected_tentacle;

  if (MotorControl_IsValidTentacleIndex(i) == 0U)
  {
    return;
  }

  s_mctrl_restore_pending[i] = 0U;
  s_mctrl_restore_done[i] = 1U;
  s_mctrl_actual_save_enabled[i] = 1U;
  s_mctrl_restore_last_try_ms[i] = 0U;
  s_mctrl_restore_invalid_count[i] = 0U;
  s_mctrl_restore_invalid_since_ms[i] = 0U;
  s_mctrl_restore_is_startup[i] = 0U;
  s_mctrl_ever_synced[i] = 1U;
  s_mctrl_link_lost[i] = 0U;
  s_mctrl_position_unknown[i] = 0U;
  s_mctrl_restore_actual_q[i][0] = g_mctrl.neutral_q[0];
  s_mctrl_restore_actual_q[i][1] = g_mctrl.neutral_q[1];
  s_mctrl_restore_actual_q[i][2] = g_mctrl.neutral_q[2];
}

/**
  * @brief  设置等待G4 RESTORE确认的位置。
  * @param  q0/q1/q2 希望G4回传接近的位置，单位0.01rad。
  * @note   只重建确认状态，不发送HOME；手动CANHOME时才会让电机运动。
  */
void MotorControl_StartRestoreConfirmQ(int32_t q0, int32_t q1, int32_t q2)
{
  uint8_t i = s_mctrl_selected_tentacle;

  if (MotorControl_IsValidTentacleIndex(i) == 0U)
  {
    return;
  }

  s_mctrl_restore_actual_q[i][0] = q0;
  s_mctrl_restore_actual_q[i][1] = q1;
  s_mctrl_restore_actual_q[i][2] = q2;
  s_mctrl_restore_pending[i] = 1U;
  s_mctrl_restore_done[i] = 0U;
  s_mctrl_actual_save_enabled[i] = 0U;
  s_mctrl_restore_last_try_ms[i] = 0U;
  s_mctrl_restore_invalid_count[i] = 0U;
  s_mctrl_restore_invalid_since_ms[i] = 0U;
  s_mctrl_restore_is_startup[i] = 0U; /* CANZERO等人工确认恢复。 */
}

/**
  * @brief  更新H7内存中下一次RESTORE使用的ACTUAL源数据。
  * @param  q0/q1/q2 最新可信G4实际位置，单位0.01rad。
  * @note   只更新保存值，不改变pending/done状态；用于W25SAVE或自动保存成功后同步RAM状态。
  */
void MotorControl_UpdateRestoreActualQ(int32_t q0, int32_t q1, int32_t q2)
{
  uint8_t i = s_mctrl_selected_tentacle;

  if (MotorControl_IsValidTentacleIndex(i) == 0U)
  {
    return;
  }

  s_mctrl_restore_actual_q[i][0] = q0;
  s_mctrl_restore_actual_q[i][1] = q1;
  s_mctrl_restore_actual_q[i][2] = q2;
}

/**
  * @brief  查询G4多圈坐标是否仍在等待恢复确认。
  * @param  无。
  * @return 1表示RESTORE尚未完成，0表示可以正常控制和保存ACTUAL。
  */
uint8_t MotorControl_IsRestorePending(void)
{
  if (MotorControl_IsValidTentacleIndex(s_mctrl_selected_tentacle) == 0U)
  {
    return 0U;
  }

  return s_mctrl_restore_pending[s_mctrl_selected_tentacle];
}

void MotorControl_Task(void)
{
  const VL53_App_AiEstimate_t *ai;
  uint32_t now_ms;

  now_ms = HAL_GetTick();
  MotorControl_UpdateSafetyStop(now_ms);
  MotorControl_RestoreG4Task(now_ms);
  MotorControl_PositionSaveTask(now_ms);

  /* 只有AI来源执行手势映射；PC和ACT均由各自模块发送目标。 */
  if (g_mctrl.control_source != MOTOR_CONTROL_SOURCE_AI)
  {
    return;
  }

  if (VL53_App_GetMode() != VL53_APP_MODE_INFER)
  {
    return;
  }

  /*
   * 单击确认依赖双击窗口超时，不能放在AI有效帧之后；
   * 否则手离开或AI短暂无效时，KEY1/KEY2会一直 pending。
   */
  MotorControl_UpdateKeySingleTimeout(now_ms);

  MotorControl_UpdateSelectTimeout(now_ms);

  ai = VL53_App_GetAiEstimate();
  if ((ai == NULL) || (ai->valid == 0U))
  {
    g_mctrl.hand_present = 0U;
    g_mctrl.distance_zone = MCTRL_DISTANCE_ZONE_NONE;
    MotorControl_ResetCenterSelectState();
    (void)MotorControl_UpdateKeyPendingLeave(MCTRL_DISTANCE_ZONE_NONE, now_ms);
    return;
  }

  if (ai->frame_count == s_mctrl_last_ai_frame)
  {
    return;
  }
  s_mctrl_last_ai_frame = ai->frame_count;

  if (MotorControl_KeyProcess(ai, now_ms) != 0U)
  {
    return;
  }

  /*
   * RESTORE未完成时仍允许虚拟按键识别，方便调试进入按键模式；
   * 但后续映射控制和CAN目标发送仍然禁止，避免G4坐标未恢复时误动作。
   */
  if (MotorControl_IsRestorePending() != 0U)
  {
    return;
  }

  if (g_mctrl.enable == 0U)
  {
    return;
  }

  MotorControl_UpdateFromAiEstimate(ai);
}

const MotorControl_Status_t *MotorControl_GetStatus(void)
{
  return &g_mctrl;
}

const MotorControl_Config_t *MotorControl_GetConfig(void)
{
  return &g_mctrl_config;
}

const MotorControl_KeyInfo_t *MotorControl_GetKeyInfo(void)
{
  return &s_mctrl_key_info;
}

uint8_t MotorControl_TakeKeyNotice(MotorControl_KeyInfo_t *info)
{
  if (s_mctrl_key_notice_pending == 0U)
  {
    return 0U;
  }

  if (info != NULL)
  {
    *info = s_mctrl_key_info;
  }

  s_mctrl_key_notice_pending = 0U;
  return 1U;
}

uint8_t MotorControl_TakeKeyDebug(MotorControl_KeyInfo_t *info)
{
  uint32_t now_ms;

  if (s_mctrl_key_dbg_active == 0U)
  {
    return 0U;
  }

  now_ms = HAL_GetTick();
  if ((s_mctrl_key_pending_zone == MCTRL_DISTANCE_ZONE_NONE) &&
      (s_mctrl_key_grip_state != MCTRL_KEY_GRIP_WAIT_OPEN) &&
      ((now_ms - s_mctrl_key_grip_ms) > MCTRL_KEY_PULSE_TIMEOUT_MS))
  {
    MotorControl_ResetKeyPulse();
    MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
    MotorControl_KeyDebugStop();
    return 0U;
  }

  MotorControl_UpdateKeyDebug(s_mctrl_key_info.raw_bend, now_ms);
  if ((now_ms - s_mctrl_key_dbg_last_ms) < MCTRL_KEY_DBG_PERIOD_MS)
  {
    return 0U;
  }

  s_mctrl_key_dbg_last_ms = now_ms;
  if (info != NULL)
  {
    *info = s_mctrl_key_info;
  }

  return 1U;
}

uint8_t MotorControl_SetConfig(const MotorControl_Config_t *config, uint8_t save_to_flash)
{
  W25Q64_App_MotorConfig_t flash_cfg;

  if ((config == 0) ||
      (config->bend_gain_q < 0) || (config->bend_gain_q > MCTRL_RELEASE_LIMIT_Q) ||
      (config->stiff_gain_q < 0) || (config->stiff_gain_q > 2000L) ||
      (config->input_deadband < 0) || (config->input_deadband > 60L) ||
      (config->auto_dir_full < 1L) || (config->auto_dir_full > 100L) ||
      (config->motor_speed_base_q < 100L) || (config->motor_speed_base_q > 5000L) ||
      (config->motor_speed_max_q < config->motor_speed_base_q) ||
      (config->motor_speed_max_q > 6000L))
  {
    return W25Q64_APP_ERR_PARAM;
  }

  g_mctrl_config = *config;
  MotorControl_RebuildTarget();

  if (save_to_flash == 0U)
  {
    return W25Q64_APP_OK;
  }

  flash_cfg.bend_gain_q = config->bend_gain_q;
  flash_cfg.stiff_gain_q = config->stiff_gain_q;
  flash_cfg.input_deadband = config->input_deadband;
  flash_cfg.auto_dir_full = config->auto_dir_full;
  flash_cfg.motor_speed_base_q = config->motor_speed_base_q;
  flash_cfg.motor_speed_max_q = config->motor_speed_max_q;

  return W25Q64_App_SaveMotorConfig(&flash_cfg);
}
