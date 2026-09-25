/**
 * @file act_policy_app.c
 * @brief ACT策略模型初始化、历史帧构建和触手3安全控制。
 */

#include "act_policy_app.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "act_policy.h"
#include "act_policy_data.h"
#include "motor_control_app.h"
#include "vl53_app.h"

/* 生成模型的两个输入和一个输出在缓冲数组中的位置。 */
#define ACT_POLICY_APP_TOF_INPUT_INDEX      0U
#define ACT_POLICY_APP_STATE_INPUT_INDEX    1U
#define ACT_POLICY_APP_OUTPUT_INDEX         0U

#define ACT_POLICY_APP_TOF_CHANNELS         18U
#define ACT_POLICY_APP_ZONE_COUNT           64U
#define ACT_POLICY_APP_FRAME_VALUES         \
  (ACT_POLICY_APP_TOF_CHANNELS * ACT_POLICY_APP_ZONE_COUNT)

/* 执行模型预测的下一帧目标，避免跳过抬升、偏移等中间路径。 */
#define ACT_POLICY_APP_ACTION_HORIZON_INDEX 0U
/* 一次锁定并顺序执行同一条预测的3步，防止相邻推理立即反向覆盖目标。 */
#define ACT_POLICY_APP_COMMIT_STEPS         3U
/* 最多保存5次推理结果；启动时只使用最新chunk。 */
#define ACT_POLICY_APP_MAX_ENSEMBLE_COUNT   5U

/* 模型目标是相对HOME位置，限制范围防止异常输出造成过度收放线。 */
#define ACT_POLICY_APP_TARGET_LIMIT_Q       30000L
/* 10Hz下约等于32rad/s，接近当前G4基础速度。 */
#define ACT_POLICY_APP_TARGET_STEP_Q        320L
/* 融合期望值最多领先实际位置两帧，防止负载卡住后目标持续跑远。 */
#define ACT_POLICY_APP_TARGET_LEAD_Q        640L
#define ACT_POLICY_APP_SENSOR_TIMEOUT_MS    500U
#define ACT_POLICY_APP_CAN_HOLD_MS          1000U
#define ACT_POLICY_APP_CAN_STOP_MS          3000U

/* 实际位置看最近8帧；目标多保留2帧，用于覆盖电机响应滞后。 */
#define ACT_POLICY_APP_OSC_ACTUAL_FRAMES    8U
#define ACT_POLICY_APP_OSC_HISTORY_FRAMES   10U
#define ACT_POLICY_APP_OSC_ACTUAL_DEAD_Q    25L
#define ACT_POLICY_APP_OSC_TARGET_DEAD_Q    35L
#define ACT_POLICY_APP_OSC_SPAN_Q           70L
#define ACT_POLICY_APP_OSC_BACKTRACK_Q      70L
#define ACT_POLICY_APP_OSC_NET_PERCENT      40U
#define ACT_POLICY_APP_OSC_LEVEL_WAIT_MS    400U
#define ACT_POLICY_APP_OSC_FRESH_FRAMES     4U
#define ACT_POLICY_APP_OSC_RECOVER_Q        240L
#define ACT_POLICY_APP_OSC_RECOVER_MS       500U
#define ACT_POLICY_APP_OSC_MAX_HOLD_MS      1000U
#define ACT_POLICY_APP_OSC_FRAME_GAP_MS     300U
#define ACT_POLICY_APP_FRAME_RESET_DELTA    10U

typedef char ActPolicyAppTofSizeCheck[
  (AI_ACT_POLICY_IN_1_SIZE ==
   (ACT_POLICY_APP_HISTORY_FRAMES * ACT_POLICY_APP_FRAME_VALUES)) ? 1 : -1];
typedef char ActPolicyAppStateSizeCheck[
  (AI_ACT_POLICY_IN_2_SIZE ==
   (ACT_POLICY_APP_HISTORY_FRAMES * ACT_POLICY_APP_MOTOR_COUNT)) ? 1 : -1];
typedef char ActPolicyAppEnsembleSizeCheck[
  ((ACT_POLICY_APP_ACTION_HORIZON_INDEX +
    ACT_POLICY_APP_MAX_ENSEMBLE_COUNT +
    ACT_POLICY_APP_COMMIT_STEPS - 1U) <=
   (AI_ACT_POLICY_OUT_1_SIZE / ACT_POLICY_APP_MOTOR_COUNT)) ? 1 : -1];

typedef enum
{
  ACT_POLICY_APP_OSC_MODEL = 0U,
  ACT_POLICY_APP_OSC_SENT,
  ACT_POLICY_APP_OSC_ACTUAL
} ActPolicyApp_OscSource_t;

/* 一条记录对应一次成功发送给G4的ACT控制目标。 */
typedef struct
{
  uint32_t timestamp_ms;
  int32_t model_target_q[ACT_POLICY_APP_MOTOR_COUNT];
  int32_t sent_target_q[ACT_POLICY_APP_MOTOR_COUNT];
  int32_t actual_q[ACT_POLICY_APP_MOTOR_COUNT];
} ActPolicyApp_OscSample_t;

typedef struct
{
  int64_t travel_q;
  int64_t net_q;
  int64_t span_q;
  int64_t backtrack_q;
  uint8_t reversal_count;
} ActPolicyApp_OscStats_t;

/* ACT模型使用独立激活区，不能与手势模型共用。 */
AI_ALIGNED(32)
static uint8_t s_act_policy_activations[
  AI_ACT_POLICY_DATA_ACTIVATION_1_SIZE];

static ai_handle s_activation_handles[
  AI_ACT_POLICY_DATA_ACTIVATIONS_COUNT] =
{
  s_act_policy_activations
};

static ai_handle s_network = AI_HANDLE_NULL;
static ai_buffer *s_inputs = NULL;
static ai_buffer *s_outputs = NULL;
static ActPolicyApp_Status_t s_status;

/*
 * CubeAI运行时可能复用输入所在的激活区，因此12帧历史必须独立保存。
 * 两块缓存合计约55.5KB，只由CPU访问，不参与DMA。
 */
AI_ALIGNED(32)
static float s_tof_history[AI_ACT_POLICY_IN_1_SIZE];
AI_ALIGNED(32)
static float s_state_history[AI_ACT_POLICY_IN_2_SIZE];

/* newest在下标0；每个chunk包含10组、每组3个电机目标。 */
static float s_action_chunks[ACT_POLICY_APP_MAX_ENSEMBLE_COUNT]
                            [AI_ACT_POLICY_OUT_1_SIZE];
static uint32_t
  s_action_chunk_frame_seq[ACT_POLICY_APP_MAX_ENSEMBLE_COUNT];
static uint8_t s_action_chunk_count;
static int32_t s_last_target_q[ACT_POLICY_APP_MOTOR_COUNT];
static uint8_t s_last_target_valid;
static float
  s_committed_action_q[ACT_POLICY_APP_COMMIT_STEPS]
                      [ACT_POLICY_APP_MOTOR_COUNT];
static uint8_t s_committed_action_step;
static uint8_t s_committed_action_valid;

static uint32_t s_last_frame_seq;
static uint32_t s_last_frame_ms;
static uint32_t s_last_frame_delta;

static ActPolicyApp_OscSample_t
  s_osc_samples[ACT_POLICY_APP_OSC_HISTORY_FRAMES];
static uint8_t s_osc_sample_count;
static uint8_t s_osc_samples_since_level_change;
static uint32_t s_last_ensemble_change_ms;
static uint32_t s_recovery_start_ms;
static uint32_t s_max_oscillation_start_ms;
static ActPolicyApp_OscDebug_t s_osc_debug;

static const float
  s_ensemble_weights[ACT_POLICY_APP_MAX_ENSEMBLE_COUNT] =
{
  1.00f, 0.67f, 0.45f, 0.30f, 0.20f
};

/**
 * @brief 判断一个模型输出是否为有效有限数值。
 * @param value 需要检查的浮点数。
 * @return 1表示有效，0表示NaN或正负无穷。
 */
static uint8_t ActPolicyApp_IsFinite(float value)
{
  if (value != value)
  {
    return 0U;
  }

  if ((value > FLT_MAX) || (value < -FLT_MAX))
  {
    return 0U;
  }

  return 1U;
}

/**
 * @brief 返回64位有符号数的绝对值。
 * @param value 输入值。
 * @return 输入值的绝对值。
 */
static int64_t ActPolicyApp_AbsI64(int64_t value)
{
  return (value >= 0) ? value : -value;
}

/**
 * @brief 读取一条振荡记录中的指定信号。
 * @param sample 振荡记录。
 * @param source 模型目标、发送目标或实际位置。
 * @param motor 电机下标，范围0~2。
 * @return 对应位置，单位0.01rad。
 */
static int32_t ActPolicyApp_GetOscValue(
  const ActPolicyApp_OscSample_t *sample,
  ActPolicyApp_OscSource_t source,
  uint8_t motor)
{
  if (source == ACT_POLICY_APP_OSC_MODEL)
  {
    return sample->model_target_q[motor];
  }
  if (source == ACT_POLICY_APP_OSC_SENT)
  {
    return sample->sent_target_q[motor];
  }
  return sample->actual_q[motor];
}

/**
 * @brief 统计一个电机在当前窗口中的运动量、净位移和反转次数。
 * @param motor 电机下标，范围0~2。
 * @param source 需要分析的信号来源。
 * @param deadband_q 小于该值的单帧变化不参与方向和路程统计。
 * @param window_frames 从最新样本向前统计的最大帧数。
 * @param stats 输出统计结果。
 */
static void ActPolicyApp_AnalyzeOscillation(
  uint8_t motor,
  ActPolicyApp_OscSource_t source,
  int32_t deadband_q,
  uint8_t window_frames,
  ActPolicyApp_OscStats_t *stats)
{
  int32_t first;
  int32_t last;
  int32_t minimum;
  int32_t maximum;
  int8_t previous_direction = 0;
  uint8_t start_index;
  uint8_t i;

  memset(stats, 0, sizeof(*stats));
  if ((s_osc_sample_count < 2U) || (window_frames < 2U))
  {
    return;
  }

  start_index = (s_osc_sample_count > window_frames) ?
                (uint8_t)(s_osc_sample_count - window_frames) : 0U;
  first = ActPolicyApp_GetOscValue(
    &s_osc_samples[start_index], source, motor);
  last = ActPolicyApp_GetOscValue(
    &s_osc_samples[s_osc_sample_count - 1U], source, motor);
  minimum = first;
  maximum = first;

  for (i = (uint8_t)(start_index + 1U);
       i < s_osc_sample_count;
       i++)
  {
    int32_t current =
      ActPolicyApp_GetOscValue(&s_osc_samples[i], source, motor);
    int32_t previous =
      ActPolicyApp_GetOscValue(&s_osc_samples[i - 1U], source, motor);
    int64_t delta = (int64_t)current - (int64_t)previous;
    int64_t magnitude = ActPolicyApp_AbsI64(delta);
    int8_t direction;

    if (current < minimum)
    {
      minimum = current;
    }
    if (current > maximum)
    {
      maximum = current;
    }
    if (magnitude < deadband_q)
    {
      continue;
    }

    stats->travel_q += magnitude;
    direction = (delta > 0) ? 1 : -1;
    if ((previous_direction != 0) &&
        (direction != previous_direction))
    {
      stats->reversal_count++;
    }
    previous_direction = direction;
  }

  stats->net_q = ActPolicyApp_AbsI64((int64_t)last - first);
  stats->span_q = (int64_t)maximum - minimum;
  if (stats->travel_q > stats->net_q)
  {
    stats->backtrack_q =
      (stats->travel_q - stats->net_q) / 2;
  }
}

/**
 * @brief 清空振荡检测状态，并恢复单chunk控制。
 * @note 不会清除模型的12帧输入历史。
 */
static void ActPolicyApp_ResetAdaptiveFusion(void)
{
  memset(s_osc_samples, 0, sizeof(s_osc_samples));
  memset(&s_osc_debug, 0, sizeof(s_osc_debug));
  s_osc_sample_count = 0U;
  s_osc_samples_since_level_change = 0U;
  s_last_ensemble_change_ms = 0U;
  s_recovery_start_ms = 0U;
  s_max_oscillation_start_ms = 0U;
  s_status.active_ensemble_count = 1U;
  s_status.oscillation_mask = 0U;
}

/**
 * @brief 保存一次成功下发目标对应的模型、发送和实际位置。
 * @param timestamp_ms 当前HAL毫秒时间。
 * @param model_target_q 最新模型未经融合的目标。
 * @param sent_target_q 本次成功发送给G4的目标。
 * @param actual_q 本次推理使用的G4实际位置。
 */
static void ActPolicyApp_RecordOscillationSample(
  uint32_t timestamp_ms,
  const int32_t model_target_q[ACT_POLICY_APP_MOTOR_COUNT],
  const int32_t sent_target_q[ACT_POLICY_APP_MOTOR_COUNT],
  const int32_t actual_q[ACT_POLICY_APP_MOTOR_COUNT])
{
  uint8_t index;

  if (s_osc_sample_count < ACT_POLICY_APP_OSC_HISTORY_FRAMES)
  {
    index = s_osc_sample_count++;
  }
  else
  {
    memmove(&s_osc_samples[0],
            &s_osc_samples[1],
            sizeof(s_osc_samples[0]) *
            (ACT_POLICY_APP_OSC_HISTORY_FRAMES - 1U));
    index = ACT_POLICY_APP_OSC_HISTORY_FRAMES - 1U;
  }

  s_osc_samples[index].timestamp_ms = timestamp_ms;
  memcpy(s_osc_samples[index].model_target_q,
         model_target_q,
         sizeof(s_osc_samples[index].model_target_q));
  memcpy(s_osc_samples[index].sent_target_q,
         sent_target_q,
         sizeof(s_osc_samples[index].sent_target_q));
  memcpy(s_osc_samples[index].actual_q,
         actual_q,
         sizeof(s_osc_samples[index].actual_q));

  if (s_osc_samples_since_level_change < UINT8_MAX)
  {
    s_osc_samples_since_level_change++;
  }
}

/**
 * @brief 找出模型目标、发送目标和实际位置均发生振荡的电机。
 * @return 位0~2分别对应三台电机；0表示没有确认模型振荡。
 */
static uint8_t ActPolicyApp_BuildOscillationMask(void)
{
  uint8_t mask = 0U;
  uint8_t motor;
  int64_t largest_backtrack_q = -1;

  memset(&s_osc_debug, 0, sizeof(s_osc_debug));
  if (s_osc_sample_count < ACT_POLICY_APP_OSC_ACTUAL_FRAMES)
  {
    return 0U;
  }

  for (motor = 0U; motor < ACT_POLICY_APP_MOTOR_COUNT; motor++)
  {
    ActPolicyApp_OscStats_t actual;
    ActPolicyApp_OscStats_t sent;
    ActPolicyApp_OscStats_t model;
    uint8_t actual_oscillating;

    ActPolicyApp_AnalyzeOscillation(
      motor, ACT_POLICY_APP_OSC_ACTUAL,
      ACT_POLICY_APP_OSC_ACTUAL_DEAD_Q,
      ACT_POLICY_APP_OSC_ACTUAL_FRAMES, &actual);
    ActPolicyApp_AnalyzeOscillation(
      motor, ACT_POLICY_APP_OSC_SENT,
      ACT_POLICY_APP_OSC_TARGET_DEAD_Q,
      ACT_POLICY_APP_OSC_HISTORY_FRAMES, &sent);
    ActPolicyApp_AnalyzeOscillation(
      motor, ACT_POLICY_APP_OSC_MODEL,
      ACT_POLICY_APP_OSC_TARGET_DEAD_Q,
      ACT_POLICY_APP_OSC_HISTORY_FRAMES, &model);

    if (actual.backtrack_q > largest_backtrack_q)
    {
      largest_backtrack_q = actual.backtrack_q;
      s_osc_debug.motor = (uint8_t)(motor + 1U);
      s_osc_debug.actual_reversals = actual.reversal_count;
      s_osc_debug.sent_reversals = sent.reversal_count;
      s_osc_debug.model_reversals = model.reversal_count;
      s_osc_debug.actual_span_q =
        (actual.span_q > UINT32_MAX) ?
        UINT32_MAX : (uint32_t)actual.span_q;
      s_osc_debug.actual_backtrack_q =
        (actual.backtrack_q > UINT32_MAX) ?
        UINT32_MAX : (uint32_t)actual.backtrack_q;
    }

    actual_oscillating =
      ((actual.span_q >= ACT_POLICY_APP_OSC_SPAN_Q) &&
       (actual.reversal_count >= 2U) &&
       (actual.backtrack_q >= ACT_POLICY_APP_OSC_BACKTRACK_Q) &&
       (actual.travel_q > 0) &&
       ((actual.net_q * 100) <=
        (actual.travel_q * ACT_POLICY_APP_OSC_NET_PERCENT))) ? 1U : 0U;

    if ((actual_oscillating != 0U) &&
        (sent.reversal_count >= 1U) &&
        (model.reversal_count >= 1U))
    {
      mask |= (uint8_t)(1U << motor);
    }
  }

  return mask;
}

/**
 * @brief 判断触手是否已恢复为有明确净位移的稳定运动。
 * @return 1表示至少一台电机稳定前进，0表示尚未确认。
 */
static uint8_t ActPolicyApp_HasStableProgress(void)
{
  uint8_t motor;

  for (motor = 0U; motor < ACT_POLICY_APP_MOTOR_COUNT; motor++)
  {
    ActPolicyApp_OscStats_t actual;
    ActPolicyApp_OscStats_t sent;
    ActPolicyApp_OscStats_t model;

    ActPolicyApp_AnalyzeOscillation(
      motor, ACT_POLICY_APP_OSC_ACTUAL,
      ACT_POLICY_APP_OSC_ACTUAL_DEAD_Q,
      ACT_POLICY_APP_OSC_ACTUAL_FRAMES, &actual);
    ActPolicyApp_AnalyzeOscillation(
      motor, ACT_POLICY_APP_OSC_SENT,
      ACT_POLICY_APP_OSC_TARGET_DEAD_Q,
      ACT_POLICY_APP_OSC_HISTORY_FRAMES, &sent);
    ActPolicyApp_AnalyzeOscillation(
      motor, ACT_POLICY_APP_OSC_MODEL,
      ACT_POLICY_APP_OSC_TARGET_DEAD_Q,
      ACT_POLICY_APP_OSC_HISTORY_FRAMES, &model);

    if ((actual.net_q >= ACT_POLICY_APP_OSC_RECOVER_Q) &&
        (actual.reversal_count <= 1U) &&
        (sent.reversal_count <= 1U) &&
        (model.reversal_count <= 1U))
    {
      return 1U;
    }
  }

  return 0U;
}

/**
 * @brief 更新振荡检测和自适应融合等级。
 * @param now_ms 当前HAL毫秒时间。
 * @param model_target_q 最新模型原始目标。
 * @param sent_target_q 本次成功发送目标。
 * @param actual_q 当前G4实际位置。
 * @return 1表示5级融合后仍持续振荡，需要安全停止。
 */
static uint8_t ActPolicyApp_UpdateAdaptiveFusion(
  uint32_t now_ms,
  const int32_t model_target_q[ACT_POLICY_APP_MOTOR_COUNT],
  const int32_t sent_target_q[ACT_POLICY_APP_MOTOR_COUNT],
  const int32_t actual_q[ACT_POLICY_APP_MOTOR_COUNT])
{
  uint8_t oscillation_mask;

  ActPolicyApp_RecordOscillationSample(
    now_ms, model_target_q, sent_target_q, actual_q);
  oscillation_mask = ActPolicyApp_BuildOscillationMask();
  s_status.oscillation_mask = oscillation_mask;

  if (oscillation_mask != 0U)
  {
    s_recovery_start_ms = 0U;

    if (s_status.active_ensemble_count <
        ACT_POLICY_APP_MAX_ENSEMBLE_COUNT)
    {
      s_max_oscillation_start_ms = 0U;
      if ((s_osc_samples_since_level_change >=
           ACT_POLICY_APP_OSC_FRESH_FRAMES) &&
          ((uint32_t)(now_ms - s_last_ensemble_change_ms) >=
           ACT_POLICY_APP_OSC_LEVEL_WAIT_MS))
      {
        s_status.active_ensemble_count++;
        s_osc_samples_since_level_change = 0U;
        s_last_ensemble_change_ms = now_ms;
      }
      return 0U;
    }

    if (s_max_oscillation_start_ms == 0U)
    {
      s_max_oscillation_start_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - s_max_oscillation_start_ms) >=
             ACT_POLICY_APP_OSC_MAX_HOLD_MS)
    {
      return 1U;
    }
    return 0U;
  }

  s_max_oscillation_start_ms = 0U;
  if ((s_status.active_ensemble_count > 1U) &&
      (ActPolicyApp_HasStableProgress() != 0U))
  {
    if (s_recovery_start_ms == 0U)
    {
      s_recovery_start_ms = now_ms;
    }
    else if (((uint32_t)(now_ms - s_recovery_start_ms) >=
              ACT_POLICY_APP_OSC_RECOVER_MS) &&
             (s_osc_samples_since_level_change >=
              ACT_POLICY_APP_OSC_FRESH_FRAMES))
    {
      s_status.active_ensemble_count--;
      s_osc_samples_since_level_change = 0U;
      s_last_ensemble_change_ms = now_ms;
      s_recovery_start_ms = 0U;
    }
  }
  else
  {
    s_recovery_start_ms = 0U;
  }

  return 0U;
}

/**
 * @brief 判断一个VL53区域是否具有有效距离。
 * @param frame 当前ACT原始帧。
 * @param zone 区域编号，范围0~63。
 * @return 1有效，0无效。
 */
static uint8_t ActPolicyApp_IsZoneValid(
  const VL53_App_ActFrame_t *frame,
  uint8_t zone)
{
  uint8_t status = frame->target_status[zone];

  return (((status == 5U) || (status == 9U)) &&
          (frame->target_count[zone] > 0U) &&
          (frame->distance_mm[zone] > 0)) ? 1U : 0U;
}

/**
 * @brief 将一帧VL53原始数据转换为模型需要的18通道特征。
 * @param frame 当前ACT原始帧。
 * @param output 输出1152个float，排列为18通道乘64区域。
 * @return 1成功，0表示参数或CNH缩放指数非法。
 * @note 通道顺序必须与training/preprocess.py完全一致。
 */
static uint8_t ActPolicyApp_BuildTofFrame(
  const VL53_App_ActFrame_t *frame,
  float output[ACT_POLICY_APP_FRAME_VALUES])
{
  uint8_t zone;
  uint8_t bin;

  if ((frame == NULL) || (output == NULL))
  {
    return 0U;
  }

  for (zone = 0U; zone < ACT_POLICY_APP_ZONE_COUNT; zone++)
  {
    uint8_t valid = ActPolicyApp_IsZoneValid(frame, zone);

    output[0U * 64U + zone] =
      (valid != 0U) ? (float)frame->distance_mm[zone] : 0.0f;
    output[1U * 64U + zone] = (float)valid;
    output[2U * 64U + zone] = (float)frame->target_status[zone];
    output[3U * 64U + zone] = (float)frame->target_count[zone];
    output[4U * 64U + zone] = (float)frame->signal_per_spad[zone];
    output[5U * 64U + zone] = (float)frame->ambient_per_spad[zone];
    output[6U * 64U + zone] = (float)frame->reflectance[zone];
    output[7U * 64U + zone] = (float)frame->range_sigma_mm[zone];

    for (bin = 0U; bin < VL53_APP_ACT_CNH_BINS; bin++)
    {
      int8_t scaler = frame->cnh_scaler[bin][zone];
      uint32_t divisor;

      if ((scaler < 0) || (scaler > 30))
      {
        return 0U;
      }

      /* 与NumPy ldexp(raw, -(scaler + 1))保持一致。 */
      divisor = 1UL << ((uint8_t)scaler + 1U);
      output[(8U + bin) * 64U + zone] =
        (float)frame->cnh_raw[bin][zone] / (float)divisor;
    }
  }

  return 1U;
}

/**
 * @brief 清空ACT历史，不改变已经创建的CubeAI模型。
 */
static void ActPolicyApp_ClearHistory(void)
{
  memset(s_tof_history, 0, sizeof(s_tof_history));
  memset(s_state_history, 0, sizeof(s_state_history));
  memset(s_action_chunks, 0, sizeof(s_action_chunks));
  memset(s_action_chunk_frame_seq, 0, sizeof(s_action_chunk_frame_seq));
  memset(s_committed_action_q, 0, sizeof(s_committed_action_q));
  memset(s_last_target_q, 0, sizeof(s_last_target_q));
  s_status.history_count = 0U;
  s_action_chunk_count = 0U;
  s_committed_action_step = 0U;
  s_committed_action_valid = 0U;
  s_last_target_valid = 0U;
  s_last_frame_seq = 0U;
  s_last_frame_ms = 0U;
  s_last_frame_delta = 0U;
  ActPolicyApp_ResetAdaptiveFusion();
}

/**
 * @brief 保存最新完整动作chunk，供同一绝对时刻预测融合。
 * @param model_output 模型输出的10组、共30个目标。
 * @param frame_seq 产生该chunk的VL53帧序号。
 */
static void ActPolicyApp_SaveActionChunk(const float *model_output,
                                         uint32_t frame_seq)
{
  uint8_t move_count = s_action_chunk_count;

  if (move_count >= ACT_POLICY_APP_MAX_ENSEMBLE_COUNT)
  {
    move_count = ACT_POLICY_APP_MAX_ENSEMBLE_COUNT - 1U;
  }
  else
  {
    s_action_chunk_count++;
  }

  if (move_count > 0U)
  {
    memmove(s_action_chunks[1],
            s_action_chunks[0],
            sizeof(s_action_chunks[0]) * move_count);
    memmove(&s_action_chunk_frame_seq[1],
            &s_action_chunk_frame_seq[0],
            sizeof(s_action_chunk_frame_seq[0]) * move_count);
  }
  memcpy(s_action_chunks[0], model_output, sizeof(s_action_chunks[0]));
  s_action_chunk_frame_seq[0] = frame_seq;
}

/**
 * @brief 融合多次推理中指向同一未来时刻的电机目标。
 * @param motor 电机下标，范围0至2。
 * @param current_frame_seq 当前推理使用的VL53帧序号。
 * @param future_step 相对当前控制时刻继续向后执行的步数。
 * @return 融合后的相对HOME目标，单位0.01rad。
 */
static float ActPolicyApp_BuildFusedTarget(uint8_t motor,
                                           uint32_t current_frame_seq,
                                           uint8_t future_step)
{
  float weighted_sum = 0.0f;
  float weight_sum = 0.0f;
  uint8_t used_count = 0U;
  uint8_t age;

  if ((s_status.active_ensemble_count == 0U) ||
      (s_status.active_ensemble_count >
       ACT_POLICY_APP_MAX_ENSEMBLE_COUNT))
  {
    s_status.active_ensemble_count = 1U;
  }
  for (age = 0U;
       (age < s_action_chunk_count) &&
       (used_count < s_status.active_ensemble_count);
       age++)
  {
    uint32_t frame_delta =
      current_frame_seq - s_action_chunk_frame_seq[age];
    uint8_t step;
    uint32_t offset =
      (ACT_POLICY_APP_ACTION_HORIZON_INDEX + future_step) *
      ACT_POLICY_APP_MOTOR_COUNT + motor;
    float weight;

    if (frame_delta >= ACT_POLICY_APP_MAX_ENSEMBLE_COUNT)
    {
      continue;
    }

    step = (uint8_t)(ACT_POLICY_APP_ACTION_HORIZON_INDEX +
                     future_step + frame_delta);
    if (step >=
        (AI_ACT_POLICY_OUT_1_SIZE / ACT_POLICY_APP_MOTOR_COUNT))
    {
      continue;
    }

    offset = (uint32_t)step * ACT_POLICY_APP_MOTOR_COUNT + motor;
    weight = s_ensemble_weights[frame_delta];

    weighted_sum += s_action_chunks[age][offset] * weight;
    weight_sum += weight;
    used_count++;
  }

  if (weight_sum > 0.0f)
  {
    return weighted_sum / weight_sum;
  }

  /* 最新chunk正常情况下必定有效；该回退避免异常序号导致目标突变为0。 */
  return (s_action_chunk_count > 0U) ?
         s_action_chunks[0][
           (ACT_POLICY_APP_ACTION_HORIZON_INDEX + future_step) *
           ACT_POLICY_APP_MOTOR_COUNT + motor] : 0.0f;
}

/**
 * @brief 将最新模型目标限制到允许范围并转换为整数Q值。
 * @param model_q 模型原始输出，单位0.01rad。
 * @return 限幅并四舍五入后的整数目标。
 */
static int32_t ActPolicyApp_ModelTargetToQ(float model_q)
{
  if (model_q > (float)ACT_POLICY_APP_TARGET_LIMIT_Q)
  {
    model_q = (float)ACT_POLICY_APP_TARGET_LIMIT_Q;
  }
  else if (model_q < (float)-ACT_POLICY_APP_TARGET_LIMIT_Q)
  {
    model_q = (float)-ACT_POLICY_APP_TARGET_LIMIT_Q;
  }

  return (model_q >= 0.0f) ?
         (int32_t)(model_q + 0.5f) :
         (int32_t)(model_q - 0.5f);
}

/**
 * @brief 将一帧同步的VL53和电机数据加入12帧历史。
 * @param frame 当前VL53帧。
 * @param motor 与该帧同步读取的触手3电机状态。
 * @return 1成功，0表示输入数据非法。
 */
static uint8_t ActPolicyApp_AppendHistory(
  const VL53_App_ActFrame_t *frame,
  const MotorControl_ActData_t *motor)
{
  uint32_t frame_index;
  uint8_t i;

  if ((frame == NULL) || (motor == NULL))
  {
    return 0U;
  }

  if (s_status.history_count < ACT_POLICY_APP_HISTORY_FRAMES)
  {
    frame_index = s_status.history_count;
    s_status.history_count++;
  }
  else
  {
    memmove(s_tof_history,
            &s_tof_history[ACT_POLICY_APP_FRAME_VALUES],
            sizeof(float) *
            (AI_ACT_POLICY_IN_1_SIZE - ACT_POLICY_APP_FRAME_VALUES));
    memmove(s_state_history,
            &s_state_history[ACT_POLICY_APP_MOTOR_COUNT],
            sizeof(float) *
            (AI_ACT_POLICY_IN_2_SIZE - ACT_POLICY_APP_MOTOR_COUNT));
    frame_index = ACT_POLICY_APP_HISTORY_FRAMES - 1U;
  }

  if (ActPolicyApp_BuildTofFrame(
        frame,
        &s_tof_history[frame_index * ACT_POLICY_APP_FRAME_VALUES]) == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < ACT_POLICY_APP_MOTOR_COUNT; i++)
  {
    s_state_history[
      frame_index * ACT_POLICY_APP_MOTOR_COUNT + i] =
      (float)motor->actual_q[i];
  }

  return 1U;
}

/**
 * @brief 限制模型目标范围、实际位置领先量和连续发送斜率。
 * @param model_q 融合后的目标，单位0.01rad。
 * @param actual_q 当前实际位置，单位0.01rad。
 * @param previous_q 上一次成功发送的目标。
 * @return 本次允许发送的相对HOME目标。
 */
static int32_t ActPolicyApp_LimitTarget(float model_q,
                                       int32_t actual_q,
                                       int32_t previous_q)
{
  int64_t target_q;
  int64_t delta_q;

  if (model_q > (float)ACT_POLICY_APP_TARGET_LIMIT_Q)
  {
    model_q = (float)ACT_POLICY_APP_TARGET_LIMIT_Q;
  }
  else if (model_q < (float)-ACT_POLICY_APP_TARGET_LIMIT_Q)
  {
    model_q = (float)-ACT_POLICY_APP_TARGET_LIMIT_Q;
  }

  target_q = (int64_t)((model_q >= 0.0f) ?
                       (model_q + 0.5f) :
                       (model_q - 0.5f));
  delta_q = target_q - (int64_t)actual_q;

  if (delta_q > ACT_POLICY_APP_TARGET_LEAD_Q)
  {
    target_q = (int64_t)actual_q + ACT_POLICY_APP_TARGET_LEAD_Q;
  }
  else if (delta_q < -ACT_POLICY_APP_TARGET_LEAD_Q)
  {
    target_q = (int64_t)actual_q - ACT_POLICY_APP_TARGET_LEAD_Q;
  }

  delta_q = target_q - (int64_t)previous_q;

  if (delta_q > ACT_POLICY_APP_TARGET_STEP_Q)
  {
    target_q = (int64_t)previous_q + ACT_POLICY_APP_TARGET_STEP_Q;
  }
  else if (delta_q < -ACT_POLICY_APP_TARGET_STEP_Q)
  {
    target_q = (int64_t)previous_q - ACT_POLICY_APP_TARGET_STEP_Q;
  }

  return (int32_t)target_q;
}

/**
 * @brief 清理未完成初始化的ACT模型实例。
 * @note 只在初始化失败时调用，确保后续可以再次尝试初始化。
 */
static void ActPolicyApp_Cleanup(void)
{
  if (s_network != AI_HANDLE_NULL)
  {
    s_network = ai_act_policy_destroy(s_network);
  }

  s_inputs = NULL;
  s_outputs = NULL;
  s_status.initialized = 0U;
}

/**
 * @brief 锁存ACT故障并触发带重试的安全STOP。
 * @param result 本次停止原因。
 * @note 只在首次进入FAULT时启动STOP，不会在主循环中反复触发。
 */
static void ActPolicyApp_Fail(ActPolicyApp_Result_t result)
{
  if (s_status.run_state == ACT_POLICY_APP_STATE_FAULT)
  {
    return;
  }

  s_status.result = result;
  s_status.run_state = ACT_POLICY_APP_STATE_FAULT;
  s_status.history_count = 0U;
  s_committed_action_step = 0U;
  s_committed_action_valid = 0U;
  (void)MotorControl_EmergencyStop();
}

/**
 * @brief 使用当前12帧历史执行一次模型推理。
 * @param frame_seq 当前推理所对应的VL53帧序号。
 * @return 推理结果。
 */
static ActPolicyApp_Result_t ActPolicyApp_RunHistory(uint32_t frame_seq)
{
  const float *model_output;
  uint32_t start_ms;
  uint32_t i;
  ai_i32 batch;

  memcpy(s_inputs[ACT_POLICY_APP_TOF_INPUT_INDEX].data,
         s_tof_history,
         sizeof(s_tof_history));
  memcpy(s_inputs[ACT_POLICY_APP_STATE_INPUT_INDEX].data,
         s_state_history,
         sizeof(s_state_history));

  start_ms = HAL_GetTick();
  batch = ai_act_policy_run(s_network, s_inputs, s_outputs);
  s_status.inference_time_ms = HAL_GetTick() - start_ms;
  if (batch != 1)
  {
    return ACT_POLICY_APP_RESULT_RUN_FAILED;
  }

  model_output =
    (const float *)s_outputs[ACT_POLICY_APP_OUTPUT_INDEX].data;
  for (i = 0U; i < AI_ACT_POLICY_OUT_1_SIZE; i++)
  {
    if (ActPolicyApp_IsFinite(model_output[i]) == 0U)
    {
      return ACT_POLICY_APP_RESULT_OUTPUT_INVALID;
    }
  }

  ActPolicyApp_SaveActionChunk(model_output, frame_seq);
  memcpy(s_status.first_action_q,
         model_output,
         sizeof(s_status.first_action_q));
  s_status.run_count++;
  return ACT_POLICY_APP_RESULT_OK;
}

ActPolicyApp_Result_t ActPolicyApp_Init(void)
{
  ai_error error;
  ai_u16 input_count = 0U;
  ai_u16 output_count = 0U;

  if (s_status.initialized != 0U)
  {
    return ACT_POLICY_APP_RESULT_OK;
  }

  memset(&s_status, 0, sizeof(s_status));
  ActPolicyApp_Cleanup();

  error = ai_act_policy_create_and_init(
    &s_network,
    s_activation_handles,
    NULL);

  if (error.type != AI_ERROR_NONE)
  {
    ActPolicyApp_Cleanup();
    s_status.result = ACT_POLICY_APP_RESULT_CREATE_FAILED;
    return s_status.result;
  }

  s_inputs = ai_act_policy_inputs_get(s_network, &input_count);
  s_outputs = ai_act_policy_outputs_get(s_network, &output_count);

  if ((s_inputs == NULL) ||
      (s_outputs == NULL) ||
      (input_count != AI_ACT_POLICY_IN_NUM) ||
      (output_count != AI_ACT_POLICY_OUT_NUM) ||
      (s_inputs[ACT_POLICY_APP_TOF_INPUT_INDEX].data == NULL) ||
      (s_inputs[ACT_POLICY_APP_STATE_INPUT_INDEX].data == NULL) ||
      (s_outputs[ACT_POLICY_APP_OUTPUT_INDEX].data == NULL))
  {
    ActPolicyApp_Cleanup();
    s_status.result = ACT_POLICY_APP_RESULT_CREATE_FAILED;
    return s_status.result;
  }

  s_status.initialized = 1U;
  s_status.selected_tentacle = ACT_POLICY_APP_TENTACLE;
  s_status.run_state = ACT_POLICY_APP_STATE_OFF;
  s_status.result = ACT_POLICY_APP_RESULT_OK;
  s_status.active_ensemble_count = 1U;
  return s_status.result;
}

ActPolicyApp_Result_t ActPolicyApp_RunSelfTest(void)
{
  const float *model_output;
  uint32_t start_ms;
  uint32_t i;
  ai_i32 batch;

  if ((s_status.initialized == 0U) ||
      (s_network == AI_HANDLE_NULL) ||
      (s_inputs == NULL) ||
      (s_outputs == NULL))
  {
    s_status.result = ACT_POLICY_APP_RESULT_NOT_READY;
    return s_status.result;
  }
  if (s_status.run_state != ACT_POLICY_APP_STATE_OFF)
  {
    s_status.result = ACT_POLICY_APP_RESULT_BUSY;
    return s_status.result;
  }

  memset(s_status.first_action_q, 0, sizeof(s_status.first_action_q));
  s_status.inference_time_ms = 0U;
  memset(s_inputs[ACT_POLICY_APP_TOF_INPUT_INDEX].data,
         0,
         AI_ACT_POLICY_IN_1_SIZE_BYTES);
  memset(s_inputs[ACT_POLICY_APP_STATE_INPUT_INDEX].data,
         0,
         AI_ACT_POLICY_IN_2_SIZE_BYTES);

  start_ms = HAL_GetTick();
  batch = ai_act_policy_run(s_network, s_inputs, s_outputs);
  s_status.inference_time_ms = HAL_GetTick() - start_ms;
  if (batch != 1)
  {
    s_status.result = ACT_POLICY_APP_RESULT_RUN_FAILED;
    return s_status.result;
  }

  model_output =
    (const float *)s_outputs[ACT_POLICY_APP_OUTPUT_INDEX].data;
  for (i = 0U; i < AI_ACT_POLICY_OUT_1_SIZE; i++)
  {
    if (ActPolicyApp_IsFinite(model_output[i]) == 0U)
    {
      s_status.result = ACT_POLICY_APP_RESULT_OUTPUT_INVALID;
      return s_status.result;
    }
  }

  memcpy(s_status.first_action_q,
         model_output,
         sizeof(s_status.first_action_q));
  s_status.run_count++;
  s_status.result = ACT_POLICY_APP_RESULT_OK;
  return s_status.result;
}

ActPolicyApp_Result_t ActPolicyApp_SetRun(uint8_t enable,
                                         uint8_t tentacle)
{
  const VL53_App_ActFrame_t *frame;
  const MotorControl_Status_t *motor_status;
  MotorControl_Summary_t summary;
  uint32_t now_ms = HAL_GetTick();
  uint8_t mask;
  uint8_t status;

  if (enable == 0U)
  {
    status = 0U;
    motor_status = MotorControl_GetStatus();
    if ((s_status.run_state != ACT_POLICY_APP_STATE_OFF) &&
        (motor_status->control_source == MOTOR_CONTROL_SOURCE_ACT) &&
        (motor_status->enable != 0U))
    {
      status = MotorControl_StopActive();
    }

    ActPolicyApp_ClearHistory();
    s_status.run_state = ACT_POLICY_APP_STATE_OFF;
    s_status.result = (status == 0U) ?
                      ACT_POLICY_APP_RESULT_OK :
                      ACT_POLICY_APP_RESULT_TX_FAILED;
    return s_status.result;
  }

  /* 当前模型只使用触手3示教数据，禁止误控制其他触手。 */
  if (tentacle != ACT_POLICY_APP_TENTACLE)
  {
    return ACT_POLICY_APP_RESULT_BAD_ARGUMENT;
  }
  if (s_status.run_state != ACT_POLICY_APP_STATE_OFF)
  {
    return ACT_POLICY_APP_RESULT_BUSY;
  }
  if (ActPolicyApp_Init() != ACT_POLICY_APP_RESULT_OK)
  {
    return s_status.result;
  }

  frame = VL53_App_GetLatestActFrame();
  if ((VL53_App_IsReady() == 0U) ||
      (frame == NULL) ||
      (frame->frame_seq == 0U) ||
      ((uint32_t)(now_ms - frame->timestamp_ms) >
       ACT_POLICY_APP_SENSOR_TIMEOUT_MS))
  {
    return ACT_POLICY_APP_RESULT_SENSOR_TIMEOUT;
  }

  motor_status = MotorControl_GetStatus();
  if (motor_status->enable != 0U)
  {
    return ACT_POLICY_APP_RESULT_BUSY;
  }

  mask = MOTOR_CONTROL_T3_MASK;
  MotorControl_GetSummary(&summary);
  if ((summary.fault_mask & mask) != 0U)
  {
    return ACT_POLICY_APP_RESULT_MOTOR_FAULT;
  }
  if (((summary.online_mask & mask) == 0U) ||
      ((summary.ready_mask & mask) == 0U))
  {
    return ACT_POLICY_APP_RESULT_MOTOR_NOT_READY;
  }

  status = MotorControl_SelectWork(mask, 0U);
  if ((status == 2U) || (status == 3U))
  {
    return ACT_POLICY_APP_RESULT_MOTOR_NOT_READY;
  }
  if (status != 0U)
  {
    return ACT_POLICY_APP_RESULT_TX_FAILED;
  }

  status = MotorControl_SetControlSource(MOTOR_CONTROL_SOURCE_ACT);
  if (status != 0U)
  {
    return ACT_POLICY_APP_RESULT_TX_FAILED;
  }

  MotorControl_SetEnable(1U);
  motor_status = MotorControl_GetStatus();
  if (motor_status->enable == 0U)
  {
    return ACT_POLICY_APP_RESULT_MOTOR_NOT_READY;
  }

  ActPolicyApp_ClearHistory();
  s_status.selected_tentacle = ACT_POLICY_APP_TENTACLE;
  s_status.run_state = ACT_POLICY_APP_STATE_WARMUP;
  s_status.result = ACT_POLICY_APP_RESULT_OK;
  s_last_frame_seq = frame->frame_seq;
  s_last_frame_ms = frame->timestamp_ms;
  return s_status.result;
}

void ActPolicyApp_Task(void)
{
  const VL53_App_ActFrame_t *frame;
  const MotorControl_Status_t *control;
  MotorControl_ActData_t motor;
  ActPolicyApp_Result_t result;
  int32_t model_target_q[ACT_POLICY_APP_MOTOR_COUNT];
  int32_t target_q[ACT_POLICY_APP_MOTOR_COUNT];
  uint32_t previous_frame_seq;
  uint32_t previous_frame_ms;
  uint32_t now_ms;
  uint8_t status;
  uint8_t commit_step;
  uint8_t i;

  if ((s_status.run_state != ACT_POLICY_APP_STATE_WARMUP) &&
      (s_status.run_state != ACT_POLICY_APP_STATE_RUNNING))
  {
    return;
  }

  control = MotorControl_GetStatus();
  if ((control->enable == 0U) ||
      (control->control_source != MOTOR_CONTROL_SOURCE_ACT))
  {
    s_status.result = ACT_POLICY_APP_RESULT_MOTOR_NOT_READY;
    s_status.run_state = ACT_POLICY_APP_STATE_FAULT;
    return;
  }

  now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - s_last_frame_ms) >
      ACT_POLICY_APP_SENSOR_TIMEOUT_MS)
  {
    ActPolicyApp_Fail(ACT_POLICY_APP_RESULT_SENSOR_TIMEOUT);
    return;
  }

  frame = VL53_App_GetLatestActFrame();
  if ((frame == NULL) ||
      (frame->frame_seq == 0U) ||
      (frame->frame_seq == s_last_frame_seq))
  {
    return;
  }

  previous_frame_seq = s_last_frame_seq;
  previous_frame_ms = s_last_frame_ms;
  s_last_frame_seq = frame->frame_seq;
  s_last_frame_ms = frame->timestamp_ms;
  s_last_frame_delta = frame->frame_seq - previous_frame_seq;

  /* 普通跳帧由chunk帧序号补偿；仅长时间中断或序号异常才重置。 */
  if ((s_last_frame_delta > ACT_POLICY_APP_FRAME_RESET_DELTA) ||
      ((uint32_t)(frame->timestamp_ms - previous_frame_ms) >
       ACT_POLICY_APP_OSC_FRAME_GAP_MS))
  {
    s_action_chunk_count = 0U;
    s_committed_action_step = 0U;
    s_committed_action_valid = 0U;
    memset(s_action_chunk_frame_seq,
           0,
           sizeof(s_action_chunk_frame_seq));
    ActPolicyApp_ResetAdaptiveFusion();
  }

  MotorControl_CopyActData(ACT_POLICY_APP_TENTACLE, &motor);

  if ((motor.valid_flags & MOTOR_CONTROL_ACT_FAULT) != 0U)
  {
    ActPolicyApp_Fail(ACT_POLICY_APP_RESULT_MOTOR_FAULT);
    return;
  }
  if (motor.status_age_ms > ACT_POLICY_APP_CAN_STOP_MS)
  {
    ActPolicyApp_Fail(ACT_POLICY_APP_RESULT_MOTOR_NOT_READY);
    return;
  }

  /* CAN短暂延迟或RESTORE波动时保持上一目标，不积压新命令。 */
  if ((motor.status_age_ms > ACT_POLICY_APP_CAN_HOLD_MS) ||
      ((motor.valid_flags & MOTOR_CONTROL_ACT_POSITION_VALID) == 0U))
  {
    s_action_chunk_count = 0U;
    s_committed_action_step = 0U;
    s_committed_action_valid = 0U;
    memset(s_action_chunk_frame_seq,
           0,
           sizeof(s_action_chunk_frame_seq));
    ActPolicyApp_ResetAdaptiveFusion();
    return;
  }

  if (ActPolicyApp_AppendHistory(frame, &motor) == 0U)
  {
    ActPolicyApp_Fail(ACT_POLICY_APP_RESULT_OUTPUT_INVALID);
    return;
  }
  if (s_status.history_count < ACT_POLICY_APP_HISTORY_FRAMES)
  {
    return;
  }

  s_status.run_state = ACT_POLICY_APP_STATE_RUNNING;
  result = ActPolicyApp_RunHistory(frame->frame_seq);
  if (result != ACT_POLICY_APP_RESULT_OK)
  {
    ActPolicyApp_Fail(result);
    return;
  }

  if ((s_committed_action_valid == 0U) ||
      (s_committed_action_step >= ACT_POLICY_APP_COMMIT_STEPS))
  {
    /* 一次生成并锁定连续三步，后续两帧的新推理不能改写该轨迹。 */
    for (commit_step = 0U;
         commit_step < ACT_POLICY_APP_COMMIT_STEPS;
         commit_step++)
    {
      for (i = 0U; i < ACT_POLICY_APP_MOTOR_COUNT; i++)
      {
        s_committed_action_q[commit_step][i] =
          ActPolicyApp_BuildFusedTarget(
            i, frame->frame_seq, commit_step);
      }
    }
    s_committed_action_step = 0U;
    s_committed_action_valid = 1U;
  }

  /*
   * 推理约需一个控制帧，不能继续使用推理前的actual做目标限幅。
   * 模型历史保留上面与VL53帧同步的actual；这里重新读取仅供本次控制使用。
   */
  MotorControl_CopyActData(ACT_POLICY_APP_TENTACLE, &motor);
  if ((motor.valid_flags & MOTOR_CONTROL_ACT_FAULT) != 0U)
  {
    ActPolicyApp_Fail(ACT_POLICY_APP_RESULT_MOTOR_FAULT);
    return;
  }
  if (motor.status_age_ms > ACT_POLICY_APP_CAN_STOP_MS)
  {
    ActPolicyApp_Fail(ACT_POLICY_APP_RESULT_MOTOR_NOT_READY);
    return;
  }
  if ((motor.status_age_ms > ACT_POLICY_APP_CAN_HOLD_MS) ||
      ((motor.valid_flags & MOTOR_CONTROL_ACT_POSITION_VALID) == 0U))
  {
    s_action_chunk_count = 0U;
    s_committed_action_step = 0U;
    s_committed_action_valid = 0U;
    memset(s_action_chunk_frame_seq,
           0,
           sizeof(s_action_chunk_frame_seq));
    ActPolicyApp_ResetAdaptiveFusion();
    return;
  }

  for (i = 0U; i < ACT_POLICY_APP_MOTOR_COUNT; i++)
  {
    float committed_target_q =
      s_committed_action_q[s_committed_action_step][i];
    int32_t previous_q = motor.actual_q[i];

    if (s_last_target_valid != 0U)
    {
      int64_t previous_delta_q =
        (int64_t)s_last_target_q[i] - (int64_t)motor.actual_q[i];

      /* 实际位置异常跳变时旧目标已失去参考意义，从当前位置重新限速。 */
      if ((previous_delta_q <= ACT_POLICY_APP_TARGET_LEAD_Q) &&
          (previous_delta_q >= -ACT_POLICY_APP_TARGET_LEAD_Q))
      {
        previous_q = s_last_target_q[i];
      }
    }

    model_target_q[i] = ActPolicyApp_ModelTargetToQ(
      s_action_chunks[0][
        ACT_POLICY_APP_ACTION_HORIZON_INDEX *
        ACT_POLICY_APP_MOTOR_COUNT + i]);
    target_q[i] = ActPolicyApp_LimitTarget(
      committed_target_q,
      motor.actual_q[i],
      previous_q);
  }

  status = MotorControl_SendActTarget(
    ACT_POLICY_APP_TENTACLE,
    target_q[0],
    target_q[1],
    target_q[2]);

  if (status == 2U)
  {
    /* FIFO暂忙时保留当前承诺步；下一帧重试，不能直接跳到下一步。 */
    s_status.result = ACT_POLICY_APP_RESULT_TX_BUSY;
    return;
  }
  if (status != 0U)
  {
    ActPolicyApp_Fail(ACT_POLICY_APP_RESULT_TX_FAILED);
    return;
  }

  memcpy(s_last_target_q, target_q, sizeof(s_last_target_q));
  s_last_target_valid = 1U;

  if (ActPolicyApp_UpdateAdaptiveFusion(
        HAL_GetTick(),
        model_target_q,
        target_q,
        motor.actual_q) != 0U)
  {
    /* ACTOSC保留具体振荡信息；ACTSTAT复用旧MOTOR_FAULT结果码。 */
    ActPolicyApp_Fail(ACT_POLICY_APP_RESULT_MOTOR_FAULT);
    return;
  }

  s_committed_action_step++;
  if (s_committed_action_step >= ACT_POLICY_APP_COMMIT_STEPS)
  {
    s_committed_action_step = 0U;
    s_committed_action_valid = 0U;
  }

  s_status.result = ACT_POLICY_APP_RESULT_OK;
}

ActPolicyApp_RunState_t ActPolicyApp_GetRunState(void)
{
  return s_status.run_state;
}

void ActPolicyApp_GetStatus(ActPolicyApp_Status_t *status)
{
  if (status != NULL)
  {
    *status = s_status;
  }
}

void ActPolicyApp_GetOscDebug(ActPolicyApp_OscDebug_t *debug)
{
  if (debug == NULL)
  {
    return;
  }

  *debug = s_osc_debug;
  debug->active_ensemble_count = s_status.active_ensemble_count;
  debug->oscillation_mask = s_status.oscillation_mask;
  debug->sample_count = s_osc_sample_count;
  debug->frame_delta = s_last_frame_delta;
}
