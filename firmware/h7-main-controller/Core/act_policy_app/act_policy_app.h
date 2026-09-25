#ifndef ACT_POLICY_APP_H
#define ACT_POLICY_APP_H

/**
 * @file act_policy_app.h
 * @brief ACT策略模型的最小运行接口。
 *
 * 本模块负责ACT模型初始化、12帧历史构建、推理和触手3安全控制。
 * 模型输出使用相对HOME的统一线坐标，正数表示收线。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 每个ACT动作包含三台电机的位置目标。 */
#define ACT_POLICY_APP_MOTOR_COUNT    3U
#define ACT_POLICY_APP_HISTORY_FRAMES 12U
#define ACT_POLICY_APP_TENTACLE       3U

/**
 * @brief ACT自主控制运行状态。
 */
typedef enum
{
  ACT_POLICY_APP_STATE_OFF = 0U,     /* 未启用，不会发送电机目标。 */
  ACT_POLICY_APP_STATE_WARMUP,       /* 正在收集12帧历史，不运动。 */
  ACT_POLICY_APP_STATE_RUNNING,      /* 正在根据新VL53帧执行推理。 */
  ACT_POLICY_APP_STATE_FAULT         /* 已安全停止，必须先RUN 0清除。 */
} ActPolicyApp_RunState_t;

/**
 * @brief ACT模型接口执行结果。
 */
typedef enum
{
  ACT_POLICY_APP_RESULT_OK = 0U,          /* 操作执行成功。 */
  ACT_POLICY_APP_RESULT_NOT_READY,        /* 模型尚未成功初始化。 */
  ACT_POLICY_APP_RESULT_CREATE_FAILED,    /* CubeAI模型创建或初始化失败。 */
  ACT_POLICY_APP_RESULT_RUN_FAILED,       /* CubeAI执行单次推理失败。 */
  ACT_POLICY_APP_RESULT_OUTPUT_INVALID,   /* 模型输出包含NaN或无穷值。 */
  ACT_POLICY_APP_RESULT_BAD_ARGUMENT,     /* 参数非法或不是触手3。 */
  ACT_POLICY_APP_RESULT_BUSY,             /* 其他控制来源正在运行。 */
  ACT_POLICY_APP_RESULT_SENSOR_TIMEOUT,   /* VL53数据持续超时。 */
  ACT_POLICY_APP_RESULT_MOTOR_NOT_READY,  /* CAN或RESTORE状态不可控制。 */
  ACT_POLICY_APP_RESULT_MOTOR_FAULT,      /* G4报告堵转等故障。 */
  ACT_POLICY_APP_RESULT_TX_BUSY,          /* CAN FIFO暂时没有空间。 */
  ACT_POLICY_APP_RESULT_TX_FAILED         /* CAN目标或STOP发送失败。 */
} ActPolicyApp_Result_t;

/**
 * @brief ACT模型运行状态摘要。
 *
 * 该结构只用于调试和状态查询，不包含模型输入、激活区等大块缓存。
 */
typedef struct
{
  uint8_t initialized;            /* 1=模型已初始化，可以推理；0=不可推理。 */
  uint8_t selected_tentacle;      /* 固定为训练所用的触手3。 */
  uint8_t history_count;          /* 当前已收集历史帧数，最大12。 */
  uint8_t active_ensemble_count;  /* 当前参与时间融合的chunk数量，范围1~5。 */
  uint8_t oscillation_mask;       /* 已确认模型振荡的电机位掩码。 */
  ActPolicyApp_RunState_t run_state;
  ActPolicyApp_Result_t result;   /* 最近一次初始化或推理的执行结果。 */
  uint32_t run_count;             /* 成功完成的推理次数。 */
  uint32_t inference_time_ms;     /* 最近一次推理耗时，单位ms。 */
  float first_action_q[ACT_POLICY_APP_MOTOR_COUNT];
                                  /* 最近输出序列的第一组目标，单位0.01rad。 */
} ActPolicyApp_Status_t;

/**
 * @brief ACT振荡检测调试摘要。
 * @note motor为1~3；0表示尚未收集到足够样本。
 */
typedef struct
{
  uint8_t active_ensemble_count; /* 当前允许参与融合的chunk数量。 */
  uint8_t oscillation_mask;      /* 已确认模型振荡的电机位掩码。 */
  uint8_t sample_count;          /* 当前振荡历史样本数，最大10。 */
  uint8_t motor;                 /* 当前回退量最大的电机，范围1~3。 */
  uint8_t actual_reversals;      /* 该电机最近8个实际位置样本的反转次数。 */
  uint8_t sent_reversals;        /* 该电机最近10个发送目标的反转次数。 */
  uint8_t model_reversals;       /* 该电机最近10个模型目标的反转次数。 */
  uint32_t actual_span_q;        /* 实际位置峰峰值，单位0.01rad。 */
  uint32_t actual_backtrack_q;   /* 实际位置累计回退量，单位0.01rad。 */
  uint32_t frame_delta;          /* 最近两次处理的VL53帧序号差。 */
} ActPolicyApp_OscDebug_t;

/**
 * @brief 创建并初始化ACT策略模型。
 * @return ACT_POLICY_APP_RESULT_OK表示初始化成功，其他值表示失败。
 *
 * @note
 * 1. 必须在系统时钟和DCache初始化完成后调用。
 * 2. 该函数只初始化模型，不执行推理，也不控制电机。
 * 3. 重复调用不会重复创建模型实例。
 */
ActPolicyApp_Result_t ActPolicyApp_Init(void);

/**
 * @brief 使用模块内部的固定输入执行一次测试推理。
 * @return ACT_POLICY_APP_RESULT_OK表示推理完成且输出数值有效。
 *
 * @note
 * 该接口只用于确认模型能够在H7上运行。测试输出不会发送给G4，
 * 也不会改变当前触手控制状态。
 */
ActPolicyApp_Result_t ActPolicyApp_RunSelfTest(void);

/**
 * @brief 开启或关闭ACT自主控制。
 * @param enable 0关闭，非0开启。
 * @param tentacle 开启时必须为ACT_POLICY_APP_TENTACLE，即触手3。
 * @return ACT策略执行结果。
 * @note 开启前严格检查VL53、CAN、RESTORE和FAULT；停止后不会自动恢复。
 */
ActPolicyApp_Result_t ActPolicyApp_SetRun(uint8_t enable,
                                         uint8_t tentacle);

/**
 * @brief 执行ACT主循环任务。
 * @note 仅在获得新VL53帧时执行一次推理，禁止在中断中调用。
 */
void ActPolicyApp_Task(void);

/**
 * @brief 查询ACT自主控制状态。
 */
ActPolicyApp_RunState_t ActPolicyApp_GetRunState(void);

/**
 * @brief 复制ACT模型当前运行状态。
 * @param status 用于接收状态的结构体地址。
 *
 * @note
 * status为NULL时函数直接返回。只有run_count大于0时，
 * inference_time_ms和first_action_q才表示有效的推理结果。
 */
void ActPolicyApp_GetStatus(ActPolicyApp_Status_t *status);

/**
 * @brief 复制当前振荡检测调试摘要。
 * @param debug 用于接收调试数据的结构体地址；NULL时直接返回。
 */
void ActPolicyApp_GetOscDebug(ActPolicyApp_OscDebug_t *debug);

#ifdef __cplusplus
}
#endif

#endif /* ACT_POLICY_APP_H */
