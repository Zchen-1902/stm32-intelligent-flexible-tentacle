#include "motor_core.h"
#include "Config.h"
#include "../adc_app/adc_app.h"
#include "../spi_encoder/spi_encoder.h"
#include "../drv8323S_drive/drv_drive.h"
#include "foc_drive.h"
#include "../timer_app/timer_app.h"
#include <string.h>

/*
 * motor_core 电机核心层实现
 *
 * 当前文件先作为 motor_t 对象管理和后续控制链路的统一入口。
 * 后续会逐步补充：
 * 1. 三电机对象创建；
 * 2. PID 默认参数装载；
 * 3. 电流/编码器数据接入；
 * 4. 单环、双环和三环控制更新；
 * 5. FOC 输出调用。
 */

static motor_t s_motors[MOTOR_COUNT];                         // 三电机运行对象，由motor_core统一管理

#define MOTOR_CURRENT_ZERO_CAL_SETTLE_MS      5U               // CAL拉高后等待CSA输出稳定的时间
#define MOTOR_CURRENT_ZERO_CAL_TIMEOUT_MS     2000U            // 电流零漂校准等待ADC新帧的最大超时时间
#define MOTOR_CURRENT_ZERO_TRIM_SETTLE_MS     5U               // CAL退出后等待CSA回到真实采样状态的时间
#define MOTOR_CURRENT_ZERO_TRIM_SAMPLES       500U             // CAL退出后零点残差微调采样次数
#define MOTOR_ADC_TRIGGER_DYNAMIC_ENABLE      1U               // 动态ADC触发点使能：0固定中点，1按duty动态计算
#define MOTOR_ADC_TRIGGER_GUARD_PERCENT       5U               // ADC动态触发保护边界，避开PWM边沿、死区和振铃区
#define MOTOR_RECONSTRUCT_PHASE_HYST_DUTY     0.13f            // 重构相切换duty滞回，候选相至少高出当前相该比例才切换
#define MOTOR_RECONSTRUCT_PHASE_MIN_HOLD_CNT  16U               // 重构相切换后最少保持的电流环周期数，8/16kHz约0.5ms
#define MOTOR_RECONSTRUCT_FORCE_ENABLE        0U               // 调试开关：1强制固定重构相，排查重构相切换是否引入id周期抖动
#define MOTOR_RECONSTRUCT_FORCE_PHASE         MOTOR_CURRENT_PHASE_U // 强制重构相：U/V/W分别填MOTOR_CURRENT_PHASE_U/V/W
#define MOTOR_RESTORE_Q_SCALE                 100.0f           // RESTORE协议单位换算：q = rad * 100
#define MOTOR_RESTORE_MAX_ABS_RAD             ((float)ENCODER_MULTI_TURN_MAX_ABS * (2.0f * M_PI_F)) // RESTORE外部位置保护上限
#define MOTOR_POSITION_SPEED_LIMIT_MIN_RAD_S  1.0f             // 位置运动速度下限，防止配置为0后无法运动
#ifndef MOTOR1_MECH_FB_SIGN
#define MOTOR1_MECH_FB_SIGN                   (1.0f)           // 电机1机械反馈方向：正iq时反馈应向正方向变化
#endif
#ifndef MOTOR2_MECH_FB_SIGN
#define MOTOR2_MECH_FB_SIGN                   (-1.0f)           // 电机2机械反馈方向：位置环和速度环共用
#endif
#ifndef MOTOR3_MECH_FB_SIGN
#define MOTOR3_MECH_FB_SIGN                   (1.0f)           // 电机3机械反馈方向：位置环和速度环共用
#endif
#define MOTOR_FOC_OFFSET_CALIB_SETTLE_MS      200U             // FOC零偏校准每个吸附点稳定等待时间
#define MOTOR_FOC_OFFSET_CALIB_SAMPLE_COUNT   32U              // FOC零偏校准每个吸附点编码器平均采样次数
#define MOTOR_FOC_OFFSET_CALIB_SAMPLE_MS      1U               // FOC零偏校准采样间隔，避免主循环过快重复取同一帧
#define MOTOR_FOC_OFFSET_CALIB_INVALID_MAX    5U               // 校准中允许的连续编码器无效帧次数，避免单次SPI抖动直接退出
#define MOTOR_FOC_OFFSET_CALIB_TWO_POINT_ENABLE 0U             // 先禁用两点校准，避免ramp过程污染offset判断
#ifndef MOTOR_FOC_OFFSET_ALIGN_ELEC_RAD
#define MOTOR_FOC_OFFSET_ALIGN_ELEC_RAD       M_PI_F             // d轴吸附时认为的目标电角度；若差180度可改为M_PI_F
#endif
#ifndef MOTOR_FOC_OFFSET_MECH_SIGN
#define MOTOR_FOC_OFFSET_MECH_SIGN            1.0f            // offset公式机械角符号；若方向相反可改为+1.0f
#endif
#ifndef MOTOR1_FOC_ELEC_TRIM_RAD
#define MOTOR1_FOC_ELEC_TRIM_RAD              0.5f * M_PI_F             // 电机1校正后一次性并入offset，当前先不加90度补偿
#endif
#ifndef MOTOR2_FOC_ELEC_TRIM_RAD
#define MOTOR2_FOC_ELEC_TRIM_RAD              0           // 电机2校正后一次性并入offset，默认不补偿
#endif
#ifndef MOTOR3_FOC_ELEC_TRIM_RAD
#define MOTOR3_FOC_ELEC_TRIM_RAD              0.5f * M_PI_F // 电机3校正后一次性并入offset，先试-90度
#endif
#define MOTOR_FOC_OFFSET_CALIB_RAMP_MS        1500U            // d轴吸附磁场旋转一个电周期的时间
#define MOTOR_FOC_OFFSET_CALIB_MECH_TOL_RAD   0.05f            // 相邻零点机械间隔允许误差
#define MOTOR_FOC_OFFSET_CALIB_OFFSET_TOL_RAD 0.12f            // 两个零点offset允许误差，约15度电角度

typedef enum
{
    MOTOR_CURRENT_PHASE_U = 0U,                                // U/A相电流
    MOTOR_CURRENT_PHASE_V,                                     // V/B相电流
    MOTOR_CURRENT_PHASE_W                                      // W/C相电流
} motor_current_phase_t;

typedef struct
{
    uint8_t enable_reconstruct;                                // 1表示按上一周期计划重构一相，0表示暂不重构
    motor_current_phase_t reconstruct_phase;                   // 需要重构的相，通常选择上一周期duty最大的相
    uint16_t adc_trigger_ccr;                                  // 预留：下一周期ADC触发CCR，阶段4动态采样点使用
    uint8_t phase_hold_count;                                  // 重构相最小保持计数，避免边界附近频繁切换
} motor_current_sample_plan_t;

typedef enum
{
    MOTOR_FOC_OFFSET_CALIB_IDLE = 0U,                           // 空闲
    MOTOR_FOC_OFFSET_CALIB_POINT0_SETTLE,                       // 第一个电角度0点吸附稳定
    MOTOR_FOC_OFFSET_CALIB_POINT0_SAMPLE,                       // 第一个电角度0点采样
    MOTOR_FOC_OFFSET_CALIB_RAMP_TO_POINT1,                      // d轴吸附磁场旋转一个电周期到相邻零点
    MOTOR_FOC_OFFSET_CALIB_POINT1_SETTLE,                       // 第二个相邻电角度0点吸附稳定
    MOTOR_FOC_OFFSET_CALIB_POINT1_SAMPLE                        // 第二个相邻电角度0点采样
} motor_foc_offset_calib_state_t;

typedef struct
{
    motor_foc_offset_calib_state_t state;                       // 当前自动校准状态
    uint8_t motor_id;                                           // 当前校准电机编号
    uint8_t valid;                                              // 最近一次校准结果是否有效
    uint32_t state_tick;                                        // 当前状态进入时间
    uint32_t sample_tick;                                       // 上一次采样时间
    uint16_t sample_count;                                      // 当前点已累计采样次数
    uint8_t invalid_count;                                      // 连续编码器无效帧计数
    float vd_norm;                                              // 本次校准使用的d轴吸附电压
    float sample_sum;                                           // 当前点机械角累计和
    float mech_0;                                               // 第一个电角度0点平均机械角
    float mech_1;                                               // 第二个相邻电角度0点平均机械角
    float offset_0;                                             // 第一个点计算得到的offset
    float offset_1;                                             // 第二个点计算得到的offset
    float offset_final;                                         // 最终平均后的offset
} motor_foc_offset_calib_t;

static motor_current_sample_plan_t s_current_sample_plan[MOTOR_COUNT]; // 三电机电流采样计划，当前先用于TIM20/ADC1快环
static uint8_t s_position_lock_active[MOTOR_COUNT];                    // 位置锁定区状态，带滞回，避免到位附近反复切换
static uint8_t s_position_approach_active[MOTOR_COUNT];                // 位置远距离巡航状态，带滞回，避免接近边界来回切换
static float s_position_speed_limit_rad_s[MOTOR_COUNT];                // 位置环速度上限，单位rad/s，由H7映射控制下发
static uint8_t s_position_restore_valid[MOTOR_COUNT];                  // 1表示已建立H7位置映射
static float s_position_restore_saved_rad[MOTOR_COUNT];                // H7保存的位置，单位rad
static float s_position_restore_base_raw_rad[MOTOR_COUNT];             // 建立映射时G4当前真实位置，单位rad
static uint8_t s_position_stall_valid[MOTOR_COUNT];                    // 堵转检测窗口是否有效
static float s_position_stall_start_pos[MOTOR_COUNT];                  // 堵转检测窗口起点位置，单位rad
static uint16_t s_position_stall_count[MOTOR_COUNT];                   // 堵转检测连续计数，按位置环周期累计
static motor_core_fast_loop_profile_t s_fast_loop_profile[MOTOR_COUNT]; // 三电机快环DWT耗时统计，用于判断三路同时运行余量
static motor_foc_offset_calib_t s_foc_offset_calib;                         // FOC电角度零偏自动校准运行状态，Motor_Core_Init中显式清零

static const foc_pwm_output_t s_motor_pwm_output[MOTOR_COUNT] =
{
    {&htim1,  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},    // 电机1：TIM1 CH1/CH2/CH3
    {&htim8,  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},    // 电机2：TIM8 CH1/CH2/CH3
    {&htim20, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_4},    // 电机3：TIM20 CH1/CH2/CH4
};

static const float s_motor_mech_fb_sign[MOTOR_COUNT] =
{
    MOTOR1_MECH_FB_SIGN,
    MOTOR2_MECH_FB_SIGN,
    MOTOR3_MECH_FB_SIGN,
};

static void Motor_Core_InitOne(motor_t *motor, uint8_t motor_id);           // 初始化单个motor_t基础字段
static void Motor_Core_LoadDefaultPid(motor_t *motor);                      // 从Config.h装载四个PID默认参数
static void Motor_Core_LoadDefaultCalib(motor_t *motor, uint8_t motor_id);  // 从Config.h装载单电机默认校准参数
static uint8_t Motor_Core_UpdateCurrentMeasure(motor_t *motor);             // 更新三相电流、Clarke和Park结果，返回是否获得新采样
static uint8_t Motor_Core_UpdateEncoderAngle(motor_t *motor);               // 更新机械角、速度和电角度，返回编码器数据是否有效
static uint8_t Motor_Core_UpdateEncoderAngleFast(motor_t *motor);           // 快环轻量更新电角度，只更新电流环必需字段
static uint8_t Motor_Core_UpdateCurrentMeasureFast(motor_t *motor,
                                                   float sin_theta,
                                                   float cos_theta);         // 快环轻量更新三相电流和id/iq
static void Motor_Core_ReconstructPhaseCurrent(const motor_current_sample_plan_t *plan,
                                               float *ia,
                                               float *ib,
                                               float *ic);                    // 根据上一周期采样计划重构低侧窗口不足的一相
static void Motor_Core_UpdateCurrentSamplePlan(motor_t *motor);              // 根据本周期FOC duty生成下一周期采样计划
static void Motor_Core_WriteAdcTriggerCcr(const motor_t *motor,
                                          const motor_current_sample_plan_t *plan); // 写入ADC触发CCR，第一版仅支持TIM20 CCR3
#if (MOTOR_ADC_TRIGGER_DYNAMIC_ENABLE != 0U)
static uint16_t Motor_Core_CalcAdcTriggerCcrFromDuty(const motor_t *motor,
                                                     motor_current_phase_t reconstruct_phase); // 根据duty计算下一周期ADC触发CCR
#endif
static void Motor_Core_UpdatePositionPid(motor_t *motor, float dt);          // 执行位置环PID，输出speed_ref_rad_s
static float Motor_Core_GetPositionSpeedLimit(uint8_t motor_id);             // 获取位置环速度上限，单位rad/s
static void Motor_Core_UpdatePositionRefRamp(motor_t *motor,
                                             float cruise_speed_rad_s,
                                             float dt);                       // 按巡航速度推进位置参考
static uint8_t Motor_Core_CheckPositionStall(motor_t *motor,
                                             float pos_feedback,
                                             float pos_error_abs,
                                             uint8_t lock_active);            // 位置环堵转检测与软保护
static void Motor_Core_UpdateSpeedPid(motor_t *motor, float dt);             // 执行速度环PID，输出iq_ref
static void Motor_Core_UpdateCurrentPid(motor_t *motor, float dt);           // 执行id/iq电流环PID
static void Motor_Core_UpdateCurrentPidSeparated(control_pid_t *pid,
                                                 float ref,
                                                 float feedback,
                                                 float dt,
                                                 float *output);              // 电流环积分分离PID更新，误差大时清积分
static void Motor_Core_RunFocMath(motor_t *motor);                          // 执行FOC电压矢量计算，不直接启动PWM
static uint32_t Motor_Core_DutyToTimCompareFast(float duty, uint32_t arr);   // duty快速转换为TIM比较值
static void Motor_Core_WritePwmCcrFast(motor_t *motor);                      // 快环PWM CCR快速写入
static uint8_t Motor_Core_RecordFastLoopProfile(uint8_t motor_id,
                                                uint32_t start_cycles,
                                                uint8_t result);             // 记录一次快环DWT耗时并返回原result
static void Motor_Core_ResetControlRuntime(motor_t *motor);                  // 模式切换时清除控制器运行状态，不清外部目标值
static adc_app_motor_adc_t Motor_Core_GetCurrentAdcId(uint8_t motor_id);     // 根据电机ID选择对应电流ADC组
static encoder_id_t Motor_Core_GetEncoderId(uint8_t motor_id);              // 根据电机ID选择对应编码器ID
static uint8_t Motor_Core_IsEncoderReady(uint8_t motor_id);                  // 判断指定电机编码器数据是否有效
static uint8_t Motor_Core_IsDutySafe(const motor_t *motor);                  // 判断当前FOC duty是否处于可启动范围
static float Motor_Core_CalcFocElecAngle(const motor_t *motor,
                                           float mech_angle_total_rad);         // 根据机械角和FOC电角度零偏计算电角度
static float Motor_Core_GetSignedRawPositionRad(uint8_t motor_id);             // 获取带机械反馈方向的G4原始多圈位置
static float Motor_Core_ExternalToRawTargetRad(uint8_t motor_id,
                                               float external_target_rad);      // 将H7目标位置转换为G4原始位置环目标
static float Motor_Core_CalcFocOffsetFromMech(float mech_angle_total_rad);     // 根据吸附角定义和机械角计算FOC电角度offset
static uint8_t Motor_Core_OutputFocAlignVoltageAtAngle(uint8_t motor_id,
                                                       float vd_norm,
                                                       float elec_angle_rad);  // 输出指定电角度的d轴吸附电压
static void Motor_Core_FocOffsetCalibResetSample(void);                       // 清空自动校准采样累计
static uint8_t Motor_Core_FocOffsetCalibAccumulate(float *avg_mech_angle);    // 累计编码器机械角采样，满样本后输出平均值
#if (MOTOR_FOC_OFFSET_CALIB_TWO_POINT_ENABLE != 0U)
static float Motor_Core_AngleDiffSigned(float target_rad,
                                        float source_rad);                    // 计算target-source的最短有符号角度差
#endif
static void Motor_Core_FocOffsetCalibFinish(uint8_t success);                 // 结束自动校准并输出中性PWM
static float Motor_Core_GetPhaseZeroOffsetV(const motor_t *motor,
                                            adc_app_phase_t phase);             // 根据ADC采样相返回对应零漂电压
static float Motor_Core_RawToCurrentA(uint16_t raw, float zero_offset_v);    // ADC raw转换为相电流A
static void Motor_Core_InitPidParam(control_pid_param_t *param,
                                      float kp,
                                      float ki,
                                    float kd,
                                    float integral_limit,
                                    float output_limit,
                                    float deadband);                        // 生成对称限幅的PID参数

/**
 * @brief  初始化motor_core管理的所有电机对象。
 * @param  None
 * @retval None
 * @note   使用方式：在main初始化阶段调用一次。
 *         当前只做软件对象初始化，不启动PWM、不启动ADC、不控制DRV。
 */
void Motor_Core_Init(void)
{
    uint8_t i;

    memset(s_motors, 0, sizeof(s_motors));
    memset(s_current_sample_plan, 0, sizeof(s_current_sample_plan));
    memset(s_position_lock_active, 0, sizeof(s_position_lock_active));
    memset(s_position_approach_active, 0, sizeof(s_position_approach_active));
    memset(s_position_stall_valid, 0, sizeof(s_position_stall_valid));
    memset(s_position_stall_start_pos, 0, sizeof(s_position_stall_start_pos));
    memset(s_position_stall_count, 0, sizeof(s_position_stall_count));
    memset(s_position_restore_valid, 0, sizeof(s_position_restore_valid));
    memset(s_position_restore_saved_rad, 0, sizeof(s_position_restore_saved_rad));
    memset(s_position_restore_base_raw_rad, 0, sizeof(s_position_restore_base_raw_rad));
    memset(s_fast_loop_profile, 0, sizeof(s_fast_loop_profile));
    memset(&s_foc_offset_calib, 0, sizeof(s_foc_offset_calib));
    s_foc_offset_calib.state = MOTOR_FOC_OFFSET_CALIB_IDLE;

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        s_position_speed_limit_rad_s[i] = POS_APPROACH_SPEED_RAD_S;
        Motor_Core_InitOne(&s_motors[i], i);
    }
}

/**
 * @brief  根据电机编号获取motor_t对象。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @retval motor_t指针；编号非法时返回0。
 * @note   外部模块后续通过该接口访问电机对象，避免直接暴露s_motors数组。
 */
motor_t *Motor_Core_GetMotor(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return 0;
    }

    return &s_motors[motor_id];
}

/**
 * @brief  获取当前工程配置的电机数量。
 * @param  None
 * @retval 电机数量。
 */
uint8_t Motor_Core_GetMotorCount(void)
{
    return (uint8_t)MOTOR_COUNT;
}

/**
 * @brief  获取指定电机初始化时绑定的编码器ID。
 * @param  motor_id: 电机编号，范围0~MOTOR_COUNT-1。
 * @retval 编码器ID；motor_id非法时返回encoder_id_1。
 * @note   该接口用于外部调度层把TIM7外环和对应编码器缓存绑定起来。
 */
uint8_t Motor_Core_GetBoundEncoderId(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return (uint8_t)encoder_id_1;
    }

    return s_motors[motor_id].encoder_id;
}

/**
 * @brief  获取指定电机快环DWT耗时统计。
 * @param  motor_id: 电机编号，范围0~MOTOR_COUNT-1。
 * @param  profile: 输出统计结构体指针。
 * @retval None
 * @note   该接口只拷贝统计值，不清零；用于VOFA低频观察，不应在快环里调用。
 */
void Motor_Core_GetFastLoopProfile(uint8_t motor_id, motor_core_fast_loop_profile_t *profile)
{
    if ((profile == 0) || (motor_id >= MOTOR_COUNT))
    {
        return;
    }

    *profile = s_fast_loop_profile[motor_id];
}

/**
 * @brief  更新单个电机的一周期控制流程。
 * @param  motor: 电机对象指针。
 * @param  dt: 控制周期，单位s。
 * @retval None
 * @note   当前只建立控制模式分发框架，暂不执行ADC、编码器、PID和FOC。
 *         后续会根据motor->mode逐步接入电流环、速度环和位置环。
 */
void Motor_Core_UpdateMotor(motor_t *motor, float dt)
{
    if ((motor == 0) || (dt <= 0.0f))
    {
        return;
    }

    switch (motor->mode)
    {
    case MOTOR_MODE_IDLE:
        break;

    case MOTOR_MODE_CURRENT:
        (void)Motor_Core_UpdateCurrentLoop(motor, dt);
        break;

    case MOTOR_MODE_SPEED:
        (void)Motor_Core_UpdateSpeedLoop(motor, dt);
        break;

    case MOTOR_MODE_POSITION:
        (void)Motor_Core_UpdatePositionLoop(motor, dt);
        break;

    case MOTOR_MODE_SPEED_CURRENT:
        (void)Motor_Core_UpdateSpeedCurrentLoop(motor, dt);
        break;

    case MOTOR_MODE_POSITION_SPEED:
        (void)Motor_Core_UpdatePositionSpeedLoop(motor, dt);
        break;

    case MOTOR_MODE_POSITION_SPEED_CURRENT:
        (void)Motor_Core_UpdatePositionSpeedCurrentLoop(motor, dt);
        break;

    default:
        motor->mode = MOTOR_MODE_IDLE;
        break;
    }
}

/**
 * @brief  更新全部电机的一周期控制流程。
 * @param  dt: 控制周期，单位s。
 * @retval None
 * @note   外部调度层可调用本函数统一更新三台电机。
 */
void Motor_Core_UpdateAll(float dt)
{
    uint8_t i;

    if (dt <= 0.0f)
    {
        return;
    }

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        Motor_Core_UpdateMotor(&s_motors[i], dt);
    }
}

/**
 * @brief  PWM同步请求到达后尝试更新指定电机高频控制。
 * @param  motor_id：电机编号，范围0~MOTOR_COUNT-1。
 * @param  dt：距离该电机上一次实际控制执行的时间，单位s。
 * @return uint8_t
 *         0：未执行控制，例如ID无效、模式不需要电流环、无新ADC帧
 *         1：已执行一次控制更新
 * @note   第一版只允许带电流环/FOC输出的模式在PWM同步请求下执行。
 *         ADC新数据判断仍复用Motor_Core_UpdateCurrentMeasure()内部update_count机制。
 */
uint8_t Motor_Core_TryUpdateFromPwmRequest(uint8_t motor_id, float dt)
{
    motor_t *motor;
    uint8_t updated = 0U;

    if ((motor_id >= MOTOR_COUNT) || (dt <= 0.0f))
    {
        return 0U;
    }

    motor = &s_motors[motor_id];

    switch (motor->mode)
    {
    case MOTOR_MODE_CURRENT:
        updated = Motor_Core_UpdateCurrentLoop(motor, dt);
        break;

    case MOTOR_MODE_SPEED_CURRENT:
        updated = Motor_Core_UpdateSpeedCurrentLoop(motor, dt);
        break;

    case MOTOR_MODE_POSITION_SPEED_CURRENT:
        updated = Motor_Core_UpdatePositionSpeedCurrentLoop(motor, dt);
        break;

    default:
        return 0U;
    }

    if (updated == 0U)
    {
        return 0U;
    }

    if (FOC_WritePwmDuty(&s_motor_pwm_output[motor_id],
                         motor->control.duty_u,
                         motor->control.duty_v,
                         motor->control.duty_w) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  ADC完成后执行已启动电机的轻量电流快环。
 * @param  motor_id: 电机编号，调用方保证为0/1/2。
 * @param  dt: 电流环周期，单位s，应与实际ADC触发周期一致。
 * @retval 1表示本次完成电流环和FOC更新，0表示条件不满足。
 * @note   使用方式：放在对应ADC DMA完成回调中调用，且应先调用ADC_App_ConvCpltCallback()更新ADC缓存。
 *         高频映射在Motor_Core_InitOne()中绑定到motor_t，快环不再调用ID映射函数。
 *         本函数不检查mode、不启动PWM、不做模式切换，由调用方保证该电机已进入闭环运行状态。
 */
uint8_t Motor_Core_RunCurrentFastLoopDirectFromAdc(uint8_t motor_id, float dt)
{
    motor_t *motor;
    float sin_theta;
    float cos_theta;
    const uint32_t profile_start_cycles = Timer_App_GetRuntimeCycles();

    if (dt <= 0.0f)
    {
        return Motor_Core_RecordFastLoopProfile(motor_id, profile_start_cycles, 0U);
    }

    motor = &s_motors[motor_id];
    if (Motor_Core_UpdateEncoderAngleFast(motor) == 0U)
    {
        return Motor_Core_RecordFastLoopProfile(motor_id, profile_start_cycles, 0U);
    }

    FOC_CalcSinCos(motor->encoder.elec_angle, &sin_theta, &cos_theta);

    if (Motor_Core_UpdateCurrentMeasureFast(motor, sin_theta, cos_theta) == 0U)
    {
        return Motor_Core_RecordFastLoopProfile(motor_id, profile_start_cycles, 0U);
    }

    Motor_Core_UpdateCurrentPid(motor, dt);
    (void)FOC_RunVoltageNormSinCos(motor,
                                   motor->control.vd_norm,
                                   motor->control.vq_norm,
                                   motor->encoder.elec_angle,
                                   sin_theta,
                                   cos_theta,
                                   0);

    Motor_Core_WritePwmCcrFast(motor);
    Motor_Core_UpdateCurrentSamplePlan(motor);

    return Motor_Core_RecordFastLoopProfile(motor_id, profile_start_cycles, 1U);
}

/**
 * @brief  ADC完成后执行指定电机的安全轻量电流快环。
 * @param  motor_id: 电机编号，调用方保证为0/1/2。
 * @param  dt: 电流环周期，单位s，应与实际ADC触发周期一致。
 * @retval 1表示本次完成电流环和FOC更新，0表示条件不满足。
 * @note   保留给通用路径使用；本函数会检查当前模式是否允许电流内环运行。
 */
uint8_t Motor_Core_RunCurrentFastLoopFromAdc(uint8_t motor_id, float dt)
{
    motor_t *motor;

    motor = &s_motors[motor_id];
    if ((motor->mode != MOTOR_MODE_CURRENT) &&
        (motor->mode != MOTOR_MODE_SPEED_CURRENT) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED_CURRENT))
    {
        return Motor_Core_RecordFastLoopProfile(motor_id, Timer_App_GetRuntimeCycles(), 0U);
    }

    return Motor_Core_RunCurrentFastLoopDirectFromAdc(motor_id, dt);
}

/**
 * @brief  兼容旧调试入口：ADC1完成后执行motor2轻量电流快环。
 * @param  dt: 电流环周期，单位s。
 * @retval 1表示本次完成电流环和FOC更新，0表示条件不满足。
 */
uint8_t Motor_Core_RunMotor2CurrentFastLoopFromAdc(float dt)
{
    return Motor_Core_RunCurrentFastLoopFromAdc(2U, dt);
}

/**
 * @brief  TIM7速度外环入口。
 * @param  motor_id: 电机编号，范围0~MOTOR_COUNT-1。
 * @param  dt: 速度环周期，单位s，应与TIM7分频后的速度环周期一致。
 * @retval 1表示速度外环已更新iq_ref，0表示当前模式或参数不满足。
 * @note   本函数只执行速度PID，把speed_ref转换为iq_ref。
 *         不读取ADC、不执行电流PID、不执行FOC、不写PWM CCR，避免TIM7中断负担过重。
 */
uint8_t Motor_Core_RunSpeedOuterLoopFromTim7(uint8_t motor_id, float dt)
{
    motor_t *motor;
    encoder_id_t encoder_id;

    if ((motor_id >= MOTOR_COUNT) || (dt <= 0.0f))
    {
        return 0U;
    }

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0U;
    }

    if ((motor->mode != MOTOR_MODE_SPEED) &&
        (motor->mode != MOTOR_MODE_SPEED_CURRENT) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED_CURRENT))
    {
        return 0U;
    }

    encoder_id = Motor_Core_GetEncoderId(motor_id);
    if (SPI_Encoder_Is_Valid(encoder_id) == 0U)
    {
        motor->encoder.valid = 0U;
        return 0U;
    }

    motor->encoder.speed_rad_s = SPI_Encoder_Get_Speed(encoder_id);
    motor->encoder.speed_rpm = RAD_S_TO_RPM(motor->encoder.speed_rad_s);
    motor->encoder.valid = 1U;

    Motor_Core_UpdateSpeedPid(motor, dt);

    return 1U;
}

/**
 * @brief  TIM7位置外环入口。
 * @param  motor_id: 电机编号，范围0~MOTOR_COUNT-1。
 * @param  dt: 位置环周期，单位s，应与TIM7位置环分频后的周期一致。
 * @retval 1表示位置外环已更新speed_ref，0表示当前模式或参数不满足。
 * @note   本函数只执行位置PID，把mech_angle_ref转换为speed_ref_rad_s。
 *         不执行速度PID、不执行电流PID、不执行FOC、不写PWM CCR。
 */
uint8_t Motor_Core_RunPositionOuterLoopFromTim7(uint8_t motor_id, float dt)
{
    motor_t *motor;
    encoder_id_t encoder_id;
    float mech_angle_total;

    if ((motor_id >= MOTOR_COUNT) || (dt <= 0.0f))
    {
        return 0U;
    }

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0U;
    }

    if ((motor->mode != MOTOR_MODE_POSITION) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED_CURRENT))
    {
        return 0U;
    }

    encoder_id = Motor_Core_GetEncoderId(motor_id);
    if (SPI_Encoder_Is_Valid(encoder_id) == 0U)
    {
        motor->encoder.valid = 0U;
        return 0U;
    }

    mech_angle_total = SPI_Encoder_Get_Total_Angle(encoder_id);

    motor->encoder.mech_angle_last = motor->encoder.mech_angle;
    motor->encoder.elec_angle_last = motor->encoder.elec_angle;
    motor->encoder.turn_count = SPI_Encoder_Get_Turn_Count(encoder_id);
    motor->encoder.mech_angle_total_rad = mech_angle_total;
    motor->encoder.mech_angle = FOC_WrapAngle0To2Pi(mech_angle_total);
    motor->encoder.elec_angle = Motor_Core_CalcFocElecAngle(motor, mech_angle_total);
    motor->encoder.valid = 1U;

    Motor_Core_UpdatePositionPid(motor, dt);

    return 1U;
}


/**
 * @brief  应用单电机控制命令。
 * @param  cmd: 单电机控制命令指针，不能为NULL。
 * @retval None
 * @note   使用方式：上位机、CAN、按键调试或主控逻辑生成 motor_core_command_t 后调用。
 *         本函数按“目标值先写入、模式最后切换”的顺序执行，避免模式切换后短暂使用旧目标。
 *         本函数不启动PWM、不使能DRV、不做状态机安全互锁。
 */
void Motor_Core_ApplyCommand(const motor_core_command_t *cmd)
{
    float id_ref;
    float iq_ref;
    float speed_ref_rad_s;

    if ((cmd == 0) || (cmd->motor_id >= MOTOR_COUNT))
    {
        return;
    }

    id_ref = LIMIT(cmd->id_ref, -MOTOR_CURRENT_MAX_A, MOTOR_CURRENT_MAX_A);
    iq_ref = LIMIT(cmd->iq_ref, -MOTOR_CURRENT_MAX_A, MOTOR_CURRENT_MAX_A);
    speed_ref_rad_s = LIMIT(cmd->speed_ref_rad_s,
                            -MOTOR_SPEED_REF_MAX_RAD_S,
                            MOTOR_SPEED_REF_MAX_RAD_S);

    Motor_Core_SetCurrentRef(cmd->motor_id, id_ref, iq_ref);
    Motor_Core_SetSpeedRef(cmd->motor_id, speed_ref_rad_s);
    Motor_Core_SetPositionRef(cmd->motor_id, cmd->position_ref_rad);
    Motor_Core_SetMode(cmd->motor_id, cmd->mode);
}

/**
 * @brief  设置指定电机控制模式。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  mode: 目标控制模式。
 * @retval None
 * @note   当前只负责写入mode，不做状态机互锁；后续接入状态机后再增加模式切换条件检查。
 */
void Motor_Core_SetMode(uint8_t motor_id, motor_mode_t mode)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return;
    }

    if (motor->mode == mode)
    {
        return;
    }

    Motor_Core_ResetControlRuntime(motor);
    motor->mode = mode;
}

/**
 * @brief  清除指定电机运行态保护和控制缓存。
 * @param  motor_id 电机编号，范围0~MOTOR_COUNT-1。
 * @retval None
 * @note   用于CANSTOP等人工恢复入口；只清PID、锁定区、堵转窗口和stall_fault，
 *         不修改位置目标、编码器零点和H7 RESTORE坐标映射。
 */
void Motor_Core_ClearRuntimeFault(uint8_t motor_id)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return;
    }

    Motor_Core_ResetControlRuntime(motor);
}

/**
 * @brief  设置指定电机d/q轴电流目标。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  id_ref: d轴电流目标，单位A；常规表贴PMSM/BLDC第一版通常设为0。
 * @param  iq_ref: q轴电流目标，单位A；正负号决定期望电磁转矩方向。
 * @retval None
 * @note   使用方式：调试单电流环或速度/位置串级环时调用。
 *         本函数只写入目标值，不切换控制模式、不启动PWM、不做安全互锁。
 */
void Motor_Core_SetCurrentRef(uint8_t motor_id, float id_ref, float iq_ref)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return;
    }

    motor->ref.id_ref = id_ref;
    motor->ref.iq_ref = iq_ref;
    motor->ref.iq_ref_ramp = iq_ref;
}

/**
 * @brief  设置指定电机机械角速度目标。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  speed_ref_rad_s: 机械角速度目标，单位rad/s。
 * @retval None
 * @note   使用方式：调试单速度环、速度电流双环或三环时调用。
 *         本函数同步更新speed_ref_rpm，仅用于观察；控制主单位仍为rad/s。
 */
void Motor_Core_SetSpeedRef(uint8_t motor_id, float speed_ref_rad_s)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return;
    }

    motor->ref.speed_ref_rad_s = speed_ref_rad_s;
    motor->ref.speed_ref_ramp_rad_s = speed_ref_rad_s;
    motor->ref.speed_ref_rpm = RAD_S_TO_RPM(speed_ref_rad_s);
}

/**
 * @brief  设置指定电机的位置运动速度上限。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  speed_limit_rad_s: 巡航速度，单位rad/s；小于等于0时使用默认巡航速度。
 * @retval None
 * @note   只修改已有速度限幅，不增加位置环周期内的计算量。
 */
void Motor_Core_SetPositionSpeedLimit(uint8_t motor_id, float speed_limit_rad_s)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return;
    }

    if (speed_limit_rad_s <= 0.0f)
    {
        speed_limit_rad_s = POS_APPROACH_SPEED_RAD_S;
    }

    speed_limit_rad_s = LIMIT(speed_limit_rad_s,
                              MOTOR_POSITION_SPEED_LIMIT_MIN_RAD_S,
                              POS_PID_OUT_LIM_RAD_S_DEFAULT);

    s_position_speed_limit_rad_s[motor_id] = speed_limit_rad_s;
}

/**
 * @brief  设置三路位置运动速度上限。
 * @param  speed_limit_rad_s: 巡航速度，单位rad/s；小于等于0时使用默认巡航速度。
 * @retval None
 */
void Motor_Core_SetPositionSpeedLimitAll(float speed_limit_rad_s)
{
    uint8_t motor_id;

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        Motor_Core_SetPositionSpeedLimit(motor_id, speed_limit_rad_s);
    }
}

/**
 * @brief  设置指定电机机械角位置目标。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  mech_angle_ref_rad: 机械角位置目标，单位rad；多圈位置时可传连续角度。
 * @retval None
 * @note   使用方式：调试单位置环、位置速度双环或三环时调用。
 *         本函数只设置电机机械角目标，不在这里换算线轮角或绳长。
 */
void Motor_Core_SetPositionRef(uint8_t motor_id, float mech_angle_ref_rad)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return;
    }

    motor->ref.mech_angle_ref = mech_angle_ref_rad;

    /*
     * 非位置闭环状态下收到第一个位置目标时，从当前实际位置启动轨迹；
     * 已在位置闭环中时只更新最终目标，ramp继续平滑追踪，避免CAN新目标导致参考突跳。
     */
    if ((motor->mode != MOTOR_MODE_POSITION) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED_CURRENT))
    {
        motor->ref.mech_angle_ref_ramp = Motor_Core_GetSignedRawPositionRad(motor_id);
    }
}

/**
 * @brief  按巡航速度推进位置参考。
 * @param  motor 电机对象指针。
 * @param  cruise_speed_rad_s 巡航速度，单位rad/s。
 * @param  dt 位置环周期，单位s。
 * @note   mech_angle_ref是最终目标，mech_angle_ref_ramp是位置环实际使用的平滑参考。
 */
static void Motor_Core_UpdatePositionRefRamp(motor_t *motor,
                                             float cruise_speed_rad_s,
                                             float dt)
{
    float delta;
    float step;

    if ((motor == 0) || (dt <= 0.0f))
    {
        return;
    }

    cruise_speed_rad_s = LIMIT(cruise_speed_rad_s,
                               MOTOR_POSITION_SPEED_LIMIT_MIN_RAD_S,
                               POS_PID_OUT_LIM_RAD_S_DEFAULT);

    delta = motor->ref.mech_angle_ref - motor->ref.mech_angle_ref_ramp;
    step = cruise_speed_rad_s * dt;

    if (delta > step)
    {
        motor->ref.mech_angle_ref_ramp += step;
    }
    else if (delta < -step)
    {
        motor->ref.mech_angle_ref_ramp -= step;
    }
    else
    {
        motor->ref.mech_angle_ref_ramp = motor->ref.mech_angle_ref;
    }
}

/**
 * @brief  批量设置三路电机绝对机械角目标。
 * @param  target_abs_rad: 三路电机目标机械角，单位rad，下标0/1/2对应motor0/1/2。
 * @retval None
 * @note   只写目标值和模式，不启动PWM，不做安全互锁。
 */
void Motor_Core_SetPositionAbsoluteAll(const float *target_abs_rad)
{
    uint8_t motor_id;
    float raw_target_rad;
    motor_t *motor;

    if (target_abs_rad == 0)
    {
        return;
    }

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        motor = Motor_Core_GetMotor(motor_id);
        if (motor == 0)
        {
            continue;
        }

        raw_target_rad = Motor_Core_ExternalToRawTargetRad(motor_id, target_abs_rad[motor_id]);

        /*
         * 连续CAN映射控制时，已在位置闭环中的电机只更新最终目标；
         * 不反复清speed_ref_ramp/iq_ref，避免每个新目标都打断巡航速度轨迹。
         */
        if ((motor->mode != MOTOR_MODE_POSITION) &&
            (motor->mode != MOTOR_MODE_POSITION_SPEED) &&
            (motor->mode != MOTOR_MODE_POSITION_SPEED_CURRENT))
        {
            Motor_Core_SetCurrentRef(motor_id, 0.0f, 0.0f);
            Motor_Core_SetSpeedRef(motor_id, 0.0f);
        }

        Motor_Core_SetPositionRef(motor_id, raw_target_rad);
        Motor_Core_SetMode(motor_id, MOTOR_MODE_POSITION_SPEED_CURRENT);
    }
}

/**
 * @brief  批量设置三路电机相对当前位置的位置目标。
 * @param  delta_rad: 三路电机相对当前位置的机械角增量，单位rad，下标0/1/2对应motor0/1/2。
 * @retval None
 * @note   这里先按当前编码器绝对角换算成目标绝对角，再转入绝对位置接口。
 */
void Motor_Core_SetPositionRelativeAll(const float *delta_rad)
{
    float target_abs_rad[MOTOR_COUNT];
    uint8_t motor_id;

    if (delta_rad == 0)
    {
        return;
    }

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        target_abs_rad[motor_id] = Motor_Core_GetPositionFeedbackRad(motor_id) + delta_rad[motor_id];
    }

    Motor_Core_SetPositionAbsoluteAll(target_abs_rad);
}

/**
 * @brief  获取外部命令坐标系下的实际多圈机械角。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @retval 实际多圈位置，单位rad；参数非法时返回0。
 * @note   这里返回的是CAN/H7/VOFA看到的位置坐标，只包含统一机械反馈方向，
 *         因此与G4位置环目标和相对位置命令处于同一符号约定。
 */
float Motor_Core_GetPositionFeedbackRad(uint8_t motor_id)
{
    float raw_rad;

    if (motor_id >= MOTOR_COUNT)
    {
        return 0.0f;
    }

    raw_rad = Motor_Core_GetSignedRawPositionRad(motor_id);
    if (s_position_restore_valid[motor_id] == 0U)
    {
        return raw_rad;
    }

    /*
     * H7看到的位置 = H7保存的位置 + G4从建立映射后真实转过的量。
     * 这里只用于CANSTAT/上层坐标，不修改FOC使用的编码器角。
     */
    return s_position_restore_saved_rad[motor_id] +
           (raw_rad - s_position_restore_base_raw_rad[motor_id]);
}

/**
 * @brief  查询G4是否已经收到H7发来的ACTUAL并建立位置映射。
 * @retval 1表示三路aq可信，0表示至少一路aq还不能使用。
 * @note   不判断ACTUAL数值本身，因为ACTUAL=0也是合法位置。
 */
uint8_t Motor_Core_IsPositionRestoreValid(void)
{
    uint8_t motor_id;

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        if (s_position_restore_valid[motor_id] == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  恢复外部命令坐标系下的三电机实际多圈位置。
 * @param  position_q 三路实际位置，单位0.01rad，坐标系与CANSTAT的aq一致。
 * @retval 1恢复成功，0恢复失败。
 * @note   不改spi_encoder真实多圈角，只建立外部坐标偏移；因此不会改变FOC电角度零点。
 */
uint8_t Motor_Core_RestorePositionFeedbackQ(const int32_t position_q[3])
{
    uint8_t motor_id;
    float saved_rad[MOTOR_COUNT];
    float base_raw_rad[MOTOR_COUNT];

    if (position_q == 0)
    {
        return 0U;
    }

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        encoder_id_t encoder_id = (encoder_id_t)Motor_Core_GetBoundEncoderId(motor_id);
        float target_external_rad = (float)position_q[motor_id] / MOTOR_RESTORE_Q_SCALE;
        float raw_external_rad;

        if (SPI_Encoder_Is_Valid(encoder_id) == 0U)
        {
            return 0U;
        }

        raw_external_rad = Motor_Core_GetSignedRawPositionRad(motor_id);
        if ((target_external_rad != target_external_rad) ||
            (raw_external_rad != raw_external_rad) ||
            (target_external_rad > MOTOR_RESTORE_MAX_ABS_RAD) ||
            (target_external_rad < -MOTOR_RESTORE_MAX_ABS_RAD) ||
            (raw_external_rad > MOTOR_RESTORE_MAX_ABS_RAD) ||
            (raw_external_rad < -MOTOR_RESTORE_MAX_ABS_RAD))
        {
            return 0U;
        }

        saved_rad[motor_id] = target_external_rad;
        base_raw_rad[motor_id] = raw_external_rad;
    }

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        s_position_restore_saved_rad[motor_id] = saved_rad[motor_id];
        s_position_restore_base_raw_rad[motor_id] = base_raw_rad[motor_id];
        s_position_restore_valid[motor_id] = 1U;
    }

    return 1U;
}

/**
 * @brief  清除H7 RESTORE建立的位置映射。
 * @param  motor_id 电机编号，范围0~MOTOR_COUNT-1。
 * @retval None
 * @note   重新设置编码器零点后必须清除，避免旧H7坐标关系继续叠加。
 */
void Motor_Core_ClearPositionRestoreOffset(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return;
    }

    s_position_restore_valid[motor_id] = 0U;
    s_position_restore_saved_rad[motor_id] = 0.0f;
    s_position_restore_base_raw_rad[motor_id] = 0.0f;
}

/**
 * @brief  设置指定电机编码器机械零点偏移。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  offset_rad: 编码器机械零点偏移，单位rad。
 * @retval None
 * @note   使用方式：调试或标定时调用本函数修正编码器零点误差。
 *         本函数会同时更新motor->calib.encoder_zero_offset和spi_encoder内部offset。
 *         motor_core后续直接使用spi_encoder处理后的角度，不再额外扣除该offset。
 */
void Motor_Core_SetEncoderZeroOffset(uint8_t motor_id, float offset_rad)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return;
    }

    motor->calib.encoder_zero_offset = offset_rad;
    SPI_Encoder_Set_Offset(Motor_Core_GetEncoderId(motor_id), offset_rad);
    Motor_Core_ClearPositionRestoreOffset(motor_id);
}

/**
 * @brief  应用Config.h中的编码器零点配置。
 * @param  None
 * @retval None
 * @note   使用方式：SPI_Encoder_Init()之后调用一次。
 *         Motor_Core_Init()已经把ENCODER_ZERO_OFFSET_Mx_RAD加载到motor->calib.encoder_zero_offset，
 *         本函数负责把这些配置同步给spi_encoder，避免motor_core中重复扣除零点。
 */
void Motor_Core_ApplyEncoderOffsetConfig(void)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        Motor_Core_SetEncoderZeroOffset(i, s_motors[i].calib.encoder_zero_offset);
    }
}

/**
 * @brief  执行三路电机电流零漂校准。
 * @param  None
 * @return HAL_OK表示校准成功；HAL_TIMEOUT表示等待ADC新帧超时；HAL_ERROR表示配置参数异常。
 * @note   使用方式：应在ADC DMA启动、TIM1/TIM8/TIM20采样触发已经运行、PWM输出未开启时调用。
 *         本函数会拉高三路DRV CAL，使CSA输出零电流参考，再采集ADC平均值写入motor->calib零漂。
 *         本函数是阻塞式校准函数，只适合上电初始化或手动调试阶段，不应在中断中调用。
 */
HAL_StatusTypeDef Motor_Core_CalibrateCurrentZero(void)
{
    uint32_t sum_raw[MOTOR_COUNT][ADC_APP_PHASE_NUM] = {{0U}};
    uint32_t last_count[MOTOR_COUNT] = {0U};
    uint32_t start_tick;
    uint16_t sample_idx;
    uint8_t motor_id;
    uint8_t all_ready;

    if ((MOTOR_COUNT != 3U) || (CURRENT_ZERO_CALIB_SAMPLES == 0U))
    {
        return HAL_ERROR;
    }

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        adc_app_motor_adc_t adc_id = Motor_Core_GetCurrentAdcId(motor_id);

        last_count[motor_id] = ADC_App_GetMotorUpdateCount(adc_id);
    }

    DRV_Drive_CsaCalibBeginAll();
    HAL_Delay(MOTOR_CURRENT_ZERO_CAL_SETTLE_MS);

    start_tick = HAL_GetTick();

    for (sample_idx = 0U; sample_idx < CURRENT_ZERO_CALIB_SAMPLES; sample_idx++)
    {
        do
        {
            all_ready = 1U;

            for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
            {
                adc_app_motor_adc_t adc_id = Motor_Core_GetCurrentAdcId(motor_id);

                if (ADC_App_GetMotorUpdateCount(adc_id) == last_count[motor_id])
                {
                    all_ready = 0U;
                    break;
                }
            }

            if ((HAL_GetTick() - start_tick) > MOTOR_CURRENT_ZERO_CAL_TIMEOUT_MS)
            {
                DRV_Drive_CsaCalibEndAll();
                return HAL_TIMEOUT;
            }
        } while (all_ready == 0U);

        for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
        {
            adc_app_motor_adc_t adc_id = Motor_Core_GetCurrentAdcId(motor_id);

            last_count[motor_id] = ADC_App_GetMotorUpdateCount(adc_id);
            sum_raw[motor_id][ADC_APP_PHASE_A] += ADC_App_GetMotorRaw(adc_id, ADC_APP_PHASE_A);
            sum_raw[motor_id][ADC_APP_PHASE_B] += ADC_App_GetMotorRaw(adc_id, ADC_APP_PHASE_B);
            sum_raw[motor_id][ADC_APP_PHASE_C] += ADC_App_GetMotorRaw(adc_id, ADC_APP_PHASE_C);
        }
    }

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        motor_t *motor = &s_motors[motor_id];
        float avg_a = (float)sum_raw[motor_id][ADC_APP_PHASE_A] / (float)CURRENT_ZERO_CALIB_SAMPLES;
        float avg_b = (float)sum_raw[motor_id][ADC_APP_PHASE_B] / (float)CURRENT_ZERO_CALIB_SAMPLES;
        float avg_c = (float)sum_raw[motor_id][ADC_APP_PHASE_C] / (float)CURRENT_ZERO_CALIB_SAMPLES;

        motor->calib.ia_zero_offset_v = ((avg_a * ADC_VREF_V) / ADC_MAX_COUNT) - CURRENT_ADC_MID_V;
        motor->calib.ib_zero_offset_v = ((avg_b * ADC_VREF_V) / ADC_MAX_COUNT) - CURRENT_ADC_MID_V;
        motor->calib.ic_zero_offset_v = ((avg_c * ADC_VREF_V) / ADC_MAX_COUNT) - CURRENT_ADC_MID_V;
    }

    DRV_Drive_CsaCalibEndAll();

    return HAL_OK;
}

/**
 * @brief  CAL退出后按真实ADC采样链路微调三相电流零点残差。
 * @param  None
 * @return HAL_OK表示微调成功；HAL_TIMEOUT表示等待ADC新帧超时；HAL_ERROR表示配置参数异常。
 * @note   使用方式：应在Motor_Core_CalibrateCurrentZero()之后、PWM未输出转矩时调用。
 *         本函数不拉高DRV CAL，而是直接采集真实运行状态下的零电流ADC平均值，
 *         用于修正CAL状态和真实采样状态之间的几count残余偏差。
 */
HAL_StatusTypeDef Motor_Core_TrimCurrentZeroResidual(void)
{
    uint32_t sum_raw[MOTOR_COUNT][ADC_APP_PHASE_NUM] = {{0U}};
    uint32_t last_count[MOTOR_COUNT] = {0U};
    uint32_t start_tick;
    uint16_t sample_idx;
    uint8_t motor_id;
    uint8_t all_ready;

    if ((MOTOR_COUNT != 3U) || (MOTOR_CURRENT_ZERO_TRIM_SAMPLES == 0U))
    {
        return HAL_ERROR;
    }

    HAL_Delay(MOTOR_CURRENT_ZERO_TRIM_SETTLE_MS);

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        adc_app_motor_adc_t adc_id = Motor_Core_GetCurrentAdcId(motor_id);

        last_count[motor_id] = ADC_App_GetMotorUpdateCount(adc_id);
    }

    start_tick = HAL_GetTick();

    for (sample_idx = 0U; sample_idx < MOTOR_CURRENT_ZERO_TRIM_SAMPLES; sample_idx++)
    {
        do
        {
            all_ready = 1U;

            for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
            {
                adc_app_motor_adc_t adc_id = Motor_Core_GetCurrentAdcId(motor_id);

                if (ADC_App_GetMotorUpdateCount(adc_id) == last_count[motor_id])
                {
                    all_ready = 0U;
                    break;
                }
            }

            if ((HAL_GetTick() - start_tick) > MOTOR_CURRENT_ZERO_CAL_TIMEOUT_MS)
            {
                return HAL_TIMEOUT;
            }
        } while (all_ready == 0U);

        for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
        {
            adc_app_motor_adc_t adc_id = Motor_Core_GetCurrentAdcId(motor_id);

            last_count[motor_id] = ADC_App_GetMotorUpdateCount(adc_id);
            sum_raw[motor_id][ADC_APP_PHASE_A] += ADC_App_GetMotorRaw(adc_id, ADC_APP_PHASE_A);
            sum_raw[motor_id][ADC_APP_PHASE_B] += ADC_App_GetMotorRaw(adc_id, ADC_APP_PHASE_B);
            sum_raw[motor_id][ADC_APP_PHASE_C] += ADC_App_GetMotorRaw(adc_id, ADC_APP_PHASE_C);
        }
    }

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
        motor_t *motor = &s_motors[motor_id];
        float avg_a = (float)sum_raw[motor_id][ADC_APP_PHASE_A] / (float)MOTOR_CURRENT_ZERO_TRIM_SAMPLES;
        float avg_b = (float)sum_raw[motor_id][ADC_APP_PHASE_B] / (float)MOTOR_CURRENT_ZERO_TRIM_SAMPLES;
        float avg_c = (float)sum_raw[motor_id][ADC_APP_PHASE_C] / (float)MOTOR_CURRENT_ZERO_TRIM_SAMPLES;

        motor->calib.ia_zero_offset_v = ((avg_a * ADC_VREF_V) / ADC_MAX_COUNT) - CURRENT_ADC_MID_V;
        motor->calib.ib_zero_offset_v = ((avg_b * ADC_VREF_V) / ADC_MAX_COUNT) - CURRENT_ADC_MID_V;
        motor->calib.ic_zero_offset_v = ((avg_c * ADC_VREF_V) / ADC_MAX_COUNT) - CURRENT_ADC_MID_V;

        motor->current.ia = 0.0f;
        motor->current.ib = 0.0f;
        motor->current.ic = 0.0f;
        motor->current.i_sum = 0.0f;
        motor->current.i_alpha = 0.0f;
        motor->current.i_beta = 0.0f;
        motor->current.id = 0.0f;
        motor->current.iq = 0.0f;
        motor->current.adc_update_count = 0U;
    }

    return HAL_OK;
}

/**
 * @brief  启动指定电机的三相PWM和互补PWM输出。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @return HAL_OK表示启动成功，HAL_ERROR表示电机编号无效或底层PWM启动失败。
 * @note   先启动已经准备好的PWM，再退出对应DRV的COAST状态。
 *         COAST清除失败时重新停止PWM，避免输入状态不确定。
 */
HAL_StatusTypeDef Motor_Core_StartPwmOutput(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return HAL_ERROR;
    }

    if (FOC_StartPwmOutput(&s_motor_pwm_output[motor_id]) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (DRV_Drive_UpdateReg((drv_id_t)motor_id,
                            DRV8323_REG_DRIVER_CONTROL,
                            DRV8323_DRIVER_COAST,
                            0U) != HAL_OK)
    {
        (void)FOC_StopPwmOutput(&s_motor_pwm_output[motor_id]);
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  停止指定电机的三相PWM和互补PWM输出。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @return HAL_OK表示停止完成，HAL_ERROR表示电机编号无效或底层PWM停止异常。
 * @note   先置对应DRV的COAST位，再停止PWM，保证停机后功率管处于高阻状态。
 *         即使SPI设置失败也继续关闭PWM，并通过返回值报告异常。
 */
HAL_StatusTypeDef Motor_Core_StopPwmOutput(uint8_t motor_id)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (motor_id >= MOTOR_COUNT)
    {
        return HAL_ERROR;
    }

    if (DRV_Drive_UpdateReg((drv_id_t)motor_id,
                            DRV8323_REG_DRIVER_CONTROL,
                            DRV8323_DRIVER_COAST,
                            DRV8323_DRIVER_COAST) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    if (FOC_StopPwmOutput(&s_motor_pwm_output[motor_id]) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    return status;
}

/**
 * @brief  输出指定电机的三相中性PWM。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @return HAL_OK表示写入成功，HAL_ERROR表示电机编号无效或底层CCR写入失败。
 * @note   使用方式：调试空闲态或退出对齐输出时调用。
 *         三相duty全部写0.5，理论上线电压为0，不产生转矩；
 *         本函数不停止PWM、不关闭MOE、不停止TIM计数，因此不会打断PWM定时器触发ADC。
 *         真正故障停机仍应使用Motor_Core_StopPwmOutput()。
 */
HAL_StatusTypeDef Motor_Core_OutputNeutralPwm(uint8_t motor_id)
{
    motor_t *motor;

    if (motor_id >= MOTOR_COUNT)
    {
        return HAL_ERROR;
    }

    motor = Motor_Core_GetMotor(motor_id);
    if (motor != 0)
    {
        motor->control.vd_norm = 0.0f;
        motor->control.vq_norm = 0.0f;
        motor->control.v_alpha_norm = 0.0f;
        motor->control.v_beta_norm = 0.0f;
        motor->control.duty_u = 0.5f;
        motor->control.duty_v = 0.5f;
        motor->control.duty_w = 0.5f;
    }

    return FOC_WritePwmDuty(&s_motor_pwm_output[motor_id], 0.5f, 0.5f, 0.5f);
}

/**
 * @brief  判断指定电机当前是否允许启动PWM输出。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @retval 1表示允许启动，0表示不允许启动。
 * @note   使用方式：KEY1调试启动PWM或后续状态机进入RUN前调用。
 *         当前只是调试阶段的最小软件门禁，不替代DRV保护、硬件限流和后续安全状态机。
 *         第一版只检查：电机对象存在、编码器数据有效、FOC duty处于安全范围。
 */
uint8_t Motor_Core_IsPwmStartAllowed(uint8_t motor_id)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0U;
    }

    if (Motor_Core_IsEncoderReady(motor_id) == 0U)
    {
        return 0U;
    }

    if (Motor_Core_IsDutySafe(motor) == 0U)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  输出单电机FOC电角度对齐电压。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  vd_norm: d轴归一化电压，建议首次真实测试使用0.01f~0.02f。
 * @retval 1表示CCR写入成功，0表示参数错误或FOC/PWM写入失败。
 * @note   使用方式：先调用本函数写入固定电角度0的d轴电压，再由外部显式启动PWM输出。
 *         本函数只写CCR，不自动启动PWM，不修改控制模式，不控制DRV。
 *         对齐时vq固定为0，目标电角度固定为0，使转子吸合到已知电角度方向。
 */
uint8_t Motor_Core_OutputFocAlignVoltage(uint8_t motor_id, float vd_norm)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0U;
    }

    if (FOC_RunVoltageNorm(motor, vd_norm, 0.0f, 0.0f, 0) != HAL_OK)
    {
        return 0U;
    }

    if (FOC_WritePwmDuty(&s_motor_pwm_output[motor_id],
                         motor->control.duty_u,
                         motor->control.duty_v,
                         motor->control.duty_w) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  按当前FOC电角度输出q轴开环电压。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  vq_norm: q轴归一化电压，建议首次真实旋转从0.01~0.02开始。
 * @retval 1表示输出成功，0表示电机编号无效、编码器无效或PWM写入失败。
 * @note   使用方式：FOC电角度零偏校准完成后周期性调用。
 *         本函数会读取当前编码器角度并计算电角度，然后输出vd=0、vq=vq_norm。
 *         本函数只更新CCR，不自动启动PWM；外部必须先调用Motor_Core_StartPwmOutput()。
 */
uint8_t Motor_Core_OutputFocQVoltage(uint8_t motor_id, float vq_norm)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0U;
    }

    if (Motor_Core_UpdateEncoderAngle(motor) == 0U)
    {
        return 0U;
    }

    if (FOC_RunVoltageNorm(motor, 0.0f, vq_norm, motor->encoder.elec_angle, 0) != HAL_OK)
    {
        return 0U;
    }

    if (FOC_WritePwmDuty(&s_motor_pwm_output[motor_id],
                         motor->control.duty_u,
                         motor->control.duty_v,
                         motor->control.duty_w) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  捕获当前机械角并计算FOC电角度零位偏置。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @retval 1表示计算成功，0表示电机编号无效或编码器数据无效。
 * @note   使用方式：在OutputFocAlignVoltage输出固定d轴电压、转子吸合稳定后调用。
 *         计算公式：foc_elec_offset = wrap(0 - mech_angle * POLE_PAIRS)。
 *         该偏置只用于FOC换相，不等同于编码器机械零点和线轮位置零点。
 */
uint8_t Motor_Core_CaptureFocElecOffset(uint8_t motor_id)
{
    motor_t *motor;
    encoder_id_t encoder_id;
    float mech_angle_total;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0U;
    }

    encoder_id = Motor_Core_GetEncoderId(motor_id);
    if (SPI_Encoder_Is_Valid(encoder_id) == 0U)
    {
        return 0U;
    }

    mech_angle_total = SPI_Encoder_Get_Total_Angle(encoder_id);
    motor->calib.foc_elec_offset_rad = Motor_Core_CalcFocOffsetFromMech(mech_angle_total);
    if (motor_id == 0U)
    {
        motor->calib.foc_elec_offset_rad =
            FOC_WrapAngle0To2Pi(motor->calib.foc_elec_offset_rad + MOTOR1_FOC_ELEC_TRIM_RAD);
    }
    else if (motor_id == 1U)
    {
        motor->calib.foc_elec_offset_rad =
            FOC_WrapAngle0To2Pi(motor->calib.foc_elec_offset_rad + MOTOR2_FOC_ELEC_TRIM_RAD);
    }
    else if (motor_id == 2U)
    {
        motor->calib.foc_elec_offset_rad =
            FOC_WrapAngle0To2Pi(motor->calib.foc_elec_offset_rad + MOTOR3_FOC_ELEC_TRIM_RAD);
    }

    (void)Motor_Core_UpdateEncoderAngle(motor);

    return 1U;
}

/**
 * @brief  启动FOC电角度零偏自动校准。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @param  vd_norm: d轴吸附电压，沿用外部当前调试参数，不在motor_core内部强行改校准力度。
 * @retval None
 * @note   使用方式：KEY1按下后调用一次。
 *         本函数只启动流程并输出第一个d轴吸附电压；
 *         等待、ramp、采样和结果计算由 Motor_Core_FocOffsetAutoCalibTask() 在主循环中推进。
 */
void Motor_Core_StartFocOffsetAutoCalib(uint8_t motor_id, float vd_norm)
{
    if ((motor_id >= MOTOR_COUNT) || (vd_norm <= 0.0f) || (Motor_Core_IsEncoderReady(motor_id) == 0U))
    {
        return;
    }

    memset(&s_foc_offset_calib, 0, sizeof(s_foc_offset_calib));
    s_foc_offset_calib.motor_id = motor_id;
    s_foc_offset_calib.vd_norm = vd_norm;
    s_foc_offset_calib.state = MOTOR_FOC_OFFSET_CALIB_POINT0_SETTLE;
    s_foc_offset_calib.state_tick = HAL_GetTick();
    s_foc_offset_calib.sample_tick = s_foc_offset_calib.state_tick;

    (void)Motor_Core_OutputFocAlignVoltageAtAngle(motor_id, vd_norm, 0.0f);
}

/**
 * @brief  FOC电角度零偏自动校准任务。
 * @param  None
 * @retval None
 * @note   使用方式：放在main while中周期调用。
 *         默认流程：电角度0点吸附 -> 编码器机械角平均采样 -> 计算并保存FOC电角度offset。
 *         若 MOTOR_FOC_OFFSET_CALIB_TWO_POINT_ENABLE = 1，则恢复两点校验流程。
 */
void Motor_Core_FocOffsetAutoCalibTask(void)
{
    motor_t *motor;
    uint32_t now_tick;
    uint32_t elapsed_ms;
    float avg_mech;
#if (MOTOR_FOC_OFFSET_CALIB_TWO_POINT_ENABLE != 0U)
    float progress;
    float elec_cmd;
    float expected_delta;
    float delta_mech;
    float mech_err;
    float offset_err;
#endif

    if (s_foc_offset_calib.state == MOTOR_FOC_OFFSET_CALIB_IDLE)
    {
        return;
    }

    motor = Motor_Core_GetMotor(s_foc_offset_calib.motor_id);
    if (motor == 0)
    {
        Motor_Core_FocOffsetCalibFinish(0U);
        return;
    }

    if (Motor_Core_IsEncoderReady(s_foc_offset_calib.motor_id) == 0U)
    {
        s_foc_offset_calib.invalid_count++;
        if (s_foc_offset_calib.invalid_count > MOTOR_FOC_OFFSET_CALIB_INVALID_MAX)
        {
            Motor_Core_FocOffsetCalibFinish(0U);
        }
        return;
    }
    s_foc_offset_calib.invalid_count = 0U;

    now_tick = HAL_GetTick();
    elapsed_ms = now_tick - s_foc_offset_calib.state_tick;

    switch (s_foc_offset_calib.state)
    {
    case MOTOR_FOC_OFFSET_CALIB_POINT0_SETTLE:
        (void)Motor_Core_OutputFocAlignVoltageAtAngle(s_foc_offset_calib.motor_id,
                                                      s_foc_offset_calib.vd_norm,
                                                      0.0f);
        if (elapsed_ms >= MOTOR_FOC_OFFSET_CALIB_SETTLE_MS)
        {
            Motor_Core_FocOffsetCalibResetSample();
            s_foc_offset_calib.state = MOTOR_FOC_OFFSET_CALIB_POINT0_SAMPLE;
            s_foc_offset_calib.state_tick = now_tick;
        }
        break;

    case MOTOR_FOC_OFFSET_CALIB_POINT0_SAMPLE:
        (void)Motor_Core_OutputFocAlignVoltageAtAngle(s_foc_offset_calib.motor_id,
                                                      s_foc_offset_calib.vd_norm,
                                                      0.0f);
        if (Motor_Core_FocOffsetCalibAccumulate(&avg_mech) != 0U)
        {
            s_foc_offset_calib.mech_0 = avg_mech;
            s_foc_offset_calib.offset_0 = Motor_Core_CalcFocOffsetFromMech(avg_mech);
#if (MOTOR_FOC_OFFSET_CALIB_TWO_POINT_ENABLE != 0U)
            s_foc_offset_calib.state = MOTOR_FOC_OFFSET_CALIB_RAMP_TO_POINT1;
            s_foc_offset_calib.state_tick = now_tick;
#else
            s_foc_offset_calib.offset_final = s_foc_offset_calib.offset_0;
            if (s_foc_offset_calib.motor_id == 0U)
            {
                s_foc_offset_calib.offset_final =
                    FOC_WrapAngle0To2Pi(s_foc_offset_calib.offset_final + MOTOR1_FOC_ELEC_TRIM_RAD);
            }
            else if (s_foc_offset_calib.motor_id == 1U)
            {
                s_foc_offset_calib.offset_final =
                    FOC_WrapAngle0To2Pi(s_foc_offset_calib.offset_final + MOTOR2_FOC_ELEC_TRIM_RAD);
            }
            else if (s_foc_offset_calib.motor_id == 2U)
            {
                s_foc_offset_calib.offset_final =
                    FOC_WrapAngle0To2Pi(s_foc_offset_calib.offset_final + MOTOR3_FOC_ELEC_TRIM_RAD);
            }
            motor->calib.foc_elec_offset_rad = s_foc_offset_calib.offset_final;
            (void)Motor_Core_UpdateEncoderAngle(motor);
            Motor_Core_FocOffsetCalibFinish(1U);
#endif
        }
        break;

#if (MOTOR_FOC_OFFSET_CALIB_TWO_POINT_ENABLE != 0U)
    case MOTOR_FOC_OFFSET_CALIB_RAMP_TO_POINT1:
        progress = (float)elapsed_ms / (float)MOTOR_FOC_OFFSET_CALIB_RAMP_MS;
        progress = (progress > 1.0f) ? 1.0f : progress;
        elec_cmd = FOC_TWO_PI_F * progress;

        (void)Motor_Core_OutputFocAlignVoltageAtAngle(s_foc_offset_calib.motor_id,
                                                      s_foc_offset_calib.vd_norm,
                                                      elec_cmd);

        if (elapsed_ms >= MOTOR_FOC_OFFSET_CALIB_RAMP_MS)
        {
            s_foc_offset_calib.state = MOTOR_FOC_OFFSET_CALIB_POINT1_SETTLE;
            s_foc_offset_calib.state_tick = now_tick;
        }
        break;

    case MOTOR_FOC_OFFSET_CALIB_POINT1_SETTLE:
        (void)Motor_Core_OutputFocAlignVoltageAtAngle(s_foc_offset_calib.motor_id,
                                                      s_foc_offset_calib.vd_norm,
                                                      FOC_TWO_PI_F);
        if (elapsed_ms >= MOTOR_FOC_OFFSET_CALIB_SETTLE_MS)
        {
            Motor_Core_FocOffsetCalibResetSample();
            s_foc_offset_calib.state = MOTOR_FOC_OFFSET_CALIB_POINT1_SAMPLE;
            s_foc_offset_calib.state_tick = now_tick;
        }
        break;

    case MOTOR_FOC_OFFSET_CALIB_POINT1_SAMPLE:
        (void)Motor_Core_OutputFocAlignVoltageAtAngle(s_foc_offset_calib.motor_id,
                                                      s_foc_offset_calib.vd_norm,
                                                      FOC_TWO_PI_F);
        if (Motor_Core_FocOffsetCalibAccumulate(&avg_mech) != 0U)
        {
            s_foc_offset_calib.mech_1 = avg_mech;
            s_foc_offset_calib.offset_1 =
                FOC_WrapAngle0To2Pi(FOC_TWO_PI_F + MOTOR_FOC_OFFSET_ALIGN_ELEC_RAD +
                                     (MOTOR_FOC_OFFSET_MECH_SIGN * avg_mech * POLE_PAIRS));

            expected_delta = FOC_TWO_PI_F / (float)POLE_PAIRS;
            delta_mech = s_foc_offset_calib.mech_1 - s_foc_offset_calib.mech_0;
            mech_err = delta_mech - expected_delta;
            mech_err = (mech_err < 0.0f) ? -mech_err : mech_err;

            offset_err = Motor_Core_AngleDiffSigned(s_foc_offset_calib.offset_1,
                                                    s_foc_offset_calib.offset_0);
            offset_err = (offset_err < 0.0f) ? -offset_err : offset_err;

            if ((mech_err <= MOTOR_FOC_OFFSET_CALIB_MECH_TOL_RAD) &&
                (offset_err <= MOTOR_FOC_OFFSET_CALIB_OFFSET_TOL_RAD))
            {
                s_foc_offset_calib.offset_final =
                    FOC_WrapAngle0To2Pi(s_foc_offset_calib.offset_0 +
                                         (0.5f * Motor_Core_AngleDiffSigned(s_foc_offset_calib.offset_1,
                                                                            s_foc_offset_calib.offset_0)));
                motor->calib.foc_elec_offset_rad = s_foc_offset_calib.offset_final;
                (void)Motor_Core_UpdateEncoderAngle(motor);
                Motor_Core_FocOffsetCalibFinish(1U);
            }
            else
            {
                Motor_Core_FocOffsetCalibFinish(0U);
            }
        }
        break;
#endif

    default:
        Motor_Core_FocOffsetCalibFinish(0U);
      break;
  }
}

/**
 * @brief  判断FOC零偏自动校准当前是否正在运行。
 * @retval 1表示忙，0表示空闲。
 */
uint8_t Motor_Core_IsFocOffsetAutoCalibBusy(void)
{
    return (s_foc_offset_calib.state == MOTOR_FOC_OFFSET_CALIB_IDLE) ? 0U : 1U;
}

/**
 * @brief  获取最近一次FOC零偏自动校准结果。
 * @retval 1表示成功，0表示失败或尚未完成。
 */
uint8_t Motor_Core_GetFocOffsetAutoCalibResult(void)
{
    return s_foc_offset_calib.valid;
}

/**
 * @brief  获取指定电机当前FOC电角度。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @retval 带FOC电角度零偏修正后的电角度，单位rad，范围0~2pi。
 * @note   使用方式：FOC测试、FOC闭环或调试观测需要真实换相角时调用。
 *         编码器无效或编号非法时返回0，外部应优先检查SPI_Encoder_Is_Valid()。
 */
float Motor_Core_GetFocElecAngle(uint8_t motor_id)
{
    motor_t *motor;
    encoder_id_t encoder_id;
    float mech_angle_total;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0.0f;
    }

    encoder_id = Motor_Core_GetEncoderId(motor_id);
    if (SPI_Encoder_Is_Valid(encoder_id) == 0U)
    {
        return 0.0f;
    }

    mech_angle_total = SPI_Encoder_Get_Total_Angle(encoder_id);

    return Motor_Core_CalcFocElecAngle(motor, mech_angle_total);
}

/**
 * @brief  只刷新指定电机的电流观测数据。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @retval uint8_t
 *         1: 成功读取到新的ADC稳定帧并完成电流换算
 *         0: 电机编号无效，或当前没有新的ADC稳定帧
 * @note   使用方式：用于VOFA调试相电流，不执行PID、不执行FOC输出、不写PWM CCR。
 *         本函数不主动更新编码器，避免和控制链路重复读取角度；
 *         id/iq 使用 motor->encoder.elec_angle 当前缓存值，raw/ia/ib/ic/i_sum 不依赖编码器。
 */
uint8_t Motor_Core_UpdateCurrentObserve(uint8_t motor_id)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0U;
    }

    return Motor_Core_UpdateCurrentMeasure(motor);
}

/**
 * @brief  获取指定电机当前电流采样重构调试信息。
 * @param  motor_id: 电机编号。
 * @param  reconstruct_enable: 输出是否启用重构，允许为NULL。
 * @param  reconstruct_phase: 输出重构相，0=U/A，1=V/B，2=W/C，允许为NULL。
 * @param  adc_trigger_ccr: 输出当前ADC触发CCR，允许为NULL。
 * @retval None
 * @note   只暴露采样计划状态，供VOFA诊断；不修改控制状态。
 */
void Motor_Core_GetCurrentSampleDebug(uint8_t motor_id,
                                      uint8_t *reconstruct_enable,
                                      uint8_t *reconstruct_phase,
                                      uint16_t *adc_trigger_ccr)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return;
    }

    if (reconstruct_enable != 0)
    {
        *reconstruct_enable = s_current_sample_plan[motor_id].enable_reconstruct;
    }
    if (reconstruct_phase != 0)
    {
        *reconstruct_phase = (uint8_t)s_current_sample_plan[motor_id].reconstruct_phase;
    }
    if (adc_trigger_ccr != 0)
    {
        *adc_trigger_ccr = s_current_sample_plan[motor_id].adc_trigger_ccr;
    }
}

/**
 * @brief  更新单电流环完整链路。
 * @param  motor: 电机对象指针。
 * @param  dt: 控制周期，单位s。
 * @retval 1表示本周期完成更新，0表示参数错误、编码器无效或没有新电流采样。
 * @note   完整链路包括：编码器电角度、电流采样、id/iq PID和FOC数学。
 *         当前只计算duty，不启动PWM，也不直接写CCR。
 */
uint8_t Motor_Core_UpdateCurrentLoop(motor_t *motor, float dt)
{
    if ((motor == 0) || (dt <= 0.0f))
    {
        return 0U;
    }

    if ((Motor_Core_UpdateEncoderAngle(motor) == 0U) ||
        (Motor_Core_UpdateCurrentMeasure(motor) == 0U))
    {
        return 0U;
    }

    Motor_Core_UpdateCurrentPid(motor, dt);
    Motor_Core_RunFocMath(motor);

    return 1U;
}

/**
 * @brief  更新单速度环。
 * @param  motor: 电机对象指针。
 * @param  dt: 控制周期，单位s。
 * @retval 1表示本周期速度环更新成功，0表示参数错误或编码器无效。
 * @note   本函数只执行速度外环，输出motor->ref.iq_ref，不执行电流环和FOC。
 *         当前speed_ref_ramp_rad_s直接跟随speed_ref_rad_s，后续可接入速度目标斜坡。
 */
uint8_t Motor_Core_UpdateSpeedLoop(motor_t *motor, float dt)
{
    if ((motor == 0) || (dt <= 0.0f))
    {
        return 0U;
    }

    if (Motor_Core_UpdateEncoderAngle(motor) == 0U)
    {
        return 0U;
    }

    Motor_Core_UpdateSpeedPid(motor, dt);

    return 1U;
}

/**
 * @brief  更新速度-电流双环。
 * @param  motor: 电机对象指针。
 * @param  dt: 电流环控制周期，单位s。
 * @retval 1表示本周期电流环和FOC已更新，0表示参数错误、编码器无效或没有新电流采样。
 * @note   外部应按电流环频率调用本函数。
 *         本函数只在开头更新一次编码器，然后复用内部PID步骤函数；
 *         不调用Motor_Core_UpdateSpeedLoop()或Motor_Core_UpdateCurrentLoop()，避免重复更新编码器。
 *         速度环按MOTOR_SPEED_LOOP_DIV分频运行，电流环每次有新采样时运行。
 */
uint8_t Motor_Core_UpdateSpeedCurrentLoop(motor_t *motor, float dt)
{
    if ((motor == 0) || (dt <= 0.0f))
    {
        return 0U;
    }

    if (Motor_Core_UpdateEncoderAngle(motor) == 0U)
    {
        return 0U;
    }

    if (Motor_Core_UpdateCurrentMeasure(motor) == 0U)
    {
        return 0U;
    }

    motor->control.speed_loop_div_cnt++;
    if (motor->control.speed_loop_div_cnt >= MOTOR_SPEED_LOOP_DIV)
    {
        motor->control.speed_loop_div_cnt = 0U;
        Motor_Core_UpdateSpeedPid(motor, MOTOR_SPEED_LOOP_DT_S);
    }

    Motor_Core_UpdateCurrentPid(motor, dt);
    Motor_Core_RunFocMath(motor);

    return 1U;
}

/**
 * @brief  更新单位置环。
 * @param  motor: 电机对象指针。
 * @param  dt: 控制周期，单位s。
 * @retval 1表示本周期位置环更新成功，0表示参数错误或编码器无效。
 * @note   本函数只执行位置外环，输出motor->ref.speed_ref_rad_s。
 *         不执行速度环、电流环和FOC，方便后续按不同频率组合串级控制。
 */
uint8_t Motor_Core_UpdatePositionLoop(motor_t *motor, float dt)
{
    if ((motor == 0) || (dt <= 0.0f))
    {
        return 0U;
    }

    if (Motor_Core_UpdateEncoderAngle(motor) == 0U)
    {
        return 0U;
    }

    Motor_Core_UpdatePositionPid(motor, dt);

    return 1U;
}

/**
 * @brief  更新位置-速度双环。
 * @param  motor: 电机对象指针。
 * @param  dt: 调度周期，单位s；当前只做合法性检查，位置/速度PID使用各自固定dt。
 * @retval 1表示编码器有效且本周期完成调度，0表示参数错误或编码器无效。
 * @note   本函数只在开头更新一次编码器，然后复用内部PID步骤函数；
 *         不调用Motor_Core_UpdatePositionLoop()或Motor_Core_UpdateSpeedLoop()，避免重复更新编码器。
 *         本函数不执行电流环和FOC，只把位置目标逐级转换为iq_ref。
 */
uint8_t Motor_Core_UpdatePositionSpeedLoop(motor_t *motor, float dt)
{
    if ((motor == 0) || (dt <= 0.0f))
    {
        return 0U;
    }

    if (Motor_Core_UpdateEncoderAngle(motor) == 0U)
    {
        return 0U;
    }

    motor->control.pos_loop_div_cnt++;
    if (motor->control.pos_loop_div_cnt >= MOTOR_POSITION_LOOP_DIV)
    {
        motor->control.pos_loop_div_cnt = 0U;
        Motor_Core_UpdatePositionPid(motor, MOTOR_POSITION_LOOP_DT_S);
    }

    motor->control.speed_loop_div_cnt++;
    if (motor->control.speed_loop_div_cnt >= MOTOR_SPEED_LOOP_DIV)
    {
        motor->control.speed_loop_div_cnt = 0U;
        Motor_Core_UpdateSpeedPid(motor, MOTOR_SPEED_LOOP_DT_S);
    }

    return 1U;
}

/**
 * @brief  更新位置-速度-电流三环。
 * @param  motor: 电机对象指针。
 * @param  dt: 电流环控制周期，单位s。
 * @retval 1表示本周期电流环和FOC已更新，0表示参数错误、编码器无效或没有新电流采样。
 * @note   外部应按电流环频率调用本函数。
 *         本函数只在开头更新一次编码器，然后复用内部PID步骤函数；
 *         不调用完整单环/双环接口，避免重复更新编码器。
 *         位置环按MOTOR_POSITION_LOOP_DIV分频运行，速度环按MOTOR_SPEED_LOOP_DIV分频运行。
 */
uint8_t Motor_Core_UpdatePositionSpeedCurrentLoop(motor_t *motor, float dt)
{
    if ((motor == 0) || (dt <= 0.0f))
    {
        return 0U;
    }

    if (Motor_Core_UpdateEncoderAngle(motor) == 0U)
    {
        return 0U;
    }

    if (Motor_Core_UpdateCurrentMeasure(motor) == 0U)
    {
        return 0U;
    }

    motor->control.pos_loop_div_cnt++;
    if (motor->control.pos_loop_div_cnt >= MOTOR_POSITION_LOOP_DIV)
    {
        motor->control.pos_loop_div_cnt = 0U;
        Motor_Core_UpdatePositionPid(motor, MOTOR_POSITION_LOOP_DT_S);
    }

    motor->control.speed_loop_div_cnt++;
    if (motor->control.speed_loop_div_cnt >= MOTOR_SPEED_LOOP_DIV)
    {
        motor->control.speed_loop_div_cnt = 0U;
        Motor_Core_UpdateSpeedPid(motor, MOTOR_SPEED_LOOP_DT_S);
    }

    Motor_Core_UpdateCurrentPid(motor, dt);
    Motor_Core_RunFocMath(motor);

    return 1U;
}

/**
 * @brief  更新单个电机的电流测量量。
 * @param  motor: 电机对象指针。
 * @retval None
 * @note   数据流：ADC raw -> ia/ib/ic -> Clarke -> Park。
 *         Park变换使用motor->encoder.elec_angle，编码器未接入前该角度默认为0。
 */
static uint8_t Motor_Core_UpdateCurrentMeasure(motor_t *motor)
{
    adc_app_motor_adc_t adc_id;
    foc_alpha_beta_t alpha_beta;
    foc_dq_t dq;
    uint32_t update_count_before;
    uint32_t update_count_after;
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    float ia_now;
    float ib_now;
    float ic_now;
    uint8_t first_current_sample;

    if (motor == 0)
    {
        return 0U;
    }

    adc_id = Motor_Core_GetCurrentAdcId(motor->motor_id);
    update_count_before = ADC_App_GetMotorUpdateCount(adc_id);
    if ((update_count_before == 0U) || (update_count_before == motor->current.adc_update_count))
    {
        return 0U;
    }

    ia_raw = ADC_App_GetMotorRaw(adc_id, MOTOR_CURRENT_U_ADC_PHASE);
    ib_raw = ADC_App_GetMotorRaw(adc_id, MOTOR_CURRENT_V_ADC_PHASE);
    ic_raw = ADC_App_GetMotorRaw(adc_id, MOTOR_CURRENT_W_ADC_PHASE);

    update_count_after = ADC_App_GetMotorUpdateCount(adc_id);
    if (update_count_after != update_count_before)
    {
        return 0U;
    }

    motor->current.ia_raw = ia_raw;
    motor->current.ib_raw = ib_raw;
    motor->current.ic_raw = ic_raw;

    first_current_sample = (motor->current.adc_update_count == 0U) ? 1U : 0U;
    motor->current.adc_update_count = update_count_after;

    ia_now = Motor_Core_RawToCurrentA(motor->current.ia_raw,
                                      Motor_Core_GetPhaseZeroOffsetV(motor, MOTOR_CURRENT_U_ADC_PHASE));
    ib_now = Motor_Core_RawToCurrentA(motor->current.ib_raw,
                                      Motor_Core_GetPhaseZeroOffsetV(motor, MOTOR_CURRENT_V_ADC_PHASE));
    ic_now = Motor_Core_RawToCurrentA(motor->current.ic_raw,
                                      Motor_Core_GetPhaseZeroOffsetV(motor, MOTOR_CURRENT_W_ADC_PHASE));

    if (first_current_sample != 0U)
    {
        motor->current.ia = ia_now;
        motor->current.ib = ib_now;
        motor->current.ic = ic_now;
    }
    else
    {
        motor->current.ia += CURRENT_FILTER_ALPHA * (ia_now - motor->current.ia);
        motor->current.ib += CURRENT_FILTER_ALPHA * (ib_now - motor->current.ib);
        motor->current.ic += CURRENT_FILTER_ALPHA * (ic_now - motor->current.ic);
    }

    motor->current.i_sum = motor->current.ia + motor->current.ib + motor->current.ic;

    alpha_beta = FOC_Clarke(motor->current.ia, motor->current.ib, motor->current.ic);
    motor->current.i_alpha = alpha_beta.alpha;
    motor->current.i_beta = alpha_beta.beta;

    dq = FOC_Park(motor->current.i_alpha, motor->current.i_beta, motor->encoder.elec_angle);
    motor->current.id = dq.d;
    motor->current.iq = dq.q;

    return 1U;
}

/**
 * @brief  更新电机编码器角度和电角度。
 * @param  motor: 电机对象指针。
 * @retval 1表示编码器数据有效，0表示无效或参数错误。
 * @note   数据流：spi_encoder已处理方向/零点后的机械角 -> 乘极对数 -> 归一化电角度。
 *         当前采用严格模式：编码器无效时不允许继续执行电流环闭环。
 */
static uint8_t Motor_Core_UpdateEncoderAngle(motor_t *motor)
{
    encoder_id_t encoder_id;
    float mech_angle_total;

    if (motor == 0)
    {
        return 0U;
    }

    encoder_id = Motor_Core_GetEncoderId(motor->motor_id);
    if (SPI_Encoder_Is_Valid(encoder_id) == 0U)
    {
        motor->encoder.valid = 0U;
        return 0U;
    }

    mech_angle_total = SPI_Encoder_Get_Total_Angle(encoder_id);

    motor->encoder.mech_angle_last = motor->encoder.mech_angle;
    motor->encoder.elec_angle_last = motor->encoder.elec_angle;

    motor->encoder.turn_count = SPI_Encoder_Get_Turn_Count(encoder_id);
    motor->encoder.mech_angle_total_rad = mech_angle_total;
    motor->encoder.mech_angle = FOC_WrapAngle0To2Pi(mech_angle_total);
    motor->encoder.elec_angle = Motor_Core_CalcFocElecAngle(motor, mech_angle_total);
    motor->encoder.delta_angle = motor->encoder.mech_angle - motor->encoder.mech_angle_last;
    motor->encoder.speed_rad_s = SPI_Encoder_Get_Speed(encoder_id);
    motor->encoder.speed_rpm = RAD_S_TO_RPM(motor->encoder.speed_rad_s);
    motor->encoder.valid = 1U;

    return 1U;
}

/**
 * @brief  快环轻量更新编码器电角度。
 * @param  motor: 电机对象指针。
 * @retval 1表示编码器有效且电角度已更新，0表示无效。
 * @note   使用motor_t初始化时绑定的编码器unchecked快照，并按样本时间戳预测当前电角度。
 */
static uint8_t Motor_Core_UpdateEncoderAngleFast(motor_t *motor)
{
    spi_encoder_fast_snapshot_t enc;
    float mech_angle_total;
    float mech_angle_for_foc;

    SPI_Encoder_Get_FastSnapshot_Unchecked((encoder_id_t)motor->encoder_id, &enc);
    if (enc.valid == 0U)
    {
        motor->encoder.valid = 0U;
        return 0U;
    }

    mech_angle_total = enc.angle_total;
    mech_angle_for_foc = mech_angle_total;

#if (FOC_ELEC_ANGLE_PREDICT_ENABLE != 0U)
    if (enc.sample_cycles != 0U)
    {
        uint32_t now_cycles;
        uint32_t delta_cycles;
        float age_s;

        now_cycles = Timer_App_GetRuntimeCycles();
        delta_cycles = now_cycles - enc.sample_cycles;
        age_s = (float)delta_cycles / (float)SystemCoreClock;

        if (age_s <= FOC_ELEC_ANGLE_PREDICT_MAX_AGE_S)
        {
            mech_angle_for_foc += enc.speed_raw * (age_s + FOC_ELEC_ANGLE_PREDICT_EXTRA_DELAY_S);
        }
    }
#endif

    motor->encoder.elec_angle_last = motor->encoder.elec_angle;
    motor->encoder.mech_angle_total_rad = mech_angle_total;
    motor->encoder.elec_angle = Motor_Core_CalcFocElecAngle(motor, mech_angle_for_foc);
    motor->encoder.speed_rad_s = enc.speed;
    motor->encoder.speed_rpm = RAD_S_TO_RPM(motor->encoder.speed_rad_s);
    motor->encoder.valid = 1U;

    return 1U;
}

/**
 * @brief  快环轻量更新当前电机三相电流和id/iq。
 * @param  motor: 电机对象指针。
 * @param  sin_theta: 当前电角度sin值。
 * @param  cos_theta: 当前电角度cos值。
 * @retval 1表示电流观测量已更新，0表示ADC尚无有效帧或参数错误。
 * @note   本函数假设调用点就是对应ADC DMA完成之后，因此不再做通用路径里的二次update_count快照检查。
 */
static uint8_t Motor_Core_UpdateCurrentMeasureFast(motor_t *motor, float sin_theta, float cos_theta)
{
    foc_alpha_beta_t alpha_beta;
    foc_dq_t dq;
    uint32_t update_count;
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    float ia_now;
    float ib_now;
    float ic_now;
    uint8_t first_current_sample;

    if (motor == 0)
    {
        return 0U;
    }

    if (ADC_App_GetMotorRaw3FastUnchecked(motor->current_adc_id,
                                          &ia_raw,
                                          &ib_raw,
                                          &ic_raw,
                                          &update_count) == 0U)
    {
        return 0U;
    }

    motor->current.ia_raw = ia_raw;
    motor->current.ib_raw = ib_raw;
    motor->current.ic_raw = ic_raw;

    first_current_sample = (motor->current.adc_update_count == 0U) ? 1U : 0U;
    motor->current.adc_update_count = update_count;

    ia_now = Motor_Core_RawToCurrentA(ia_raw, motor->calib.ia_zero_offset_v);
    ib_now = Motor_Core_RawToCurrentA(ib_raw, motor->calib.ib_zero_offset_v);
    ic_now = Motor_Core_RawToCurrentA(ic_raw, motor->calib.ic_zero_offset_v);

    /*
     * 三低侧采样不是每个PWM状态下三相都可信。
     * 这里用上一周期FOC duty生成的采样计划，把低侧窗口最短的一相用另外两相重构。
     * 重构放在滤波前，避免无效相raw污染滤波状态。
     */
    Motor_Core_ReconstructPhaseCurrent(&s_current_sample_plan[motor->motor_id],
                                       &ia_now,
                                       &ib_now,
                                       &ic_now);

    if (first_current_sample != 0U)
    {
        motor->current.ia = ia_now;
        motor->current.ib = ib_now;
        motor->current.ic = ic_now;
    }
    else
    {
        motor->current.ia += CURRENT_FILTER_ALPHA * (ia_now - motor->current.ia);
        motor->current.ib += CURRENT_FILTER_ALPHA * (ib_now - motor->current.ib);
        motor->current.ic += CURRENT_FILTER_ALPHA * (ic_now - motor->current.ic);
    }

    motor->current.i_sum = motor->current.ia + motor->current.ib + motor->current.ic;

    alpha_beta = FOC_Clarke(motor->current.ia, motor->current.ib, motor->current.ic);
    motor->current.i_alpha = alpha_beta.alpha;
    motor->current.i_beta = alpha_beta.beta;

    dq = FOC_ParkSinCos(motor->current.i_alpha, motor->current.i_beta, sin_theta, cos_theta);
    motor->current.id = dq.d;
    motor->current.iq = dq.q;

    return 1U;
}

/**
 * @brief  根据上一周期采样计划重构一相电流。
 * @param  plan: 采样计划指针。
 * @param  ia: A/U相电流指针。
 * @param  ib: B/V相电流指针。
 * @param  ic: C/W相电流指针。
 * @retval None
 * @note   三相无中性线时满足 ia + ib + ic = 0。
 *         如果某相低侧采样窗口不足，则用另外两相重构该相。
 */
static void Motor_Core_ReconstructPhaseCurrent(const motor_current_sample_plan_t *plan,
                                               float *ia,
                                               float *ib,
                                               float *ic)
{
    if ((plan == 0) || (ia == 0) || (ib == 0) || (ic == 0) || (plan->enable_reconstruct == 0U))
    {
        return;
    }

    switch (plan->reconstruct_phase)
    {
    case MOTOR_CURRENT_PHASE_U:
        *ia = -(*ib) - (*ic);
        break;

    case MOTOR_CURRENT_PHASE_V:
        *ib = -(*ia) - (*ic);
        break;

    case MOTOR_CURRENT_PHASE_W:
    default:
        *ic = -(*ia) - (*ib);
        break;
    }
}

/**
 * @brief  根据本周期FOC duty生成下一周期电流采样计划。
 * @param  motor: 电机对象指针。
 * @retval None
 * @note   duty最大的相低侧导通窗口最短，下一周期优先重构该相。
 *         为避免U/V/W在边界附近来回切换，加入最小保持周期和duty滞回。
 *         动态采样点放在另外两相共同低侧有效窗口内，当前仅用于TIM20/ADC1调试链路。
 */
static void Motor_Core_UpdateCurrentSamplePlan(motor_t *motor)
{
    motor_current_sample_plan_t *plan;
#if (MOTOR_RECONSTRUCT_FORCE_ENABLE == 0U)
    motor_current_phase_t candidate_phase;
    float candidate_duty;
    float current_duty;
#endif

    if ((motor == 0) || (motor->motor_id >= MOTOR_COUNT))
    {
        return;
    }

    plan = &s_current_sample_plan[motor->motor_id];
    plan->enable_reconstruct = 1U;

#if (MOTOR_RECONSTRUCT_FORCE_ENABLE != 0U)
    plan->reconstruct_phase = MOTOR_RECONSTRUCT_FORCE_PHASE;
    plan->phase_hold_count = MOTOR_RECONSTRUCT_PHASE_MIN_HOLD_CNT;
#else
    if ((motor->control.duty_u >= motor->control.duty_v) &&
        (motor->control.duty_u >= motor->control.duty_w))
    {
        candidate_phase = MOTOR_CURRENT_PHASE_U;
        candidate_duty = motor->control.duty_u;
    }
    else if ((motor->control.duty_v >= motor->control.duty_u) &&
             (motor->control.duty_v >= motor->control.duty_w))
    {
        candidate_phase = MOTOR_CURRENT_PHASE_V;
        candidate_duty = motor->control.duty_v;
    }
    else
    {
        candidate_phase = MOTOR_CURRENT_PHASE_W;
        candidate_duty = motor->control.duty_w;
    }

    switch (plan->reconstruct_phase)
    {
    case MOTOR_CURRENT_PHASE_U:
        current_duty = motor->control.duty_u;
        break;

    case MOTOR_CURRENT_PHASE_V:
        current_duty = motor->control.duty_v;
        break;

    case MOTOR_CURRENT_PHASE_W:
    default:
        current_duty = motor->control.duty_w;
        break;
    }

    if (plan->phase_hold_count > 0U)
    {
        plan->phase_hold_count--;
    }
    else if ((candidate_phase != plan->reconstruct_phase) &&
             (candidate_duty > (current_duty + MOTOR_RECONSTRUCT_PHASE_HYST_DUTY)))
    {
        plan->reconstruct_phase = candidate_phase;
        plan->phase_hold_count = MOTOR_RECONSTRUCT_PHASE_MIN_HOLD_CNT;
    }
#endif

#if (MOTOR_ADC_TRIGGER_DYNAMIC_ENABLE != 0U)
    plan->adc_trigger_ccr = Motor_Core_CalcAdcTriggerCcrFromDuty(motor, plan->reconstruct_phase);
#else
    plan->adc_trigger_ccr = (uint16_t)(motor->pwm_arr / 2U);
#endif
    Motor_Core_WriteAdcTriggerCcr(motor, plan);
}

/**
 * @brief  写入ADC触发比较值。
 * @param  motor: 电机对象指针。
 * @param  plan: 电流采样计划指针。
 * @retval None
 * @note   写入初始化阶段绑定的ADC触发CCR。
 *         motor2当前对应TIM20 CCR3；motor0/1预留TIM1/TIM8 CCR4，需CubeMX侧配合OC4REF才会真正生效。
 */
static void Motor_Core_WriteAdcTriggerCcr(const motor_t *motor,
                                          const motor_current_sample_plan_t *plan)
{
    if ((motor == 0) || (plan == 0))
    {
        return;
    }

    *(motor->adc_trigger_ccr) = plan->adc_trigger_ccr;
}

/**
 * @brief  根据三相duty计算下一周期ADC触发CCR。
 * @param  motor: 电机对象指针。
 * @param  reconstruct_phase: 计划重构的相，通常为duty最大的相。
 * @retval TIM比较值，用于写入ADC触发通道CCR。
 * @note   第一版轻量策略：
 *         1. duty最大的相低侧窗口最短，计划重构该相；
 *         2. 另外两相作为可信采样相；
 *         3. TIM20使用PWM1互补输出时，低侧有效窗口大致在CNT大于该相CCR之后；
 *         4. 因此取可信两相CCR较大值作为共同窗口起点，并把采样点放到窗口中间。
 */
#if (MOTOR_ADC_TRIGGER_DYNAMIC_ENABLE != 0U)
static uint16_t Motor_Core_CalcAdcTriggerCcrFromDuty(const motor_t *motor,
                                                     motor_current_phase_t reconstruct_phase)
{
    uint32_t ccr_u;
    uint32_t ccr_v;
    uint32_t ccr_w;
    uint32_t trusted_max_ccr;
    uint32_t window_start;
    uint32_t window_end;
    uint32_t sample_ccr;

    if (motor == 0)
    {
        return 0U;
    }

    if (motor->pwm_arr == 0U)
    {
        return 0U;
    }

    ccr_u = motor->pwm_ccr_u_last;
    ccr_v = motor->pwm_ccr_v_last;
    ccr_w = motor->pwm_ccr_w_last;

    switch (reconstruct_phase)
    {
    case MOTOR_CURRENT_PHASE_U:
        trusted_max_ccr = (ccr_v > ccr_w) ? ccr_v : ccr_w;
        break;

    case MOTOR_CURRENT_PHASE_V:
        trusted_max_ccr = (ccr_u > ccr_w) ? ccr_u : ccr_w;
        break;

    case MOTOR_CURRENT_PHASE_W:
    default:
        trusted_max_ccr = (ccr_u > ccr_v) ? ccr_u : ccr_v;
        break;
    }

    window_start = trusted_max_ccr + motor->adc_guard_ccr;
    window_end = motor->pwm_arr - motor->adc_guard_ccr;

    if (window_end <= motor->adc_guard_ccr)
    {
        return (uint16_t)(motor->pwm_arr / 2U);
    }

    if (window_start >= window_end)
    {
        /*
         * 可信两相共同低侧窗口过窄时，先退到靠近ARR但避开边沿的位置。
         * 这仍然是降级采样，后续应结合VOFA观察raw是否继续回到2048。
         */
        sample_ccr = window_end;
    }
    else
    {
        sample_ccr = window_start + ((window_end - window_start) / 2U);
    }

    if (sample_ccr >= motor->pwm_arr)
    {
        sample_ccr = motor->pwm_arr - 1U;
    }

    return (uint16_t)sample_ccr;
}
#endif

/**
 * @brief  执行单电机id/iq电流环。
 * @param  motor: 电机对象指针。
 * @param  dt: 控制周期，单位s。
 * @retval None
 * @note   id_ref/iq_ref单位为A，PID输出为vd_norm/vq_norm归一化电压指令。
 *         当前没有额外斜坡参数，iq_ref_ramp先直接跟随iq_ref。
 */
static void Motor_Core_UpdateCurrentPid(motor_t *motor, float dt)
{
    float vd_norm;
    float vq_norm;
#if (MOTOR_BEMF_FF_ENABLE != 0U)
    float speed_abs;
    float vq_bemf_ff;
#endif

    if ((motor == 0) || (dt <= 0.0f))
    {
        return;
    }

    vd_norm = motor->control.vd_norm;
    vq_norm = motor->control.vq_norm;
    motor->ref.iq_ref_ramp = motor->ref.iq_ref;

    (void)Control_PID_Update(&motor->control.id_pid,
                             motor->ref.id_ref,
                             motor->current.id,
                             dt,
                             &vd_norm);

    Motor_Core_UpdateCurrentPidSeparated(&motor->control.iq_pid,
                                         motor->ref.iq_ref_ramp,
                                          motor->current.iq,
                                          dt,
                                          &vq_norm);

#if (MOTOR_BEMF_FF_ENABLE != 0U)
    /*
     * q轴反电动势前馈：
     * vq_ff = omega_e * flux / Vbus
     * 只在电流PI之后叠加到vq，不改变id环，也暂不做Ld/Lq交叉解耦。
     */
    speed_abs = motor->encoder.speed_rad_s;
    speed_abs = (speed_abs < 0.0f) ? -speed_abs : speed_abs;
    if (speed_abs >= MOTOR_BEMF_FF_SPEED_MIN_RAD_S)
    {
        vq_bemf_ff = MOTOR_BEMF_FF_GAIN
                   * motor->encoder.speed_rad_s
                   * (float)POLE_PAIRS
                   * MOTOR_FLUX_WB
                   / MOTOR_VOLTAGE_NOMINAL_V;
        vq_norm += LIMIT(vq_bemf_ff, -MOTOR_BEMF_FF_LIMIT_NORM, MOTOR_BEMF_FF_LIMIT_NORM);
    }
#endif

    motor->control.vd_norm = vd_norm;
    motor->control.vq_norm = vq_norm;
}

/**
 * @brief  电流环积分分离PID更新。
 * @param  pid: PID对象指针。
 * @param  ref: 电流目标，单位A。
 * @param  feedback: 电流反馈，单位A。
 * @param  dt: 控制周期，单位s。
 * @param  output: PID输出指针。
 * @retval None
 * @note   电流环P/D始终参与控制；只有误差不大于阈值时才允许积分。
 *         误差超过阈值时清空积分并临时关闭积分，避免大误差下积分积住导致退电流很慢。
 */
static void Motor_Core_UpdateCurrentPidSeparated(control_pid_t *pid,
                                                 float ref,
                                                 float feedback,
                                                 float dt,
                                                 float *output)
{
    float error_abs;
    uint8_t integral_enable_saved;

    if ((pid == 0) || (dt <= 0.0f))
    {
        return;
    }

    error_abs = ref - feedback;
    error_abs = (error_abs < 0.0f) ? -error_abs : error_abs;
    integral_enable_saved = pid->param.enable_integral;

    if (error_abs > CURRENT_PID_INTEGRAL_ENABLE_ERROR_A)
    {
        /* 误差较大时缓慢泄放积分，避免清零断崖，也避免错误积分长期卡住。 */
        pid->integral *= CURRENT_PID_INTEGRAL_LEAK_FACTOR;
        pid->param.enable_integral = 0U;
    }

    (void)Control_PID_Update(pid, ref, feedback, dt, output);
    pid->param.enable_integral = integral_enable_saved;
}

/**
 * @brief  位置环堵转检测与软保护。
 * @param  motor 电机对象。
 * @param  pos_feedback 当前实际位置，单位rad，已转换到外部命令坐标系。
 * @param  pos_error_abs 当前位置误差绝对值，单位rad。
 * @param  lock_active 当前是否处于位置锁定区。
 * @retval 1表示触发堵转保护，0表示未触发。
 * @note   触发后锁定当前位置并清控制状态，避免继续顶住原目标。
 */
static uint8_t Motor_Core_CheckPositionStall(motor_t *motor,
                                             float pos_feedback,
                                             float pos_error_abs,
                                             uint8_t lock_active)
{
    uint8_t id;
    float iq_abs;
    float move_abs;

    if ((motor == 0) || (motor->motor_id >= MOTOR_COUNT))
    {
        return 0U;
    }

    id = motor->motor_id;
    iq_abs = motor->current.iq;
    iq_abs = (iq_abs < 0.0f) ? -iq_abs : iq_abs;

    if ((lock_active != 0U) ||
        (pos_error_abs <= STALL_POS_ERROR_MIN_RAD) ||
        (iq_abs <= STALL_CURRENT_THRESHOLD_A))
    {
        s_position_stall_valid[id] = 0U;
        s_position_stall_count[id] = 0U;
        return 0U;
    }

    if (s_position_stall_valid[id] == 0U)
    {
        s_position_stall_valid[id] = 1U;
        s_position_stall_start_pos[id] = pos_feedback;
        s_position_stall_count[id] = 0U;
        return 0U;
    }

    move_abs = pos_feedback - s_position_stall_start_pos[id];
    move_abs = (move_abs < 0.0f) ? -move_abs : move_abs;
    if (move_abs >= STALL_MOVE_MIN_RAD)
    {
        s_position_stall_start_pos[id] = pos_feedback;
        s_position_stall_count[id] = 0U;
        return 0U;
    }

    if (s_position_stall_count[id] < 0xFFFFU)
    {
        s_position_stall_count[id]++;
    }
    if (s_position_stall_count[id] < STALL_CHECK_COUNT_LIMIT)
    {
        return 0U;
    }

    motor->ref.mech_angle_ref = pos_feedback;
    motor->ref.mech_angle_ref_ramp = pos_feedback;
    motor->ref.speed_ref_rad_s = 0.0f;
    motor->ref.speed_ref_ramp_rad_s = 0.0f;
    motor->ref.id_ref = 0.0f;
    motor->ref.iq_ref = 0.0f;
    motor->ref.iq_ref_ramp = 0.0f;

    Control_PID_Reset(&motor->control.pos_pid);
    Control_PID_Reset(&motor->control.speed_pid);
    Control_PID_Reset(&motor->control.iq_pid);
    Control_PID_Reset(&motor->control.id_pid);

    s_position_approach_active[id] = 0U;
    s_position_lock_active[id] = 0U;
    s_position_stall_valid[id] = 0U;
    s_position_stall_count[id] = 0U;

    motor->protect.stall_fault = 1U;
    if (motor->protect.stall_count < 0xFFFFU)
    {
        motor->protect.stall_count++;
    }

    motor->mode = MOTOR_MODE_IDLE;
    (void)Motor_Core_OutputNeutralPwm(id);

    return 1U;
}

/**
 * @brief  获取位置环速度上限。
 * @param  motor_id: 电机编号。
 * @retval 速度上限，单位rad/s。
 * @note   对内部缓存做防御限幅，避免外部配置错误导致位置环输出异常。
 */
static float Motor_Core_GetPositionSpeedLimit(uint8_t motor_id)
{
    float speed_limit;

    if (motor_id >= MOTOR_COUNT)
    {
        return POS_APPROACH_SPEED_RAD_S;
    }

    speed_limit = s_position_speed_limit_rad_s[motor_id];
    if (speed_limit <= 0.0f)
    {
        speed_limit = POS_APPROACH_SPEED_RAD_S;
    }

    return LIMIT(speed_limit,
                 MOTOR_POSITION_SPEED_LIMIT_MIN_RAD_S,
                 POS_PID_OUT_LIM_RAD_S_DEFAULT);
}

/**
 * @brief  执行位置环PID。
 * @param  motor: 电机对象指针。
 * @param  dt: 控制周期，单位s。
 * @retval None
 * @note   位置环输入为mech_angle_ref和连续机械角反馈，输出为机械角速度目标speed_ref_rad_s。
 *         mech_angle_ref是最终目标，mech_angle_ref_ramp按巡航速度平滑追踪。
 *         位置环不再使用“到位清零”死区；进入锁定区后使用小范围位置刚度和速度阻尼保持伺服锁死。
 *         锁定区只改变speed_ref，不清speed_pid和iq_ref，避免带负载时突然泄力。
 */
static void Motor_Core_UpdatePositionPid(motor_t *motor, float dt)
{
    float pos_error;
    float pos_error_abs;
    float pos_feedback;
    float speed_feedback;
    float pos_error_eff;
    float speed_ref;
    float speed_ref_delta;
    float speed_ref_step;
    float speed_ref_target;
    float speed_limit;
    uint8_t lock_active = 0U;
    uint8_t approach_active;

    if ((motor == 0) || (dt <= 0.0f))
    {
        return;
    }

    pos_feedback = motor->encoder.mech_angle_total_rad * s_motor_mech_fb_sign[motor->motor_id];
    speed_feedback = motor->encoder.speed_rad_s * s_motor_mech_fb_sign[motor->motor_id];
    speed_limit = Motor_Core_GetPositionSpeedLimit(motor->motor_id);
    Motor_Core_UpdatePositionRefRamp(motor, speed_limit, dt);

    /*
     * mech_angle_total_rad来自编码器累计角。
     * spi_encoder已负责方向和零点处理，这里直接使用连续机械角，避免零点被重复扣除。
     */
    pos_error = motor->ref.mech_angle_ref_ramp - pos_feedback;
    pos_error_abs = (pos_error < 0.0f) ? -pos_error : pos_error;

    approach_active = s_position_approach_active[motor->motor_id];
    if (approach_active != 0U)
    {
        if (pos_error_abs <= POS_APPROACH_ENTER_RAD)
        {
            approach_active = 0U;
        }
    }
    else if (pos_error_abs >= POS_APPROACH_EXIT_RAD)
    {
        approach_active = 1U;
    }
    s_position_approach_active[motor->motor_id] = approach_active;

    if (approach_active != 0U)
    {
        s_position_lock_active[motor->motor_id] = 0U;
        if (Motor_Core_CheckPositionStall(motor, pos_feedback, pos_error_abs, 0U) != 0U)
        {
            return;
        }
        pos_error_eff = pos_error;
        speed_ref_target = (pos_error >= 0.0f) ? speed_limit : -speed_limit;
        speed_ref = LIMIT(speed_ref_target,
                          -speed_limit,
                           speed_limit);
    }
    else
    {
        lock_active = s_position_lock_active[motor->motor_id];
        if (lock_active != 0U)
        {
            if (pos_error_abs >= POS_LOCK_EXIT_RAD)
            {
                lock_active = 0U;
            }
        }
        else if (pos_error_abs <= POS_LOCK_ENTER_RAD)
        {
            lock_active = 1U;
        }

        s_position_lock_active[motor->motor_id] = lock_active;

        if (Motor_Core_CheckPositionStall(motor, pos_feedback, pos_error_abs, lock_active) != 0U)
        {
            return;
        }

        if (lock_active != 0U)
        {
            pos_error_eff = pos_error;
            speed_ref_target = (POS_LOCK_KP_DEFAULT * pos_error_eff)
                      - (POS_LOCK_KD_DEFAULT * speed_feedback);
            speed_ref = LIMIT(speed_ref_target,
                              -POS_LOCK_SPEED_LIM_RAD_S,
                               POS_LOCK_SPEED_LIM_RAD_S);

            /*
             * 锁定区允许速度环积分提供保持力，但积分不能把电机往目标外推。
             * 若积分方向与位置误差方向相反，说明积分正在帮助越过目标，快速泄放；
             * 目标中心附近再轻微泄放，避免积分长期堆住导致锁定角慢慢漂移。
             */
            if ((pos_error * motor->control.speed_pid.integral) < 0.0f)
            {
                motor->control.speed_pid.integral *= POS_LOCK_SPEED_I_REVERSE_LEAK_FACTOR;
            }
            else if (pos_error_abs <= POS_LOCK_SPEED_I_CENTER_LEAK_RAD)
            {
                motor->control.speed_pid.integral *= POS_LOCK_SPEED_I_CENTER_LEAK_FACTOR;
            }
        }
        else
        {
            pos_error_eff = pos_error;
            speed_ref_target = POS_PID_KP_DEFAULT * pos_error_eff;
            speed_ref_target -= POS_VEL_DAMPING_GAIN_DEFAULT * speed_feedback;
            speed_ref = LIMIT(speed_ref_target,
                              -speed_limit,
                               speed_limit);
        }
    }

    /*
     * 位置环当前使用显式P + 速度阻尼，不使用通用PID内部死区和积分。
     * 锁定区仍然保留位置误差，不把误差清零，保证小偏差也能产生保持力。
    */
    motor->ref.speed_ref_rad_s = speed_ref;
    motor->control.pos_pid.ref = motor->ref.mech_angle_ref_ramp;
    motor->control.pos_pid.feedback = pos_feedback;
    motor->control.pos_pid.error = pos_error_eff;
    motor->control.pos_pid.proportional = speed_ref;
    motor->control.pos_pid.integral = 0.0f;
    motor->control.pos_pid.derivative = 0.0f;
    motor->control.pos_pid.output_raw = speed_ref;
    motor->control.pos_pid.output = speed_ref;
    motor->control.pos_pid.saturated = 0U;

    if (lock_active != 0U)
    {
        speed_ref_step = POS_LOCK_SPEED_REF_ACCEL_LIMIT_RAD_S2 * dt;
    }
    else
    {
        speed_ref_step = POS_SPEED_REF_ACCEL_LIMIT_RAD_S2 * dt;
    }

    speed_ref_delta = speed_ref - motor->ref.speed_ref_ramp_rad_s;
  speed_ref_delta = LIMIT(speed_ref_delta, -speed_ref_step, speed_ref_step);

  motor->ref.speed_ref_ramp_rad_s += speed_ref_delta;
}

/**
 * @brief  执行速度环PID。
 * @param  motor: 电机对象指针。
 * @param  dt: 控制周期，单位s。
 * @retval None
 * @note   速度环输入为speed_ref_rad_s和encoder.speed_rad_s，输出为q轴电流目标iq_ref。
 *         当前不做速度目标斜坡，speed_ref_ramp_rad_s直接等于speed_ref_rad_s。
 */
static void Motor_Core_UpdateSpeedPid(motor_t *motor, float dt)
{
    float iq_ref_target;
    float iq_ref_filtered;
    float speed_feedback;
    float speed_error;
    float speed_error_abs;
    float speed_integral_abs;
    float speed_kp_saved;
    float speed_kp_scale;
    float speed_ki_saved;

    if ((motor == 0) || (dt <= 0.0f))
    {
        return;
    }

    iq_ref_target = motor->ref.iq_ref;
    speed_feedback = motor->encoder.speed_rad_s * s_motor_mech_fb_sign[motor->motor_id];

    if ((motor->mode != MOTOR_MODE_POSITION) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED) &&
        (motor->mode != MOTOR_MODE_POSITION_SPEED_CURRENT))
    {
        motor->ref.speed_ref_ramp_rad_s = motor->ref.speed_ref_rad_s;
    }
    speed_error = motor->ref.speed_ref_ramp_rad_s - speed_feedback;
    speed_error_abs = speed_error;
    speed_error_abs = (speed_error_abs < 0.0f) ? -speed_error_abs : speed_error_abs;
    speed_kp_saved = motor->control.speed_pid.param.kp;
    speed_ki_saved = motor->control.speed_pid.param.ki;

    speed_kp_scale = 1.0f;
    if (speed_error_abs < SPEED_PID_KP_FULL_ERROR_RAD_S)
    {
        speed_kp_scale = SPEED_PID_KP_MIN_SCALE
                       + ((1.0f - SPEED_PID_KP_MIN_SCALE)
                       * speed_error_abs / SPEED_PID_KP_FULL_ERROR_RAD_S);
    }
    motor->control.speed_pid.param.kp = speed_kp_saved * speed_kp_scale;

    if (speed_error_abs > SPEED_PID_INTEGRAL_ENABLE_ERROR_RAD_S)
    {
        motor->control.speed_pid.param.ki = speed_ki_saved * SPEED_PID_LARGE_ERROR_KI_SCALE;
    }

    speed_integral_abs = motor->control.speed_pid.integral;
    speed_integral_abs = (speed_integral_abs < 0.0f) ? -speed_integral_abs : speed_integral_abs;
    if ((speed_integral_abs > SPEED_PID_INTEGRAL_REVERSE_MIN_A) &&
        ((speed_error * motor->control.speed_pid.integral) < 0.0f))
    {
        motor->control.speed_pid.integral *= SPEED_PID_INTEGRAL_REVERSE_LEAK_FACTOR;
    }

    (void)Control_PID_Update(&motor->control.speed_pid,
                             motor->ref.speed_ref_ramp_rad_s,
                             speed_feedback,
                             dt,
                             &iq_ref_target);

    motor->control.speed_pid.param.kp = speed_kp_saved;
    motor->control.speed_pid.param.ki = speed_ki_saved;

    iq_ref_filtered = motor->ref.iq_ref
                    + (SPEED_IQ_REF_LPF_ALPHA * (iq_ref_target - motor->ref.iq_ref));

    motor->ref.iq_ref = iq_ref_filtered;
    motor->ref.iq_ref_ramp = iq_ref_filtered;
}

/**
 * @brief  执行FOC电压矢量计算。
 * @param  motor: 电机对象指针。
 * @retval None
 * @note   当前只调用FOC_RunVoltageNorm()更新FOC数学结果和duty，不启动PWM，也不直接写CCR。
 */
static void Motor_Core_RunFocMath(motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    (void)FOC_RunVoltageNorm(motor,
                             motor->control.vd_norm,
                             motor->control.vq_norm,
                             motor->encoder.elec_angle,
                               0);
}

/**
 * @brief  duty快速转换为TIM CCR比较值。
 * @param  duty: PWM占空比，理论范围0~1。
 * @param  arr:  TIM自动重装载值。
 * @retval CCR比较值，范围0~arr。
 * @note   使用三目运算符做轻量限幅，避免异常duty写出有效范围。
 */
static uint32_t Motor_Core_DutyToTimCompareFast(float duty, uint32_t arr)
{
    duty = (duty < PWM_DUTY_MIN) ? PWM_DUTY_MIN : ((duty > PWM_DUTY_MAX) ? PWM_DUTY_MAX : duty);

    return (uint32_t)((duty * (float)arr) + 0.5f);
}

/**
 * @brief  快环PWM CCR快速写入。
 * @param  motor: 电机对象指针，使用其control.duty_u/v/w和初始化时绑定的CCR指针。
 * @retval None
 * @note   相比FOC_WritePwmDuty()，本函数不做通用通道分发和HAL封装，减少ADC快环开销。
 */
static void Motor_Core_WritePwmCcrFast(motor_t *motor)
{
    uint32_t ccr_u;
    uint32_t ccr_v;
    uint32_t ccr_w;

    if ((motor == 0) || (motor->pwm_arr == 0U))
    {
        return;
    }

    ccr_u = Motor_Core_DutyToTimCompareFast(motor->control.duty_u, motor->pwm_arr);
    ccr_v = Motor_Core_DutyToTimCompareFast(motor->control.duty_v, motor->pwm_arr);
    ccr_w = Motor_Core_DutyToTimCompareFast(motor->control.duty_w, motor->pwm_arr);

    *(motor->pwm_ccr_u) = ccr_u;
    *(motor->pwm_ccr_v) = ccr_v;
    *(motor->pwm_ccr_w) = ccr_w;

    motor->pwm_ccr_u_last = ccr_u;
    motor->pwm_ccr_v_last = ccr_v;
    motor->pwm_ccr_w_last = ccr_w;
}

/**
 * @brief  记录一次ADC快环DWT耗时。
 * @param  motor_id: 电机编号，调用方保证来自固定ADC回调。
 * @param  start_cycles: 快环入口DWT cycle。
 * @param  result: 本次快环原始返回值。
 * @retval 原样返回result，便于快环多出口保持简洁。
 * @note   只使用整数cycle计数，避免在高频路径中做浮点换算。
 */
static uint8_t Motor_Core_RecordFastLoopProfile(uint8_t motor_id,
                                                uint32_t start_cycles,
                                                uint8_t result)
{
    motor_core_fast_loop_profile_t *profile;
    uint32_t elapsed_cycles;
    uint32_t limit_cycles;

    if (motor_id >= MOTOR_COUNT)
    {
        return result;
    }

    elapsed_cycles = Timer_App_GetRuntimeCycles() - start_cycles;
    limit_cycles = SystemCoreClock / MOTOR_CURRENT_LOOP_FREQ_HZ;
    profile = &s_fast_loop_profile[motor_id];

    profile->last_cycles = elapsed_cycles;
    if (elapsed_cycles > profile->max_cycles)
    {
        profile->max_cycles = elapsed_cycles;
    }
    if (elapsed_cycles > limit_cycles)
    {
        profile->overrun_count++;
    }
    profile->run_count++;
    profile->last_result = result;

    return result;
}

/**
 * @brief  清除模式切换时不应继承的控制运行状态。
 * @param  motor: 电机对象指针。
 * @retval None
 * @note   使用方式：Motor_Core_SetMode() 检测到模式变化时调用。
 *         本函数只清PID内部积分/误差/输出、环路分频计数和FOC输出缓存；
 *         不清 motor->ref 中的外部目标值，避免切换模式时把用户设置的目标意外擦掉。
 */
static void Motor_Core_ResetControlRuntime(motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    Control_PID_Reset(&motor->control.id_pid);
    Control_PID_Reset(&motor->control.iq_pid);
    Control_PID_Reset(&motor->control.speed_pid);
    Control_PID_Reset(&motor->control.pos_pid);

    motor->control.speed_loop_div_cnt = 0U;
    motor->control.pos_loop_div_cnt = 0U;
    s_position_lock_active[motor->motor_id] = 0U;
    s_position_approach_active[motor->motor_id] = 0U;
    s_position_stall_valid[motor->motor_id] = 0U;
    s_position_stall_start_pos[motor->motor_id] = 0.0f;
    s_position_stall_count[motor->motor_id] = 0U;
    motor->protect.stall_fault = 0U;

    motor->control.vd_norm = 0.0f;
    motor->control.vq_norm = 0.0f;
    motor->control.v_alpha_norm = 0.0f;
    motor->control.v_beta_norm = 0.0f;

    motor->control.duty_u = FOC_DUTY_CENTER;
    motor->control.duty_v = FOC_DUTY_CENTER;
    motor->control.duty_w = FOC_DUTY_CENTER;
}

/**
 * @brief  根据电机ID选择对应的电流采样ADC组。
 * @param  motor_id: 电机编号。
 * @retval adc_app_motor_adc_t
 * @note   当前映射按现有定时器触发关系：
 *         motor0/TIM1 -> ADC4，motor1/TIM8 -> ADC3，motor2/TIM20 -> ADC1。
 */
static adc_app_motor_adc_t Motor_Core_GetCurrentAdcId(uint8_t motor_id)
{
    switch (motor_id)
    {
    case 0U:
        return ADC_APP_MOTOR_ADC4;

    case 1U:
        return ADC_APP_MOTOR_ADC3;

    case 2U:
        return ADC_APP_MOTOR_ADC1;

    default:
        return ADC_APP_MOTOR_ADC4;
    }
}

/**
 * @brief  根据电机ID选择对应编码器ID。
 * @param  motor_id: 电机编号。
 * @retval encoder_id_t
 * @note   当前默认motor0/1/2分别对应encoder_id_1/2/3。
 */
static encoder_id_t Motor_Core_GetEncoderId(uint8_t motor_id)
{
    switch (motor_id)
    {
    case 0U:
        return encoder_id_1;

    case 1U:
        return encoder_id_2;

    case 2U:
        return encoder_id_3;

    default:
        return encoder_id_1;
    }
}

/**
 * @brief  判断指定电机对应编码器是否已经有有效数据。
 * @param  motor_id: 电机编号，范围0 ~ MOTOR_COUNT-1。
 * @retval 1表示编码器有效，0表示电机编号无效或编码器无效。
 * @note   使用方式：PWM启动前检查真实角度是否可信。
 *         本函数只读取spi_encoder有效标志，不主动触发SPI读取。
 */
static uint8_t Motor_Core_IsEncoderReady(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return 0U;
    }

    return SPI_Encoder_Is_Valid(Motor_Core_GetEncoderId(motor_id));
}

/**
 * @brief  判断当前FOC duty是否处于允许启动PWM的范围。
 * @param  motor: 电机对象指针。
 * @retval 1表示duty安全，0表示duty未初始化或越界。
 * @note   使用方式：PWM启动前确认FOC已经计算过合理占空比。
 *         如果三相duty仍为0，通常表示FOC尚未产生有效输出，禁止启动。
 */
static uint8_t Motor_Core_IsDutySafe(const motor_t *motor)
{
    if (motor == 0)
    {
        return 0U;
    }

    if ((motor->control.duty_u == 0.0f) &&
        (motor->control.duty_v == 0.0f) &&
        (motor->control.duty_w == 0.0f))
    {
        return 0U;
    }

    if ((motor->control.duty_u < PWM_DUTY_MIN) ||
        (motor->control.duty_u > PWM_DUTY_MAX) ||
        (motor->control.duty_v < PWM_DUTY_MIN) ||
        (motor->control.duty_v > PWM_DUTY_MAX) ||
        (motor->control.duty_w < PWM_DUTY_MIN) ||
        (motor->control.duty_w > PWM_DUTY_MAX))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  根据机械角和FOC电角度零偏计算电角度。
 * @param  motor: 电机对象指针。
 * @param  mech_angle_total_rad: 编码器累计机械角，单位rad。
 * @retval FOC电角度，单位rad，范围0~2pi。
 * @note   使用方式：所有FOC换相角都应通过本函数统一计算，避免机械零点和电角度零点混用。
 *         公式：elec_angle = wrap(mech_angle * POLE_PAIRS + foc_elec_offset)。
 */
static float Motor_Core_CalcFocElecAngle(const motor_t *motor, float mech_angle_total_rad)
{
    if (motor == 0)
    {
        return 0.0f;
    }

    return FOC_WrapAngle0To2Pi((mech_angle_total_rad * POLE_PAIRS) + motor->calib.foc_elec_offset_rad);
}

/**
 * @brief  获取G4位置环真实使用的原始多圈位置。
 * @param  motor_id 电机编号。
 * @retval 带机械反馈方向的原始多圈位置，单位rad。
 * @note   该值不包含H7 RESTORE映射，只表示G4本机编码器连续角。
 */
static float Motor_Core_GetSignedRawPositionRad(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return 0.0f;
    }

    return SPI_Encoder_Get_Total_Angle((encoder_id_t)Motor_Core_GetBoundEncoderId(motor_id)) *
           s_motor_mech_fb_sign[motor_id];
}

/**
 * @brief  将H7位置坐标下的目标转换为G4位置环原始目标。
 * @param  motor_id 电机编号。
 * @param  external_target_rad H7下发的目标位置，单位rad。
 * @retval G4位置环使用的原始目标，单位rad。
 * @note   RESTORE未建立时直接使用输入值；建立后按目标相对H7保存位置的差值换算。
 */
static float Motor_Core_ExternalToRawTargetRad(uint8_t motor_id, float external_target_rad)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return 0.0f;
    }

    if (s_position_restore_valid[motor_id] == 0U)
    {
        return external_target_rad;
    }

    return s_position_restore_base_raw_rad[motor_id] +
           (external_target_rad - s_position_restore_saved_rad[motor_id]);
}

/**
 * @brief  根据d轴吸附后的机械角计算FOC电角度offset。
 * @param  mech_angle_total_rad: 编码器方向/零点处理后的累计机械角，单位rad。
 * @retval FOC电角度offset，单位rad，范围0~2pi。
 * @note   用于排查校正公式定义。MOTOR_FOC_OFFSET_ALIGN_ELEC_RAD表示吸附命令对应的目标电角度；
 *         MOTOR_FOC_OFFSET_MECH_SIGN用于验证编码器方向和FOC正方向是否一致。
 */
static float Motor_Core_CalcFocOffsetFromMech(float mech_angle_total_rad)
{
    return FOC_WrapAngle0To2Pi(MOTOR_FOC_OFFSET_ALIGN_ELEC_RAD +
                               (MOTOR_FOC_OFFSET_MECH_SIGN * mech_angle_total_rad * POLE_PAIRS));
}

/**
 * @brief  输出指定电角度的d轴吸附电压。
 * @param  motor_id: 电机编号。
 * @param  vd_norm: d轴归一化吸附电压。
 * @param  elec_angle_rad: 指令电角度，单位rad。
 * @retval 1表示写入成功，0表示参数错误或FOC/PWM写入失败。
 * @note   自动offset校准内部使用；只写CCR，不启动PWM。
 */
static uint8_t Motor_Core_OutputFocAlignVoltageAtAngle(uint8_t motor_id,
                                                       float vd_norm,
                                                       float elec_angle_rad)
{
    motor_t *motor;

    motor = Motor_Core_GetMotor(motor_id);
    if (motor == 0)
    {
        return 0U;
    }

    if (FOC_RunVoltageNorm(motor, vd_norm, 0.0f, elec_angle_rad, 0) != HAL_OK)
    {
        return 0U;
    }

    if (FOC_WritePwmDuty(&s_motor_pwm_output[motor_id],
                         motor->control.duty_u,
                         motor->control.duty_v,
                         motor->control.duty_w) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  清空FOC offset自动校准的当前采样累计。
 * @param  None
 * @retval None
 * @note   每进入一个吸附点采样阶段前调用。
 */
static void Motor_Core_FocOffsetCalibResetSample(void)
{
    s_foc_offset_calib.sample_count = 0U;
    s_foc_offset_calib.sample_sum = 0.0f;
    s_foc_offset_calib.sample_tick = HAL_GetTick();
}

/**
 * @brief  累计当前编码器机械角并在满样本后输出平均值。
 * @param  avg_mech_angle: 平均机械角输出指针，单位rad。
 * @retval 1表示采样完成，0表示仍在采样。
 * @note   读取SPI编码器最近一次DMA更新后的累计机械角缓存，不阻塞SPI通信。
 */
static uint8_t Motor_Core_FocOffsetCalibAccumulate(float *avg_mech_angle)
{
    encoder_id_t encoder_id;
    uint32_t now_tick;

    if (avg_mech_angle == 0)
    {
        return 0U;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - s_foc_offset_calib.sample_tick) < MOTOR_FOC_OFFSET_CALIB_SAMPLE_MS)
    {
        return 0U;
    }

    s_foc_offset_calib.sample_tick = now_tick;
    encoder_id = Motor_Core_GetEncoderId(s_foc_offset_calib.motor_id);
    s_foc_offset_calib.sample_sum += SPI_Encoder_Get_Total_Angle(encoder_id);
    s_foc_offset_calib.sample_count++;

    if (s_foc_offset_calib.sample_count < MOTOR_FOC_OFFSET_CALIB_SAMPLE_COUNT)
    {
        return 0U;
    }

    *avg_mech_angle = s_foc_offset_calib.sample_sum / (float)MOTOR_FOC_OFFSET_CALIB_SAMPLE_COUNT;
    return 1U;
}

#if (MOTOR_FOC_OFFSET_CALIB_TWO_POINT_ENABLE != 0U)
/**
 * @brief  计算 target - source 的最短有符号角度差。
 * @param  target_rad: 目标角度，单位rad。
 * @param  source_rad: 源角度，单位rad。
 * @retval 最短角度差，范围约为 -pi ~ +pi。
 * @note   用于两个offset的圆周平均，避免359度和1度普通平均得到180度。
 */
static float Motor_Core_AngleDiffSigned(float target_rad, float source_rad)
{
    float diff;

    diff = FOC_WrapAngle0To2Pi(target_rad - source_rad);
    if (diff > M_PI_F)
    {
        diff -= FOC_TWO_PI_F;
    }

    return diff;
}
#endif

/**
 * @brief  结束FOC offset自动校准。
 * @param  success: 1表示校准成功，0表示校准失败。
 * @retval None
 * @note   无论成功失败，都会输出中性PWM并退出校准任务；失败时不会覆盖旧offset。
 */
static void Motor_Core_FocOffsetCalibFinish(uint8_t success)
{
    s_foc_offset_calib.valid = (success != 0U) ? 1U : 0U;
    (void)Motor_Core_OutputNeutralPwm(s_foc_offset_calib.motor_id);
    (void)Motor_Core_StopPwmOutput(s_foc_offset_calib.motor_id);
    s_foc_offset_calib.state = MOTOR_FOC_OFFSET_CALIB_IDLE;
}

/**
 * @brief  根据ADC采样相选择对应零漂电压。
 * @param  motor: 电机对象指针。
 * @param  phase: ADC采样相，取值ADC_APP_PHASE_A/B/C。
 * @retval 对应ADC采样通道的零漂电压，单位V。
 * @note   使用方式：软件调整电流采样相序时，raw和零漂必须一起换相，
 *         否则静态电流会因为零漂通道不匹配产生额外偏置。
 */
static float Motor_Core_GetPhaseZeroOffsetV(const motor_t *motor, adc_app_phase_t phase)
{
    if (motor == 0)
    {
        return 0.0f;
    }

    switch (phase)
    {
    case ADC_APP_PHASE_A:
        return motor->calib.ia_zero_offset_v;

    case ADC_APP_PHASE_B:
        return motor->calib.ib_zero_offset_v;

    case ADC_APP_PHASE_C:
        return motor->calib.ic_zero_offset_v;

    default:
        return 0.0f;
    }
}

/**
 * @brief  将ADC原始值转换为相电流。
 * @param  raw: ADC原始计数。
 * @param  zero_offset_v: 零漂修正电压，单位V。
 * @retval 相电流，单位A。
 * @note   公式：(采样电压 - 中点偏置 - 零漂修正) / (采样电阻 * 放大倍数)。
 */
static float Motor_Core_RawToCurrentA(uint16_t raw, float zero_offset_v)
{
    float voltage;

    voltage = ((float)raw * ADC_VREF_V) / ADC_MAX_COUNT;

    return (voltage - CURRENT_ADC_MID_V - zero_offset_v) / (CURRENT_SHUNT_OHM * CURRENT_AMP_GAIN);
}

/**
 * @brief  初始化单个电机对象的基础字段。
 * @param  motor: 电机对象指针。
 * @param  motor_id: 电机编号。
 * @retval None
 * @note   该函数只初始化软件状态和默认参数，不访问任何硬件外设。
 */
static void Motor_Core_InitOne(motor_t *motor, uint8_t motor_id)
{
    if (motor == 0)
    {
        return;
    }

    motor->motor_id = motor_id;

    switch (motor_id)
    {
    case 0U:
        motor->current_adc_id = (uint8_t)ADC_APP_MOTOR_ADC4;
        motor->encoder_id = (uint8_t)encoder_id_1;
        motor->pwm_arr = TIM1->ARR;
        motor->adc_guard_ccr = (motor->pwm_arr * MOTOR_ADC_TRIGGER_GUARD_PERCENT) / 100U;
        motor->pwm_ccr_u = &TIM1->CCR1;
        motor->pwm_ccr_v = &TIM1->CCR2;
        motor->pwm_ccr_w = &TIM1->CCR3;
        motor->adc_trigger_ccr = &TIM1->CCR4;
        break;

    case 1U:
        motor->current_adc_id = (uint8_t)ADC_APP_MOTOR_ADC3;
        motor->encoder_id = (uint8_t)encoder_id_2;
        motor->pwm_arr = TIM8->ARR;
        motor->adc_guard_ccr = (motor->pwm_arr * MOTOR_ADC_TRIGGER_GUARD_PERCENT) / 100U;
        motor->pwm_ccr_u = &TIM8->CCR1;
        motor->pwm_ccr_v = &TIM8->CCR2;
        motor->pwm_ccr_w = &TIM8->CCR3;
        motor->adc_trigger_ccr = &TIM8->CCR4;
        break;

    case 2U:
    default:
        motor->current_adc_id = (uint8_t)ADC_APP_MOTOR_ADC1;
        motor->encoder_id = (uint8_t)encoder_id_3;
        motor->pwm_arr = TIM20->ARR;
        motor->adc_guard_ccr = (motor->pwm_arr * MOTOR_ADC_TRIGGER_GUARD_PERCENT) / 100U;
        motor->pwm_ccr_u = &TIM20->CCR1;
        motor->pwm_ccr_v = &TIM20->CCR2;
        motor->pwm_ccr_w = &TIM20->CCR4;
        motor->adc_trigger_ccr = &TIM20->CCR3;
        break;
    }

    motor->pwm_ccr_u_last = Motor_Core_DutyToTimCompareFast(FOC_DUTY_CENTER, motor->pwm_arr);
    motor->pwm_ccr_v_last = motor->pwm_ccr_u_last;
    motor->pwm_ccr_w_last = motor->pwm_ccr_u_last;

    motor->mode = MOTOR_MODE_IDLE;
    motor->run_state = MOTOR_STATE_OFF;
    motor->enable_cmd = 0U;
    motor->start_cmd = 0U;

    Motor_Core_LoadDefaultPid(motor);
    Motor_Core_LoadDefaultCalib(motor, motor_id);
}

/**
 * @brief  从Config.h装载四个控制环PID默认参数。
 * @param  motor: 电机对象指针。
 * @retval None
 * @note   control_pid模块只负责单个PID计算；这里负责把电机项目中的id/iq/speed/pos参数装入motor_t。
 *         当前三电机共用公共默认参数；如果后续三路PID不同，可在本函数内按motor_id分支覆盖。
 */
static void Motor_Core_LoadDefaultPid(motor_t *motor)
{
    control_pid_param_t param;
    float id_kp = ID_PID_KP_DEFAULT;
    float id_ki = ID_PID_KI_DEFAULT;
    float id_kd = ID_PID_KD_DEFAULT;
    float id_int_lim = ID_PID_INT_LIM_DEFAULT;
    float id_out_lim = ID_PID_OUT_LIM_NORM_DEFAULT;
    float iq_kp = IQ_PID_KP_DEFAULT;
    float iq_ki = IQ_PID_KI_DEFAULT;
    float iq_kd = IQ_PID_KD_DEFAULT;
    float iq_int_lim = IQ_PID_INT_LIM_DEFAULT;
    float iq_out_lim = IQ_PID_OUT_LIM_NORM_DEFAULT;

    if (motor == 0)
    {
        return;
    }

    if (motor->motor_id == 2U)
    {
        id_kp = M3_ID_PID_KP_DEFAULT;
        id_ki = M3_ID_PID_KI_DEFAULT;
        id_kd = M3_ID_PID_KD_DEFAULT;
        id_int_lim = M3_ID_PID_INT_LIM_DEFAULT;
        id_out_lim = M3_ID_PID_OUT_LIM_NORM_DEFAULT;

        iq_kp = M3_IQ_PID_KP_DEFAULT;
        iq_ki = M3_IQ_PID_KI_DEFAULT;
        iq_kd = M3_IQ_PID_KD_DEFAULT;
        iq_int_lim = M3_IQ_PID_INT_LIM_DEFAULT;
        iq_out_lim = M3_IQ_PID_OUT_LIM_NORM_DEFAULT;
    }

    Motor_Core_InitPidParam(&param,
                            id_kp,
                            id_ki,
                            id_kd,
                            id_int_lim,
                            id_out_lim,
                            CURRENT_DEADBAND_A);
    (void)Control_PID_Init(&motor->control.id_pid, &param);

    Motor_Core_InitPidParam(&param,
                            iq_kp,
                            iq_ki,
                            iq_kd,
                            iq_int_lim,
                            iq_out_lim,
                            CURRENT_DEADBAND_A);
    (void)Control_PID_Init(&motor->control.iq_pid, &param);

    Motor_Core_InitPidParam(&param,
                            SPEED_PID_KP_DEFAULT,
                            SPEED_PID_KI_DEFAULT,
                            SPEED_PID_KD_DEFAULT,
                            SPEED_PID_INT_LIM_DEFAULT,
                            SPEED_PID_OUT_LIM_A_DEFAULT,
                            SPEED_DEADBAND_RAD_S);
    (void)Control_PID_Init(&motor->control.speed_pid, &param);

    Motor_Core_InitPidParam(&param,
                            POS_PID_KP_DEFAULT,
                            POS_PID_KI_DEFAULT,
                            POS_PID_KD_DEFAULT,
                            POS_PID_INT_LIM_DEFAULT,
                            POS_PID_OUT_LIM_RAD_S_DEFAULT,
                            POS_DEADBAND_RAD);
    (void)Control_PID_Init(&motor->control.pos_pid, &param);
}

/**
 * @brief  从Config.h装载单个电机的默认校准参数。
 * @param  motor: 电机对象指针。
 * @param  motor_id: 电机编号。
 * @retval None
 * @note   当前只装载静态默认值；上电零漂校准完成后可覆盖这些字段。
 */
static void Motor_Core_LoadDefaultCalib(motor_t *motor, uint8_t motor_id)
{
    if (motor == 0)
    {
        return;
    }

    motor->calib.current_limit = MOTOR_CURRENT_MAX_A;
    motor->calib.temp_limit = TEMP_PROTECT_C;

    switch (motor_id)
    {
    case 0U:
        motor->calib.ia_zero_offset_v = M1_IA_ZERO_CORRECT_V;
        motor->calib.ib_zero_offset_v = M1_IB_ZERO_CORRECT_V;
        motor->calib.ic_zero_offset_v = M1_IC_ZERO_CORRECT_V;
        motor->calib.encoder_zero_offset = ENCODER_ZERO_OFFSET_M1_RAD;
        break;

    case 1U:
        motor->calib.ia_zero_offset_v = M2_IA_ZERO_CORRECT_V;
        motor->calib.ib_zero_offset_v = M2_IB_ZERO_CORRECT_V;
        motor->calib.ic_zero_offset_v = M2_IC_ZERO_CORRECT_V;
        motor->calib.encoder_zero_offset = ENCODER_ZERO_OFFSET_M2_RAD;
        break;

    case 2U:
        motor->calib.ia_zero_offset_v = M3_IA_ZERO_CORRECT_V;
        motor->calib.ib_zero_offset_v = M3_IB_ZERO_CORRECT_V;
        motor->calib.ic_zero_offset_v = M3_IC_ZERO_CORRECT_V;
        motor->calib.encoder_zero_offset = ENCODER_ZERO_OFFSET_M3_RAD;
        break;

    default:
        break;
    }
}

/**
 * @brief  生成一组PID参数。
 * @param  param: PID参数结构体指针。
 * @param  kp/ki/kd: PID系数。
 * @param  integral_limit: 积分对称限幅，实际范围为[-integral_limit, +integral_limit]。
 * @param  output_limit: 输出对称限幅，实际范围为[-output_limit, +output_limit]。
 * @param  deadband: 误差死区。
 * @retval None
 * @note   当前四个环都使用对称限幅，后续如果某个环需要非对称限幅，再单独扩展。
 */
static void Motor_Core_InitPidParam(control_pid_param_t *param,
                                    float kp,
                                    float ki,
                                    float kd,
                                    float integral_limit,
                                    float output_limit,
                                    float deadband)
{
    if (param == 0)
    {
        return;
    }

    param->kp = kp;
    param->ki = ki;
    param->kd = kd;
    param->integral_min = -integral_limit;
    param->integral_max = integral_limit;
    param->out_min = -output_limit;
    param->out_max = output_limit;
    param->deadband = deadband;
    param->enable_integral = (ki != 0.0f) ? 1U : 0U;
    param->enable_derivative = (kd != 0.0f) ? 1U : 0U;
    param->enable_anti_windup = 1U;
}
