#include "foc_drive.h"

#include <math.h>
#include <stddef.h>

#include "arm_math.h"

#define FOC_FLOAT_EPSILON        1.0e-6f
#define FOC_TEMP_PWM_PHASE_U     0x01U       // 临时隔离测试：U/A相PWM启动使能位
#define FOC_TEMP_PWM_PHASE_V     0x02U       // 临时隔离测试：V/B相PWM启动使能位
#define FOC_TEMP_PWM_PHASE_W     0x04U       // 临时隔离测试：W/C相PWM启动使能位
#define FOC_TEMP_PWM_PHASE_MASK  0x07U       // 临时隔离测试：0x01只A，0x02只B，0x04只C，0x07三相全启动

static float FOC_LimitFloat(float value, float min_value, float max_value);       // 单个浮点数限幅
static uint8_t FOC_LimitVectorNorm(float *x, float *y, float *scale);            // 二维归一化电压矢量幅值限幅
static uint8_t FOC_GetSvpwmSector(float u_alpha_norm, float u_beta_norm);        // 三辅助量符号法判断SVPWM扇区
static void FOC_SetZeroSvpwmResult(foc_svpwm_t *svpwm, uint8_t sector);          // 构造零矢量SVPWM安全结果
static void FOC_FillVofaData(foc_vofa_data_t *vofa_out,
                             float vd_cmd,
                             float vq_cmd,
                             float vd_limited,
                             float vq_limited,
                             float elec_angle_rad,
                             const foc_alpha_beta_t *alpha_beta,
                             const foc_svpwm_t *svpwm);                         // 填充VOFA观测数据
static uint32_t FOC_DutyToCompare(const TIM_HandleTypeDef *htim, float duty);    // duty转换为TIM CCR比较值
static HAL_StatusTypeDef FOC_StartPwmChannel(TIM_HandleTypeDef *htim, uint32_t channel); // 启动单相PWM和互补PWM
static HAL_StatusTypeDef FOC_StopPwmChannel(TIM_HandleTypeDef *htim, uint32_t channel);  // 停止单相PWM和互补PWM

/**
 * @brief  对单个浮点数做上下限保护。
 * @param  value      输入值。
 * @param  min_value  允许的最小值。
 * @param  max_value  允许的最大值。
 * @return 限幅后的结果。
 * @note   本文件内部工具函数，主要用于 duty、compare 等保护；如果 min > max，直接返回原值，避免隐藏配置错误。
 */
static float FOC_LimitFloat(float value, float min_value, float max_value)
{
    if (min_value > max_value)
    {
        return value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/**
 * @brief  对二维归一化电压矢量做幅值限幅。
 * @param  x      输入/输出的第一轴分量，例如 vd 或 u_alpha。
 * @param  y      输入/输出的第二轴分量，例如 vq 或 u_beta。
 * @param  scale  输出缩放系数，1表示未缩放，小于1表示发生限幅。
 * @return 0表示未限幅，1表示发生限幅。
 * @note   第一版 FOC 没有实时母线电压采样，所以这里用归一化电压限幅保护 SVPWM 在线性区附近工作。
 */
static uint8_t FOC_LimitVectorNorm(float *x, float *y, float *scale)
{
    float mag_sq;
    float mag;
    const float limit_sq = FOC_VOLTAGE_NORM_LIMIT * FOC_VOLTAGE_NORM_LIMIT;

    if ((x == NULL) || (y == NULL) || (scale == NULL))
    {
        return 0U;
    }

    *scale = 1.0f;
    mag_sq = (*x * *x) + (*y * *y);

    if (mag_sq <= limit_sq)
    {
        return 0U;
    }

    if ((arm_sqrt_f32(mag_sq, &mag) != ARM_MATH_SUCCESS) || (mag <= FOC_FLOAT_EPSILON))
    {
        *x = 0.0f;
        *y = 0.0f;
        *scale = 0.0f;
        return 1U;
    }

    *scale = FOC_VOLTAGE_NORM_LIMIT / mag;
    *x *= *scale;
    *y *= *scale;

    return 1U;
}

/**
 * @brief  使用 CMSIS-DSP 计算角度的 sin/cos。
 * @param  angle_rad  输入角度，单位 rad。
 * @param  sin_theta  输出 sin(angle)，可以传 NULL 表示不需要。
 * @param  cos_theta  输出 cos(angle)，可以传 NULL 表示不需要。
 * @note   外部接口统一使用 rad；本函数内部会先把角度归一化到 0~2pi，避免角度长期累加后数值过大。
 */
void FOC_CalcSinCos(float angle_rad, float *sin_theta, float *cos_theta)
{
    const float angle_wrapped = FOC_WrapAngle0To2Pi(angle_rad);

    if (sin_theta != NULL)
    {
        *sin_theta = arm_sin_f32(angle_wrapped);
    }

    if (cos_theta != NULL)
    {
        *cos_theta = arm_cos_f32(angle_wrapped);
    }
}

/**
 * @brief  使用三辅助量符号法判断 SVPWM 扇区。
 * @param  u_alpha_norm  alpha轴归一化电压分量。
 * @param  u_beta_norm   beta轴归一化电压分量。
 * @return SVPWM扇区编号，范围1~6；返回0表示零矢量或异常边界。
 * @note   A/B/C 只用于扇区判断，后续 T1/T2 使用 X/Y/Z 辅助量计算，二者不要混用。
 */
static uint8_t FOC_GetSvpwmSector(float u_alpha_norm, float u_beta_norm)
{
    uint8_t code = 0U;
    const float a = u_beta_norm;
    const float b = (FOC_SQRT3_BY_2_F * u_alpha_norm) - (FOC_ONE_BY_TWO_F * u_beta_norm);
    const float c = (-FOC_SQRT3_BY_2_F * u_alpha_norm) - (FOC_ONE_BY_TWO_F * u_beta_norm);

    code |= (a >= 0.0f) ? 1U : 0U;
    code |= (b >= 0.0f) ? 2U : 0U;
    code |= (c >= 0.0f) ? 4U : 0U;

    switch (code)
    {
        case 3U:
            return 1U;

        case 1U:
            return 2U;

        case 5U:
            return 3U;

        case 4U:
            return 4U;

        case 6U:
            return 5U;

        case 2U:
            return 6U;

        default:
            return FOC_SVPWM_SECTOR_INVALID;
    }
}

/**
 * @brief  将 SVPWM 结果设置为零矢量安全结果。
 * @param  svpwm   需要写入的 SVPWM 结果结构体。
 * @param  sector  需要保留的扇区编号，通常为0。
 * @note   用于输入为零矢量、扇区异常或保护兜底时，让三相 duty 回到中间值。
 */
static void FOC_SetZeroSvpwmResult(foc_svpwm_t *svpwm, uint8_t sector)
{
    const float duty_center = FOC_LimitFloat(FOC_DUTY_CENTER, PWM_DUTY_MIN, PWM_DUTY_MAX);

    if (svpwm == NULL)
    {
        return;
    }

    svpwm->duty_u = duty_center;
    svpwm->duty_v = duty_center;
    svpwm->duty_w = duty_center;

    svpwm->t1 = 0.0f;
    svpwm->t2 = 0.0f;
    svpwm->t0 = 1.0f;

    svpwm->voltage_scale = 1.0f;
    svpwm->sector = sector;
    svpwm->voltage_limited = 0U;
}

/**
 * @brief  填充 VOFA 观测数据结构体。
 * @param  vofa_out        VOFA观测数据输出指针；传 NULL 表示不需要保存观测数据。
 * @param  vd_cmd          d轴归一化电压原始输入。
 * @param  vq_cmd          q轴归一化电压原始输入。
 * @param  vd_limited      d轴归一化电压限幅后值。
 * @param  vq_limited      q轴归一化电压限幅后值。
 * @param  elec_angle_rad  本次计算使用的电角度，单位 rad。
 * @param  alpha_beta      逆Park输出的 alpha/beta 归一化电压。
 * @param  svpwm           SVPWM完整计算结果。
 * @note   这里把 uint8_t 类型的 sector/limited 转成 float，是为了后面能直接通过 VOFA JustFloat 发送。
 */
static void FOC_FillVofaData(foc_vofa_data_t *vofa_out,
                             float vd_cmd,
                             float vq_cmd,
                             float vd_limited,
                             float vq_limited,
                             float elec_angle_rad,
                             const foc_alpha_beta_t *alpha_beta,
                             const foc_svpwm_t *svpwm)
{
    if ((vofa_out == NULL) || (alpha_beta == NULL) || (svpwm == NULL))
    {
        return;
    }

    vofa_out->vd_cmd = vd_cmd;
    vofa_out->vq_cmd = vq_cmd;
    vofa_out->vd_limited = vd_limited;
    vofa_out->vq_limited = vq_limited;

    vofa_out->v_alpha = alpha_beta->alpha;
    vofa_out->v_beta = alpha_beta->beta;
    vofa_out->elec_angle_rad = elec_angle_rad;

    vofa_out->duty_u = svpwm->duty_u;
    vofa_out->duty_v = svpwm->duty_v;
    vofa_out->duty_w = svpwm->duty_w;

    vofa_out->t1 = svpwm->t1;
    vofa_out->t2 = svpwm->t2;
    vofa_out->t0 = svpwm->t0;

    vofa_out->sector = (float)svpwm->sector;
    vofa_out->voltage_scale = svpwm->voltage_scale;
    vofa_out->voltage_limited = (float)svpwm->voltage_limited;

}

/**
 * @brief  将 0~1 占空比转换为 TIM CCR 比较值。
 * @param  htim  PWM定时器句柄。
 * @param  duty  PWM占空比，函数内部会按 PWM_DUTY_MIN/PWM_DUTY_MAX 再限幅一次。
 * @return 写入 CCR 的 compare 数值。
 * @note   这里只做 duty 到 ARR 的线性换算，不启动 PWM，也不改变定时器配置。
 */
static uint32_t FOC_DutyToCompare(const TIM_HandleTypeDef *htim, float duty)
{
    float compare_f;
    const float duty_limited = FOC_LimitFloat(duty, PWM_DUTY_MIN, PWM_DUTY_MAX);
    uint32_t arr;

    if ((htim == NULL) || (htim->Instance == NULL))
    {
        return 0U;
    }

    arr = htim->Instance->ARR;
    compare_f = duty_limited * (float)arr;
    compare_f = FOC_LimitFloat(compare_f, 0.0f, (float)arr);

    return (uint32_t)(compare_f + 0.5f);
}

/**
 * @brief  启动单个 PWM 通道及其互补输出。
 * @param  htim     PWM定时器句柄。
 * @param  channel  TIM通道，例如 TIM_CHANNEL_1。
 * @return HAL_OK表示启动成功，否则返回 HAL_ERROR。
 * @note   若互补输出启动失败，会把已经启动的主输出关掉，避免半启动状态。
 */
static HAL_StatusTypeDef FOC_StartPwmChannel(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim == NULL)
    {
        return HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(htim, channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_TIMEx_PWMN_Start(htim, channel) != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(htim, channel);
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  停止单个 PWM 通道及其互补输出。
 * @param  htim     PWM定时器句柄。
 * @param  channel  TIM通道，例如 TIM_CHANNEL_1。
 * @return HAL_OK表示停止成功；只要主输出或互补输出任一停止失败，就返回 HAL_ERROR。
 * @note   停止时会尽量同时关闭主输出和互补输出，不会因为其中一个失败就跳过另一个。
 */
static HAL_StatusTypeDef FOC_StopPwmChannel(TIM_HandleTypeDef *htim, uint32_t channel)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (htim == NULL)
    {
        return HAL_ERROR;
    }

    if (HAL_TIMEx_PWMN_Stop(htim, channel) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    if (HAL_TIM_PWM_Stop(htim, channel) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    return status;
}

/**
 * @brief  将角度归一化到 0~2pi。
 * @param  angle_rad  输入角度，单位 rad，可以为负数或超过 2pi。
 * @return 归一化后的角度，范围 [0, 2pi)。
 * @note   后续 Park/InvPark 会调用它；这里用 fmodf 做浮点取余，避免大角度输入时反复循环减 2pi。
 */
float FOC_WrapAngle0To2Pi(float angle_rad)
{
    if (!isfinite(angle_rad))
    {
        return 0.0f;
    }

    angle_rad = fmodf(angle_rad, FOC_TWO_PI_F);

    if (angle_rad < 0.0f)
    {
        angle_rad += FOC_TWO_PI_F;
    }

    if (angle_rad >= FOC_TWO_PI_F)
    {
        angle_rad -= FOC_TWO_PI_F;
    }

    return angle_rad;
}

/**
 * @brief  Clarke 变换，将三相电流转换到 alpha/beta 静止坐标系。
 * @param  ia  U/A相电流，单位 A。
 * @param  ib  V/B相电流，单位 A。
 * @param  ic  W/C相电流，单位 A。
 * @return alpha/beta 电流分量，单位仍为 A。
 * @note   使用三相幅值保持形式，即使 ia+ib+ic 不完全为0，也能保留三相采样误差对结果的影响，方便后续调试电流采样质量。
 */
foc_alpha_beta_t FOC_Clarke(float ia, float ib, float ic)
{
    foc_alpha_beta_t output;

    output.alpha = FOC_TWO_BY_THREE_F * (ia - (FOC_ONE_BY_TWO_F * ib) - (FOC_ONE_BY_TWO_F * ic));
    output.beta = FOC_ONE_BY_SQRT3_F * (ib - ic);

    return output;
}

/**
 * @brief  Park 变换，将 alpha/beta 电流转换到 d/q 旋转坐标系。
 * @param  i_alpha         alpha轴电流，单位 A。
 * @param  i_beta          beta轴电流，单位 A。
 * @param  elec_angle_rad  电角度，单位 rad。
 * @return d/q 电流分量，单位仍为 A。
 * @note   后续电流环会用 id/iq 做反馈；电角度方向需要和编码器方向、电机相序保持一致，否则 iq 符号会异常。
 */
foc_dq_t FOC_Park(float i_alpha, float i_beta, float elec_angle_rad)
{
    float sin_theta;
    float cos_theta;

    FOC_CalcSinCos(elec_angle_rad, &sin_theta, &cos_theta);

    return FOC_ParkSinCos(i_alpha, i_beta, sin_theta, cos_theta);
}

/**
 * @brief  Park 变换，将 alpha/beta 电流转换到 d/q 旋转坐标系。
 * @param  i_alpha    alpha轴电流，单位 A。
 * @param  i_beta     beta轴电流，单位 A。
 * @param  sin_theta  电角度sin值。
 * @param  cos_theta  电角度cos值。
 * @return d/q 电流分量，单位仍为 A。
 * @note   用于快环内复用同一组sin/cos，避免Park和InvPark重复计算三角函数。
 */
foc_dq_t FOC_ParkSinCos(float i_alpha, float i_beta, float sin_theta, float cos_theta)
{
    foc_dq_t output;

    output.d = (i_alpha * cos_theta) + (i_beta * sin_theta);
    output.q = (-i_alpha * sin_theta) + (i_beta * cos_theta);

    return output;
}

/**
 * @brief  逆 Park 变换，将 d/q 归一化电压转换到 alpha/beta 静止坐标系。
 * @param  vd_norm         d轴归一化电压指令。
 * @param  vq_norm         q轴归一化电压指令。
 * @param  elec_angle_rad  电角度，单位 rad。
 * @return alpha/beta 归一化电压分量。
 * @note   第一版没有实时 Vbus 补偿，因此这里的 vd/vq 不是物理电压 V，而是相对于母线电压的归一化控制量。
 */
foc_alpha_beta_t FOC_InvPark(float vd_norm, float vq_norm, float elec_angle_rad)
{
    float sin_theta;
    float cos_theta;

    FOC_CalcSinCos(elec_angle_rad, &sin_theta, &cos_theta);

    return FOC_InvParkSinCos(vd_norm, vq_norm, sin_theta, cos_theta);
}

/**
 * @brief  逆 Park 变换，将 d/q 归一化电压转换到 alpha/beta 静止坐标系。
 * @param  vd_norm    d轴归一化电压指令。
 * @param  vq_norm    q轴归一化电压指令。
 * @param  sin_theta  电角度sin值。
 * @param  cos_theta  电角度cos值。
 * @return alpha/beta 归一化电压分量。
 * @note   用于快环内复用同一组sin/cos，控制含义和FOC_InvPark()一致。
 */
foc_alpha_beta_t FOC_InvParkSinCos(float vd_norm, float vq_norm, float sin_theta, float cos_theta)
{
    foc_alpha_beta_t output;

    output.alpha = (vd_norm * cos_theta) - (vq_norm * sin_theta);
    output.beta = (vd_norm * sin_theta) + (vq_norm * cos_theta);

    return output;
}

/**
 * @brief  根据 alpha/beta 归一化电压计算 SVPWM 占空比。
 * @param  u_alpha_norm  alpha轴归一化电压分量。
 * @param  u_beta_norm   beta轴归一化电压分量。
 * @param  svpwm_out     SVPWM计算结果输出指针，不能为 NULL。
 * @return HAL_OK表示计算完成；svpwm_out为空时返回 HAL_ERROR。
 * @note   本函数只做数学计算，不写 TIM CCR，不启动 PWM。扇区使用 A/B/C 符号法；作用时间使用 X/Y/Z 映射表。
 */
HAL_StatusTypeDef FOC_SVPWM(float u_alpha_norm, float u_beta_norm, foc_svpwm_t *svpwm_out)
{
    float x;
    float y;
    float z;
    float active_sum;
    float time_scale;
    float t0_half;
    float vector_scale = 1.0f;
    float u_alpha = u_alpha_norm;
    float u_beta = u_beta_norm;

    if (svpwm_out == NULL)
    {
        return HAL_ERROR;
    }

    FOC_SetZeroSvpwmResult(svpwm_out, FOC_SVPWM_SECTOR_INVALID);
    svpwm_out->voltage_limited = FOC_LimitVectorNorm(&u_alpha, &u_beta, &vector_scale);
    svpwm_out->voltage_scale = vector_scale;

    if (((u_alpha * u_alpha) + (u_beta * u_beta)) <= (FOC_FLOAT_EPSILON * FOC_FLOAT_EPSILON))
    {
        return HAL_OK;
    }

    svpwm_out->sector = FOC_GetSvpwmSector(u_alpha, u_beta);
    if ((svpwm_out->sector < FOC_SVPWM_SECTOR_MIN) || (svpwm_out->sector > FOC_SVPWM_SECTOR_MAX))
    {
        return HAL_OK;
    }

    /*
     * X/Y/Z 是有效矢量作用时间的辅助量，已经把 Udc 和 Ts 归一化掉。
     * 这里的 T1/T2 映射必须和 FOC_GetSvpwmSector() 的扇区编号保持一致。
     */
    x = FOC_SQRT3_F * u_beta;
    y = (1.5f * u_alpha) + (FOC_SQRT3_BY_2_F * u_beta);
    z = (-1.5f * u_alpha) + (FOC_SQRT3_BY_2_F * u_beta);

    switch (svpwm_out->sector)
    {
        case 1U:
            svpwm_out->t1 = -z;
            svpwm_out->t2 = x;
            break;

        case 2U:
            svpwm_out->t1 = z;
            svpwm_out->t2 = y;
            break;

        case 3U:
            svpwm_out->t1 = x;
            svpwm_out->t2 = -y;
            break;

        case 4U:
            svpwm_out->t1 = -x;
            svpwm_out->t2 = z;
            break;

        case 5U:
            svpwm_out->t1 = -y;
            svpwm_out->t2 = -z;
            break;

        case 6U:
            svpwm_out->t1 = y;
            svpwm_out->t2 = -x;
            break;

        default:
            return HAL_OK;
    }

    if (svpwm_out->t1 < 0.0f)
    {
        svpwm_out->t1 = 0.0f;
    }

    if (svpwm_out->t2 < 0.0f)
    {
        svpwm_out->t2 = 0.0f;
    }

    active_sum = svpwm_out->t1 + svpwm_out->t2;
    if (active_sum > 1.0f)
    {
        time_scale = 1.0f / active_sum;
        svpwm_out->t1 *= time_scale;
        svpwm_out->t2 *= time_scale;
        svpwm_out->voltage_scale *= time_scale;
        svpwm_out->voltage_limited = 1U;
        active_sum = 1.0f;
    }

    svpwm_out->t0 = 1.0f - active_sum;
    t0_half = svpwm_out->t0 * FOC_ONE_BY_TWO_F;

    switch (svpwm_out->sector)
    {
        case 1U:
            svpwm_out->duty_u = svpwm_out->t1 + svpwm_out->t2 + t0_half;
            svpwm_out->duty_v = svpwm_out->t2 + t0_half;
            svpwm_out->duty_w = t0_half;
            break;

        case 2U:
            svpwm_out->duty_u = svpwm_out->t2 + t0_half;
            svpwm_out->duty_v = svpwm_out->t1 + svpwm_out->t2 + t0_half;
            svpwm_out->duty_w = t0_half;
            break;

        case 3U:
            svpwm_out->duty_u = t0_half;
            svpwm_out->duty_v = svpwm_out->t1 + svpwm_out->t2 + t0_half;
            svpwm_out->duty_w = svpwm_out->t2 + t0_half;
            break;

        case 4U:
            svpwm_out->duty_u = t0_half;
            svpwm_out->duty_v = svpwm_out->t2 + t0_half;
            svpwm_out->duty_w = svpwm_out->t1 + svpwm_out->t2 + t0_half;
            break;

        case 5U:
            svpwm_out->duty_u = svpwm_out->t2 + t0_half;
            svpwm_out->duty_v = t0_half;
            svpwm_out->duty_w = svpwm_out->t1 + svpwm_out->t2 + t0_half;
            break;

        case 6U:
            svpwm_out->duty_u = svpwm_out->t1 + svpwm_out->t2 + t0_half;
            svpwm_out->duty_v = t0_half;
            svpwm_out->duty_w = svpwm_out->t2 + t0_half;
            break;

        default:
            break;
    }

    svpwm_out->duty_u = FOC_LimitFloat(svpwm_out->duty_u, PWM_DUTY_MIN, PWM_DUTY_MAX);
    svpwm_out->duty_v = FOC_LimitFloat(svpwm_out->duty_v, PWM_DUTY_MIN, PWM_DUTY_MAX);
    svpwm_out->duty_w = FOC_LimitFloat(svpwm_out->duty_w, PWM_DUTY_MIN, PWM_DUTY_MAX);

    return HAL_OK;
}

/**
 * @brief  执行一次归一化电压开环 FOC 输出计算。
 * @param  motor           单电机对象指针；可以传 NULL，此时只计算 vofa_out，不写 motor->control。
 * @param  vd_norm         d轴归一化电压指令。
 * @param  vq_norm         q轴归一化电压指令。
 * @param  elec_angle_rad  当前电角度，单位 rad。
 * @param  vofa_out        VOFA观测数据输出指针；传 NULL 表示不保存观测数据。
 * @return HAL_OK表示计算完成；内部 SVPWM 计算失败时返回 HAL_ERROR。
 * @note   本函数只完成 vd/vq -> InvPark -> SVPWM -> motor->control 的数据链路，不写 CCR、不启动 PWM、不做 PID、不读编码器。
 */
HAL_StatusTypeDef FOC_RunVoltageNorm(motor_t *motor,
                                     float vd_norm,
                                     float vq_norm,
                                     float elec_angle_rad,
                                     foc_vofa_data_t *vofa_out)
{
    float sin_theta;
    float cos_theta;

    FOC_CalcSinCos(elec_angle_rad, &sin_theta, &cos_theta);

    return FOC_RunVoltageNormSinCos(motor,
                                    vd_norm,
                                    vq_norm,
                                    elec_angle_rad,
                                    sin_theta,
                                    cos_theta,
                                    vofa_out);
}

/**
 * @brief  使用已计算sin/cos执行一次归一化电压开环 FOC 输出计算。
 * @param  motor           单电机对象指针；可以传 NULL，此时只计算 vofa_out，不写 motor->control。
 * @param  vd_norm         d轴归一化电压指令。
 * @param  vq_norm         q轴归一化电压指令。
 * @param  elec_angle_rad  当前电角度，单位 rad，仅用于VOFA记录。
 * @param  sin_theta       当前电角度sin值。
 * @param  cos_theta       当前电角度cos值。
 * @param  vofa_out        VOFA观测数据输出指针；传 NULL 表示不保存观测数据。
 * @return HAL_OK表示计算完成；内部 SVPWM 计算失败时返回 HAL_ERROR。
 * @note   快环入口用本函数复用同一组sin/cos；限幅、SVPWM和motor->control写回逻辑保持和FOC_RunVoltageNorm()一致。
 */
HAL_StatusTypeDef FOC_RunVoltageNormSinCos(motor_t *motor,
                                           float vd_norm,
                                           float vq_norm,
                                           float elec_angle_rad,
                                           float sin_theta,
                                           float cos_theta,
                                           foc_vofa_data_t *vofa_out)
{
    foc_alpha_beta_t alpha_beta;
    foc_svpwm_t svpwm;
    HAL_StatusTypeDef status;
    float dq_scale = 1.0f;
    float vd = vd_norm;
    float vq = vq_norm;
    const uint8_t dq_limited = FOC_LimitVectorNorm(&vd, &vq, &dq_scale);

    alpha_beta = FOC_InvParkSinCos(vd, vq, sin_theta, cos_theta);
    status = FOC_SVPWM(alpha_beta.alpha, alpha_beta.beta, &svpwm);

    if (status != HAL_OK)
    {
        FOC_SetZeroSvpwmResult(&svpwm, FOC_SVPWM_SECTOR_INVALID);
    }

    if (dq_limited != 0U)
    {
        svpwm.voltage_limited = 1U;
        svpwm.voltage_scale *= dq_scale;
    }

    if (motor != NULL)
    {
        motor->control.vd_norm = vd;
        motor->control.vq_norm = vq;
        motor->control.v_alpha_norm = alpha_beta.alpha;
        motor->control.v_beta_norm = alpha_beta.beta;
        motor->control.duty_u = svpwm.duty_u;
        motor->control.duty_v = svpwm.duty_v;
        motor->control.duty_w = svpwm.duty_w;
    }

    FOC_FillVofaData(vofa_out, vd_norm, vq_norm, vd, vq, elec_angle_rad, &alpha_beta, &svpwm);

    return status;
}

/**
 * @brief  将三相 duty 写入 PWM 定时器 CCR。
 * @param  pwm     PWM输出映射，包含定时器句柄和 U/V/W 三相通道。
 * @param  duty_u  U相占空比，范围建议 0~1。
 * @param  duty_v  V相占空比，范围建议 0~1。
 * @param  duty_w  W相占空比，范围建议 0~1。
 * @return HAL_OK表示写入成功；pwm 或 htim 为空时返回 HAL_ERROR。
 * @note   本函数只更新 CCR，不启动 PWM、不打开 MOE、不控制 DRV 使能；真正输出必须由外部明确调用 FOC_StartPwmOutput()。
 */
HAL_StatusTypeDef FOC_WritePwmDuty(const foc_pwm_output_t *pwm,
                                   float duty_u,
                                   float duty_v,
                                   float duty_w)
{
    if ((pwm == NULL) || (pwm->htim == NULL))
    {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel_u, FOC_DutyToCompare(pwm->htim, duty_u));
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel_v, FOC_DutyToCompare(pwm->htim, duty_v));
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel_w, FOC_DutyToCompare(pwm->htim, duty_w));

    return HAL_OK;
}

/**
 * @brief  启动三相 PWM 主输出和互补输出。
 * @param  pwm  PWM输出映射，包含定时器句柄和 U/V/W 三相通道。
 * @return HAL_OK表示三相全部启动成功，否则返回 HAL_ERROR。
 * @note   建议只在状态机 READY/RUN 且 DRV/保护条件满足后调用；本函数不检查功率板安全状态，也不自动写入安全 duty。
 */
HAL_StatusTypeDef FOC_StartPwmOutput(const foc_pwm_output_t *pwm)
{
    if ((pwm == NULL) || (pwm->htim == NULL))
    {
        return HAL_ERROR;
    }

    if ((FOC_TEMP_PWM_PHASE_MASK & FOC_TEMP_PWM_PHASE_U) != 0U)
    {
        if (FOC_StartPwmChannel(pwm->htim, pwm->channel_u) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    if ((FOC_TEMP_PWM_PHASE_MASK & FOC_TEMP_PWM_PHASE_V) != 0U)
    {
        if (FOC_StartPwmChannel(pwm->htim, pwm->channel_v) != HAL_OK)
        {
            if ((FOC_TEMP_PWM_PHASE_MASK & FOC_TEMP_PWM_PHASE_U) != 0U)
            {
                (void)FOC_StopPwmChannel(pwm->htim, pwm->channel_u);
            }
            return HAL_ERROR;
        }
    }

    if ((FOC_TEMP_PWM_PHASE_MASK & FOC_TEMP_PWM_PHASE_W) != 0U)
    {
        if (FOC_StartPwmChannel(pwm->htim, pwm->channel_w) != HAL_OK)
        {
            if ((FOC_TEMP_PWM_PHASE_MASK & FOC_TEMP_PWM_PHASE_V) != 0U)
            {
                (void)FOC_StopPwmChannel(pwm->htim, pwm->channel_v);
            }
            if ((FOC_TEMP_PWM_PHASE_MASK & FOC_TEMP_PWM_PHASE_U) != 0U)
            {
                (void)FOC_StopPwmChannel(pwm->htim, pwm->channel_u);
            }
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/**
 * @brief  停止三相 PWM 主输出和互补输出。
 * @param  pwm  PWM输出映射，包含定时器句柄和 U/V/W 三相通道。
 * @return HAL_OK表示三相全部停止成功；任一通道停止失败则返回 HAL_ERROR。
 * @note   故障、停机、调试退出时可以调用；本函数会尽量停止三相所有输出，不因为某一路失败就提前退出。
 */
HAL_StatusTypeDef FOC_StopPwmOutput(const foc_pwm_output_t *pwm)
{
    HAL_StatusTypeDef status = HAL_OK;

    if ((pwm == NULL) || (pwm->htim == NULL))
    {
        return HAL_ERROR;
    }

    if (FOC_StopPwmChannel(pwm->htim, pwm->channel_u) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    if (FOC_StopPwmChannel(pwm->htim, pwm->channel_v) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    if (FOC_StopPwmChannel(pwm->htim, pwm->channel_w) != HAL_OK)
    {
        status = HAL_ERROR;
    }

    return status;
}
