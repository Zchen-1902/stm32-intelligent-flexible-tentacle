#include "control_pid.h"

/*
 * PID 控制模块实现
 *
 * 当前先保留文件框架，后续逐步补充：
 * 1. PID 参数结构体；
 * 2. PID 运行状态结构体；
 * 3. 初始化、复位、单步更新；
 * 4. 死区、积分限幅、输出限幅和抗积分饱和。
 */

#define CONTROL_PID_DEFAULT_OUT_MIN        (-1.0f)
#define CONTROL_PID_DEFAULT_OUT_MAX        (1.0f)
#define CONTROL_PID_DEFAULT_INTEGRAL_MIN   (-1.0f)
#define CONTROL_PID_DEFAULT_INTEGRAL_MAX   (1.0f)

static float Control_PID_Abs(float value);                                     // 返回浮点绝对值
static float Control_PID_Limit(float value, float min_value, float max_value);  // 将value限制在[min,max]
static uint8_t Control_PID_IsParamValid(const control_pid_param_t *param);      // 检查PID参数限幅是否有效

/**
 * @brief  填充一组安全默认PID参数。
 * @param  param: 参数结构体指针，不能为NULL。
 * @retval None
 * @note   使用方式：先调用本函数得到默认参数，再根据电流环/速度环/位置环修改kp/ki/kd和限幅。
 *         默认kp/ki/kd均为0，输出限幅为[-1,1]，积分默认关闭。
 */
void Control_PID_SetDefaultParam(control_pid_param_t *param)
{
    if (param == 0)
    {
        return;
    }

    param->kp = 0.0f;
    param->ki = 0.0f;
    param->kd = 0.0f;
    param->out_min = CONTROL_PID_DEFAULT_OUT_MIN;
    param->out_max = CONTROL_PID_DEFAULT_OUT_MAX;
    param->integral_min = CONTROL_PID_DEFAULT_INTEGRAL_MIN;
    param->integral_max = CONTROL_PID_DEFAULT_INTEGRAL_MAX;
    param->deadband = 0.0f;
    param->enable_integral = 0U;
    param->enable_derivative = 0U;
    param->enable_anti_windup = 1U;
}

/**
 * @brief  初始化PID对象。
 * @param  pid: PID对象指针，不能为NULL。
 * @param  param: PID参数指针；传NULL时使用默认参数。
 * @retval CONTROL_PID_OK表示成功，CONTROL_PID_PARAM_ERROR表示参数非法。
 * @note   使用方式：控制环启动前调用一次。初始化会清空积分、误差和输出状态。
 */
control_pid_status_t Control_PID_Init(control_pid_t *pid, const control_pid_param_t *param)
{
    control_pid_param_t default_param;

    if (pid == 0)
    {
        return CONTROL_PID_PARAM_ERROR;
    }

    if (param == 0)
    {
        Control_PID_SetDefaultParam(&default_param);
        param = &default_param;
    }

    if (Control_PID_IsParamValid(param) == 0U)
    {
        return CONTROL_PID_PARAM_ERROR;
    }

    pid->param = *param;
    Control_PID_Reset(pid);
    pid->initialized = 1U;

    return CONTROL_PID_OK;
}

/**
 * @brief  复位PID运行状态。
 * @param  pid: PID对象指针，不能为NULL。
 * @retval None
 * @note   使用方式：切换模式、清故障、重新使能电机时调用。
 *         本函数不会修改kp/ki/kd和限幅参数。
 */
void Control_PID_Reset(control_pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->ref = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->output_raw = 0.0f;
    pid->saturated = 0U;
}

/**
 * @brief  更新PID参数。
 * @param  pid: PID对象指针，不能为NULL。
 * @param  param: 新参数指针，不能为NULL。
 * @retval CONTROL_PID_OK表示成功，CONTROL_PID_PARAM_ERROR表示参数非法。
 * @note   使用方式：调参时调用。该函数只更新参数，不清除积分和历史误差。
 *         若需要清状态，请在本函数后调用Control_PID_Reset()。
 */
control_pid_status_t Control_PID_SetParam(control_pid_t *pid, const control_pid_param_t *param)
{
    if ((pid == 0) || (param == 0) || (Control_PID_IsParamValid(param) == 0U))
    {
        return CONTROL_PID_PARAM_ERROR;
    }

    pid->param = *param;
    pid->integral = Control_PID_Limit(pid->integral, pid->param.integral_min, pid->param.integral_max);
    pid->output = Control_PID_Limit(pid->output, pid->param.out_min, pid->param.out_max);

    return CONTROL_PID_OK;
}

/**
 * @brief  执行一次PID计算。
 * @param  pid: PID对象指针，不能为NULL。
 * @param  ref: 目标值。
 * @param  feedback: 反馈值。
 * @param  dt: 控制周期，单位s，必须大于0。
 * @param  output: 输出指针；可为NULL，为NULL时只更新pid->output。
 * @retval CONTROL_PID_OK表示成功，CONTROL_PID_PARAM_ERROR表示参数非法。
 * @note   使用方式：每个控制周期调用一次。
 *         第一版采用位置式PID，并在输出饱和时做简单抗积分饱和。
 */
control_pid_status_t Control_PID_Update(control_pid_t *pid,
                                        float ref,
                                        float feedback,
                                        float dt,
                                        float *output)
{
    float error;
    float integral_candidate;
    float output_candidate;
    uint8_t saturated;

    if ((pid == 0) || (dt <= 0.0f) || (pid->initialized == 0U))
    {
        return CONTROL_PID_PARAM_ERROR;
    }

    pid->ref = ref;
    pid->feedback = feedback;

    error = ref - feedback;
    if (Control_PID_Abs(error) < pid->param.deadband)
    {
        error = 0.0f;
    }

    pid->error = error;
    pid->proportional = pid->param.kp * error;

    integral_candidate = pid->integral;
    if (pid->param.enable_integral != 0U)
    {
        integral_candidate += pid->param.ki * error * dt;
        integral_candidate = Control_PID_Limit(integral_candidate,
                                               pid->param.integral_min,
                                               pid->param.integral_max);
    }
    else
    {
        /* 积分冻结：禁用积分时保留已有积分项参与输出，但不继续累加。 */
        integral_candidate = pid->integral;
    }

    if (pid->param.enable_derivative != 0U)
    {
        pid->derivative = pid->param.kd * (error - pid->last_error) / dt;
    }
    else
    {
        pid->derivative = 0.0f;
    }

    output_candidate = pid->proportional + integral_candidate + pid->derivative;
    pid->output_raw = output_candidate;
    pid->output = Control_PID_Limit(output_candidate, pid->param.out_min, pid->param.out_max);
    saturated = (pid->output != output_candidate) ? 1U : 0U;

    /*
     * 简单抗积分饱和：
     * 当输出已饱和，并且当前误差仍推动输出继续向饱和方向增大时，撤销本次积分。
     */
    if ((pid->param.enable_integral != 0U) &&
        (pid->param.enable_anti_windup != 0U) &&
        (saturated != 0U) &&
        (((pid->output >= pid->param.out_max) && (error > 0.0f)) ||
         ((pid->output <= pid->param.out_min) && (error < 0.0f))))
    {
        integral_candidate = pid->integral;
        output_candidate = pid->proportional + integral_candidate + pid->derivative;
        pid->output_raw = output_candidate;
        pid->output = Control_PID_Limit(output_candidate, pid->param.out_min, pid->param.out_max);
        saturated = (pid->output != output_candidate) ? 1U : 0U;
    }

    pid->integral = integral_candidate;
    pid->saturated = saturated;
    pid->last_error = error;

    if (output != 0)
    {
        *output = pid->output;
    }

    return CONTROL_PID_OK;
}

/**
 * @brief  获取最近一次PID输出。
 * @param  pid: PID对象指针。
 * @retval 最近一次限幅后的输出；pid为NULL时返回0。
 */
float Control_PID_GetOutput(const control_pid_t *pid)
{
    return (pid == 0) ? 0.0f : pid->output;
}

/**
 * @brief  获取当前积分项。
 * @param  pid: PID对象指针。
 * @retval 当前积分项；pid为NULL时返回0。
 */
float Control_PID_GetIntegral(const control_pid_t *pid)
{
    return (pid == 0) ? 0.0f : pid->integral;
}

/**
 * @brief  判断最近一次PID输出是否触发限幅。
 * @param  pid: PID对象指针。
 * @retval 1表示触发输出限幅，0表示未触发或pid为空。
 */
uint8_t Control_PID_IsSaturated(const control_pid_t *pid)
{
    return (pid == 0) ? 0U : pid->saturated;
}

/**
 * @brief  目标值斜坡限制。
 * @param  current: 当前已经输出给控制环的目标值。
 * @param  target: 最终希望达到的目标值。
 * @param  rate_limit: 最大变化速度，单位为“目标值单位/s”；小于等于0时表示不启用斜坡。
 * @param  dt: 调用周期，单位s，必须大于0。
 * @retval 本周期更新后的目标值。
 * @note   使用方式：每个控制周期调用一次，将返回值作为下一次current继续传入。
 *         本函数只限制目标变化速度，不等同于反馈滤波，也不会自动修改PID对象。
 */
float Control_Ramp_Update(float current,
                          float target,
                          float rate_limit,
                          float dt)
{
    float error;
    float max_step;

    if (dt <= 0.0f)
    {
        return current;
    }

    if (rate_limit <= 0.0f)
    {
        return target;
    }

    error = target - current;
    max_step = rate_limit * dt;

    if (error > max_step)
    {
        return current + max_step;
    }

    if (error < -max_step)
    {
        return current - max_step;
    }

    return target;
}

static float Control_PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Control_PID_Limit(float value, float min_value, float max_value)
{
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

static uint8_t Control_PID_IsParamValid(const control_pid_param_t *param)
{
    if (param == 0)
    {
        return 0U;
    }

    if ((param->out_min > param->out_max) || (param->integral_min > param->integral_max) || (param->deadband < 0.0f))
    {
        return 0U;
    }

    return 1U;
}
