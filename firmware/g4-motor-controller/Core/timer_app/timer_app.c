#include "timer_app.h"
#include "Config.h"

//******************************************timer_app内部参数*******************************************/
#define TIMER_APP_SPEED_CALC_PERIOD_US    (100000U / SPEED_CALCU_FREQ) // TIM7编码器速度计算周期，单位us，与tim.c中TIM7 Period公式保持一致
#define TIMER_APP_CONTROL_PENDING_MAX     1U       // 控制任务只保留一次待处理请求，不做连续补算
#define TIMER_APP_MOTOR_CONTROL_PENDING_MAX 1U     // 单电机PWM同步控制请求只保留一次，不做连续补算

//******************************************timer_app内部状态*******************************************/
static volatile uint8_t s_task_5ms_flag = 0U;           // TIM4基础5ms任务标志
static volatile uint8_t s_task_10ms_flag = 0U;          // TIM4分频10ms任务标志
static volatile uint8_t s_task_20ms_flag = 0U;          // TIM4分频20ms任务标志
static volatile uint8_t s_task_100ms_flag = 0U;         // TIM4分频100ms任务标志
static volatile uint8_t s_task_500ms_flag = 0U;         // TIM4分频500ms任务标志
static volatile uint8_t s_task_temp_sample_flag = 0U;   // TIM6温度采样任务标志（预留）
static volatile uint8_t s_task_speed_calc_flag = 0U;    // TIM7速度计算任务标志（预留）
static volatile uint8_t s_task_control_pending = 0U;    // TIM3控制任务积压次数，由main每处理一次递减
static volatile uint8_t s_motor_control_pending[MOTOR_COUNT] = {0U}; // ADC DMA完成后置位的单电机控制请求
static volatile uint32_t s_motor_update_irq_count[MOTOR_COUNT] = {0U}; // TIM1/TIM8/TIM20 update中断计数，用于验证PWM定时器是否运行
static volatile uint32_t s_speed_calc_time_us = 0U;     // TIM7节拍维护的编码器采样时间戳，允许uint32_t自然回绕

static uint8_t s_tim4_div_10ms = 0U;                    // TIM4到10ms的分频计数器
static uint8_t s_tim4_div_20ms = 0U;                    // TIM4到20ms的分频计数器
static uint8_t s_tim4_div_100ms = 0U;                   // TIM4到100ms的分频计数器
static uint8_t s_tim4_div_500ms = 0U;                   // TIM4到500ms的分频计数器

//******************************************timer_app内部函数声明*******************************************/
static void Timer_App_RuntimeCounterInit(void);         // 初始化DWT运行周期计数器
static void Timer_App_TIM3_Handler(void);               // TIM3控制任务调度事件处理
static void Timer_App_TIM4_Handler(void);               // TIM4周期事件处理
static void Timer_App_TIM6_Handler(void);               // TIM6周期事件处理（预留）
static void Timer_App_TIM7_Handler(void);               // TIM7周期事件处理（预留）

/**
 * @brief  初始化timer_app内部状态
 * @param  None
 * @retval None
 * @note   该函数只初始化软件任务标志与分频计数器，不负责初始化TIM硬件。
 *         建议在MX_TIM3_Init/MX_TIM4_Init/MX_TIM6_Init/MX_TIM7_Init之后调用。
 */
void Timer_App_Init(void)
{
    uint8_t i;

    Timer_App_RuntimeCounterInit();

    s_task_5ms_flag = 0U;
    s_task_10ms_flag = 0U;
    s_task_20ms_flag = 0U;
    s_task_100ms_flag = 0U;
    s_task_500ms_flag = 0U;
    s_task_temp_sample_flag = 0U;
    s_task_speed_calc_flag = 0U;
    s_task_control_pending = 0U;
    s_speed_calc_time_us = 0U;

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        s_motor_control_pending[i] = 0U;
        s_motor_update_irq_count[i] = 0U;
    }

    s_tim4_div_10ms = 0U;
    s_tim4_div_20ms = 0U;
    s_tim4_div_100ms = 0U;
    s_tim4_div_500ms = 0U;
}

/**
 * @brief  启动timer_app当前阶段使用的基础定时器中断
 * @param  None
 * @retval None
 * @note   TIM3当前作为预留控制任务节拍，暂不启动，避免误用1kHz作为电流环主节拍。
 *         TIM4作为5ms系统软件调度基础时基。
 *         TIM7作为编码器速度计算节拍，只在中断里置标志，
 *         编码器角度采样由对应PWM定时器事件触发SPI DMA。
 *         TIM6当前只作为ADC2温度采样TRGO触发源，不启用TIM6中断。
 */
void Timer_App_Start(void)
{
    // TIM3当前只保留配置，不启动；后续如作为中频任务节拍再开启
    (void)HAL_TIM_Base_Start_IT(&htim4);
    (void)HAL_TIM_Base_Start(&htim6);
    (void)HAL_TIM_Base_Start_IT(&htim7);
}

/**
 * @brief  启动三电机PWM定时器基础计数。
 * @param  None
 * @return HAL_OK表示TIM1/TIM8/TIM20均启动成功，否则返回HAL_ERROR。
 * @note   本函数只让TIM1/TIM8/TIM20开始计数，用于PWM和ADC硬件触发。
 *         当前闭环快环由ADC DMA完成回调触发，不再启用PWM update中断，避免高频空中断占用CPU。
 *         它不会启动PWM通道输出，不会启动互补PWM输出，也不会控制DRV使能。
 *         后续如果需要真实输出PWM，仍需由上层安全流程显式调用FOC_StartPwmOutput()。
 */
HAL_StatusTypeDef Timer_App_StartMotorControlTimers(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        return HAL_ERROR;
    }
    TIM1->CR1 |= TIM_CR1_CEN;

    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);
    if (HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_4) != HAL_OK)
    {
        return HAL_ERROR;
    }
    TIM8->CR1 |= TIM_CR1_CEN;

    __HAL_TIM_CLEAR_FLAG(&htim20, TIM_FLAG_UPDATE);
    __HAL_TIM_DISABLE_IT(&htim20, TIM_IT_UPDATE);
    if (HAL_TIM_OC_Start(&htim20, TIM_CHANNEL_3) != HAL_OK)
    {
        return HAL_ERROR;
    }
    TIM20->CR1 |= TIM_CR1_CEN;

    return HAL_OK;
}

/**
 * @brief  定时器周期到达回调转发函数
 * @param  htim: 触发周期回调的定时器句柄
 * @retval None
 * @note   该函数应在HAL_TIM_PeriodElapsedCallback中调用。
 *         TIM3只置位控制任务标志，不直接运行PID/FOC，避免中断负担过重。
 */
void Timer_App_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == NULL)
    {
        return;
    }

    if (htim->Instance == TIM1)
    {
        s_motor_update_irq_count[0U]++;
    }
    else if (htim->Instance == TIM8)
    {
        s_motor_update_irq_count[1U]++;
    }
    else if (htim->Instance == TIM20)
    {
        s_motor_update_irq_count[2U]++;

        /*
         * TIM20 update当前只保留计数验证。
         * 编码器SPI DMA改由ADC1完成回调末尾触发，避免TIM20 update和ADC DMA两条高频链路同时抢占CPU。
         */
    }
    else if (htim->Instance == TIM3)
    {
        Timer_App_TIM3_Handler();
    }
    else if (htim->Instance == TIM4)
    {
        Timer_App_TIM4_Handler();
    }
    else if (htim->Instance == TIM6)
    {
        Timer_App_TIM6_Handler();
    }
    else if (htim->Instance == TIM7)
    {
        Timer_App_TIM7_Handler();
    }
    else
    {
        // 其它定时器当前不在timer_app中处理
    }
}

/**
 * @brief  查询指定软件任务标志是否到达
 * @param  task: 任务枚举，取值见timer_app_task_t
 * @retval uint8_t
 *         0: 对应任务尚未到达
 *         1: 对应任务已到达，等待主循环处理
 * @note   该函数只负责查询，不会清除任务标志。
 */
uint8_t Timer_App_IsTaskReady(timer_app_task_t task)
{
    switch (task)
    {
        case TIMER_APP_TASK_5MS:
            return s_task_5ms_flag;

        case TIMER_APP_TASK_10MS:
            return s_task_10ms_flag;

        case TIMER_APP_TASK_20MS:
            return s_task_20ms_flag;

        case TIMER_APP_TASK_100MS:
            return s_task_100ms_flag;

        case TIMER_APP_TASK_500MS:
            return s_task_500ms_flag;

        case TIMER_APP_TASK_TEMP_SAMPLE:
            return s_task_temp_sample_flag;

        case TIMER_APP_TASK_SPEED_CALC:
            return s_task_speed_calc_flag;

        case TIMER_APP_TASK_CONTROL:
            return (s_task_control_pending > 0U) ? 1U : 0U;

        default:
            return 0U;
    }
}

/**
 * @brief  清除指定软件任务标志
 * @param  task: 任务枚举，取值见timer_app_task_t
 * @retval None
 * @note   建议主循环在完成对应任务处理后立即调用本函数。
 *         对普通任务表示清除flag；对控制任务表示消费一次pending。
 */
void Timer_App_ClearTaskFlag(timer_app_task_t task)
{
    switch (task)
    {
        case TIMER_APP_TASK_5MS:
            s_task_5ms_flag = 0U;
            break;

        case TIMER_APP_TASK_10MS:
            s_task_10ms_flag = 0U;
            break;

        case TIMER_APP_TASK_20MS:
            s_task_20ms_flag = 0U;
            break;

        case TIMER_APP_TASK_100MS:
            s_task_100ms_flag = 0U;
            break;

        case TIMER_APP_TASK_500MS:
            s_task_500ms_flag = 0U;
            break;

        case TIMER_APP_TASK_TEMP_SAMPLE:
            s_task_temp_sample_flag = 0U;
            break;

        case TIMER_APP_TASK_SPEED_CALC:
            s_task_speed_calc_flag = 0U;
            break;

        case TIMER_APP_TASK_CONTROL:
            if (s_task_control_pending > 0U)
            {
                s_task_control_pending--;
            }
            break;

        default:
            break;
    }
}

/**
 * @brief  增加一次控制任务pending。
 * @param  None
 * @retval None
 * @note   使用方式：由TIM3控制节拍调用。
 *         pending采用饱和计数，避免main偶发延迟后积压过多控制任务。
 *         本函数不直接运行PID/FOC，避免中断中执行重计算。
 */
void Timer_App_SetControlTaskFlag(void)
{
    if (s_task_control_pending < TIMER_APP_CONTROL_PENDING_MAX)
    {
        s_task_control_pending++;
    }
}

/**
 * @brief  置位指定电机的控制请求。
 * @param  motor_id: 电机编号，范围0~MOTOR_COUNT-1。
 * @retval None
 * @note   当前由ADC DMA完成回调调用，只记录“该电机获得一帧新的三相电流数据”。
 *         不在中断中运行PID/FOC，避免高频中断负担过重。
 */
void Timer_App_SetMotorControlRequest(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return;
    }

    if (s_motor_control_pending[motor_id] < TIMER_APP_MOTOR_CONTROL_PENDING_MAX)
    {
        s_motor_control_pending[motor_id]++;
    }
}

/**
 * @brief  消费指定电机的一次PWM同步控制请求。
 * @param  motor_id: 电机编号，范围0~MOTOR_COUNT-1。
 * @retval uint8_t
 *         0: 当前没有待处理请求
 *         1: 成功消费一次请求
 * @note   main中调用本函数，消费成功后再进入motor_core尝试控制更新。
 */
uint8_t Timer_App_ConsumeMotorControlRequest(uint8_t motor_id)
{
    uint32_t primask;
    uint8_t consumed = 0U;

    if (motor_id >= MOTOR_COUNT)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_motor_control_pending[motor_id] > 0U)
    {
        s_motor_control_pending[motor_id]--;
        consumed = 1U;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return consumed;
}

/**
 * @brief  获取指定电机PWM定时器update中断计数。
 * @param  motor_id: 电机编号，0/1/2分别对应TIM1/TIM8/TIM20。
 * @retval update中断累计次数，参数非法时返回0。
 * @note   使用方式：通过VOFA观察该计数是否增长，用于确认TIM1/TIM8/TIM20是否真的进入更新中断。
 */
uint32_t Timer_App_GetMotorUpdateIrqCount(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT)
    {
        return 0U;
    }

    return s_motor_update_irq_count[motor_id];
}

/**
 * @brief  获取TIM7节拍维护的编码器采样时间戳。
 * @param  None
 * @retval uint32_t
 *         编码器采样时间戳，单位us。
 * @note   该时间戳在TIM7周期中断中按固定周期累加，main只读取它。
 *         uint32_t溢出后允许自然回绕，后续用无符号减法计算dt即可保持正确短间隔。
 */
uint32_t Timer_App_GetSpeedCalcTimeUs(void)
{
    return s_speed_calc_time_us;
}

/**
 * @brief  获取DWT运行周期计数。
 * @param  None
 * @retval uint32_t
 *         当前CPU周期计数值。
 * @note   main在真正执行控制前读取该值，用两次实际执行时间差计算真实dt。
 *         uint32_t自然回绕时，无符号减法仍可正确计算短时间间隔。
 */
uint32_t Timer_App_GetRuntimeCycles(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief  初始化DWT运行周期计数器。
 * @param  None
 * @retval None
 * @note   DWT->CYCCNT随CPU时钟递增，用于比HAL_GetTick更高精度的dt测量。
 */
static void Timer_App_RuntimeCounterInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  TIM3周期事件处理函数
 * @param  None
 * @retval None
 * @note   TIM3当前作为电机控制任务调度节拍。
 *         该函数只置位TIMER_APP_TASK_CONTROL，不直接执行控制算法。
 */
static void Timer_App_TIM3_Handler(void)
{
    Timer_App_SetControlTaskFlag();
}

/**
 * @brief  TIM4周期事件处理函数
 * @param  None
 * @retval None
 * @note   TIM4当前作为系统软件调度基础时基，基础周期为5ms。
 *         由该基础周期分频得到10ms、20ms、100ms与500ms软件任务标志。
 */
static void Timer_App_TIM4_Handler(void)
{
    s_task_5ms_flag = 1U;

    s_tim4_div_10ms++;
    if (s_tim4_div_10ms >= 2U)
    {
        s_tim4_div_10ms = 0U;
        s_task_10ms_flag = 1U;
    }

    s_tim4_div_20ms++;
    if (s_tim4_div_20ms >= 4U)
    {
        s_tim4_div_20ms = 0U;
        s_task_20ms_flag = 1U;
    }

    s_tim4_div_100ms++;
    if (s_tim4_div_100ms >= 20U)
    {
        s_tim4_div_100ms = 0U;
        s_task_100ms_flag = 1U;
    }

    s_tim4_div_500ms++;
    if (s_tim4_div_500ms >= 100U)
    {
        s_tim4_div_500ms = 0U;
        s_task_500ms_flag = 1U;
    }
}

/**
 * @brief  TIM6周期事件处理函数
 * @param  None
 * @retval None
 * @note   TIM6当前预留给温度采样节拍/ADC2触发相关任务。
 *         当前阶段默认不启动TIM6中断，后续温度采样接入时再启用。
 */
static void Timer_App_TIM6_Handler(void)
{
    s_task_temp_sample_flag = 1U;
}

/**
 * @brief  TIM7周期事件处理函数
 * @param  None
 * @retval None
 * @note   TIM7当前用于编码器采样/速度计算节拍。
 *         该函数只置位任务标志，不直接启动SPI DMA。
 */
static void Timer_App_TIM7_Handler(void)
{
    s_speed_calc_time_us += TIMER_APP_SPEED_CALC_PERIOD_US;
    s_task_speed_calc_flag = 1U;
}
