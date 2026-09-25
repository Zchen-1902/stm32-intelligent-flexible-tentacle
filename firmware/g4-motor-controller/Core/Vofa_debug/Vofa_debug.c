#include "Vofa_debug.h"

#include <string.h>

#include "usart.h"
#include "../adc_app/adc_app.h"
#include "../timer_app/timer_app.h"
#include "../motor/foc_drive.h"
#include "../motor/motor_core.h"
#include "../drv8323S_drive/drv_drive.h"
#include "../spi_encoder/spi_encoder.h"

//******************************************VOFA协议常量*******************************************/
static const uint8_t s_vofa_frame_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};   // JustFloat协议帧尾

//******************************************FOC测试参数*******************************************/
#define VOFA_FOC_TEST_VD_NORM              0.0f       // FOC测试d轴归一化电压
#define VOFA_FOC_TEST_VQ_NORM              0.01f      // FOC测试q轴归一化电压，首次真实转动使用小值
#define VOFA_FOC_TEST_MOTOR_ID             2U         // FOC测试默认使用TIM20输出，对应当前DRV3调试链路
#define VOFA_ENCODER_TOPIC_ID              encoder_id_1 // 编码器主题默认查看电机3编码器，当前对应CSN3
#define VOFA_ENCODER_MOTOR_ID              2U         // 编码器主题对应的电机ID，用于查看FOC电角度和offset
#define VOFA_FOC_TEST_WRITE_CCR_ENABLE     0U         // FOC测试CCR写入使能：默认只看目标值，不写真实TIM CCR
#define VOFA_CURRENT_PID_MOTOR_ID          2U         // 电流环主题临时观察M3/motor2
#define VOFA_SPEED_PID_MOTOR_ID            2U         // 速度/位置环主题临时观察M3/motor2
#define VOFA_PHASE_CURRENT_MOTOR_ID        2U         // 相电流主题临时观察M3/motor2
#define VOFA_CURRENT_SAMPLE_DIAG_MOTOR_ID  2U         // 电流采样诊断主题临时观察M3/motor2

//******************************************VOFA模块内部状态*******************************************/
static volatile uint8_t s_vofa_tx_busy = 0U;                                 // 串口DMA发送忙标志
static uint8_t s_vofa_enable = VOFA_DEFAULT_ENABLE;                          // 调试发送使能标志
static vofa_topic_t s_vofa_topic = VOFA_TOPIC_NONE;                          // 当前调试主题
static uint32_t s_vofa_period_ms = VOFA_DEFAULT_PERIOD_MS;                   // 周期发送间隔，单位ms
static uint32_t s_vofa_last_tick = 0U;                                       // 上次成功启动发送的时间戳
static uint32_t s_vofa_error_count = 0U;                                     // 串口发送错误计数

static uint8_t s_vofa_tx_buf[VOFA_TX_BUF_SIZE] = {0U};                       // 串口DMA发送缓冲区
static float s_vofa_topic_buf[VOFA_MAX_CHANNEL_NUM] = {0.0f};                // 当前主题数据缓冲区

static uint32_t s_key1_pwm_start_count = 0U;                                 // KEY1启动PWM路径触发次数
static uint8_t s_key1_pwm_start_bits = 0U;                                    // KEY1启动PWM路径状态打包
static uint32_t s_key1_pwm_ccer_before = 0U;                                  // KEY1启动PWM前TIM20->CCER
static uint32_t s_key1_pwm_ccer_after = 0U;                                   // KEY1启动PWM后TIM20->CCER
static foc_vofa_data_t s_foc_test_vofa = {0};                                 // FOC测试主题观测数据
#if (VOFA_FOC_TEST_WRITE_CCR_ENABLE != 0U)
static const foc_pwm_output_t s_foc_test_pwm_map[MOTOR_COUNT] = {
    {&htim1,  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},                  // 电机1：TIM1 U/V/W = CH1/CH2/CH3
    {&htim8,  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3},                  // 电机2：TIM8 U/V/W = CH1/CH2/CH3
    {&htim20, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_4},                  // 电机3：TIM20 U/V/W = CH1/CH2/CH4
};
#endif

//******************************************VOFA内部函数声明*******************************************/
static void VOFA_Debug_ResetState(void);                                     // 重置模块内部运行状态
static uint8_t VOFA_Debug_IsPeriodElapsed(void);                             // 判断当前是否到达发送周期
static vofa_send_status_t VOFA_Debug_BuildTopicData(float *data, uint16_t *channel_num); // 生成当前主题数据
static vofa_send_status_t VOFA_Debug_PackFloatFrame(const float *data, uint16_t channel_num, uint16_t *frame_len); // 打包JustFloat帧

/**
 * @brief  初始化VOFA调试模块内部状态
 * @param  None
 * @retval None
 * @note   该函数应在USART1初始化完成后调用一次。
 *         本函数只初始化VOFA模块内部状态，不负责初始化USART1、GPIO或DMA硬件。
 *         建议在main函数完成CubeMX外设初始化后调用。
 */
void VOFA_Debug_Init(void)
{
    VOFA_Debug_ResetState();
    s_vofa_enable = VOFA_DEFAULT_ENABLE;
    s_vofa_topic = VOFA_TOPIC_NONE;
    s_vofa_period_ms = VOFA_DEFAULT_PERIOD_MS;
    s_vofa_last_tick = HAL_GetTick();
}

/**
 * @brief  VOFA周期发送任务
 * @param  None
 * @retval None
 * @note   该函数建议放在主循环或低频调度任务中周期调用。
 *         函数内部会自动判断：
 *         1. 当前是否允许发送；
 *         2. USART1 DMA是否仍在忙；
 *         3. 是否已到发送周期；
 *         4. 当前主题是否已准备好有效数据。
 *         若上述条件均满足，则函数会自动启动一帧VOFA数据的DMA发送。
 */
void VOFA_Debug_Task(void)
{
    vofa_send_status_t status;
    uint16_t channel_num = 0U;

    if (s_vofa_enable == 0U)
    {
        return;
    }

    if (s_vofa_tx_busy != 0U)
    {
        return;
    }

    if (VOFA_Debug_IsPeriodElapsed() == 0U)
    {
        return;
    }

    status = VOFA_Debug_BuildTopicData(s_vofa_topic_buf, &channel_num);
    if (status != VOFA_SEND_OK)
    {
        return;
    }

    if (channel_num == 0U)
    {
        return;
    }

    (void)VOFA_Debug_SendFloatFrame(s_vofa_topic_buf, channel_num);
}

/**
 * @brief  发送一帧VOFA JustFloat浮点数据
 * @param  data: 指向待发送浮点数组的首地址
 * @param  channel_num: 本次发送的浮点通道数，范围为1~VOFA_MAX_CHANNEL_NUM
 * @retval vofa_send_status_t
 *         VOFA_SEND_OK        : 成功启动DMA发送
 *         VOFA_SEND_BUSY      : 当前USART1 DMA忙，上一帧尚未发送完成
 *         VOFA_SEND_PARAM_ERR : 输入参数非法
 *         VOFA_SEND_BUF_ERR   : 打包后帧长度超出发送缓冲区
 *         VOFA_SEND_UART_ERR  : 启动USART1 DMA发送失败
 * @note   本函数会调用HAL_UART_Transmit_DMA(&huart1, ...)启动USART1 DMA发送。
 *         本函数不会阻塞等待发送完成，真正的发送完成由VOFA_Debug_TxCpltCallback处理。
 */
vofa_send_status_t VOFA_Debug_SendFloatFrame(const float *data, uint16_t channel_num)
{
    HAL_StatusTypeDef hal_status;
    vofa_send_status_t status;
    uint16_t frame_len = 0U;

    if ((data == NULL) || (channel_num == 0U) || (channel_num > VOFA_MAX_CHANNEL_NUM))
    {
        return VOFA_SEND_PARAM_ERR;
    }

    if (s_vofa_tx_busy != 0U)
    {
        return VOFA_SEND_BUSY;
    }

    status = VOFA_Debug_PackFloatFrame(data, channel_num, &frame_len);
    if (status != VOFA_SEND_OK)
    {
        return status;
    }

    s_vofa_tx_busy = 1U;

    hal_status = HAL_UART_Transmit_DMA(&huart1, s_vofa_tx_buf, frame_len);
    if (hal_status != HAL_OK)
    {
        s_vofa_tx_busy = 0U;
        s_vofa_error_count++;
        return VOFA_SEND_UART_ERR;
    }

    s_vofa_last_tick = HAL_GetTick();
    return VOFA_SEND_OK;
}

/**
 * @brief  设置当前VOFA调试主题
 * @param  topic: 目标调试主题，取值见vofa_topic_t枚举
 * @retval None
 * @note   该函数只修改当前输出主题，不会立即触发发送。
 *         新主题会在后续VOFA_Debug_Task周期执行时生效。
 */
void VOFA_Debug_SetTopic(vofa_topic_t topic)
{
    s_vofa_topic = topic;
}

/**
 * @brief  获取当前VOFA调试主题
 * @param  None
 * @retval vofa_topic_t 当前主题
 * @note   可用于按键切换主题、调试状态查询或上位机显示当前主题编号。
 */
vofa_topic_t VOFA_Debug_GetTopic(void)
{
    return s_vofa_topic;
}

/**
 * @brief  使能VOFA调试发送
 * @param  None
 * @retval None
 * @note   调用后允许VOFA_Debug_Task继续发送数据。
 *         本函数不会强制立即发送当前帧。
 */
void VOFA_Debug_Enable(void)
{
    s_vofa_enable = 1U;
}

/**
 * @brief  禁用VOFA调试发送
 * @param  None
 * @retval None
 * @note   调用后VOFA_Debug_Task将不再启动新的发送。
 *         若当前已有DMA发送正在进行，本函数不会强制中止该发送。
 */
void VOFA_Debug_Disable(void)
{
    s_vofa_enable = 0U;
}

/**
 * @brief  设置VOFA周期发送间隔
 * @param  period_ms: 发送周期，单位ms，必须大于0
 * @retval None
 * @note   该函数只修改后续周期发送间隔，不影响当前已经启动的DMA发送。
 *         建议在主循环初始化阶段或调试参数切换时调用。
 */
void VOFA_Debug_SetPeriodMs(uint32_t period_ms)
{
    if (period_ms > 0U)
    {
        s_vofa_period_ms = period_ms;
    }
}

/**
 * @brief  记录KEY1触发后的PWM启动路径诊断信息。
 * @param  encoder_valid: 按键触发时编码器有效标志。
 * @param  start_allowed: Motor_Core_IsPwmStartAllowed()返回结果。
 * @param  start_called: 是否实际调用Motor_Core_StartPwmOutput()。
 * @param  start_ok: Motor_Core_StartPwmOutput()是否返回HAL_OK。
 * @param  pwm_ready_after: KEY1处理结束后的PWM ready状态。
 * @param  ccer_before: KEY1处理前TIM20->CCER。
 * @param  ccer_after: KEY1处理后TIM20->CCER。
 * @retval None
 * @note   状态打包bit：bit0 encoder_valid，bit1 start_allowed，bit2 start_called，
 *         bit3 start_ok，bit4 pwm_ready_after。
 */
void VOFA_Debug_RecordKey1PwmStart(uint8_t encoder_valid,
                                   uint8_t start_allowed,
                                   uint8_t start_called,
                                   uint8_t start_ok,
                                   uint8_t pwm_ready_after,
                                   uint32_t ccer_before,
                                   uint32_t ccer_after)
{
    s_key1_pwm_start_count++;
    s_key1_pwm_start_bits = (uint8_t)(((encoder_valid != 0U) ? 0x01U : 0U) |
                                      ((start_allowed != 0U) ? 0x02U : 0U) |
                                      ((start_called != 0U) ? 0x04U : 0U) |
                                      ((start_ok != 0U) ? 0x08U : 0U) |
                                      ((pwm_ready_after != 0U) ? 0x10U : 0U));
    s_key1_pwm_ccer_before = ccer_before;
    s_key1_pwm_ccer_after = ccer_after;
}

/**
 * @brief  查询当前VOFA发送是否忙
 * @param  None
 * @retval uint8_t
 *         0: 空闲，可启动新一帧发送
 *         1: 忙，上一帧DMA尚未发送完成
 * @note   该函数主要用于调试状态判断或外部逻辑查看发送占用状态。
 */
uint8_t VOFA_Debug_IsBusy(void)
{
    return s_vofa_tx_busy;
}

/**
 * @brief  获取VOFA串口发送错误计数
 * @param  None
 * @retval uint32_t 串口发送错误累积次数
 * @note   可用于调试串口稳定性，判断DMA发送过程中是否存在异常。
 */
uint32_t VOFA_Debug_GetErrorCount(void)
{
    return s_vofa_error_count;
}

/**
 * @brief  VOFA串口发送完成回调入口
 * @param  huart: 触发发送完成的串口句柄
 * @retval None
 * @note   该函数应在HAL_UART_TxCpltCallback中转发调用。
 *         本函数只处理USART1发送完成事件，用于清除发送忙标志。
 */
void VOFA_Debug_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        s_vofa_tx_busy = 0U;
    }
}

/**
 * @brief  VOFA串口错误回调入口
 * @param  huart: 触发错误的串口句柄
 * @retval None
 * @note   该函数应在HAL_UART_ErrorCallback中转发调用。
 *         当USART1发送过程中发生错误时，本函数会清除忙标志并累计错误计数。
 */
void VOFA_Debug_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        s_vofa_tx_busy = 0U;
        s_vofa_error_count++;
    }
}

/**
 * @brief  重置VOFA模块内部运行状态
 * @param  None
 * @retval None
 * @note   该函数仅供模块内部使用，用于初始化或异常恢复时清空状态变量和缓冲区。
 */
static void VOFA_Debug_ResetState(void)
{
    s_vofa_tx_busy = 0U;
    s_vofa_error_count = 0U;
    (void)memset(s_vofa_tx_buf, 0, sizeof(s_vofa_tx_buf));
    (void)memset(s_vofa_topic_buf, 0, sizeof(s_vofa_topic_buf));
}

/**
 * @brief  判断当前是否到达发送周期
 * @param  None
 * @retval uint8_t
 *         0: 未到发送时间
 *         1: 已到发送时间
 * @note   该函数内部基于HAL_GetTick与上次发送时间戳计算周期是否到达。
 */
static uint8_t VOFA_Debug_IsPeriodElapsed(void)
{
    uint32_t now_tick = HAL_GetTick();
    return ((now_tick - s_vofa_last_tick) >= s_vofa_period_ms) ? 1U : 0U;
}

/**
 * @brief  根据当前调试主题构建待发送的浮点数据
 * @param  data: 输出浮点数组缓冲区
 * @param  channel_num: 输出的有效通道数
 * @retval vofa_send_status_t 构建状态
 * @note   该函数负责“当前主题发什么”的数据组织。
 *         当前版本仅保留主题框架，具体ADC、电流、编码器、PID数据后续逐步接入。
 */
static vofa_send_status_t VOFA_Debug_BuildTopicData(float *data, uint16_t *channel_num)
{
    if ((data == NULL) || (channel_num == NULL))
    {
        return VOFA_SEND_PARAM_ERR;
    }

    (void)memset(data, 0, VOFA_MAX_CHANNEL_NUM * sizeof(float));

    switch (s_vofa_topic)
    {
        case VOFA_TOPIC_ADC_RAW:
            *channel_num = 6U;    // ADC采样概览：adc1_done adc3_done adc4_done tx_busy uart_err period_ms
            data[0] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC1); // ADC1 DMA完成标志
            data[1] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC3); // ADC3 DMA完成标志
            data[2] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC4); // ADC4 DMA完成标志
            data[3] = (float)s_vofa_tx_busy;                            // 当前VOFA串口DMA发送忙状态
            data[4] = (float)s_vofa_error_count;                        // VOFA串口发送错误计数
            data[5] = (float)s_vofa_period_ms;                          // 当前VOFA发送周期，单位ms
            break;

        case VOFA_TOPIC_ADC_DIAG:
            *channel_num = 10U;   // ADC诊断：adc计数/完成标志，以及系统时钟/TIM20基础参数
            data[0] = (float)ADC_App_GetMotorUpdateCount(ADC_APP_MOTOR_ADC1);      // ADC1稳定帧更新计数，判断TIM20触发链路
            data[1] = (float)ADC_App_GetMotorUpdateCount(ADC_APP_MOTOR_ADC3);      // ADC3稳定帧更新计数，判断TIM8触发链路
            data[2] = (float)ADC_App_GetMotorUpdateCount(ADC_APP_MOTOR_ADC4);      // ADC4稳定帧更新计数，判断TIM1触发链路
            data[3] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC1);           // ADC1 DMA完成标志
            data[4] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC3);           // ADC3 DMA完成标志
            data[5] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC4);           // ADC4 DMA完成标志
            data[6] = (float)SystemCoreClock / 1000000.0f;                         // 当前SystemCoreClock，单位MHz
            data[7] = (float)HAL_RCC_GetPCLK2Freq() / 1000000.0f;                  // 当前PCLK2频率，单位MHz，TIM20位于APB2
            data[8] = (float)TIM20->PSC;                                           // TIM20预分频寄存器
            data[9] = (float)TIM20->ARR;                                           // TIM20自动重装载寄存器
            break;

        case VOFA_TOPIC_ADC1_RAW:
            *channel_num = 4U;    // ADC1原始采样：raw_a raw_b raw_c dma_done
            data[0] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC1, ADC_APP_PHASE_A); // ADC1 Rank1原始值
            data[1] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC1, ADC_APP_PHASE_B); // ADC1 Rank2原始值
            data[2] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC1, ADC_APP_PHASE_C); // ADC1 Rank3原始值
            data[3] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC1);               // ADC1 DMA完成标志
            break;

        case VOFA_TOPIC_ADC3_RAW:
            *channel_num = 4U;    // ADC3原始采样：raw_a raw_b raw_c dma_done
            data[0] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC3, ADC_APP_PHASE_A); // ADC3 Rank1原始值
            data[1] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC3, ADC_APP_PHASE_B); // ADC3 Rank2原始值
            data[2] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC3, ADC_APP_PHASE_C); // ADC3 Rank3原始值
            data[3] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC3);               // ADC3 DMA完成标志
            break;

        case VOFA_TOPIC_ADC4_RAW:
            *channel_num = 4U;    // ADC4原始采样：raw_a raw_b raw_c dma_done
            data[0] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC4, ADC_APP_PHASE_A); // ADC4 Rank1原始值
            data[1] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC4, ADC_APP_PHASE_B); // ADC4 Rank2原始值
            data[2] = (float)ADC_App_GetMotorRaw(ADC_APP_MOTOR_ADC4, ADC_APP_PHASE_C); // ADC4 Rank3原始值
            data[3] = (float)ADC_App_IsMotorDmaDone(ADC_APP_MOTOR_ADC4);               // ADC4 DMA完成标志
            break;

        case VOFA_TOPIC_PHASE_CURRENT:
        {
            motor_t *motor;

            *channel_num = 10U;   // 相电流：ia_raw ib_raw ic_raw ia ib ic i_sum adc_count id iq
            motor = Motor_Core_GetMotor(VOFA_PHASE_CURRENT_MOTOR_ID);

            if (motor != 0)
            {
                data[0] = (float)motor->current.ia_raw;             // A相ADC原始值
                data[1] = (float)motor->current.ib_raw;             // B相ADC原始值
                data[2] = (float)motor->current.ic_raw;             // C相ADC原始值
                data[3] = motor->current.ia;                        // A相电流，单位A
                data[4] = motor->current.ib;                        // B相电流，单位A
                data[5] = motor->current.ic;                        // C相电流，单位A
                data[6] = motor->current.i_sum;                     // 三相电流和，正常应接近0
                data[7] = (float)motor->current.adc_update_count;   // 已消费ADC稳定帧计数
                data[8] = motor->current.id;                        // d轴电流，单位A
                data[9] = motor->current.iq;                        // q轴电流，单位A
            }
            break;
        }

        case VOFA_TOPIC_CURRENT_SAMPLE_DIAG:
        {
            motor_t *motor;
            uint8_t reconstruct_enable = 0U;
            uint8_t reconstruct_phase = 0U;
            uint16_t adc_trigger_ccr = 0U;

            *channel_num = 10U;   // raw_a raw_b raw_c ia ib ic adc_count reconstruct_en reconstruct_phase adc_ccr
            motor = Motor_Core_GetMotor(VOFA_CURRENT_SAMPLE_DIAG_MOTOR_ID);

            if (motor != 0)
            {
                Motor_Core_GetCurrentSampleDebug(VOFA_CURRENT_SAMPLE_DIAG_MOTOR_ID,
                                                 &reconstruct_enable,
                                                 &reconstruct_phase,
                                                 &adc_trigger_ccr);

                data[0] = (float)motor->current.ia_raw;             // A相ADC原始值
                data[1] = (float)motor->current.ib_raw;             // B相ADC原始值
                data[2] = (float)motor->current.ic_raw;             // C相ADC原始值
                data[3] = motor->current.ia;                        // A相电流，单位A
                data[4] = motor->current.ib;                        // B相电流，单位A
                data[5] = motor->current.ic;                        // C相电流，单位A
                data[6] = (float)motor->current.adc_update_count;   // 已消费ADC稳定帧计数
                data[7] = (float)reconstruct_enable;                // 是否启用重构
                data[8] = (float)reconstruct_phase;                 // 重构相：0=U/A，1=V/B，2=W/C
                data[9] = (float)adc_trigger_ccr;                   // 当前ADC触发CCR
            }
            break;
        }

        case VOFA_TOPIC_ENCODER:
        {
            *channel_num = 10U;   // 编码器原始帧：id raw rx0 rx1 rx2 rx3 valid no_mag parity init

            data[0] = (float)VOFA_ENCODER_TOPIC_ID;                              // 当前查看的编码器编号
            data[1] = (float)spi_encoder[VOFA_ENCODER_TOPIC_ID].raw_angle;        // 14bit原始角度，范围0~16383
            data[2] = (float)spi_encoder[VOFA_ENCODER_TOPIC_ID].last_rx[0];       // 高字节寄存器读帧rx[0]，通常为dummy
            data[3] = (float)spi_encoder[VOFA_ENCODER_TOPIC_ID].last_rx[1];       // 高字节寄存器读帧rx[1]，Angle[13:6]
            data[4] = (float)spi_encoder[VOFA_ENCODER_TOPIC_ID].last_rx[2];       // burst模式为L/status；分开读模式为dummy
            data[5] = (float)spi_encoder[VOFA_ENCODER_TOPIC_ID].last_rx[3];       // 分开读模式为L/status；burst模式固定为0
            data[6] = (float)SPI_Encoder_Is_Valid(VOFA_ENCODER_TOPIC_ID);         // 最近一次编码器数据有效标志
            data[7] = (float)spi_encoder[VOFA_ENCODER_TOPIC_ID].no_mag_warning;   // MT6816 no_mag报警位
            data[8] = (float)spi_encoder[VOFA_ENCODER_TOPIC_ID].parity_error;     // MT6816 parity校验错误位
            data[9] = (float)spi_encoder[VOFA_ENCODER_TOPIC_ID].initialized;      // 是否已完成首次有效读取
            break;
        }

        case VOFA_TOPIC_ENCODER_ALL:
        {
            uint8_t valid_mask = 0U;
            uint8_t error_mask = 0U;
            uint8_t init_mask = 0U;
            uint8_t i;

            *channel_num = 10U;   // 三路编码器：raw1 raw2 raw3 angle1 angle2 angle3 valid_mask error_mask missed init_mask

            for (i = 0U; i < ENCODER_NUM; i++)
            {
                if (SPI_Encoder_Is_Valid((encoder_id_t)i) != 0U)
                {
                    valid_mask |= (uint8_t)(1U << i);
                }

                if ((spi_encoder[i].no_mag_warning != 0U) ||
                    (spi_encoder[i].parity_error != 0U))
                {
                    error_mask |= (uint8_t)(1U << i);
                }

                if (spi_encoder[i].initialized != 0U)
                {
                    init_mask |= (uint8_t)(1U << i);
                }
            }

            data[0] = (float)spi_encoder[encoder_id_1].raw_angle;        // encoder1 14bit原始角度
            data[1] = (float)spi_encoder[encoder_id_2].raw_angle;        // encoder2 14bit原始角度
            data[2] = (float)spi_encoder[encoder_id_3].raw_angle;        // encoder3 14bit原始角度
            data[3] = SPI_Encoder_Get_Angle(encoder_id_1);               // encoder1 单圈机械角，单位rad
            data[4] = SPI_Encoder_Get_Angle(encoder_id_2);               // encoder2 单圈机械角，单位rad
            data[5] = SPI_Encoder_Get_Angle(encoder_id_3);               // encoder3 单圈机械角，单位rad
            data[6] = (float)valid_mask;                                 // bit0/1/2分别表示encoder1/2/3最近数据有效
            data[7] = (float)error_mask;                                 // bit0/1/2分别表示encoder1/2/3无磁或校验错误
            data[8] = (float)SPI_Encoder_Get_DMA_Missed_Count();         // SPI3 DMA忙导致跳过编码器读取的次数
            data[9] = (float)init_mask;                                  // bit0/1/2分别表示encoder1/2/3已完成首次有效读取
            break;
        }

        case VOFA_TOPIC_CURRENT_PID:
        {
            motor_t *motor;
            uint8_t reconstruct_phase = 0U;
            uint16_t adc_trigger_ccr = 0U;

            *channel_num = 10U;   // 电流环混合诊断：id iq i_alpha i_beta ia ib ic reconstruct_phase adc_trigger_ccr elec_angle

            motor = Motor_Core_GetMotor(VOFA_CURRENT_PID_MOTOR_ID);
            if (motor != 0)
            {
                Motor_Core_GetCurrentSampleDebug(VOFA_CURRENT_PID_MOTOR_ID,
                                                 0,
                                                 &reconstruct_phase,
                                                 &adc_trigger_ccr);

                data[0] = motor->current.id;                                 // d轴实际电流，单位A
                data[1] = motor->current.iq;                                 // q轴实际电流，单位A
                data[2] = motor->current.i_alpha;                            // Clarke alpha轴电流，单位A
                data[3] = motor->current.i_beta;                             // Clarke beta轴电流，单位A
                data[4] = motor->current.ia;                                 // U/A相电流，单位A
                data[5] = motor->current.ib;                                 // V/B相电流，单位A
                data[6] = motor->current.ic;                                 // W/C相电流，单位A
                data[7] = (float)reconstruct_phase;                          // 重构相：0=U/A，1=V/B，2=W/C
                data[8] = (float)adc_trigger_ccr;                             // 当前ADC触发CCR
                data[9] = motor->encoder.elec_angle;                         // 当前电角度，单位rad
            }
            break;
        }

        case VOFA_TOPIC_SPEED_PID:
        {
            motor_t *motor;

            *channel_num = 8U;    // 速度环：speed_ref speed_fb err iq_ref integral output_raw saturated encoder_valid
            motor = Motor_Core_GetMotor(VOFA_SPEED_PID_MOTOR_ID);
            if (motor != 0)
            {
                data[0] = motor->ref.speed_ref_ramp_rad_s;             // 实际送入速度环的机械角速度目标，单位rad/s
                data[1] = motor->control.speed_pid.feedback;           // 速度PID实际使用的反馈速度，单位rad/s
                data[2] = motor->control.speed_pid.error;              // 速度误差，单位rad/s
                data[3] = motor->ref.iq_ref;                           // 速度环输出的q轴电流目标，单位A
                data[4] = motor->control.speed_pid.integral;           // 速度环积分项
                data[5] = motor->control.speed_pid.output_raw;         // 速度环限幅前输出
                data[6] = (float)motor->control.speed_pid.saturated;   // 速度环输出是否限幅
                data[7] = (float)motor->encoder.valid;                 // 编码器有效标志
            }
            break;
        }

        case VOFA_TOPIC_POSITION_PID:
        {
            motor_t *motor;

            *channel_num = 8U;    // 位置环PID：ref fb err output integral output_raw saturated aux
            motor = Motor_Core_GetMotor(VOFA_SPEED_PID_MOTOR_ID);
            if (motor != 0)
            {
                data[0] = motor->ref.mech_angle_ref_ramp;          // 位置目标，单位rad
                data[1] = motor->control.pos_pid.feedback;         // 位置环实际使用的反馈，单位rad
                data[2] = motor->control.pos_pid.error;            // 位置误差，单位rad
                data[3] = motor->ref.speed_ref_rad_s;              // 位置环输出的速度目标，单位rad/s
                data[4] = motor->encoder.speed_rad_s;              // 当前速度反馈，单位rad/s
                data[5] = motor->ref.iq_ref;                       // 速度环输出的q轴电流目标，单位A
                data[6] = motor->control.pos_pid.output_raw;       // 位置环限幅前输出
                data[7] = (float)motor->encoder.valid;             // 编码器有效标志
            }
            break;
        }

        case VOFA_TOPIC_STATE:
            *channel_num = 4U;    // 预留：mode state fault warning
            break;

        case VOFA_TOPIC_FAST_LOOP_PROFILE:
        {
            motor_core_fast_loop_profile_t p0;
            motor_core_fast_loop_profile_t p1;
            motor_core_fast_loop_profile_t p2;
            const float cycle_to_us = 1000000.0f / (float)SystemCoreClock;

            *channel_num = 10U;   // m0_last_us m1_last_us m2_last_us m0_max_us m1_max_us m2_max_us over0 over1 over2 spi_busy

            memset(&p0, 0, sizeof(p0));
            memset(&p1, 0, sizeof(p1));
            memset(&p2, 0, sizeof(p2));

            Motor_Core_GetFastLoopProfile(0U, &p0);
            Motor_Core_GetFastLoopProfile(1U, &p1);
            Motor_Core_GetFastLoopProfile(2U, &p2);

            data[0] = (float)p0.last_cycles * cycle_to_us;       // motor0最近一次快环耗时，单位us
            data[1] = (float)p1.last_cycles * cycle_to_us;       // motor1最近一次快环耗时，单位us
            data[2] = (float)p2.last_cycles * cycle_to_us;       // motor2最近一次快环耗时，单位us
            data[3] = (float)p0.max_cycles * cycle_to_us;        // motor0最大快环耗时，单位us
            data[4] = (float)p1.max_cycles * cycle_to_us;        // motor1最大快环耗时，单位us
            data[5] = (float)p2.max_cycles * cycle_to_us;        // motor2最大快环耗时，单位us
            data[6] = (float)p0.overrun_count;                   // motor0超过电流环周期次数
            data[7] = (float)p1.overrun_count;                   // motor1超过电流环周期次数
            data[8] = (float)p2.overrun_count;                   // motor2超过电流环周期次数
            data[9] = (float)SPI_Encoder_Is_DMA_Busy();          // 当前SPI3编码器DMA忙状态
            break;
        }

        case VOFA_TOPIC_DRV_CHECK:
        {
            static const drv_id_t drv_ids[DRV_ID_NUM] =
            {
                DRV_ID_1,
                DRV_ID_2,
                DRV_ID_3
            };
            static const uint16_t drv_default_cfg[5] =
            {
                DRV8323_DRIVER_CONTROL_DEFAULT,
                DRV8323_GATE_DRIVE_HS_DEFAULT,
                DRV8323_GATE_DRIVE_LS_DEFAULT,
                DRV8323_OCP_CONTROL_DEFAULT,
                DRV8323_CSA_CONTROL_DEFAULT
            };
            uint16_t fault_status_1[DRV_ID_NUM] = {0U};
            uint16_t fault_status_2[DRV_ID_NUM] = {0U};
            uint16_t reg_value = 0U;
            uint16_t fault_summary_mask = 0U;
            uint16_t vds_ocp_mask = 0U;
            uint16_t read_error_mask = 0U;
            uint16_t config_diff_mask = 0U;
            uint8_t drv_index;
            uint8_t cfg_index;

            *channel_num = 10U;   // 三路DRV总览：fault1_1/2/3 fault2_1/2/3 fault_mask vds_mask read_err cfg_diff

            for (drv_index = 0U; drv_index < (uint8_t)DRV_ID_NUM; drv_index++)
            {
                if (DRV_Drive_ReadReg(drv_ids[drv_index],
                                      DRV8323_REG_FAULT_STATUS_1,
                                      &fault_status_1[drv_index]) != HAL_OK)
                {
                    read_error_mask |= (uint16_t)(1U << drv_index);
                }

                if (DRV_Drive_ReadReg(drv_ids[drv_index],
                                      DRV8323_REG_FAULT_STATUS_2,
                                      &fault_status_2[drv_index]) != HAL_OK)
                {
                    read_error_mask |= (uint16_t)(1U << drv_index);
                }

                if (((fault_status_1[drv_index] >> 10) & 0x1U) != 0U)
                {
                    fault_summary_mask |= (uint16_t)(1U << drv_index);
                }

                if (((fault_status_1[drv_index] >> 9) & 0x1U) != 0U)
                {
                    vds_ocp_mask |= (uint16_t)(1U << drv_index);
                }

                for (cfg_index = 0U; cfg_index < 5U; cfg_index++)
                {
                    if (DRV_Drive_ReadReg(drv_ids[drv_index],
                                          (uint8_t)(DRV8323_REG_DRIVER_CONTROL + cfg_index),
                                          &reg_value) != HAL_OK)
                    {
                        read_error_mask |= (uint16_t)(1U << drv_index);
                    }
                    else if ((reg_value & DRV8323_REG_DATA_MASK) !=
                             (drv_default_cfg[cfg_index] & DRV8323_REG_DATA_MASK))
                    {
                        config_diff_mask |= (uint16_t)(1U << drv_index);
                    }
                }
            }

            data[0] = (float)fault_status_1[0];                   // DRV1 Fault Status 1原始值
            data[1] = (float)fault_status_1[1];                   // DRV2 Fault Status 1原始值
            data[2] = (float)fault_status_1[2];                   // DRV3 Fault Status 1原始值
            data[3] = (float)fault_status_2[0];                   // DRV1 Fault Status 2原始值
            data[4] = (float)fault_status_2[1];                   // DRV2 Fault Status 2原始值
            data[5] = (float)fault_status_2[2];                   // DRV3 Fault Status 2原始值
            data[6] = (float)fault_summary_mask;                  // bit0/1/2分别表示DRV1/2/3 FAULT总故障
            data[7] = (float)vds_ocp_mask;                        // bit0/1/2分别表示DRV1/2/3 VDS_OCP
            data[8] = (float)read_error_mask;                     // bit0/1/2分别表示DRV1/2/3 SPI读失败
            data[9] = (float)config_diff_mask;                    // bit0/1/2分别表示DRV1/2/3配置寄存器与默认值不一致
            break;
        }

        case VOFA_TOPIC_FOC_TEST:
        {
            float elec_angle_rad = 0.0f;
            uint32_t tim20_ccer;

            *channel_num = 10U;   // TIM20诊断：CEN MOE CCER U_EN V_EN W_EN CCR_U CCR_V CCR_W CALIB_OK

            if (SPI_Encoder_Is_Valid(VOFA_ENCODER_TOPIC_ID) != 0U)
            {
                elec_angle_rad = Motor_Core_GetFocElecAngle(VOFA_FOC_TEST_MOTOR_ID);
                (void)FOC_RunVoltageNorm(NULL,
                                         VOFA_FOC_TEST_VD_NORM,
                                         VOFA_FOC_TEST_VQ_NORM,
                                         elec_angle_rad,
                                         &s_foc_test_vofa);
            }
            else
            {
                /* 编码器无效时输出零矢量，避免用软件假角度掩盖真实角度链路问题。 */
                (void)FOC_RunVoltageNorm(NULL,
                                         0.0f,
                                         0.0f,
                                         0.0f,
                                         &s_foc_test_vofa);
            }

#if (VOFA_FOC_TEST_WRITE_CCR_ENABLE != 0U)
            /* 这里只写当前测试电机CCR，不启动PWM输出；用于观察真实电角度驱动下的CCR变化。 */
            (void)FOC_WritePwmDuty(&s_foc_test_pwm_map[VOFA_FOC_TEST_MOTOR_ID],
                                   s_foc_test_vofa.duty_u,
                                   s_foc_test_vofa.duty_v,
                                   s_foc_test_vofa.duty_w);
#endif

            tim20_ccer = TIM20->CCER;

            data[0] = (float)((TIM20->CR1 & TIM_CR1_CEN) != 0U);    // TIM20计数器使能位，1表示定时器正在计数
            data[1] = (float)((TIM20->BDTR & TIM_BDTR_MOE) != 0U);  // TIM20主输出使能MOE，1表示高级定时器允许输出
            data[2] = (float)tim20_ccer;                            // TIM20通道输出使能寄存器原始值，用于核对实际启动相
            data[3] = (float)((((tim20_ccer & TIM_CCER_CC1E) != 0U) ? 1U : 0U) |
                              (((tim20_ccer & TIM_CCER_CC1NE) != 0U) ? 2U : 0U)); // A相CH1/CH1N使能打包值：0关，1主，2互补，3全开
            data[4] = (float)((((tim20_ccer & TIM_CCER_CC2E) != 0U) ? 1U : 0U) |
                              (((tim20_ccer & TIM_CCER_CC2NE) != 0U) ? 2U : 0U)); // B相CH2/CH2N使能打包值：0关，1主，2互补，3全开
            data[5] = (float)((((tim20_ccer & TIM_CCER_CC4E) != 0U) ? 1U : 0U) |
                              (((tim20_ccer & TIM_CCER_CC4NE) != 0U) ? 2U : 0U)); // C相CH4/CH4N使能打包值：0关，1主，2互补，3全开
            data[6] = (float)TIM20->CCR1;                            // 电机3 U相实际PWM比较值
            data[7] = (float)TIM20->CCR2;                            // 电机3 V相实际PWM比较值
            data[8] = (float)TIM20->CCR4;                            // 电机3 W相实际PWM比较值
            data[9] = (float)Motor_Core_GetFocOffsetAutoCalibResult(); // 最近一次FOC校正结果：1成功，0失败
            break;
        }

        case VOFA_TOPIC_NONE:
        default:
            *channel_num = 0U;
            break;
    }

    return VOFA_SEND_OK;
}

/**
 * @brief  将浮点数组打包为VOFA JustFloat发送帧
 * @param  data: 输入浮点数组
 * @param  channel_num: 浮点通道数
 * @param  frame_len: 输出最终帧长度，单位字节
 * @retval vofa_send_status_t 打包状态
 * @note   本函数会将float数组按内存字节序拷贝到发送缓冲区，
 *         并在末尾追加JustFloat协议帧尾。
 */
static vofa_send_status_t VOFA_Debug_PackFloatFrame(const float *data, uint16_t channel_num, uint16_t *frame_len)
{
    uint16_t payload_len;

    if ((data == NULL) || (frame_len == NULL) || (channel_num == 0U) || (channel_num > VOFA_MAX_CHANNEL_NUM))
    {
        return VOFA_SEND_PARAM_ERR;
    }

    payload_len = (uint16_t)(channel_num * sizeof(float));
    *frame_len = (uint16_t)(payload_len + sizeof(s_vofa_frame_tail));

    if (*frame_len > VOFA_TX_BUF_SIZE)
    {
        return VOFA_SEND_BUF_ERR;
    }

    (void)memcpy(s_vofa_tx_buf, data, payload_len);
    (void)memcpy(&s_vofa_tx_buf[payload_len], s_vofa_frame_tail, sizeof(s_vofa_frame_tail));

    return VOFA_SEND_OK;
}
