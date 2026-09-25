#include "adc_app.h"

//******************************************ADC应用层内部数据*******************************************/
static uint16_t s_adc1_raw[ADC_APP_PHASE_NUM] = {0U};        // ADC1三相电流原始采样值，顺序对应ADC1 Rank1/2/3
static uint16_t s_adc3_raw[ADC_APP_PHASE_NUM] = {0U};        // ADC3三相电流原始采样值，顺序对应ADC3 Rank1/2/3
static uint16_t s_adc4_raw[ADC_APP_PHASE_NUM] = {0U};        // ADC4三相电流原始采样值，顺序对应ADC4 Rank1/2/3
static uint16_t s_adc2_raw[ADC_APP_TEMP_ADC_NUM] = {0U};     // ADC2温度/辅助模拟量原始采样值，顺序对应ADC2 Rank1/2

static uint16_t s_adc1_frame_raw[2U][ADC_APP_PHASE_NUM] = {{0U}}; // ADC1软件双缓冲稳定帧
static uint16_t s_adc3_frame_raw[2U][ADC_APP_PHASE_NUM] = {{0U}}; // ADC3软件双缓冲稳定帧
static uint16_t s_adc4_frame_raw[2U][ADC_APP_PHASE_NUM] = {{0U}}; // ADC4软件双缓冲稳定帧

static volatile uint8_t s_adc1_dma_done = 0U;                // ADC1 DMA完成标志，DMA循环模式下表示至少完成过一次搬运
static volatile uint8_t s_adc3_dma_done = 0U;                // ADC3 DMA完成标志，DMA循环模式下表示至少完成过一次搬运
static volatile uint8_t s_adc4_dma_done = 0U;                // ADC4 DMA完成标志，DMA循环模式下表示至少完成过一次搬运
static volatile uint8_t s_adc2_dma_done = 0U;                // ADC2 DMA完成标志，DMA循环模式下表示至少完成过一次搬运

static volatile uint8_t s_adc1_read_idx = 0U;                // ADC1当前可读稳定帧索引
static volatile uint8_t s_adc3_read_idx = 0U;                // ADC3当前可读稳定帧索引
static volatile uint8_t s_adc4_read_idx = 0U;                // ADC4当前可读稳定帧索引

static volatile uint32_t s_adc1_update_count = 0U;           // ADC1稳定帧更新计数
static volatile uint32_t s_adc3_update_count = 0U;           // ADC3稳定帧更新计数
static volatile uint32_t s_adc4_update_count = 0U;           // ADC4稳定帧更新计数

static uint16_t (* const s_motor_fast_frame_raw[ADC_APP_MOTOR_ADC_NUM])[ADC_APP_PHASE_NUM] = {s_adc1_frame_raw, s_adc3_frame_raw, s_adc4_frame_raw}; // 快环ADC稳定帧表，索引使用adc_app_motor_adc_t
static volatile uint8_t * const s_motor_fast_read_idx[ADC_APP_MOTOR_ADC_NUM] = {&s_adc1_read_idx, &s_adc3_read_idx, &s_adc4_read_idx};              // 快环ADC稳定帧读索引表
static volatile uint32_t * const s_motor_fast_update_count[ADC_APP_MOTOR_ADC_NUM] = {&s_adc1_update_count, &s_adc3_update_count, &s_adc4_update_count}; // 快环ADC稳定帧计数表

static volatile uint8_t s_motor_adc_calib_error_mask = 0U;    // ADC自校准错误掩码：bit0 ADC1，bit1 ADC3，bit2 ADC4
static volatile uint8_t s_motor_adc_start_error_mask = 0U;    // ADC DMA启动错误掩码：bit0 ADC1，bit1 ADC3，bit2 ADC4

typedef struct
{
    uint16_t *dma_buf;                                      // DMA正在写入的原始buffer
    uint16_t (*frame_buf)[ADC_APP_PHASE_NUM];               // 软件双缓冲稳定帧
    volatile uint8_t *done_flag;                            // DMA完成标志
    volatile uint8_t *read_idx;                             // 当前可读稳定帧索引
    volatile uint32_t *update_count;                        // 稳定帧更新计数
} adc_app_motor_ctx_t;

//******************************************ADC应用层内部函数声明*******************************************/
static uint8_t ADC_App_GetMotorContext(adc_app_motor_adc_t adc_id,
                                       adc_app_motor_ctx_t *ctx);              // 根据ADC组ID集中获取该组ADC的软件资源
static void ADC_App_UpdateMotorFrame(adc_app_motor_adc_t adc_id);              // DMA完成后生成一帧可稳定读取的三相采样

/**
 * @brief  初始化ADC应用层状态
 * @param  None
 * @retval None
 * @note   建议在MX_ADC1/2/3/4_Init之后调用。
 *         该函数只清空软件buffer和DMA完成标志，不会启动ADC，也不会修改CubeMX外设配置。
 */
void ADC_App_Init(void)
{
    uint8_t i;

    for (i = 0U; i < ADC_APP_PHASE_NUM; i++)
    {
        s_adc1_raw[i] = 0U;
        s_adc3_raw[i] = 0U;
        s_adc4_raw[i] = 0U;

        s_adc1_frame_raw[0U][i] = 0U;
        s_adc1_frame_raw[1U][i] = 0U;
        s_adc3_frame_raw[0U][i] = 0U;
        s_adc3_frame_raw[1U][i] = 0U;
        s_adc4_frame_raw[0U][i] = 0U;
        s_adc4_frame_raw[1U][i] = 0U;
    }

    for (i = 0U; i < ADC_APP_TEMP_ADC_NUM; i++)
    {
        s_adc2_raw[i] = 0U;
    }

    s_adc1_dma_done = 0U;
    s_adc3_dma_done = 0U;
    s_adc4_dma_done = 0U;
    s_adc2_dma_done = 0U;

    s_adc1_read_idx = 0U;
    s_adc3_read_idx = 0U;
    s_adc4_read_idx = 0U;

    s_adc1_update_count = 0U;
    s_adc3_update_count = 0U;
    s_adc4_update_count = 0U;
    s_motor_adc_calib_error_mask = 0U;
    s_motor_adc_start_error_mask = 0U;
}

/**
 * @brief  校准三路电机电流采样ADC。
 * @param  None
 * @return HAL_OK表示ADC1/ADC3/ADC4均校准成功，否则返回对应HAL错误状态。
 * @note   使用方式：必须在HAL_ADC_Start_DMA()之前调用，且ADC不能处于转换运行状态。
 *         本函数只做STM32 ADC内部自校准，不等同于DRV CAL电流零漂校准。
 */
HAL_StatusTypeDef ADC_App_CalibrateMotorAdc(void)
{
    HAL_StatusTypeDef status;

    s_motor_adc_calib_error_mask = 0U;

    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        s_motor_adc_calib_error_mask |= (1U << 0);
        return status;
    }

    status = HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        s_motor_adc_calib_error_mask |= (1U << 1);
        return status;
    }

    status = HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        s_motor_adc_calib_error_mask |= (1U << 2);
    }

    return status;
}

/**
 * @brief  启动三组电机电流ADC的DMA循环接收
 * @param  None
 * @retval HAL_StatusTypeDef
 *         HAL_OK: ADC1/ADC3/ADC4 DMA均启动成功
 *         HAL_ERROR: 至少有一组ADC DMA启动失败
 * @note   当前ADC实际采样时刻由CubeMX配置的外部触发决定：
 *         ADC1 <- TIM20_TRGO2，ADC3 <- TIM8_TRGO2，ADC4 <- TIM1_TRGO2。
 *         DMA当前为CIRCULAR模式，本函数只需要在初始化阶段调用一次。
 */
HAL_StatusTypeDef ADC_App_StartMotorCurrentDma(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    s_motor_adc_start_error_mask = 0U;

    s_adc1_dma_done = 0U;
    s_adc3_dma_done = 0U;
    s_adc4_dma_done = 0U;
    s_adc1_read_idx = 0U;
    s_adc3_read_idx = 0U;
    s_adc4_read_idx = 0U;

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_raw, ADC_APP_PHASE_NUM) != HAL_OK)
    {
        status = HAL_ERROR;
        s_motor_adc_start_error_mask |= (1U << 0);
    }

    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)s_adc3_raw, ADC_APP_PHASE_NUM) != HAL_OK)
    {
        status = HAL_ERROR;
        s_motor_adc_start_error_mask |= (1U << 1);
    }

    if (HAL_ADC_Start_DMA(&hadc4, (uint32_t *)s_adc4_raw, ADC_APP_PHASE_NUM) != HAL_OK)
    {
        status = HAL_ERROR;
        s_motor_adc_start_error_mask |= (1U << 2);
    }

    return status;
}

/**
 * @brief  获取三路电机ADC自校准错误掩码。
 * @param  None
 * @retval 错误掩码：bit0对应ADC1，bit1对应ADC3，bit2对应ADC4；0表示未检测到错误。
 * @note   使用方式：调试ADC启动链路时通过VOFA观察，定位是哪一路ADC自校准失败。
 */
uint8_t ADC_App_GetMotorAdcCalibErrorMask(void)
{
    return s_motor_adc_calib_error_mask;
}

/**
 * @brief  获取三路电机ADC DMA启动错误掩码。
 * @param  None
 * @retval 错误掩码：bit0对应ADC1，bit1对应ADC3，bit2对应ADC4；0表示未检测到错误。
 * @note   使用方式：调试ADC启动链路时通过VOFA观察，定位是哪一路HAL_ADC_Start_DMA失败。
 */
uint8_t ADC_App_GetMotorAdcStartErrorMask(void)
{
    return s_motor_adc_start_error_mask;
}

/**
 * @brief  启动ADC2温度/辅助模拟量DMA循环接收
 * @param  None
 * @retval HAL_StatusTypeDef
 *         HAL_OK: ADC2 DMA启动成功
 *         HAL_ERROR: ADC2 DMA启动失败
 * @note   当前ADC2外部触发源为TIM6_TRGO。
 *         如果TIM6未启动或未产生TRGO，ADC2 DMA buffer不会更新。
 */
HAL_StatusTypeDef ADC_App_StartTempDma(void)
{
    s_adc2_dma_done = 0U;

    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_raw, ADC_APP_TEMP_ADC_NUM) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  读取指定电机ADC组三相原始值
 * @param  adc_id: 电机电流ADC组ID，取值见adc_app_motor_adc_t
 * @param  phase:  三相索引，取值见adc_app_phase_t
 * @retval uint16_t
 *         对应ADC Rank的原始12bit采样值；参数非法时返回0。
 * @note   返回值顺序严格跟随ADC Rank，而不是GPIO引脚编号顺序。
 */
uint16_t ADC_App_GetMotorRaw(adc_app_motor_adc_t adc_id, adc_app_phase_t phase)
{
    adc_app_motor_ctx_t ctx;
    uint8_t read_idx;

    if ((uint32_t)phase >= ADC_APP_PHASE_NUM)
    {
        return 0U;
    }

    if (ADC_App_GetMotorContext(adc_id, &ctx) == 0U)
    {
        return 0U;
    }

    read_idx = *ctx.read_idx;
    return ctx.frame_buf[read_idx][(uint32_t)phase];
}

/**
 * @brief  快环按已绑定ADC id一次性读取三相稳定帧。
 * @param  adc_id: 已初始化绑定的ADC组ID，必须为ADC_APP_MOTOR_ADC1/3/4之一。
 * @param  raw_a: 输出A相raw指针。
 * @param  raw_b: 输出B相raw指针。
 * @param  raw_c: 输出C相raw指针。
 * @param  update_count: 输出该ADC组稳定帧计数指针。
 * @retval 1表示已读到有效稳定帧，0表示该ADC组尚未产生稳定帧。
 * @note   快环专用接口：调用方保证adc_id和指针有效，避免16kHz路径重复做参数检查。
 */
uint8_t ADC_App_GetMotorRaw3FastUnchecked(uint8_t adc_id,
                                          uint16_t *raw_a,
                                          uint16_t *raw_b,
                                          uint16_t *raw_c,
                                          uint32_t *update_count)
{
    uint8_t read_idx;
    uint32_t count;

    count = *s_motor_fast_update_count[adc_id];
    if (count == 0U)
    {
        return 0U;
    }

    read_idx = *s_motor_fast_read_idx[adc_id];
    *raw_a = s_motor_fast_frame_raw[adc_id][read_idx][ADC_APP_PHASE_A];
    *raw_b = s_motor_fast_frame_raw[adc_id][read_idx][ADC_APP_PHASE_B];
    *raw_c = s_motor_fast_frame_raw[adc_id][read_idx][ADC_APP_PHASE_C];
    *update_count = count;

    return 1U;
}

/**
 * @brief  快环一次性读取ADC1三相稳定帧。
 * @param  raw_a: 输出A相raw指针，不能为NULL。
 * @param  raw_b: 输出B相raw指针，不能为NULL。
 * @param  raw_c: 输出C相raw指针，不能为NULL。
 * @param  update_count: 输出ADC1稳定帧计数指针，不能为NULL。
 * @retval uint8_t
 *         1: 已读到有效稳定帧。
 *         0: 参数错误或ADC1尚未产生稳定帧。
 * @note   当前专用于TIM20/ADC1电流快环，避免在16kHz回调中重复走通用ADC context查询。
 */
uint8_t ADC_App_GetMotorRaw3FastAdc1(uint16_t *raw_a,
                                     uint16_t *raw_b,
                                     uint16_t *raw_c,
                                     uint32_t *update_count)
{
    if ((raw_a == 0) || (raw_b == 0) || (raw_c == 0) || (update_count == 0))
    {
        return 0U;
    }

    return ADC_App_GetMotorRaw3FastUnchecked((uint8_t)ADC_APP_MOTOR_ADC1,
                                             raw_a,
                                             raw_b,
                                             raw_c,
                                             update_count);
}

/**
 * @brief  读取指定电机ADC组稳定帧更新计数
 * @param  adc_id: 电机电流ADC组ID，取值见adc_app_motor_adc_t
 * @retval uint32_t
 *         返回该ADC组稳定帧更新次数；参数非法时返回0。
 * @note   控制层用该值判断是否有新三相采样帧。
 *         如果返回值和上次消费值相同，说明没有新帧，不应重复推进电流环。
 */
uint32_t ADC_App_GetMotorUpdateCount(adc_app_motor_adc_t adc_id)
{
    adc_app_motor_ctx_t ctx;

    if (ADC_App_GetMotorContext(adc_id, &ctx) == 0U)
    {
        return 0U;
    }

    return *ctx.update_count;
}

/**
 * @brief  读取ADC2温度/辅助模拟量原始值
 * @param  ch: ADC2通道索引，取值见adc_app_temp_ch_t
 * @retval uint16_t
 *         对应ADC2 Rank的原始12bit采样值；参数非法时返回0。
 * @note   当前ADC2 Rank1为PA0/ADC2_IN1，Rank2为PB11/ADC2_IN14。
 */
uint16_t ADC_App_GetTempRaw(adc_app_temp_ch_t ch)
{
    if ((uint32_t)ch >= ADC_APP_TEMP_ADC_NUM)
    {
        return 0U;
    }

    return s_adc2_raw[(uint32_t)ch];
}

/**
 * @brief  查询指定电机ADC组DMA完成标志
 * @param  adc_id: 电机电流ADC组ID，取值见adc_app_motor_adc_t
 * @retval uint8_t
 *         0: DMA尚未完成过序列搬运
 *         1: DMA至少完成过一次序列搬运
 * @note   DMA当前为CIRCULAR模式，该标志主要用于确认采样链路是否已经跑起来。
 */
uint8_t ADC_App_IsMotorDmaDone(adc_app_motor_adc_t adc_id)
{
    adc_app_motor_ctx_t ctx;

    if (ADC_App_GetMotorContext(adc_id, &ctx) == 0U)
    {
        return 0U;
    }

    return *ctx.done_flag;
}

/**
 * @brief  清除指定电机ADC组DMA完成标志
 * @param  adc_id: 电机电流ADC组ID，取值见adc_app_motor_adc_t
 * @retval None
 * @note   如果只是想确认DMA是否持续运行，可以在VOFA读取后清零，观察它是否会再次变成1。
 */
void ADC_App_ClearMotorDmaDone(adc_app_motor_adc_t adc_id)
{
    adc_app_motor_ctx_t ctx;

    if (ADC_App_GetMotorContext(adc_id, &ctx) != 0U)
    {
        *ctx.done_flag = 0U;
    }
}

/**
 * @brief  查询ADC2 DMA完成标志
 * @param  None
 * @retval uint8_t
 *         0: ADC2 DMA尚未完成过序列搬运
 *         1: ADC2 DMA至少完成过一次序列搬运
 */
uint8_t ADC_App_IsTempDmaDone(void)
{
    return s_adc2_dma_done;
}

/**
 * @brief  清除ADC2 DMA完成标志
 * @param  None
 * @retval None
 * @note   如果后续启用TIM6触发ADC2，可以在读取温度/辅助raw后清除该标志。
 */
void ADC_App_ClearTempDmaDone(void)
{
    s_adc2_dma_done = 0U;
}

/**
 * @brief  ADC转换完成回调转发入口
 * @param  hadc: 触发转换完成回调的ADC句柄
 * @retval None
 * @note   该函数应在HAL_ADC_ConvCpltCallback中调用。
 *         它只设置对应ADC DMA完成标志，不做耗时计算，也不在中断里发送VOFA。
 */
void ADC_App_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL)
    {
        return;
    }

    if (hadc->Instance == ADC1)
    {
        ADC_App_UpdateMotorFrame(ADC_APP_MOTOR_ADC1);
        s_adc1_dma_done = 1U;
    }
    else if (hadc->Instance == ADC3)
    {
        ADC_App_UpdateMotorFrame(ADC_APP_MOTOR_ADC3);
        s_adc3_dma_done = 1U;
    }
    else if (hadc->Instance == ADC4)
    {
        ADC_App_UpdateMotorFrame(ADC_APP_MOTOR_ADC4);
        s_adc4_dma_done = 1U;
    }
    else if (hadc->Instance == ADC2)
    {
        s_adc2_dma_done = 1U;
    }
    else
    {
        // 其它ADC当前不在adc_app中处理
    }
}

/**
 * @brief  ADC错误回调转发入口
 * @param  hadc: 触发错误回调的ADC句柄
 * @retval None
 * @note   该函数应在HAL_ADC_ErrorCallback中调用。
 *         第一版暂不记录错误计数，避免增加未使用变量；后续需要VOFA观测错误状态时再补充getter。
 */
void ADC_App_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}

/**
 * @brief  根据ADC组ID获取该组ADC的软件资源
 * @param  adc_id: 电机电流ADC组ID，取值见adc_app_motor_adc_t
 * @param  ctx: 输出参数，用于返回该ADC组的DMA buffer、稳定帧、标志和计数指针
 * @retval uint8_t
 *         1: 获取成功
 *         0: adc_id非法或ctx为空
 * @note   该函数只负责集中管理ADC1/ADC3/ADC4的资源映射关系。
 *         不复制数据、不切换buffer、不修改计数，避免映射逻辑散落在多个函数中。
 */
static uint8_t ADC_App_GetMotorContext(adc_app_motor_adc_t adc_id,
                                       adc_app_motor_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return 0U;
    }

    switch (adc_id)
    {
        case ADC_APP_MOTOR_ADC1:
            ctx->dma_buf = s_adc1_raw;
            ctx->frame_buf = s_adc1_frame_raw;
            ctx->done_flag = &s_adc1_dma_done;
            ctx->read_idx = &s_adc1_read_idx;
            ctx->update_count = &s_adc1_update_count;
            return 1U;

        case ADC_APP_MOTOR_ADC3:
            ctx->dma_buf = s_adc3_raw;
            ctx->frame_buf = s_adc3_frame_raw;
            ctx->done_flag = &s_adc3_dma_done;
            ctx->read_idx = &s_adc3_read_idx;
            ctx->update_count = &s_adc3_update_count;
            return 1U;

        case ADC_APP_MOTOR_ADC4:
            ctx->dma_buf = s_adc4_raw;
            ctx->frame_buf = s_adc4_frame_raw;
            ctx->done_flag = &s_adc4_dma_done;
            ctx->read_idx = &s_adc4_read_idx;
            ctx->update_count = &s_adc4_update_count;
            return 1U;

        default:
            return 0U;
    }
}

/**
 * @brief  从DMA buffer生成一帧稳定三相采样
 * @param  adc_id: 电机电流ADC组ID，取值见adc_app_motor_adc_t
 * @retval None
 * @note   该函数只在ADC DMA转换完成回调中调用。
 *         数据流为：DMA raw buffer -> 后台稳定帧 -> 切换read_idx -> update_count++。
 *         控制层始终通过ADC_App_GetMotorRaw()读取当前稳定帧，不直接读取DMA写入区。
 */
static void ADC_App_UpdateMotorFrame(adc_app_motor_adc_t adc_id)
{
    uint8_t i;
    uint8_t write_idx;
    adc_app_motor_ctx_t ctx;

    if (ADC_App_GetMotorContext(adc_id, &ctx) == 0U)
    {
        return;
    }

    write_idx = (*ctx.read_idx == 0U) ? 1U : 0U;

    for (i = 0U; i < ADC_APP_PHASE_NUM; i++)
    {
        ctx.frame_buf[write_idx][i] = ctx.dma_buf[i];
    }

    *ctx.read_idx = write_idx;
    (*ctx.update_count)++;
    if (*ctx.update_count == 0U)
    {
        *ctx.update_count = 1U;
    }
}
