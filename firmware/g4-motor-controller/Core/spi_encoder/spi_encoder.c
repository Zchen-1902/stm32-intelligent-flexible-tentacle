#include "spi_encoder.h"
#include <math.h>
#include <string.h>
#include "../timer_app/timer_app.h"

/************************************************
* MT6816 SPI 编码器驱动
* 功能：读取机械角、设置零点、计算多圈角度和机械角速度
************************************************ */

/****************************************************** 内部宏定义 ************************************************************************** */
#define ENCODER_CYCLE_RAD      (2.0f * M_PI_F) // 机械角单圈周期，单位 rad
#define ENCODER_SPI_BURST_READ_ENABLE 1U     // 1=手册burst read：TX 83 00 00；0=原两次CS分开读0x03/0x04
#if (ENCODER_SPI_BURST_READ_ENABLE != 0U)
#define ENCODER_FRAME_LEN      3U            // MT6816 burst read：TX 83 00 00，RX xx H L/status
#else
#define ENCODER_FRAME_LEN      2U            // MT6816 8bit SPI单寄存器读帧：TX CMD 00，RX xx DATA
#endif
#define ENCODER_RX_CACHE_LEN   4U            // 组合两次读寄存器结果：rx0 H帧dummy，rx1 H，rx2 L帧dummy，rx3 L/status
#define ENCODER_ANGLE_H_CMD    0x83U         // 读0x03：Angle[13:6]
#define ENCODER_ANGLE_L_CMD    0x84U         // 读0x04：Angle[5:0] + No_Mag + Parity
#define ENCODER_FRAME_MASK     0x3FFFU       // MT6816 角度数据按 14bit 处理
#define ENCODER_NO_MAG_MASK    0x02U         // MT6816 0x04[1]，磁场不足报警位
#define ENCODER_PARITY_MASK    0x01U         // MT6816 0x04[0]，偶校验位
#define ENCODER_CS_SETUP_DELAY_LOOP  20U      // CS拉低到首个SCK前的软件延时，约数百ns~1us量级
#define ENCODER_CS_HOLD_DELAY_LOOP   10U      // SPI空闲到CS拉高前的软件延时，避免截断帧尾
#define ENCODER_SPI_IDLE_TIMEOUT     1000U    // 等待SPI不忙的最大轮询次数，防止异常时卡死
#define ENCODER1_RAW_JUMP_REJECT_COUNT 4096U  // encoder1单次raw跳变拒绝阈值，14bit整圈的1/4；用于丢弃偶发0值/毛刺帧
#define ENCODER1_RAW_JUMP_PREDICT_MAX 3U      // encoder1连续毛刺帧最多预测次数，超过后判无效避免长期盲跑

/****************************************************** 编码器数据 ************************************************************************** */
spi_encoder_t spi_encoder[ENCODER_NUM] = {0};   // 3 路编码器的软件数据缓存

/****************************************************** 编码器配置 ************************************************************************** */
static const int8_t s_encoder_dir[ENCODER_NUM] =
{
    ENCODER_DIR_M1,
    ENCODER_DIR_M2,
    ENCODER_DIR_M3,
};

/****************************************************** 内部变量 **************************************************************************** */
// MT6816 SPI发送缓存：burst模式TX 83 00 00；分开读模式TX CMD 00
static uint8_t s_spi_tx_buf[ENCODER_FRAME_LEN] = {ENCODER_ANGLE_H_CMD, 0x00U};
static uint8_t s_spi_rx_buf[ENCODER_RX_CACHE_LEN] = {0U};

static uint8_t s_spi_dma_rx_buf[ENCODER_NUM][ENCODER_RX_CACHE_LEN] = {0U}; // 三路编码器DMA接收缓存
static uint16_t s_spi_dma_raw[ENCODER_NUM] = {0U};                      // 三路编码器DMA解析后的raw缓存
static uint8_t s_spi_dma_raw_valid[ENCODER_NUM] = {0U};                 // 三路raw缓存有效标志

static volatile uint8_t s_spi_dma_busy = 0U;                            // SPI3编码器DMA忙标志
static volatile uint8_t s_spi_dma_index = 0U;                           // 当前正在读取的编码器序号
static volatile uint8_t s_spi_dma_read_low_reg = 0U;                    // 当前DMA是否已完成高字节、准备/正在读低字节
static volatile uint8_t s_spi_dma_single_mode = 0U;                     // 单路DMA模式标志，1表示本轮只读取一个编码器
static volatile uint8_t s_spi_dma_pending_service_active = 0U;          // 当前单路DMA是否来自pending调度，避免污染旧all-DMA时间基准
static volatile uint8_t s_spi_dma_update_done = 0U;                     // DMA读取并更新完成标志，单路/三路DMA完成后都会置位
static volatile uint32_t s_spi_dma_missed_count = 0U;                   // DMA忙时被跳过的启动次数
static volatile uint8_t s_spi_dma_pending_mask = 0U;                    // 单路DMA请求pending位图，bit0/1/2对应三路编码器
static volatile uint8_t s_spi_dma_next_pending_index = 0U;              // pending轮询起点，避免总是优先同一路
static volatile uint32_t s_spi_dma_pending_cycles[ENCODER_NUM] = {0U};  // 每路最近一次请求时间戳，单位DWT cycle
static volatile uint32_t s_spi_dma_pending_overwrite_count[ENCODER_NUM] = {0U}; // pending未消费时被新请求覆盖的次数

static uint32_t s_spi_dma_current_start_us = 0U;                        // 本轮DMA读取启动时间
static uint32_t s_spi_dma_last_success_start_us = 0U;                   // 上一轮成功启动时间
static uint8_t s_spi_dma_has_last_success_time = 0U;                    // 是否已经有上一轮成功采样时间
static float s_spi_dma_dt = 0.0f;                                       // 本轮速度计算使用的时间间隔，单位s
static uint8_t s_speed_calc_initialized[ENCODER_NUM] = {0U};            // TIM7速度低通是否已有基准值
static uint8_t s_encoder1_raw_jump_predict_count = 0U;                  // encoder1 raw毛刺后使用速度预测补帧的连续次数

/****************************************************** 内部函数声明 *********************************************************************** */
static uint8_t SPI_Encoder_Id_Is_Valid(encoder_id_t id);             // 判断编码器编号是否有效
static HAL_StatusTypeDef SPI_Encoder_Parse_Raw_Frame(encoder_id_t id, const uint8_t *rx_buf, uint16_t *raw); // 解析MT6816三字节返回帧
static HAL_StatusTypeDef SPI_Encoder_Update_By_Raw(encoder_id_t id, uint16_t raw_now, float dt); // 使用已缓存raw更新角度和速度
static HAL_StatusTypeDef SPI_Encoder_Start_DMA_By_Index(uint8_t index); // 按序号启动一路编码器DMA读取
static HAL_StatusTypeDef SPI_Encoder_Start_DMA_Burst_Frame(uint8_t index); // 启动一次0x03/0x04 burst read DMA读取
static HAL_StatusTypeDef SPI_Encoder_Start_DMA_Read_Reg(uint8_t index, uint8_t cmd, uint8_t *rx_buf); // 启动一次寄存器DMA读取
static void SPI_Encoder_Update_All_By_DMA_Raw(float dt);              // 使用三路DMA raw缓存统一更新角度和速度
static uint8_t SPI_Encoder_Check_Parity(uint8_t angle_high, uint8_t angle_low_status); // 检查MT6816角度帧偶校验
static float SPI_Encoder_Apply_Direction(encoder_id_t id, float angle_rad); // 按编码器方向和零点偏移换算单圈角
static void SPI_Encoder_Update_Turn_Count(encoder_id_t id, float angle_last, float angle_now, float diff); // 更新显式多圈计数
static GPIO_TypeDef *SPI_Encoder_Get_CS_Port(encoder_id_t id);       // 获取对应编码器的片选端口
static void SPI_Encoder_DelayLoop(uint32_t loop);                    // 短软件延时，用于CS建立/保持时间
static void SPI_Encoder_CS_SetupDelay(void);                         // CS拉低后的建立时间
static void SPI_Encoder_WaitSpiIdle(void);                           // 等待SPI3发送移位完全结束
static uint16_t SPI_Encoder_Get_CS_Pin(encoder_id_t id);             // 获取对应编码器的片选引脚
static void SPI_Encoder_CS_Select(encoder_id_t id);                  // 拉低对应编码器片选
static void SPI_Encoder_CS_Release(encoder_id_t id);                 // 释放对应编码器片选

/****************************************************** 初始化 **************************************************************************** */
/**
 * @brief  初始化 SPI 磁编码器模块的软件状态。
 * @param  None
 * @retval None
 * @note   使用方式：在 MX_GPIO_Init()、MX_SPI3_Init() 之后调用一次。
 *         本函数只初始化软件缓存并释放 CSN1/CSN2/CSN3，不会主动读取编码器。
 */
void SPI_Encoder_Init(void)
{
    uint8_t i = 0U;

    for (i = 0U; i < ENCODER_NUM; i++)
    {
        spi_encoder[i].raw_angle = 0U;
        spi_encoder[i].raw_angle_last = 0U;
        spi_encoder[i].last_rx[0] = 0U;
        spi_encoder[i].last_rx[1] = 0U;
        spi_encoder[i].last_rx[2] = 0U;
        spi_encoder[i].last_rx[3] = 0U;
        spi_encoder[i].angle = 0.0f;
        spi_encoder[i].angle_last = 0.0f;
        spi_encoder[i].angle_offset = 0.0f;
        spi_encoder[i].angle_single = 0.0f;
        spi_encoder[i].angle_total = 0.0f;
        spi_encoder[i].turn_count = 0;
        spi_encoder[i].angle_sample_cycles = 0U;
        spi_encoder[i].speed_raw = 0.0f;
        spi_encoder[i].speed = 0.0f;
        spi_encoder[i].speed_avg_diff_sum = 0.0f;
        spi_encoder[i].speed_avg_cycles_sum = 0U;
        spi_encoder[i].speed_avg_sample_count = 0U;
        spi_encoder[i].initialized = 0U;
        spi_encoder[i].no_mag_warning = 0U;
        spi_encoder[i].parity_error = 0U;
        spi_encoder[i].valid = 0U;

        s_spi_dma_raw[i] = 0U;
        s_spi_dma_raw_valid[i] = 0U;
        s_speed_calc_initialized[i] = 0U;
        s_spi_dma_pending_cycles[i] = 0U;
        s_spi_dma_pending_overwrite_count[i] = 0U;
        memset(s_spi_dma_rx_buf[i], 0, ENCODER_RX_CACHE_LEN);
    }

    s_spi_dma_busy = 0U;
    s_spi_dma_index = 0U;
    s_spi_dma_read_low_reg = 0U;
    s_spi_dma_single_mode = 0U;
    s_spi_dma_pending_service_active = 0U;
    s_spi_dma_update_done = 0U;
    s_spi_dma_pending_mask = 0U;
    s_spi_dma_next_pending_index = 0U;
    s_encoder1_raw_jump_predict_count = 0U;
    s_spi_dma_missed_count = 0U;
    s_spi_dma_current_start_us = 0U;
    s_spi_dma_last_success_start_us = 0U;
    s_spi_dma_has_last_success_time = 0U;
    s_spi_dma_dt = 0.0f;

    // SPI 空闲时片选保持高电平，避免误触发编码器通信
    HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CSN2_GPIO_Port, CSN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CSN3_GPIO_Port, CSN3_Pin, GPIO_PIN_SET);
}

/****************************************************** 原始值读取 ************************************************************************ */
/**
 * @brief  读取指定 MT6816 的 14bit 原始角度值。
 * @param  id   编码器编号，取值见 encoder_id_t。
 * @param  raw  原始角度输出指针，范围 0~16383。
 * @retval HAL_OK 表示读取成功；HAL_ERROR 表示参数错误、SPI失败、磁场报警或校验失败。
 * @note   使用方式：需要单次读取原始角度时调用。
 *         burst模式从0x03开始连续读0x03/0x04；分开读模式保留原两次CS读取。
 */
HAL_StatusTypeDef SPI_Encoder_Read_Raw(encoder_id_t id, uint16_t *raw)
{
    HAL_StatusTypeDef status;

    if ((raw == NULL) || (SPI_Encoder_Id_Is_Valid(id) == 0U))
    {
        return HAL_ERROR;
    }

    if (s_spi_dma_busy != 0U)
    {
        return HAL_BUSY;
    }

    memset(s_spi_rx_buf, 0, ENCODER_RX_CACHE_LEN);

#if (ENCODER_SPI_BURST_READ_ENABLE != 0U)
    s_spi_tx_buf[0] = ENCODER_ANGLE_H_CMD;
    s_spi_tx_buf[1] = 0x00U;
    s_spi_tx_buf[2] = 0x00U;

    SPI_Encoder_CS_Select(id);
    SPI_Encoder_CS_SetupDelay();
    status = HAL_SPI_TransmitReceive(&hspi3, s_spi_tx_buf, s_spi_rx_buf, ENCODER_FRAME_LEN, ENCODER_TIMEOUT);
    SPI_Encoder_CS_Release(id);

    if (status != HAL_OK)
    {
        spi_encoder[id].valid = 0U;
        return HAL_ERROR;
    }
#else
    s_spi_tx_buf[0] = ENCODER_ANGLE_H_CMD;
    s_spi_tx_buf[1] = 0x00U;
    SPI_Encoder_CS_Select(id);
    SPI_Encoder_CS_SetupDelay();
    status = HAL_SPI_TransmitReceive(&hspi3, s_spi_tx_buf, &s_spi_rx_buf[0], ENCODER_FRAME_LEN, ENCODER_TIMEOUT);
    SPI_Encoder_CS_Release(id);

    if (status != HAL_OK)
    {
        spi_encoder[id].valid = 0U;
        return HAL_ERROR;
    }

    s_spi_tx_buf[0] = ENCODER_ANGLE_L_CMD;
    s_spi_tx_buf[1] = 0x00U;
    SPI_Encoder_CS_Select(id);
    SPI_Encoder_CS_SetupDelay();
    status = HAL_SPI_TransmitReceive(&hspi3, s_spi_tx_buf, &s_spi_rx_buf[2], ENCODER_FRAME_LEN, ENCODER_TIMEOUT);
    SPI_Encoder_CS_Release(id);

    if (status != HAL_OK)
    {
        spi_encoder[id].valid = 0U;
        return HAL_ERROR;
    }
#endif

    return SPI_Encoder_Parse_Raw_Frame(id, s_spi_rx_buf, raw);
}

/**
 * @brief  读取指定编码器当前单圈机械角。
 * @param  id         编码器编号，取值见 encoder_id_t。
 * @param  angle_rad  单圈机械角输出指针，单位 rad，范围约 0~2pi。
 * @retval HAL_OK 表示读取成功；HAL_ERROR 表示参数错误或原始角读取失败。
 * @note   使用方式：只需要一次性角度值、不需要多圈和速度时调用。
 *         若用于闭环控制，更推荐周期调用 SPI_Encoder_Update()。
 */
HAL_StatusTypeDef SPI_Encoder_Read_Angle(encoder_id_t id, float *angle_rad)
{
    uint16_t raw = 0U;

    if ((angle_rad == NULL) || (SPI_Encoder_Id_Is_Valid(id) == 0U))
    {
        return HAL_ERROR;
    }

    if (SPI_Encoder_Read_Raw(id, &raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 与头文件中的 ENCODER_RAD_STEP 保持一致
    *angle_rad = (float)raw * ENCODER_RAD_STEP;
    return HAL_OK;
}

/****************************************************** 角度更新 **************************************************************************** */
/**
 * @brief  更新指定编码器的单圈角、多圈角和机械角速度。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @param  dt  两次调用之间的时间间隔，单位 s，必须大于 0。
 * @retval HAL_OK 表示更新成功；HAL_ERROR 表示参数错误或编码器读取失败。
 * @note   使用方式：在固定周期任务中调用，例如 1ms/5ms 周期。
 *         首次有效读取只建立初始基准，不计算 diff 和 speed，避免上电速度尖峰。
 */
HAL_StatusTypeDef SPI_Encoder_Update(encoder_id_t id, float dt)
{
    uint16_t raw_now = 0U;      // 当前读取到的原始角度值

    if ((SPI_Encoder_Id_Is_Valid(id) == 0U) || (dt <= 0.0f))
    {
        return HAL_ERROR;
    }

    if (s_spi_dma_busy != 0U)
    {
        return HAL_BUSY;
    }

    // 先读取原始角度值
    if (SPI_Encoder_Read_Raw(id, &raw_now) != HAL_OK)
    {
        spi_encoder[id].valid = 0U;
        return HAL_ERROR;
    }

    return SPI_Encoder_Update_By_Raw(id, raw_now, dt);
}

/****************************************************** DMA更新 **************************************************************************** */
/**
 * @brief  启动一次三路编码器DMA读取。
 * @param  now_us  本轮采样启动时间，单位us，建议来自固定时基计数。
 * @retval HAL_OK表示成功启动；HAL_BUSY表示上一轮DMA尚未结束；HAL_ERROR表示启动失败。
 * @note   使用方式：在固定周期任务中调用，例如TIM7 5ms任务。
 *         本函数只启动第一路DMA，后续两路会在DMA完成回调中自动串行启动。
 */
HAL_StatusTypeDef SPI_Encoder_Start_Update_All_DMA(uint32_t now_us)
{
    HAL_StatusTypeDef status;
    uint8_t i = 0U;

    if (s_spi_dma_busy != 0U)
    {
        s_spi_dma_missed_count++;
        return HAL_BUSY;
    }

    for (i = 0U; i < ENCODER_NUM; i++)
    {
        s_spi_dma_raw_valid[i] = 0U;
    }

    s_spi_dma_update_done = 0U;
    s_spi_dma_current_start_us = now_us;

    if (s_spi_dma_has_last_success_time == 0U)
    {
        s_spi_dma_dt = 0.0f;
    }
    else
    {
        s_spi_dma_dt = (float)(s_spi_dma_current_start_us - s_spi_dma_last_success_start_us) * 0.000001f;
    }

    s_spi_dma_index = 0U;
    s_spi_dma_read_low_reg = 0U;
    s_spi_dma_single_mode = 0U;
    s_spi_dma_pending_service_active = 0U;
    s_spi_dma_busy = 1U;

    status = SPI_Encoder_Start_DMA_By_Index(0U);
    if (status != HAL_OK)
    {
        s_spi_dma_busy = 0U;
        s_spi_dma_read_low_reg = 0U;
        s_spi_dma_missed_count++;
        return status;
    }

    return HAL_OK;
}

/**
 * @brief  启动一次单路编码器DMA读取。
 * @param  id      编码器编号，取值见 encoder_id_t。
 * @param  now_us  本次采样启动时间，单位us，用于记录本次角度采样时刻。
 * @retval HAL_OK表示成功启动；HAL_BUSY表示SPI DMA忙；HAL_ERROR表示参数错误或DMA启动失败。
 * @note   使用方式：由PWM定时器事件调用，用于刷新指定编码器角度缓存。
 *         本函数只启动DMA，不等待完成；角度解析和缓存更新在SPI DMA完成回调中执行。
 */
HAL_StatusTypeDef SPI_Encoder_Start_Update_One_DMA(encoder_id_t id, uint32_t now_us)
{
    HAL_StatusTypeDef status;

    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return HAL_ERROR;
    }

    if (s_spi_dma_busy != 0U)
    {
        s_spi_dma_missed_count++;
        return HAL_BUSY;
    }

    s_spi_dma_update_done = 0U;
    s_spi_dma_current_start_us = now_us;
    s_spi_dma_index = (uint8_t)id;
    s_spi_dma_read_low_reg = 0U;
    s_spi_dma_single_mode = 1U;
    s_spi_dma_pending_service_active = 0U;
    s_spi_dma_raw_valid[s_spi_dma_index] = 0U;
    s_spi_dma_busy = 1U;

    status = SPI_Encoder_Start_DMA_By_Index(s_spi_dma_index);
    if (status != HAL_OK)
    {
        s_spi_dma_busy = 0U;
        s_spi_dma_read_low_reg = 0U;
        s_spi_dma_single_mode = 0U;
        s_spi_dma_missed_count++;
        return status;
    }

    return HAL_OK;
}

/**
 * @brief  提交一路编码器DMA刷新请求。
 * @param  id          编码器编号，取值见 encoder_id_t。
 * @param  now_cycles  请求发生时的DWT cycle时间戳，仅用于后续调试/扩展。
 * @retval HAL_OK表示请求已记录；HAL_ERROR表示id无效。
 * @note   本函数不直接启动SPI，不等待DMA完成。
 *         如果同一路pending尚未被消费，新请求会覆盖旧请求时间戳，只保留最新一次需求。
 */
HAL_StatusTypeDef SPI_Encoder_Request_Update_DMA(encoder_id_t id, uint32_t now_cycles)
{
    uint32_t primask;
    uint8_t bit;

    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return HAL_ERROR;
    }

    bit = (uint8_t)(1U << (uint8_t)id);

    primask = __get_PRIMASK();
    __disable_irq();
    if ((s_spi_dma_pending_mask & bit) != 0U)
    {
        s_spi_dma_pending_overwrite_count[(uint8_t)id]++;
    }
    s_spi_dma_pending_cycles[(uint8_t)id] = now_cycles;
    s_spi_dma_pending_mask |= bit;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return HAL_OK;
}

/**
 * @brief  SPI空闲时最多启动一路pending编码器DMA。
 * @param  None
 * @retval HAL_OK表示已启动一路DMA；HAL_BUSY表示SPI忙或当前没有pending；HAL_ERROR表示启动失败。
 * @note   本函数只消费一路pending，不在循环里连续启动多路，避免ADC回调中形成过长执行时间。
 */
HAL_StatusTypeDef SPI_Encoder_ServicePendingOnce(void)
{
    HAL_StatusTypeDef status;
    uint32_t primask;
    uint8_t pending_mask;
    uint8_t selected = ENCODER_NUM;
    uint8_t i;

    if ((s_spi_dma_busy != 0U) || (s_spi_dma_pending_mask == 0U))
    {
        return HAL_BUSY;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    pending_mask = s_spi_dma_pending_mask;
    for (i = 0U; i < ENCODER_NUM; i++)
    {
        const uint8_t index = (uint8_t)((s_spi_dma_next_pending_index + i) % ENCODER_NUM);
        const uint8_t bit = (uint8_t)(1U << index);

        if ((pending_mask & bit) != 0U)
        {
            selected = index;
            s_spi_dma_pending_mask &= (uint8_t)(~bit);
            s_spi_dma_next_pending_index = (uint8_t)((index + 1U) % ENCODER_NUM);
            break;
        }
    }

    if (selected >= ENCODER_NUM)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return HAL_BUSY;
    }

    s_spi_dma_update_done = 0U;
    s_spi_dma_current_start_us = 0U;
    s_spi_dma_index = selected;
    s_spi_dma_read_low_reg = 0U;
    s_spi_dma_single_mode = 1U;
    s_spi_dma_pending_service_active = 1U;
    s_spi_dma_raw_valid[selected] = 0U;
    s_spi_dma_busy = 1U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    status = SPI_Encoder_Start_DMA_By_Index(selected);
    if (status != HAL_OK)
    {
        s_spi_dma_busy = 0U;
        s_spi_dma_read_low_reg = 0U;
        s_spi_dma_single_mode = 0U;
        s_spi_dma_pending_service_active = 0U;
        s_spi_dma_missed_count++;
        return status;
    }

    return HAL_OK;
}

/**
 * @brief  SPI3 DMA收发完成回调入口。
 * @param  hspi  HAL传入的SPI句柄。
 * @retval None
 * @note   使用方式：在 HAL_SPI_TxRxCpltCallback() 中调用本函数。
 *         三路DMA模式：每完成一路后自动启动下一路，三路完成后统一更新角度。
 *         单路DMA模式：完成后只更新当前编码器角度缓存和样本累计，不连续拉起下一路DMA。
 */
void SPI_Encoder_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef status;
    encoder_id_t id;
    uint16_t raw_now = 0U;

    if ((hspi != &hspi3) || (s_spi_dma_busy == 0U))
    {
        return;
    }

    if (s_spi_dma_index >= ENCODER_NUM)
    {
        s_spi_dma_busy = 0U;
        s_spi_dma_read_low_reg = 0U;
        s_spi_dma_single_mode = 0U;
        s_spi_dma_pending_service_active = 0U;
        return;
    }

    id = (encoder_id_t)s_spi_dma_index;
    SPI_Encoder_CS_Release(id);

#if (ENCODER_SPI_BURST_READ_ENABLE == 0U)
    if (s_spi_dma_read_low_reg == 0U)
    {
        s_spi_dma_read_low_reg = 1U;
        status = SPI_Encoder_Start_DMA_Read_Reg(s_spi_dma_index,
                                                ENCODER_ANGLE_L_CMD,
                                                &s_spi_dma_rx_buf[s_spi_dma_index][2]);
        if (status != HAL_OK)
        {
            s_spi_dma_busy = 0U;
            s_spi_dma_read_low_reg = 0U;
            s_spi_dma_single_mode = 0U;
            s_spi_dma_pending_service_active = 0U;
            s_spi_dma_missed_count++;
        }
        return;
    }

    s_spi_dma_read_low_reg = 0U;
#endif

    if (SPI_Encoder_Parse_Raw_Frame(id, s_spi_dma_rx_buf[s_spi_dma_index], &raw_now) == HAL_OK)
    {
        s_spi_dma_raw[s_spi_dma_index] = raw_now;
        s_spi_dma_raw_valid[s_spi_dma_index] = 1U;
    }
    else
    {
        s_spi_dma_raw_valid[s_spi_dma_index] = 0U;
    }

    if (s_spi_dma_single_mode != 0U)
    {
        if (s_spi_dma_raw_valid[s_spi_dma_index] != 0U)
        {
            (void)SPI_Encoder_Update_By_Raw(id, raw_now, 0.0f);
        }
        else
        {
            s_spi_dma_missed_count++;
        }

        if (s_spi_dma_pending_service_active == 0U)
        {
            s_spi_dma_last_success_start_us = s_spi_dma_current_start_us;
            s_spi_dma_has_last_success_time = 1U;
        }
        s_spi_dma_busy = 0U;
        s_spi_dma_read_low_reg = 0U;
        s_spi_dma_single_mode = 0U;
        s_spi_dma_pending_service_active = 0U;
        s_spi_dma_update_done = 1U;
        return;
    }

    s_spi_dma_index++;

    if (s_spi_dma_index < ENCODER_NUM)
    {
        status = SPI_Encoder_Start_DMA_By_Index(s_spi_dma_index);
        if (status != HAL_OK)
        {
            s_spi_dma_busy = 0U;
            s_spi_dma_read_low_reg = 0U;
            s_spi_dma_pending_service_active = 0U;
            s_spi_dma_missed_count++;
        }
        return;
    }

    SPI_Encoder_Update_All_By_DMA_Raw(s_spi_dma_dt);
    s_spi_dma_last_success_start_us = s_spi_dma_current_start_us;
    s_spi_dma_has_last_success_time = 1U;
    s_spi_dma_busy = 0U;
    s_spi_dma_read_low_reg = 0U;
    s_spi_dma_pending_service_active = 0U;
    s_spi_dma_update_done = 1U;
}

/**
 * @brief  SPI3错误回调入口。
 * @param  hspi  HAL传入的SPI句柄。
 * @retval None
 * @note   使用方式：在 HAL_SPI_ErrorCallback() 中调用本函数。
 *         如果DMA过程中出错，会释放当前CSN并退出忙状态，避免系统卡死在busy。
 */
void SPI_Encoder_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != &hspi3) || (s_spi_dma_busy == 0U))
    {
        return;
    }

    if (s_spi_dma_index < ENCODER_NUM)
    {
        SPI_Encoder_CS_Release((encoder_id_t)s_spi_dma_index);
    }

    s_spi_dma_busy = 0U;
    s_spi_dma_read_low_reg = 0U;
    s_spi_dma_single_mode = 0U;
    s_spi_dma_pending_service_active = 0U;
    s_spi_dma_missed_count++;
}

/**
 * @brief  判断编码器DMA是否忙。
 * @param  None
 * @retval 1表示正在DMA读取，0表示空闲。
 * @note   使用方式：上层调度可用来判断是否允许启动下一轮编码器读取。
 */
uint8_t SPI_Encoder_Is_DMA_Busy(void)
{
    return s_spi_dma_busy;
}

/**
 * @brief  判断最近一次编码器DMA更新是否完成。
 * @param  None
 * @retval 1表示完成，0表示未完成。
 * @note   使用方式：主循环或调试任务可以根据该标志决定是否读取最新角度缓存。
 *         单路DMA和三路DMA完成后都会置位该标志。
 */
uint8_t SPI_Encoder_Is_Update_Done(void)
{
    return s_spi_dma_update_done;
}

/**
 * @brief  清除DMA更新完成标志。
 * @param  None
 * @retval None
 * @note   使用方式：上层读取完本轮角度缓存后调用。
 */
void SPI_Encoder_Clear_Update_Done(void)
{
    s_spi_dma_update_done = 0U;
}

/**
 * @brief  获取DMA忙导致跳过更新的次数。
 * @param  None
 * @retval 跳过次数。
 * @note   使用方式：调试SPI读取周期是否过快，若持续增加说明采样周期太短或SPI异常。
 */
uint32_t SPI_Encoder_Get_DMA_Missed_Count(void)
{
    return s_spi_dma_missed_count;
}

/**
 * @brief  将速度环周期内的SPI样本累计量转换为平均速度。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @param  dt  TIM7调用周期，单位s，仅用于参数合法性检查。
 * @retval HAL_OK表示计算成功；HAL_BUSY表示本周期无新样本；HAL_ERROR表示参数错误或编码器尚无有效角度。
 * @note   使用方式：由TIM7固定周期调用。
 *         速度语义为“上一次速度环调用后所有有效SPI样本的累计角度差 / 累计真实DWT时间”。
 *         这样比直接低通最近一次speed_raw更抗量化噪声，且延迟小于重低通单样本速度。
 */
HAL_StatusTypeDef SPI_Encoder_Update_Speed_By_Cache(encoder_id_t id, float dt)
{
    uint32_t primask;
    float diff_sum;
    uint32_t cycles_sum;
    uint16_t sample_count;
    float speed_avg;
    float speed_dt_s;

    if ((SPI_Encoder_Id_Is_Valid(id) == 0U) || (dt <= 0.0f) || (spi_encoder[id].valid == 0U))
    {
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    diff_sum = spi_encoder[id].speed_avg_diff_sum;
    cycles_sum = spi_encoder[id].speed_avg_cycles_sum;
    sample_count = spi_encoder[id].speed_avg_sample_count;
    spi_encoder[id].speed_avg_diff_sum = 0.0f;
    spi_encoder[id].speed_avg_cycles_sum = 0U;
    spi_encoder[id].speed_avg_sample_count = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    if ((sample_count == 0U) || (cycles_sum == 0U))
    {
        return HAL_BUSY;
    }

    speed_dt_s = (float)cycles_sum / (float)SystemCoreClock;
    speed_avg = diff_sum / speed_dt_s;
    if (speed_avg > ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S)
    {
        speed_avg = ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S;
    }
    else if (speed_avg < -ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S)
    {
        speed_avg = -ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S;
    }

    if (s_speed_calc_initialized[id] == 0U)
    {
        spi_encoder[id].speed = speed_avg;
        s_speed_calc_initialized[id] = 1U;
        return HAL_OK;
    }

    spi_encoder[id].speed += ENCODER_SPEED_LPF_ALPHA * (speed_avg - spi_encoder[id].speed);
    return HAL_OK;
}

/****************************************************** 零点设置 **************************************************************************** */
/**
 * @brief  将当前编码器角度设置为机械零点。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval None
 * @note   使用方式：至少完成一次 SPI_Encoder_Update() 后调用。
 *         调用后 angle_single、angle_total、turn_count、speed 都会清零。
 */
void SPI_Encoder_Set_Zero(encoder_id_t id)
{
    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return;
    }

    spi_encoder[id].angle_offset = spi_encoder[id].angle;
    spi_encoder[id].angle_single = 0.0f;
    spi_encoder[id].angle_total = 0.0f;
    spi_encoder[id].turn_count = 0;
    spi_encoder[id].speed_raw = 0.0f;
    spi_encoder[id].speed = 0.0f;
    spi_encoder[id].speed_avg_diff_sum = 0.0f;
    spi_encoder[id].speed_avg_cycles_sum = 0U;
    spi_encoder[id].speed_avg_sample_count = 0U;
    spi_encoder[id].angle_last = spi_encoder[id].angle;
    spi_encoder[id].raw_angle_last = spi_encoder[id].raw_angle;
    spi_encoder[id].angle_sample_cycles = Timer_App_GetRuntimeCycles();
}

/**
 * @brief  按当前单圈角恢复编码器多圈累计角。
 * @param  id 编码器编号。
 * @param  target_total_rad 希望恢复到的多圈机械角，单位rad。
 * @param  max_error_rad 允许的单圈匹配误差，单位rad；超过则认为断电期间机构可能被移动。
 * @retval 1恢复成功，0恢复失败。
 * @note   本函数只修改turn_count、angle_total和速度缓存，不修改angle_offset，
 *         因此不会改变FOC电角度零点。
 */
uint8_t SPI_Encoder_Restore_Total_Angle(encoder_id_t id, float target_total_rad, float max_error_rad)
{
    float turns_f;
    float restored_total;
    float error;
    int32_t turn_count;

    if ((SPI_Encoder_Id_Is_Valid(id) == 0U) || (spi_encoder[id].initialized == 0U))
    {
        return 0U;
    }

    turns_f = (target_total_rad - spi_encoder[id].angle_single) / ENCODER_CYCLE_RAD;
    turn_count = (turns_f >= 0.0f) ? (int32_t)(turns_f + 0.5f)
                                   : (int32_t)(turns_f - 0.5f);

    restored_total = ((float)turn_count * ENCODER_CYCLE_RAD) + spi_encoder[id].angle_single;
    error = target_total_rad - restored_total;
    if (error < 0.0f)
    {
        error = -error;
    }

    if ((max_error_rad > 0.0f) && (error > max_error_rad))
    {
        return 0U;
    }

    if (turn_count > ENCODER_MULTI_TURN_MAX_ABS)
    {
        turn_count = ENCODER_MULTI_TURN_MAX_ABS;
    }
    else if (turn_count < -ENCODER_MULTI_TURN_MAX_ABS)
    {
        turn_count = -ENCODER_MULTI_TURN_MAX_ABS;
    }

    spi_encoder[id].turn_count = turn_count;
    spi_encoder[id].angle_total = ((float)turn_count * ENCODER_CYCLE_RAD) + spi_encoder[id].angle_single;
    spi_encoder[id].speed_raw = 0.0f;
    spi_encoder[id].speed = 0.0f;
    spi_encoder[id].speed_avg_diff_sum = 0.0f;
    spi_encoder[id].speed_avg_cycles_sum = 0U;
    spi_encoder[id].speed_avg_sample_count = 0U;
    spi_encoder[id].angle_sample_cycles = Timer_App_GetRuntimeCycles();

    return 1U;
}

/**
 * @brief  手动设置编码器机械零点偏移。
 * @param  id          编码器编号，取值见 encoder_id_t。
 * @param  offset_rad  机械零点偏移，单位 rad。
 * @retval None
 * @note   使用方式：当零点来自标定参数而不是现场 Set_Zero() 时调用。
 *         设置后会立即刷新 angle_single，并重置多圈计数和速度。
 */
void SPI_Encoder_Set_Offset(encoder_id_t id, float offset_rad)
{
    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return;
    }

    spi_encoder[id].angle_offset = SPI_Encoder_Normalize_Angle(offset_rad);

    // 同步刷新当前单圈机械角，保证设置偏移后立即生效
    spi_encoder[id].angle_single = SPI_Encoder_Apply_Direction(id, spi_encoder[id].angle);
    spi_encoder[id].angle_total = spi_encoder[id].angle_single;
    spi_encoder[id].turn_count = 0;
    spi_encoder[id].speed_raw = 0.0f;
    spi_encoder[id].speed = 0.0f;
    spi_encoder[id].speed_avg_diff_sum = 0.0f;
    spi_encoder[id].speed_avg_cycles_sum = 0U;
    spi_encoder[id].speed_avg_sample_count = 0U;
    spi_encoder[id].angle_sample_cycles = Timer_App_GetRuntimeCycles();
}

/****************************************************** 数据获取 **************************************************************************** */
/**
 * @brief  获取当前扣除零点和方向后的单圈机械角。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval 单圈机械角，单位 rad，范围 0~2pi；id无效时返回0。
 * @note   使用方式：读取最近一次 SPI_Encoder_Update() 计算后的缓存值，不会触发SPI通信。
 */
float SPI_Encoder_Get_Angle(encoder_id_t id)
{
    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return 0.0f;
    }

    return spi_encoder[id].angle_single;
}

/**
 * @brief  获取当前多圈累计机械角。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval 多圈累计机械角，单位 rad；id无效时返回0。
 * @note   使用方式：用于位置环、线轮角度/绳长换算等需要连续角度的场景。
 */
float SPI_Encoder_Get_Total_Angle(encoder_id_t id)
{
    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return 0.0f;
    }

    return spi_encoder[id].angle_total;
}

/**
 * @brief  获取当前显式多圈计数。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval 当前多圈计数；id无效时返回0。
 * @note   使用方式：用于判断机构是否跨圈、是否超过允许行程范围。
 */
int32_t SPI_Encoder_Get_Turn_Count(encoder_id_t id)
{
    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return 0;
    }

    return spi_encoder[id].turn_count;
}

/**
 * @brief  获取当前滤波后的机械角速度。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval 机械角速度，单位 rad/s；id无效时返回0。
 * @note   使用方式：读取最近一次 SPI_Encoder_Update() 的速度缓存值，不会触发SPI通信。
 */
float SPI_Encoder_Get_Speed(encoder_id_t id)
{
    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return 0.0f;
    }

    return spi_encoder[id].speed;
}

/**
 * @brief  获取编码器快环快照，不做id和指针合法性检查。
 * @param  id        编码器编号，由初始化阶段固定绑定保证有效。
 * @param  snapshot  快照输出指针，由调用方保证有效。
 * @retval None
 * @note   使用方式：仅用于高速控制路径；公共/调试路径仍使用带检查的getter。
 */
void SPI_Encoder_Get_FastSnapshot_Unchecked(encoder_id_t id, spi_encoder_fast_snapshot_t *snapshot)
{
    const spi_encoder_t *encoder = &spi_encoder[id];

    snapshot->angle_total = encoder->angle_total;
    snapshot->speed_raw = encoder->speed_raw;
    snapshot->speed = encoder->speed;
    snapshot->sample_cycles = encoder->angle_sample_cycles;
    snapshot->valid = encoder->valid;
}

/**
 * @brief  判断最近一次编码器读取是否有效。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval 1表示最近一次数据有效，0表示无效或id错误。
 * @note   使用方式：上层状态机可在使用角度前先检查该标志。
 */
uint8_t SPI_Encoder_Is_Valid(encoder_id_t id)
{
    if (SPI_Encoder_Id_Is_Valid(id) == 0U)
    {
        return 0U;
    }

    return spi_encoder[id].valid;
}

/****************************************************** 工具函数 **************************************************************************** */
/**
 * @brief  将角度归一化到 0~2pi。
 * @param  angle  输入角度，单位 rad，可以为负数或超过 2pi。
 * @retval 归一化后的角度，单位 rad，范围 [0, 2pi)。
 * @note   使用方式：用于单圈角度、零点偏移等周期量归一化。
 *         这里使用 fmodf 做浮点取余，避免大角度输入时 while 循环次数不可控。
 */
float SPI_Encoder_Normalize_Angle(float angle)
{
    if (!isfinite(angle))
    {
        return 0.0f;
    }

    angle = fmodf(angle, ENCODER_CYCLE_RAD);

    if (angle < 0.0f)
    {
        angle += ENCODER_CYCLE_RAD;
    }

    return angle;
}

/**
 * @brief  计算周期量的最短差值。
 * @param  diff   原始差值，例如 angle_now - angle_last。
 * @param  cycle  周期长度，例如 2pi。
 * @retval 最短路径差值，范围约为 [-cycle/2, cycle/2]。
 * @note   使用方式：用于跨零点角度差计算。
 *         例如从 359deg 到 1deg，差值应接近 +2deg，而不是 -358deg。
 */
float SPI_Encoder_Cycle_Diff(float diff, float cycle)
{
    if (diff > (cycle / 2.0f))
    {
        diff -= cycle;
    }
    else if (diff < (-cycle / 2.0f))
    {
        diff += cycle;
    }

    return diff;
}

/****************************************************** 内部函数 **************************************************************************** */
/**
 * @brief  判断编码器编号是否有效。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval 1表示有效，0表示无效。
 * @note   使用方式：本文件内部访问 spi_encoder[] 数组前调用，避免数组越界。
 */
static uint8_t SPI_Encoder_Id_Is_Valid(encoder_id_t id)
{
    if ((uint32_t)id >= ENCODER_NUM)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  解析 MT6816 角度寄存器返回帧。
 * @param  id      编码器编号，取值见 encoder_id_t。
 * @param  rx_buf  SPI接收缓存，长度至少为4字节。
 * @param  raw     解析出的14bit原始角度输出指针。
 * @retval HAL_OK表示解析成功；HAL_ERROR表示参数错误、磁场报警或奇偶校验失败。
 * @note   使用方式：阻塞读取和DMA读取共用该函数，保证两条路径的数据解析一致。
 *         burst模式为RX xx H L；分开读模式为RX xx H xx L。
 */
static HAL_StatusTypeDef SPI_Encoder_Parse_Raw_Frame(encoder_id_t id, const uint8_t *rx_buf, uint16_t *raw)
{
    uint16_t raw_now = 0U;
    uint8_t angle_high = 0U;
    uint8_t angle_low_status = 0U;

    if ((raw == NULL) || (rx_buf == NULL) || (SPI_Encoder_Id_Is_Valid(id) == 0U))
    {
        return HAL_ERROR;
    }

    spi_encoder[id].last_rx[0] = rx_buf[0];
    spi_encoder[id].last_rx[1] = rx_buf[1];
    spi_encoder[id].last_rx[2] = rx_buf[2];
    spi_encoder[id].last_rx[3] = rx_buf[3];

    angle_high = rx_buf[1];
#if (ENCODER_SPI_BURST_READ_ENABLE != 0U)
    angle_low_status = rx_buf[2];
#else
    angle_low_status = rx_buf[3];
#endif

    spi_encoder[id].no_mag_warning = ((angle_low_status & ENCODER_NO_MAG_MASK) != 0U) ? 1U : 0U;
    spi_encoder[id].parity_error = (SPI_Encoder_Check_Parity(angle_high, angle_low_status) == 0U) ? 1U : 0U;

    if ((spi_encoder[id].no_mag_warning != 0U) || (spi_encoder[id].parity_error != 0U))
    {
        spi_encoder[id].valid = 0U;
        return HAL_ERROR;
    }

    raw_now = ((uint16_t)angle_high << 6) | (((uint16_t)angle_low_status >> 2) & 0x003FU);
    raw_now &= ENCODER_FRAME_MASK;

    *raw = raw_now;
    spi_encoder[id].valid = 1U;

    return HAL_OK;
}

/**
 * @brief  使用已解析的raw值更新编码器角度、多圈计数和速度。
 * @param  id       编码器编号，取值见 encoder_id_t。
 * @param  raw_now  当前14bit原始角度值。
 * @param  dt       两次有效更新之间的时间间隔，单位s。
 * @retval HAL_OK表示更新成功；HAL_ERROR表示参数错误或dt无效。
 * @note   使用方式：阻塞更新和DMA统一更新共用该函数。
 *         速度优先使用DWT样本时间戳计算；dt只作为同周期时间戳异常时的备用输入。
 */
static HAL_StatusTypeDef SPI_Encoder_Update_By_Raw(encoder_id_t id, uint16_t raw_now, float dt)
{
    float angle_now = 0.0f;
    float angle_single_now = 0.0f;
    float angle_single_last = 0.0f;
    float diff = 0.0f;
    float speed_dt_s = 0.0f;
    uint32_t sample_cycles;
    uint32_t delta_cycles;
    uint16_t raw_last;
    uint16_t raw_diff;
    float predict_diff;
    uint32_t primask;

    sample_cycles = Timer_App_GetRuntimeCycles();
    angle_now = (float)raw_now * ENCODER_RAD_STEP;
    angle_single_now = SPI_Encoder_Apply_Direction(id, angle_now);

    if (spi_encoder[id].initialized == 0U)
    {
        spi_encoder[id].raw_angle = raw_now;
        spi_encoder[id].raw_angle_last = raw_now;
        spi_encoder[id].angle = angle_now;
        spi_encoder[id].angle_last = angle_now;
        spi_encoder[id].angle_single = angle_single_now;
        spi_encoder[id].angle_total = angle_single_now;
        spi_encoder[id].turn_count = 0;
        spi_encoder[id].angle_sample_cycles = sample_cycles;
        spi_encoder[id].speed_raw = 0.0f;
        spi_encoder[id].speed = 0.0f;
        spi_encoder[id].speed_avg_diff_sum = 0.0f;
        spi_encoder[id].speed_avg_cycles_sum = 0U;
        spi_encoder[id].speed_avg_sample_count = 0U;
        spi_encoder[id].initialized = 1U;
        spi_encoder[id].valid = 1U;
        return HAL_OK;
    }

    if (id == encoder_id_1)
    {
        raw_last = spi_encoder[id].raw_angle;
        if (raw_now >= raw_last)
        {
            raw_diff = raw_now - raw_last;
        }
        else
        {
            raw_diff = raw_last - raw_now;
        }

        if (raw_diff > (ENCODER_COUNT_PER_TURN - raw_diff))
        {
            raw_diff = ENCODER_COUNT_PER_TURN - raw_diff;
        }

        if (raw_diff > ENCODER1_RAW_JUMP_REJECT_COUNT)
        {
            delta_cycles = sample_cycles - spi_encoder[id].angle_sample_cycles;
            if (delta_cycles != 0U)
            {
                speed_dt_s = (float)delta_cycles / (float)SystemCoreClock;
            }
            else if (dt > 0.0f)
            {
                speed_dt_s = dt;
            }

            if ((speed_dt_s <= 0.0f) || (s_encoder1_raw_jump_predict_count >= ENCODER1_RAW_JUMP_PREDICT_MAX))
            {
                spi_encoder[id].valid = 0U;
                return HAL_ERROR;
            }

            /*
             * encoder1偶发raw跳变时，不接受坏raw，也不让快环丢失电角度。
             * 使用上一帧speed_raw外推一帧angle_total，保持FOC电角度连续。
             */
            predict_diff = spi_encoder[id].speed_raw * speed_dt_s;
            spi_encoder[id].angle_last = spi_encoder[id].angle;
            angle_single_last = spi_encoder[id].angle_single;

            spi_encoder[id].angle_total += predict_diff;
            spi_encoder[id].angle_single = SPI_Encoder_Normalize_Angle(angle_single_last + predict_diff);
            spi_encoder[id].angle_sample_cycles = sample_cycles;
            spi_encoder[id].valid = 1U;
            s_encoder1_raw_jump_predict_count++;

            if (delta_cycles != 0U)
            {
                primask = __get_PRIMASK();
                __disable_irq();
                spi_encoder[id].speed_avg_diff_sum += predict_diff;
                spi_encoder[id].speed_avg_cycles_sum += delta_cycles;
                if (spi_encoder[id].speed_avg_sample_count < UINT16_MAX)
                {
                    spi_encoder[id].speed_avg_sample_count++;
                }
                if (primask == 0U)
                {
                    __enable_irq();
                }
            }

            return HAL_OK;
        }
    }

    if (id == encoder_id_1)
    {
        s_encoder1_raw_jump_predict_count = 0U;
    }

    spi_encoder[id].raw_angle_last = spi_encoder[id].raw_angle;
    spi_encoder[id].angle_last = spi_encoder[id].angle;
    angle_single_last = spi_encoder[id].angle_single;

    spi_encoder[id].raw_angle = raw_now;
    spi_encoder[id].angle = angle_now;
    spi_encoder[id].angle_single = angle_single_now;

    diff = SPI_Encoder_Cycle_Diff(angle_single_now - angle_single_last, ENCODER_CYCLE_RAD);
    SPI_Encoder_Update_Turn_Count(id, angle_single_last, angle_single_now, diff);

    if (ENCODER_MULTI_TURN_ENABLE != 0)
    {
        spi_encoder[id].angle_total = ((float)spi_encoder[id].turn_count * ENCODER_CYCLE_RAD) + angle_single_now;
    }
    else
    {
        spi_encoder[id].angle_total = angle_single_now;
    }

    delta_cycles = sample_cycles - spi_encoder[id].angle_sample_cycles;
    if (delta_cycles != 0U)
    {
        speed_dt_s = (float)delta_cycles / (float)SystemCoreClock;
    }
    else if (dt > 0.0f)
    {
        speed_dt_s = dt;
    }

    if (speed_dt_s > 0.0f)
    {
        spi_encoder[id].speed_raw = diff / speed_dt_s;
        if (spi_encoder[id].speed_raw > ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S)
        {
            spi_encoder[id].speed_raw = ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S;
        }
        else if (spi_encoder[id].speed_raw < -ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S)
        {
            spi_encoder[id].speed_raw = -ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S;
        }

        /*
         * 速度环反馈使用周期平均速度：
         * 在SPI样本路径累计角度差和真实DWT时间，TIM7速度环再一次性消费。
         * 若speed_raw被限幅，则按限幅后的速度反推等效角度差，避免单帧毛刺污染平均速度。
         */
        if (delta_cycles != 0U)
        {
            primask = __get_PRIMASK();
            __disable_irq();
            spi_encoder[id].speed_avg_diff_sum += spi_encoder[id].speed_raw * speed_dt_s;
            spi_encoder[id].speed_avg_cycles_sum += delta_cycles;
            if (spi_encoder[id].speed_avg_sample_count < UINT16_MAX)
            {
                spi_encoder[id].speed_avg_sample_count++;
            }
            if (primask == 0U)
            {
                __enable_irq();
            }
        }
    }

    spi_encoder[id].angle_sample_cycles = sample_cycles;
    spi_encoder[id].valid = 1U;

    return HAL_OK;
}

/**
 * @brief  按编码器序号启动一路SPI DMA读取。
 * @param  index  编码器序号，范围0~ENCODER_NUM-1。
 * @retval HAL_OK表示启动成功；其他值表示HAL SPI DMA启动失败。
 * @note   使用方式：DMA读取流程内部调用。
 */
static HAL_StatusTypeDef SPI_Encoder_Start_DMA_By_Index(uint8_t index)
{
    if (index >= ENCODER_NUM)
    {
        return HAL_ERROR;
    }

#if (ENCODER_SPI_BURST_READ_ENABLE != 0U)
    return SPI_Encoder_Start_DMA_Burst_Frame(index);
#else
    return SPI_Encoder_Start_DMA_Read_Reg(index, ENCODER_ANGLE_H_CMD, &s_spi_dma_rx_buf[index][0]);
#endif
}

/**
 * @brief  启动一次MT6816 0x03/0x04 burst read DMA读取。
 * @param  index  编码器序号，范围0~ENCODER_NUM-1。
 * @retval HAL_OK表示启动成功；其他值表示HAL SPI DMA启动失败。
 * @note   使用方式：DMA读取流程内部调用。TX 83 00 00，RX xx H L/status。
 *         调用后CS保持低电平，直到DMA完成回调释放。
 */
static HAL_StatusTypeDef SPI_Encoder_Start_DMA_Burst_Frame(uint8_t index)
{
    HAL_StatusTypeDef status;
    encoder_id_t id;

    if (index >= ENCODER_NUM)
    {
        return HAL_ERROR;
    }

    id = (encoder_id_t)index;
    s_spi_tx_buf[0] = ENCODER_ANGLE_H_CMD;
    s_spi_tx_buf[1] = 0x00U;
    s_spi_tx_buf[2] = 0x00U;

    SPI_Encoder_CS_Select(id);
    SPI_Encoder_CS_SetupDelay();
    status = HAL_SPI_TransmitReceive_DMA(&hspi3,
                                         s_spi_tx_buf,
                                         s_spi_dma_rx_buf[index],
                                         ENCODER_FRAME_LEN);
    if (status != HAL_OK)
    {
        SPI_Encoder_CS_Release(id);
    }

    return status;
}

/**
 * @brief  启动一次MT6816寄存器DMA读取。
 * @param  index   编码器序号，范围0~ENCODER_NUM-1。
 * @param  cmd     读寄存器命令，例如0x83或0x84。
 * @param  rx_buf  本次2字节接收缓存。
 * @retval HAL_OK表示启动成功；其他值表示HAL SPI DMA启动失败。
 * @note   使用方式：DMA读取流程内部调用。调用后CS保持低电平，直到DMA完成回调释放。
 */
static HAL_StatusTypeDef SPI_Encoder_Start_DMA_Read_Reg(uint8_t index, uint8_t cmd, uint8_t *rx_buf)
{
    HAL_StatusTypeDef status;
    encoder_id_t id;

    if ((index >= ENCODER_NUM) || (rx_buf == NULL))
    {
        return HAL_ERROR;
    }

    id = (encoder_id_t)index;
    s_spi_tx_buf[0] = cmd;
    s_spi_tx_buf[1] = 0x00U;

    SPI_Encoder_CS_Select(id);
    SPI_Encoder_CS_SetupDelay();
    status = HAL_SPI_TransmitReceive_DMA(&hspi3, s_spi_tx_buf, rx_buf, ENCODER_FRAME_LEN);
    if (status != HAL_OK)
    {
        SPI_Encoder_CS_Release(id);
    }

    return status;
}

/**
 * @brief  使用三路DMA raw缓存统一更新编码器数据。
 * @param  dt  本轮统一使用的时间间隔，单位s。
 * @retval None
 * @note   使用方式：三路DMA读取全部完成后调用。
 *         无效帧不会更新对应编码器，避免坏数据污染角度和速度。
 */
static void SPI_Encoder_Update_All_By_DMA_Raw(float dt)
{
    uint8_t i = 0U;

    for (i = 0U; i < ENCODER_NUM; i++)
    {
        if (s_spi_dma_raw_valid[i] != 0U)
        {
            (void)SPI_Encoder_Update_By_Raw((encoder_id_t)i, s_spi_dma_raw[i], dt);
        }
    }
}

/**
 * @brief  检查 MT6816 角度帧偶校验。
 * @param  angle_high        角度高8位，对应 0x03。
 * @param  angle_low_status  角度低6位和状态位，对应 0x04。
 * @retval 1表示校验通过，0表示校验失败。
 * @note   使用方式：SPI_Encoder_Read_Raw() 内部调用。
 *         MT6816 的 PC 位用于让 0x03[7:0] 和 0x04[7:1] 加上 PC 后为偶数个1。
 */
static uint8_t SPI_Encoder_Check_Parity(uint8_t angle_high, uint8_t angle_low_status)
{
    uint8_t value;
    uint8_t one_count = 0U;
    uint8_t expected_pc;

    value = angle_high;
    while (value != 0U)
    {
        one_count += (value & 0x01U);
        value >>= 1;
    }

    value = (angle_low_status & 0xFEU);
    while (value != 0U)
    {
        one_count += (value & 0x01U);
        value >>= 1;
    }

    expected_pc = one_count & 0x01U;

    return ((angle_low_status & ENCODER_PARITY_MASK) == expected_pc) ? 1U : 0U;
}

/**
 * @brief  按编码器方向和机械零点偏移换算单圈机械角。
 * @param  id         编码器编号，取值见 encoder_id_t。
 * @param  angle_rad  原始单圈机械角，单位 rad。
 * @retval 处理方向和零点后的单圈机械角，范围 0~2pi。
 * @note   使用方式：SPI_Encoder_Update() 内部调用。
 *         编码器方向来自 Config.h 中的 ENCODER_DIR_M1/M2/M3。
 */
static float SPI_Encoder_Apply_Direction(encoder_id_t id, float angle_rad)
{
    float angle = angle_rad - spi_encoder[id].angle_offset;

    if (s_encoder_dir[id] < 0)
    {
        angle = -angle;
    }

    return SPI_Encoder_Normalize_Angle(angle);
}

/**
 * @brief  根据跨零点方向更新显式多圈计数。
 * @param  id          编码器编号，取值见 encoder_id_t。
 * @param  angle_last  上一次单圈角，单位 rad，范围 0~2pi。
 * @param  angle_now   当前单圈角，单位 rad，范围 0~2pi。
 * @param  diff        最短路径角度差，单位 rad。
 * @retval None
 * @note   使用方式：SPI_Encoder_Update() 内部调用。
 *         正向从接近2pi跨到0附近时 turn_count++，反向从0附近跨到2pi附近时 turn_count--。
 */
static void SPI_Encoder_Update_Turn_Count(encoder_id_t id, float angle_last, float angle_now, float diff)
{
    if (ENCODER_MULTI_TURN_ENABLE == 0)
    {
        spi_encoder[id].turn_count = 0;
        return;
    }

    if ((diff > 0.0f) && (angle_now < angle_last))
    {
        spi_encoder[id].turn_count++;
    }
    else if ((diff < 0.0f) && (angle_now > angle_last))
    {
        spi_encoder[id].turn_count--;
    }

    if (spi_encoder[id].turn_count > ENCODER_MULTI_TURN_MAX_ABS)
    {
        spi_encoder[id].turn_count = ENCODER_MULTI_TURN_MAX_ABS;
    }
    else if (spi_encoder[id].turn_count < -ENCODER_MULTI_TURN_MAX_ABS)
    {
        spi_encoder[id].turn_count = -ENCODER_MULTI_TURN_MAX_ABS;
    }
}

/**
 * @brief  获取指定编码器的片选 GPIO 端口。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval GPIO端口指针；id无效时返回NULL。
 * @note   使用方式：片选控制内部调用。当前 CSN1/CSN2/CSN3 属于编码器片选。
 */
static GPIO_TypeDef *SPI_Encoder_Get_CS_Port(encoder_id_t id)
{
    switch (id)
    {
        case encoder_id_1:
            return CSN1_GPIO_Port;

        case encoder_id_2:
            return CSN2_GPIO_Port;

        case encoder_id_3:
            return CSN3_GPIO_Port;

        default:
            return NULL;
    }
}

/**
 * @brief  获取指定编码器的片选 GPIO 引脚。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval GPIO引脚；id无效时返回0。
 * @note   使用方式：片选控制内部调用。
 */
static uint16_t SPI_Encoder_Get_CS_Pin(encoder_id_t id)
{
    switch (id)
    {
        case encoder_id_1:
            return CSN1_Pin;

        case encoder_id_2:
            return CSN2_Pin;

        case encoder_id_3:
            return CSN3_Pin;

        default:
            return 0U;
    }
}

/**
 * @brief  短软件延时。
 * @param  loop: 空循环次数。
 * @retval None
 * @note   只用于SPI片选建立/保持时间微调；不要用于毫秒级等待。
 */
static void SPI_Encoder_DelayLoop(uint32_t loop)
{
    volatile uint32_t i;

    for (i = 0U; i < loop; i++)
    {
        __NOP();
    }
}

/**
 * @brief  CS拉低后的建立时间。
 * @param  None
 * @retval None
 * @note   确保MT6816在首个SCK前已经看到CS有效，避免首位输出不稳定。
 */
static void SPI_Encoder_CS_SetupDelay(void)
{
    SPI_Encoder_DelayLoop(ENCODER_CS_SETUP_DELAY_LOOP);
}

/**
 * @brief  等待SPI3总线完全空闲。
 * @param  None
 * @retval None
 * @note   DMA完成不完全等价于SPI移位结束；释放CS前等待BSY清零，避免截断帧尾。
 */
static void SPI_Encoder_WaitSpiIdle(void)
{
    uint32_t timeout = ENCODER_SPI_IDLE_TIMEOUT;

    while (((hspi3.Instance->SR & SPI_SR_BSY) != 0U) && (timeout > 0U))
    {
        timeout--;
    }

    SPI_Encoder_DelayLoop(ENCODER_CS_HOLD_DELAY_LOOP);
}

/**
 * @brief  拉低指定编码器片选。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval None
 * @note   使用方式：SPI通信开始前调用，CSN低电平有效。
 */
static void SPI_Encoder_CS_Select(encoder_id_t id)
{
    GPIO_TypeDef *port = SPI_Encoder_Get_CS_Port(id);
    uint16_t pin = SPI_Encoder_Get_CS_Pin(id);

    if ((port != NULL) && (pin != 0U))
    {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief  释放指定编码器片选。
 * @param  id  编码器编号，取值见 encoder_id_t。
 * @retval None
 * @note   使用方式：SPI通信结束后调用，CSN恢复高电平。
 */
static void SPI_Encoder_CS_Release(encoder_id_t id)
{
    GPIO_TypeDef *port = SPI_Encoder_Get_CS_Port(id);
    uint16_t pin = SPI_Encoder_Get_CS_Pin(id);

    if ((port != NULL) && (pin != 0U))
    {
        SPI_Encoder_WaitSpiIdle();
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    }
}
