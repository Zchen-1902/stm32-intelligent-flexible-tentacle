#include "drv_drive.h"

#define DRV8323_POWER_STABLE_DELAY_MS 300U // 上电后等待VM/DVDD稳定，避免DRV初始化过早锁存UVLO

//******************************************DRV8323S硬件映射*******************************************/
static const drv8323_hw_t s_drv_hw[DRV_ID_NUM] =
{
    {&hspi1, NSCS1_GPIO_Port, NSCS1_Pin, CAL1_GPIO_Port, CAL1_Pin}, // DRV1：电机1驱动器
    {&hspi1, NSCS2_GPIO_Port, NSCS2_Pin, CAL2_GPIO_Port, CAL2_Pin}, // DRV2：电机2驱动器
    {&hspi1, NSCS3_GPIO_Port, NSCS3_Pin, CAL3_GPIO_Port, CAL3_Pin}, // DRV3：电机3驱动器
};

/**
 * @brief  判断DRV编号是否有效。
 * @param  id：DRV编号，取值应为DRV_ID_1、DRV_ID_2、DRV_ID_3。
 * @return 1表示有效，0表示无效。
 * @note   内部保护函数，避免数组越界访问硬件映射表。
 */
static uint8_t DRV_Drive_IsValidId(drv_id_t id)
{
    return ((uint32_t)id < (uint32_t)DRV_ID_NUM);
}

/**
 * @brief  释放所有DRV片选。
 * @param  无。
 * @return 无。
 * @note   DRV8323S的nSCS为低电平有效，该函数把三路片选全部拉高，避免多个DRV同时响应SPI。
 */
static void DRV_Drive_DeselectAll(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)DRV_ID_NUM; i++)
    {
        HAL_GPIO_WritePin(s_drv_hw[i].ncs_port, s_drv_hw[i].ncs_pin, GPIO_PIN_SET);
    }
}

/**
 * @brief  选中指定DRV。
 * @param  id：DRV编号。
 * @return 无。
 * @note   选中前会先释放所有片选，保证同一时刻最多只有一个DRV的nSCS被拉低。
 */
static void DRV_Drive_Select(drv_id_t id)
{
    DRV_Drive_DeselectAll();
    HAL_GPIO_WritePin(s_drv_hw[id].ncs_port, s_drv_hw[id].ncs_pin, GPIO_PIN_RESET);
}

/**
 * @brief  通过SPI与指定DRV交换一个16bit数据帧。
 * @param  id：DRV编号。
 * @param  tx_frame：发送帧，高位包含读写位和寄存器地址，低11bit为数据。
 * @param  rx_frame：接收帧指针，用于返回DRV输出的16bit数据。
 * @return HAL状态。
 * @note   当前SPI保持8bit数据宽度，因此手动拆成高字节、低字节发送。
 */
static HAL_StatusTypeDef DRV_Drive_TransferFrame(drv_id_t id, uint16_t tx_frame, uint16_t *rx_frame)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2] = {0U, 0U};
    HAL_StatusTypeDef status;

    if ((DRV_Drive_IsValidId(id) == 0U) || (rx_frame == NULL))
    {
        return HAL_ERROR;
    }

    tx_buf[0] = (uint8_t)(tx_frame >> 8);
    tx_buf[1] = (uint8_t)(tx_frame & 0xFFU);

    DRV_Drive_Select(id);
    status = HAL_SPI_TransmitReceive(s_drv_hw[id].hspi,
                                     tx_buf,
                                     rx_buf,
                                     sizeof(tx_buf),
                                     DRV8323_SPI_TIMEOUT_MS);
    DRV_Drive_DeselectAll();

    *rx_frame = ((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1];

    return status;
}

/**
 * @brief  设置单个DRV的CSA校准引脚。
 * @param  id：DRV编号。
 * @param  enable：1表示CAL拉高进入CSA校准，0表示CAL拉低退出CSA校准。
 * @return 无。
 * @note   该函数只控制CAL引脚，不启动ADC零漂采样；ADC零漂采样由adc_app流程负责。
 */
void DRV_Drive_SetCal(drv_id_t id, uint8_t enable)
{
    GPIO_PinState pin_state;

    if (DRV_Drive_IsValidId(id) == 0U)
    {
        return;
    }

    pin_state = (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(s_drv_hw[id].cal_port, s_drv_hw[id].cal_pin, pin_state);
}

/**
 * @brief  同时设置三路DRV的CSA校准引脚。
 * @param  enable：1表示三路CAL全部拉高，0表示三路CAL全部拉低。
 * @return 无。
 * @note   用于上电零漂校准前后的统一入口，保证三路电流采样状态一致。
 */
void DRV_Drive_SetCalAll(uint8_t enable)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)DRV_ID_NUM; i++)
    {
        DRV_Drive_SetCal((drv_id_t)i, enable);
    }
}

/**
 * @brief  读取指定DRV8323S寄存器。
 * @param  id：DRV编号。
 * @param  reg_addr：寄存器地址，当前有效范围为0x00~0x06。
 * @param  data：返回寄存器低11bit有效数据。
 * @return HAL状态。
 * @note   函数内部会自动完成nSCS拉低、SPI交换、nSCS拉高。
 */
HAL_StatusTypeDef DRV_Drive_ReadReg(drv_id_t id, uint8_t reg_addr, uint16_t *data)
{
    uint16_t tx_frame;
    uint16_t rx_frame = 0U;
    HAL_StatusTypeDef status;

    if ((DRV_Drive_IsValidId(id) == 0U) || (data == NULL))
    {
        return HAL_ERROR;
    }

    tx_frame = DRV8323_SPI_READ_BIT |
               (((uint16_t)reg_addr << DRV8323_SPI_ADDR_SHIFT) & ~DRV8323_SPI_READ_BIT);

    status = DRV_Drive_TransferFrame(id, tx_frame, &rx_frame);
    if (status == HAL_OK)
    {
        *data = rx_frame & DRV8323_REG_DATA_MASK;
    }

    return status;
}

/**
 * @brief  写入指定DRV8323S寄存器。
 * @param  id：DRV编号。
 * @param  reg_addr：寄存器地址，当前有效范围为0x00~0x06。
 * @param  data：写入数据，仅低11bit有效。
 * @return HAL状态。
 * @note   该函数只负责寄存器写入，不做写后回读校验，后续需要时可单独增加验证函数。
 */
HAL_StatusTypeDef DRV_Drive_WriteReg(drv_id_t id, uint8_t reg_addr, uint16_t data)
{
    uint16_t tx_frame;
    uint16_t rx_frame = 0U;

    if (DRV_Drive_IsValidId(id) == 0U)
    {
        return HAL_ERROR;
    }

    tx_frame = DRV8323_SPI_WRITE_BIT |
               ((uint16_t)reg_addr << DRV8323_SPI_ADDR_SHIFT) |
               (data & DRV8323_REG_DATA_MASK);

    return DRV_Drive_TransferFrame(id, tx_frame, &rx_frame);
}

/**
 * @brief  读改写指定DRV8323S寄存器。
 * @param  id：DRV编号。
 * @param  reg_addr：寄存器地址。
 * @param  mask：需要修改的位掩码，mask为1的位会被value更新。
 * @param  value：待写入的新位值，只有mask覆盖的位有效。
 * @return HAL状态。
 * @note   用法示例：只想置位某些bit时，mask填对应bit，value也填对应bit。
 */
HAL_StatusTypeDef DRV_Drive_UpdateReg(drv_id_t id, uint8_t reg_addr, uint16_t mask, uint16_t value)
{
    uint16_t old_data = 0U;
    uint16_t new_data;
    HAL_StatusTypeDef status;

    status = DRV_Drive_ReadReg(id, reg_addr, &old_data);
    if (status != HAL_OK)
    {
        return status;
    }

    new_data = (old_data & (uint16_t)(~mask)) | (value & mask);

    return DRV_Drive_WriteReg(id, reg_addr, new_data);
}

/**
 * @brief  解锁指定DRV8323S配置寄存器。
 * @param  id：DRV编号。
 * @return HAL状态。
 * @note   DRV8323S的LOCK字段位于GATE_DRIVE_HS寄存器bit10~bit8，解锁后才方便写入配置寄存器。
 */
HAL_StatusTypeDef DRV_Drive_UnlockReg(drv_id_t id)
{
    return DRV_Drive_UpdateReg(id,
                               DRV8323_REG_GATE_DRIVE_HS,
                               (0x7U << 8),
                               DRV8323_LOCK_UNLOCK);
}

/**
 * @brief  锁定指定DRV8323S配置寄存器。
 * @param  id：DRV编号。
 * @return HAL状态。
 * @note   第一版初始化流程暂不自动锁定，便于后续在线调试寄存器参数；稳定后可由上层手动调用。
 */
HAL_StatusTypeDef DRV_Drive_LockReg(drv_id_t id)
{
    return DRV_Drive_UpdateReg(id,
                               DRV8323_REG_GATE_DRIVE_HS,
                               (0x7U << 8),
                               DRV8323_LOCK_LOCK);
}

/**
 * @brief  对单个DRV8323S写入第一版默认配置。
 * @param  id：DRV编号。
 * @return HAL状态。
 * @note   该函数只配置DRV寄存器，不启动PWM、不使能电机、不执行ADC零漂校准。
 */
HAL_StatusTypeDef DRV_Drive_ApplyDefaultConfig(drv_id_t id)
{
    HAL_StatusTypeDef status;

    // 1. 解锁配置寄存器，允许后续写入默认参数
    status = DRV_Drive_UnlockReg(id);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(1);

    // 2. 配置驱动控制寄存器：6x PWM模式、过温警告上报
    status = DRV_Drive_WriteReg(id, DRV8323_REG_DRIVER_CONTROL, DRV8323_DRIVER_CONTROL_DEFAULT);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(1);

    // 3. 配置高侧栅极驱动：LOCK字段保持解锁、高侧驱动电流
    status = DRV_Drive_WriteReg(id, DRV8323_REG_GATE_DRIVE_HS, DRV8323_GATE_DRIVE_HS_DEFAULT);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(1);

    // 4. 配置低侧栅极驱动：CBC、TDRIVE、低侧驱动电流
    status = DRV_Drive_WriteReg(id, DRV8323_REG_GATE_DRIVE_LS, DRV8323_GATE_DRIVE_LS_DEFAULT);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(1);

    // 5. 配置过流保护：DRV内部死区、OCP模式、去抖、VDS阈值
    status = DRV_Drive_WriteReg(id, DRV8323_REG_OCP_CONTROL, DRV8323_OCP_CONTROL_DEFAULT);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(1);

    // 6. 配置电流采样放大器：VREF/2中心、CSA增益、Sense OCP阈值
    status = DRV_Drive_WriteReg(id, DRV8323_REG_CSA_CONTROL, DRV8323_CSA_CONTROL_DEFAULT);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(1);

    return HAL_OK;
}

/**
 * @brief  对三路DRV8323S依次写入第一版默认配置。
 * @param  无。
 * @return HAL状态。
 * @note   任意一路配置失败会立即返回错误，便于上层定位初始化失败。
 */
HAL_StatusTypeDef DRV_Drive_ApplyDefaultConfigAll(void)
{
    uint32_t i;
    HAL_StatusTypeDef status;

    for (i = 0U; i < (uint32_t)DRV_ID_NUM; i++)
    {
        status = DRV_Drive_ApplyDefaultConfig((drv_id_t)i);
        if (status != HAL_OK)
        {
            return status;
        }
    }

    return HAL_OK;
}

/**
 * @brief  三路DRV进入CSA校准状态。
 * @param  无。
 * @return 无。
 * @note   该函数只拉高CAL引脚，不做延时、不采ADC；CAL稳定等待和ADC零漂采样由上层校准流程控制。
 */
void DRV_Drive_CsaCalibBeginAll(void)
{
    DRV_Drive_SetCalAll(1U);
}

/**
 * @brief  三路DRV退出CSA校准状态。
 * @param  无。
 * @return 无。
 * @note   该函数只拉低CAL引脚，使CSA恢复正常电流采样状态。
 */
void DRV_Drive_CsaCalibEndAll(void)
{
    DRV_Drive_SetCalAll(0U);
}

/**
 * @brief  初始化三路DRV8323S的基础状态和默认寄存器配置。
 * @param  无。
 * @return HAL状态。
 * @note   该函数不启动PWM、不使能电机，只保证片选释放、CAL退出校准，并写入第一版默认配置。
 */
HAL_StatusTypeDef DRV_Drive_Init(void)
{
    // 1. 释放全部片选，避免上电后多个DRV同时挂在SPI总线上响应
    DRV_Drive_DeselectAll();

    // 2. 默认退出CSA校准状态，正常运行前不让CAL保持高电平
    DRV_Drive_CsaCalibEndAll();

    // 3. 等待DRV功率电源和内部电源稳定，避免上电瞬间UVLO锁存影响首次读回
    HAL_Delay(DRV8323_POWER_STABLE_DELAY_MS);

    // 4. 写入三路DRV第一版默认寄存器配置
    return DRV_Drive_ApplyDefaultConfigAll();
}
