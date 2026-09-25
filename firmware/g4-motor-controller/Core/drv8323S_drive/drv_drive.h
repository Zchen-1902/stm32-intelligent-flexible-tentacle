#ifndef __DRV_DRIVE_H
#define __DRV_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "spi.h"

//******************************************DRV8323S基础配置*******************************************/
#define DRV8323_NUM                         3U          // DRV8323S数量，对应三路电机驱动
#define DRV8323_SPI_TIMEOUT_MS              10U         // SPI阻塞读写超时时间，单位ms
#define DRV8323_REG_DATA_MASK               0x07FFU     // DRV8323S寄存器有效数据宽度为11bit
#define DRV8323_SPI_READ_BIT                0x8000U     // SPI读命令标志位，bit15=1表示读
#define DRV8323_SPI_WRITE_BIT               0x0000U     // SPI写命令标志位，bit15=0表示写
#define DRV8323_SPI_ADDR_SHIFT              11U         // SPI帧中寄存器地址左移位数

//******************************************DRV8323S寄存器地址*******************************************/
#define DRV8323_REG_FAULT_STATUS_1          0x00U       // 故障状态寄存器1，只读
#define DRV8323_REG_FAULT_STATUS_2          0x01U       // 故障状态寄存器2，只读
#define DRV8323_REG_DRIVER_CONTROL          0x02U       // 驱动控制寄存器，配置PWM模式/COAST/BRAKE/清故障
#define DRV8323_REG_GATE_DRIVE_HS           0x03U       // 高侧栅极驱动寄存器，包含LOCK锁定位
#define DRV8323_REG_GATE_DRIVE_LS           0x04U       // 低侧栅极驱动寄存器，配置TDRIVE/低侧驱动电流
#define DRV8323_REG_OCP_CONTROL             0x05U       // 过流保护寄存器，配置OCP模式/死区/VDS阈值
#define DRV8323_REG_CSA_CONTROL             0x06U       // 电流采样放大器寄存器，配置CSA增益/VREF/校准位

//******************************************DRV8323S寄存器字段*******************************************/
#define DRV8323_DRIVER_OTW_REP              (1U << 7)   // 过温警告上报到FAULT，调试阶段建议打开
#define DRV8323_DRIVER_PWM_MODE_6X          (0U << 5)   // 6x PWM模式，由MCU提供上下桥互补PWM
#define DRV8323_DRIVER_COAST                (1U << 2)   // 置1使六个MOS全部进入高阻态
#define DRV8323_DRIVER_CLR_FLT              (1U << 0)   // 清除锁存故障位，写1后芯片自动复位该位

#define DRV8323_LOCK_UNLOCK                 (0x3U << 8) // 解锁寄存器写保护，允许修改配置寄存器
#define DRV8323_LOCK_LOCK                   (0x6U << 8) // 锁定寄存器写保护，运行稳定后可使用

#define DRV8323_CBC_ENABLE                  (1U << 10)  // 周期性过流保护使能，配合OCP模式使用
#define DRV8323_TDRIVE_4000NS               (0x3U << 8) // 栅极驱动峰值时间4000ns，第一版沿用保守默认

#define DRV8323_DEAD_TIME_100NS             (0x1U << 8) // DRV内部死区100ns，第一版先使用DRV侧死区
#define DRV8323_OCP_MODE_LATCH              (0x0U << 6) // 过流锁存模式，调试阶段便于保留故障现场
#define DRV8323_OCP_DEG_4US                 (0x1U << 4) // 过流去抖4us，第一版沿用默认量级
#define DRV8323_VDS_LVL_150V                0xDU        // VDS过流阈值1.5V，调试阶段临时放宽VDS保护阈值

#define DRV8323_CSA_VREF_DIV_2              (1U << 9)   // CSA输出以VREF/2为中心，支持正负电流观测
#define DRV8323_CSA_GAIN_20V_V              (0x2U << 6) // CSA增益20V/V，第一版适合先做中低电流调试
#define DRV8323_CSA_SEN_LVL_1V              0x3U        // Sense OCP阈值1V，第一版先保留保护余量
#define DRV8323_CSA_CAL_A                   (1U << 4)   // SPI方式使能A相CSA校准，当前主路径优先用CAL引脚
#define DRV8323_CSA_CAL_B                   (1U << 3)   // SPI方式使能B相CSA校准，当前主路径优先用CAL引脚
#define DRV8323_CSA_CAL_C                   (1U << 2)   // SPI方式使能C相CSA校准，当前主路径优先用CAL引脚

//******************************************DRV8323S默认写入值*******************************************/
#define DRV8323_IDRIVEP_HS_CFG              0x4U        // 高侧栅极源电流配置，第一版先用偏保守值，后续按波形调
#define DRV8323_IDRIVEN_HS_CFG              0x4U        // 高侧栅极灌电流配置，第一版先用偏保守值，后续按波形调
#define DRV8323_IDRIVEP_LS_CFG              0x4U        // 低侧栅极源电流配置，第一版先用偏保守值，后续按波形调
#define DRV8323_IDRIVEN_LS_CFG              0x4U        // 低侧栅极灌电流配置，第一版先用偏保守值，后续按波形调

#define DRV8323_DRIVER_CONTROL_DEFAULT      (DRV8323_DRIVER_OTW_REP | DRV8323_DRIVER_PWM_MODE_6X) // 6x PWM，保护开启，OTW上报
#define DRV8323_GATE_DRIVE_HS_DEFAULT       (DRV8323_LOCK_UNLOCK | (DRV8323_IDRIVEP_HS_CFG << 4) | DRV8323_IDRIVEN_HS_CFG) // 高侧驱动默认配置，保持解锁
#define DRV8323_GATE_DRIVE_LS_DEFAULT       (DRV8323_CBC_ENABLE | DRV8323_TDRIVE_4000NS | (DRV8323_IDRIVEP_LS_CFG << 4) | DRV8323_IDRIVEN_LS_CFG) // 低侧驱动默认配置
#define DRV8323_OCP_CONTROL_DEFAULT         (DRV8323_DEAD_TIME_100NS | DRV8323_OCP_MODE_LATCH | DRV8323_OCP_DEG_4US | DRV8323_VDS_LVL_150V) // OCP锁存，DRV死区100ns，VDS阈值1.5V
#define DRV8323_CSA_CONTROL_DEFAULT         (DRV8323_CSA_VREF_DIV_2 | DRV8323_CSA_GAIN_20V_V | DRV8323_CSA_SEN_LVL_1V) // CSA正常采样默认配置

//******************************************DRV8323S编号与硬件映射*******************************************/
typedef enum
{
    DRV_ID_1 = 0,                            // 第1路DRV8323S，对应电机1
    DRV_ID_2,                                // 第2路DRV8323S，对应电机2
    DRV_ID_3,                                // 第3路DRV8323S，对应电机3
    DRV_ID_NUM                               // DRV数量边界，不作为有效ID使用
} drv_id_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;                 // DRV使用的SPI句柄，当前三路DRV共用SPI1
    GPIO_TypeDef *ncs_port;                  // DRV片选GPIO端口，当前使用NSCS1/NSCS2/NSCS3
    uint16_t ncs_pin;                        // DRV片选GPIO引脚，低电平选中
    GPIO_TypeDef *cal_port;                  // DRV CSA校准GPIO端口，当前使用CAL1/CAL2/CAL3
    uint16_t cal_pin;                        // DRV CSA校准GPIO引脚，高电平进入CSA校准状态
} drv8323_hw_t;

//******************************************DRV8323S对外接口*******************************************/
HAL_StatusTypeDef DRV_Drive_Init(void);                                      // 初始化三路DRV：CS拉高、CAL退出校准、写入默认寄存器配置

HAL_StatusTypeDef DRV_Drive_ReadReg(drv_id_t id,
                                    uint8_t reg_addr,
                                    uint16_t *data);                         // 读取指定DRV寄存器，data返回11bit有效数据

HAL_StatusTypeDef DRV_Drive_WriteReg(drv_id_t id,
                                     uint8_t reg_addr,
                                     uint16_t data);                         // 写入指定DRV寄存器，data仅低11bit有效

HAL_StatusTypeDef DRV_Drive_UpdateReg(drv_id_t id,
                                      uint8_t reg_addr,
                                      uint16_t mask,
                                      uint16_t value);                       // 读改写指定寄存器，只修改mask覆盖的位

HAL_StatusTypeDef DRV_Drive_ApplyDefaultConfig(drv_id_t id);                 // 对单个DRV写入第一版默认配置，不启动PWM
HAL_StatusTypeDef DRV_Drive_ApplyDefaultConfigAll(void);                     // 对三路DRV依次写入第一版默认配置，不启动PWM

HAL_StatusTypeDef DRV_Drive_UnlockReg(drv_id_t id);                          // 解锁指定DRV配置寄存器，调参或初始化前调用
HAL_StatusTypeDef DRV_Drive_LockReg(drv_id_t id);                            // 锁定指定DRV配置寄存器，运行稳定后可调用

void DRV_Drive_SetCal(drv_id_t id, uint8_t enable);                          // 设置单个DRV的CAL引脚：1进入CSA校准，0退出CSA校准
void DRV_Drive_SetCalAll(uint8_t enable);                                     // 同时设置三路DRV的CAL引脚
void DRV_Drive_CsaCalibBeginAll(void);                                        // 三路DRV进入CSA校准状态，后续由ADC模块采集零漂
void DRV_Drive_CsaCalibEndAll(void);                                          // 三路DRV退出CSA校准状态，恢复正常电流采样

#ifdef __cplusplus
}
#endif

#endif /* __DRV_DRIVE_H */
