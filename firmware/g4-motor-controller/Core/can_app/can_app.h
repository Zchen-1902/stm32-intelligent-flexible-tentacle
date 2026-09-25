#ifndef CAN_APP_H
#define CAN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "../motor/Config.h"

#ifndef CAN_APP_BOARD_ID
#define CAN_APP_BOARD_ID              (0U)
#endif
#define CAN_APP_PROTOCOL_VERSION      (2U)

#define CAN_APP_ID_H7_CONTROL_BASE    (0x100U)
#define CAN_APP_ID_H7_TARGET_BASE     (0x110U)
#define CAN_APP_ID_G4_STATUS_BASE     (0x180U)
#define CAN_APP_ID_H7_POSITION_FD_BASE (0x120U)

#define CAN_APP_ID_H7_CONTROL         (CAN_APP_ID_H7_CONTROL_BASE + CAN_APP_BOARD_ID)
#define CAN_APP_ID_H7_TARGET0         (CAN_APP_ID_H7_TARGET_BASE + CAN_APP_BOARD_ID)
#define CAN_APP_ID_G4_STATUS          (CAN_APP_ID_G4_STATUS_BASE + CAN_APP_BOARD_ID)
#define CAN_APP_ID_H7_POSITION_FD     (CAN_APP_ID_H7_POSITION_FD_BASE + CAN_APP_BOARD_ID)

HAL_StatusTypeDef CAN_App_Init(void);
void CAN_App_Task(void);
void CAN_App_RequestStatusTx(void);

uint8_t CAN_App_GetActiveMotorMask(void); // 返回CAN路径已完成目标/模式配置并允许进入控制环的电机mask
uint8_t CAN_App_IsOnline(void);
uint32_t CAN_App_GetRxCount(void);
uint32_t CAN_App_GetTxCount(void);
uint32_t CAN_App_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_APP_H */
