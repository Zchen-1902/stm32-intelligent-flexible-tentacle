#ifndef TIMER_APP_H
#define TIMER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "main.h"
#include "tim.h"

//******************************************定时器应用任务枚举*******************************************/
typedef enum
{
    TIMER_APP_TASK_5MS = 0,           // TIM4基础5ms任务 
    TIMER_APP_TASK_10MS,              // TIM4分频得到的10ms任务
    TIMER_APP_TASK_20MS,              // TIM4分频得到的20ms任务
    TIMER_APP_TASK_100MS,             // TIM4分频得到的100ms任务
    TIMER_APP_TASK_500MS,             // TIM4分频得到的500ms任务
    TIMER_APP_TASK_TEMP_SAMPLE,       // TIM6温度采样节拍任务
    TIMER_APP_TASK_SPEED_CALC,        // TIM7速度计算节拍任务
    TIMER_APP_TASK_CONTROL            // TIM3控制任务调度标志，用于触发Motor_Core_UpdateAll
} timer_app_task_t;

//******************************************定时器应用层接口*******************************************/
void Timer_App_Init(void);                                            // 初始化timer_app内部状态与任务标志
void Timer_App_Start(void);                                           // 启动TIM4/TIM7中断，并启动TIM6作为ADC2触发源；TIM3暂不启动
HAL_StatusTypeDef Timer_App_StartMotorControlTimers(void);            // 启动TIM1/TIM8/TIM20基础计数与更新中断，只做PWM定时器运行计数，不启动PWM输出
void Timer_App_PeriodElapsedCallback(TIM_HandleTypeDef *htim);        // 在HAL_TIM_PeriodElapsedCallback中转发调用

uint8_t Timer_App_IsTaskReady(timer_app_task_t task);                 // 查询指定任务标志是否到达
void Timer_App_ClearTaskFlag(timer_app_task_t task);                  // 清除普通任务标志；对控制任务表示消费一次pending
void Timer_App_SetControlTaskFlag(void);                              // 外部控制节拍调用，置位控制环更新任务标志
void Timer_App_SetMotorControlRequest(uint8_t motor_id);              // ADC DMA完成后置位指定电机控制请求
uint8_t Timer_App_ConsumeMotorControlRequest(uint8_t motor_id);       // main消费指定电机控制请求，消费成功返回1
uint32_t Timer_App_GetMotorUpdateIrqCount(uint8_t motor_id);          // 获取指定电机PWM定时器update中断计数，用于验证TIM1/TIM8/TIM20是否运行
uint32_t Timer_App_GetSpeedCalcTimeUs(void);                          // 获取TIM7节拍维护的编码器采样时间戳，单位us
uint32_t Timer_App_GetRuntimeCycles(void);                             // 获取DWT运行周期计数，用于计算真实控制dt

#ifdef __cplusplus
}
#endif

#endif /* TIMER_APP_H */
