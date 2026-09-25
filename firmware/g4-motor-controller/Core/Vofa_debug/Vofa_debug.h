#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

//******************************************VOFA基础配置*******************************************/
#define VOFA_MAX_CHANNEL_NUM           10U      // 单帧最大发送通道数，编码器诊断主题需要最多10路
#define VOFA_TX_BUF_SIZE               128U     // VOFA发送缓冲区字节数
#define VOFA_DEFAULT_PERIOD_MS         20U      // 默认发送周期，单位ms
#define VOFA_DEFAULT_ENABLE            1U       // 默认发送使能状态

//******************************************VOFA调试主题*******************************************/
typedef enum
{
    VOFA_TOPIC_NONE = 0,                         // 空主题，保留
    VOFA_TOPIC_ADC_RAW,                          // ADC原始采样值概览主题，保留给全局调试
    VOFA_TOPIC_ADC_DIAG,                         // ADC诊断主题：update_count和dma_done，用于定位哪一路ADC没有更新
    VOFA_TOPIC_ADC1_RAW,                         // ADC1原始采样主题：raw_a raw_b raw_c dma_done
    VOFA_TOPIC_ADC3_RAW,                         // ADC3原始采样主题：raw_a raw_b raw_c dma_done
    VOFA_TOPIC_ADC4_RAW,                         // ADC4原始采样主题：raw_a raw_b raw_c dma_done
    VOFA_TOPIC_PHASE_CURRENT,                    // 三相电流主题
    VOFA_TOPIC_CURRENT_SAMPLE_DIAG,              // 电流采样诊断：raw/电流/重构相/触发CCR
    VOFA_TOPIC_ENCODER,                          // 编码器角度与速度主题
    VOFA_TOPIC_ENCODER_ALL,                      // 三路编码器总览主题：用于确认三路CS/角度/有效标志是否对应
    VOFA_TOPIC_CURRENT_PID,                      // 电流环混合诊断：id/iq/alpha_beta/三相电流/重构相/触发点/电角度
    VOFA_TOPIC_SPEED_PID,                        // 速度环PID主题：ref fb err output integral output_raw saturated aux
    VOFA_TOPIC_POSITION_PID,                     // 位置环PID主题：ref fb err output integral output_raw saturated aux
    VOFA_TOPIC_STATE,                            // 状态机/故障主题
    VOFA_TOPIC_FAST_LOOP_PROFILE,                // 三电机快环DWT耗时主题：last/max/overrun/spi_busy
    VOFA_TOPIC_DRV_CHECK,                        // DRV8323S寄存器检测主题：用于查看DRV故障和配置是否写入
    VOFA_TOPIC_FOC_TEST                          // FOC测试主题：target_angle actual_angle sector vd vq ccr1 ccr2 ccr3
} vofa_topic_t;

//******************************************VOFA发送状态*******************************************/
typedef enum
{
    VOFA_SEND_OK = 0,                            // 发送启动成功
    VOFA_SEND_BUSY,                              // 串口DMA忙
    VOFA_SEND_PARAM_ERR,                         // 输入参数错误
    VOFA_SEND_BUF_ERR,                           // 发送缓冲区不足
    VOFA_SEND_UART_ERR                           // 串口DMA启动失败
} vofa_send_status_t;

//******************************************VOFA对外接口*******************************************/
void VOFA_Debug_Init(void);                                                          // 初始化VOFA调试模块内部状态
void VOFA_Debug_Task(void);                                                          // 周期调用，按当前主题与周期自动发送调试数据
vofa_send_status_t VOFA_Debug_SendFloatFrame(const float *data, uint16_t channel_num); // 发送一帧JustFloat浮点数据

void VOFA_Debug_SetTopic(vofa_topic_t topic);                                        // 设置当前调试主题
vofa_topic_t VOFA_Debug_GetTopic(void);                                              // 获取当前调试主题

void VOFA_Debug_Enable(void);                                                        // 使能VOFA发送
void VOFA_Debug_Disable(void);                                                       // 禁用VOFA发送
void VOFA_Debug_SetPeriodMs(uint32_t period_ms);                                     // 设置周期发送间隔，单位ms
void VOFA_Debug_RecordKey1PwmStart(uint8_t encoder_valid,
                                   uint8_t start_allowed,
                                   uint8_t start_called,
                                   uint8_t start_ok,
                                   uint8_t pwm_ready_after,
                                   uint32_t ccer_before,
                                   uint32_t ccer_after);                             // 记录KEY1 PWM启动路径诊断信息

uint8_t VOFA_Debug_IsBusy(void);                                                     // 查询当前发送是否忙
uint32_t VOFA_Debug_GetErrorCount(void);                                             // 获取串口发送错误计数

void VOFA_Debug_TxCpltCallback(UART_HandleTypeDef *huart);                           // 串口发送完成回调入口
void VOFA_Debug_ErrorCallback(UART_HandleTypeDef *huart);                            // 串口错误回调入口

#ifdef __cplusplus
}
#endif

#endif /* VOFA_DEBUG_H */
