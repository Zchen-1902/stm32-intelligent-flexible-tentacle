#ifndef VOFA_H
#define VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* 初始化 VOFA 调试模块：启动 USART1 DMA 接收并清空内部状态。 */
void Vofa_Init(void);
/* 主循环任务：处理串口命令、采样调度和 LED 状态刷新。 */
void Vofa_Task(void);
/* 获取当前VOFA使用的UART句柄，供其它模块复用同一个调试串口。 */
UART_HandleTypeDef *Vofa_GetUartHandle(void);
/* 非阻塞写入VOFA串口；TX队列满时返回非0并丢弃本条输出。 */
uint8_t Vofa_Write(const char *text);

#ifdef __cplusplus
}
#endif

#endif
