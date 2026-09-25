#ifndef ACT_STREAM_H
#define ACT_STREAM_H

/**
 * @file act_stream.h
 * @brief 柔性触手ACT采集与USART1通信模块的公共接口。
 *
 * 本模块负责ACT上位机与H7之间的命令、状态和采样数据通信。
 * USART1专用于ACT，UART7仍由现有VOFA控制和调试模块使用。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* 上位机使用1～4表示触手编号，H7内部需要时再转换为0～3数组下标。 */
#define ACT_STREAM_TENTACLE_MIN        1U
#define ACT_STREAM_TENTACLE_MAX        4U

/**
 * @brief ACT采样数据流状态。
 *
 * H7只管理是否正在采样。停止采样后的“保存/删除”由PC上位机处理，
 * 因此H7不需要增加等待保存状态。
 */
typedef enum
{
  ACT_STREAM_CAPTURE_IDLE = 0U,   /* 当前不产生ACT_FRAME数据。 */
  ACT_STREAM_CAPTURE_RUNNING      /* 正在产生并发送ACT_FRAME数据。 */
} Act_StreamCaptureState_t;

/**
 * @brief ACT模块对上层公开的运行状态。
 *
 * 该结构体只保存状态摘要，不包含DMA缓冲区、协议解析器等内部对象。
 */
typedef struct
{
  uint8_t selected_tentacle;               /* 当前选择的触手，范围为1～4。 */
  uint8_t control_enabled;                  /* 1=USART1拥有单触手控制权，0=未启用。 */
  Act_StreamCaptureState_t capture_state;   /* 当前ACT采样数据流状态。 */
  uint32_t sent_frame_count;                /* 本次采样已完整发送的ACT_FRAME数量。 */
  uint32_t dropped_frame_count;             /* 本次采样因发送队列满而丢弃的整帧数量。 */
} Act_StreamStatus_t;

/**
 * @brief 初始化ACT模块。
 *
 * 必须在MX_USART1_UART_Init()之后调用。
 * 初始化时默认选择训练所用的触手3、关闭控制、保持采样空闲并启动RX循环DMA。
 */
void Act_Stream_Init(void);

/**
 * @brief 执行ACT非阻塞主循环任务。
 *
 * 负责处理USART1接收数据、命令解析、DMA超时和错误恢复，
 * 不允许在内部阻塞等待。
 * 应在main主循环中持续调用。
 */
void Act_Stream_Task(void);

/**
 * @brief 复制当前ACT状态摘要。
 * @param status 接收状态的结构体地址；传入NULL时不执行任何操作。
 */
void Act_Stream_GetStatus(Act_StreamStatus_t *status);

/**
 * @brief 查询USART1是否正在占用单触手控制权。
 * @return 1表示ACT正在控制，0表示UART7可以修改运动状态。
 */
uint8_t Act_Stream_HasControl(void);

/**
 * @brief 处理USART1 DMA发送完成事件。
 * @param huart HAL回调传入的串口句柄。
 *
 * 该函数由工程中已有的HAL_UART_TxCpltCallback()统一分发调用，
 * act_stream.c中不能重复定义HAL全局回调。
 */
void Act_Stream_OnUartTxComplete(UART_HandleTypeDef *huart);

/**
 * @brief 处理USART1错误事件。
 * @param huart HAL回调传入的串口句柄。
 *
 * 仅记录USART1错误，RX DMA的节流恢复由Act_Stream_Task()执行，
 * 不处理UART7错误。
 */
void Act_Stream_OnUartError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* ACT_STREAM_H */
