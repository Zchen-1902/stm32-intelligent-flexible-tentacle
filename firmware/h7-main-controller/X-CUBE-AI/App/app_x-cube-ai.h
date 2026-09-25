
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_AI_H
#define __APP_AI_H
#ifdef __cplusplus
extern "C" {
#endif
/**
  ******************************************************************************
  * @file    app_x-cube-ai.h
  * @author  X-CUBE-AI C code generator
  * @brief   AI entry function definitions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "ai_platform.h"

void MX_X_CUBE_AI_Init(void);
void MX_X_CUBE_AI_Process(void);
/* USER CODE BEGIN includes */
#include "gesture_openness.h"

/* CNN输入维度：20个通道 * 8 * 8个zone = 1280个float。 */
#define GESTURE_AI_INPUT_SIZE AI_GESTURE_OPENNESS_IN_1_SIZE

/**
  * @brief  运行一次手势开合度 AI 推理。
  * @param  input 已完成前处理和归一化的1280维CNN输入特征。
  * @param  openness_score 输出开合度，范围限制到 0~100。
  * @retval 0 成功，非 0 失败。
  */
int GestureAI_Run(const float input[GESTURE_AI_INPUT_SIZE], float *openness_score);

/* USER CODE END includes */
#ifdef __cplusplus
}
#endif
#endif /*__STMicroelectronics_X-CUBE-AI_10_2_0_H */
