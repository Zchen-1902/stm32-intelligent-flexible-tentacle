
/**
  ******************************************************************************
  * @file    app_x-cube-ai.c
  * @author  X-CUBE-AI C code generator
  * @brief   AI program body
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

 /*
  * Description
  *   v1.0 - Minimum template to show how to use the Embedded Client API
  *          model. Only one input and one output is supported. All
  *          memory resources are allocated statically (AI_NETWORK_XX, defines
  *          are used).
  *          Re-target of the printf function is out-of-scope.
  *   v2.0 - add multiple IO and/or multiple heap support
  *
  *   For more information, see the embeded documentation:
  *
  *       [1] %X_CUBE_AI_DIR%/Documentation/index.html
  *
  *   X_CUBE_AI_DIR indicates the location where the X-CUBE-AI pack is installed
  *   typical : C:\Users\[user_name]\STM32Cube\Repository\STMicroelectronics\X-CUBE-AI\7.1.0
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#if defined ( __ICCARM__ )
#elif defined ( __CC_ARM ) || ( __GNUC__ )
#endif

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "main.h"
#include "ai_datatypes_defines.h"
#include "gesture_openness.h"
#include "gesture_openness_data.h"

/* USER CODE BEGIN includes */
/* USER CODE END includes */

/* IO buffers ----------------------------------------------------------------*/

#if !defined(AI_GESTURE_OPENNESS_INPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_in_1[AI_GESTURE_OPENNESS_IN_1_SIZE_BYTES];
ai_i8* data_ins[AI_GESTURE_OPENNESS_IN_NUM] = {
data_in_1
};
#else
ai_i8* data_ins[AI_GESTURE_OPENNESS_IN_NUM] = {
NULL
};
#endif

#if !defined(AI_GESTURE_OPENNESS_OUTPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_out_1[AI_GESTURE_OPENNESS_OUT_1_SIZE_BYTES];
ai_i8* data_outs[AI_GESTURE_OPENNESS_OUT_NUM] = {
data_out_1
};
#else
ai_i8* data_outs[AI_GESTURE_OPENNESS_OUT_NUM] = {
NULL
};
#endif

/* Activations buffers -------------------------------------------------------*/

AI_ALIGNED(32)
static uint8_t pool0[AI_GESTURE_OPENNESS_DATA_ACTIVATION_1_SIZE];

ai_handle data_activations0[] = {pool0};

/* AI objects ----------------------------------------------------------------*/

static ai_handle gesture_openness = AI_HANDLE_NULL;

static ai_buffer* ai_input;
static ai_buffer* ai_output;

static void ai_log_err(const ai_error err, const char *fct)
{
  /* USER CODE BEGIN log */
  (void)err;
  (void)fct;
  /* USER CODE END log */
}

static int ai_boostrap(ai_handle *act_addr)
{
  ai_error err;

  /* Create and initialize an instance of the model */
  err = ai_gesture_openness_create_and_init(&gesture_openness, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    ai_log_err(err, "ai_gesture_openness_create_and_init");
    return -1;
  }

  ai_input = ai_gesture_openness_inputs_get(gesture_openness, NULL);
  ai_output = ai_gesture_openness_outputs_get(gesture_openness, NULL);

#if defined(AI_GESTURE_OPENNESS_INPUTS_IN_ACTIVATIONS)
  /*  In the case where "--allocate-inputs" option is used, memory buffer can be
   *  used from the activations buffer. This is not mandatory.
   */
  for (int idx=0; idx < AI_GESTURE_OPENNESS_IN_NUM; idx++) {
	data_ins[idx] = ai_input[idx].data;
  }
#else
  for (int idx=0; idx < AI_GESTURE_OPENNESS_IN_NUM; idx++) {
	  ai_input[idx].data = data_ins[idx];
  }
#endif

#if defined(AI_GESTURE_OPENNESS_OUTPUTS_IN_ACTIVATIONS)
  /*  In the case where "--allocate-outputs" option is used, memory buffer can be
   *  used from the activations buffer. This is no mandatory.
   */
  for (int idx=0; idx < AI_GESTURE_OPENNESS_OUT_NUM; idx++) {
	data_outs[idx] = ai_output[idx].data;
  }
#else
  for (int idx=0; idx < AI_GESTURE_OPENNESS_OUT_NUM; idx++) {
	ai_output[idx].data = data_outs[idx];
  }
#endif

  return 0;
}

static int ai_run(void)
{
  ai_i32 batch;

  batch = ai_gesture_openness_run(gesture_openness, ai_input, ai_output);
  if (batch != 1) {
    ai_log_err(ai_gesture_openness_get_error(gesture_openness),
        "ai_gesture_openness_run");
    return -1;
  }

  return 0;
}

/* USER CODE BEGIN 2 */
/* CubeAI初始化状态：1表示模型创建和激活区绑定成功，可以执行推理。 */
static uint8_t gesture_ai_ready = 0U;

int acquire_and_process_data(ai_i8* data[])
{
  /* CubeAI 模板函数保留不用。
   * 实际输入由 GestureAI_Run() 直接写入模型输入缓冲区。
   */
  (void)data;
  return -1;
}

int post_process(ai_i8* data[])
{
  /* CubeAI 模板函数保留不用。
   * 实际输出由 GestureAI_Run() 直接读取模型输出缓冲区。
   */
  (void)data;
  return 0;
}

/**
  * @brief  运行一次手势开合度 AI 推理。
  * @param  input 已完成前处理和归一化的1280维CNN输入特征。
  * @param  openness_score 输出开合度，范围限制到 0~100。
  * @retval 0 成功，非 0 失败。
  * @note   这里不负责 VL53 数据提取，也不负责归一化，只负责调用 CubeAI。
  */
int GestureAI_Run(const float input[GESTURE_AI_INPUT_SIZE], float *openness_score)
{
  float score;
  const float *model_output;

  if ((gesture_ai_ready == 0U) || (input == NULL) || (openness_score == NULL)) {
    return -1;
  }

  if ((data_ins[0] == NULL) || (data_outs[0] == NULL)) {
    return -2;
  }

  memcpy(data_ins[0], input, AI_GESTURE_OPENNESS_IN_1_SIZE_BYTES);

  if (ai_run() != 0) {
    return -3;
  }

  model_output = (const float *)data_outs[0];
  score = model_output[0] * 100.0f;

  if (score < 0.0f) {
    score = 0.0f;
  } else if (score > 100.0f) {
    score = 100.0f;
  }

  *openness_score = score;
  return 0;
}
/* USER CODE END 2 */

/* Entry points --------------------------------------------------------------*/

void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 5 */
  gesture_ai_ready = (ai_boostrap(data_activations0) == 0) ? 1U : 0U;
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
  /* 不使用 CubeAI 自动循环推理。
   * 后续由 VL53 有效帧触发 GestureAI_Run()。
   */
    /* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif
