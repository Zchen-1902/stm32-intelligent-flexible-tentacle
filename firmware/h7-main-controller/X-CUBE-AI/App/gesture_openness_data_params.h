/**
  ******************************************************************************
  * @file    gesture_openness_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-08-11T22:45:51+0800
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef GESTURE_OPENNESS_DATA_PARAMS_H
#define GESTURE_OPENNESS_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_GESTURE_OPENNESS_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_gesture_openness_data_weights_params[1]))
*/

#define AI_GESTURE_OPENNESS_DATA_CONFIG               (NULL)


#define AI_GESTURE_OPENNESS_DATA_ACTIVATIONS_SIZES \
  { 13312, }
#define AI_GESTURE_OPENNESS_DATA_ACTIVATIONS_SIZE     (13312)
#define AI_GESTURE_OPENNESS_DATA_ACTIVATIONS_COUNT    (1)
#define AI_GESTURE_OPENNESS_DATA_ACTIVATION_1_SIZE    (13312)



#define AI_GESTURE_OPENNESS_DATA_WEIGHTS_SIZES \
  { 85412, }
#define AI_GESTURE_OPENNESS_DATA_WEIGHTS_SIZE         (85412)
#define AI_GESTURE_OPENNESS_DATA_WEIGHTS_COUNT        (1)
#define AI_GESTURE_OPENNESS_DATA_WEIGHT_1_SIZE        (85412)



#define AI_GESTURE_OPENNESS_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_gesture_openness_activations_table[1])

extern ai_handle g_gesture_openness_activations_table[1 + 2];



#define AI_GESTURE_OPENNESS_DATA_WEIGHTS_TABLE_GET() \
  (&g_gesture_openness_weights_table[1])

extern ai_handle g_gesture_openness_weights_table[1 + 2];


#endif    /* GESTURE_OPENNESS_DATA_PARAMS_H */
