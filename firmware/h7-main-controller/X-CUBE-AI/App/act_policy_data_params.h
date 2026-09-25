/**
  ******************************************************************************
  * @file    act_policy_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-08-11T22:46:18+0800
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

#ifndef ACT_POLICY_DATA_PARAMS_H
#define ACT_POLICY_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_ACT_POLICY_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_act_policy_data_weights_params[1]))
*/

#define AI_ACT_POLICY_DATA_CONFIG               (NULL)


#define AI_ACT_POLICY_DATA_ACTIVATIONS_SIZES \
  { 110748, }
#define AI_ACT_POLICY_DATA_ACTIVATIONS_SIZE     (110748)
#define AI_ACT_POLICY_DATA_ACTIVATIONS_COUNT    (1)
#define AI_ACT_POLICY_DATA_ACTIVATION_1_SIZE    (110748)



#define AI_ACT_POLICY_DATA_WEIGHTS_SIZES \
  { 709036, }
#define AI_ACT_POLICY_DATA_WEIGHTS_SIZE         (709036)
#define AI_ACT_POLICY_DATA_WEIGHTS_COUNT        (1)
#define AI_ACT_POLICY_DATA_WEIGHT_1_SIZE        (709036)



#define AI_ACT_POLICY_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_act_policy_activations_table[1])

extern ai_handle g_act_policy_activations_table[1 + 2];



#define AI_ACT_POLICY_DATA_WEIGHTS_TABLE_GET() \
  (&g_act_policy_weights_table[1])

extern ai_handle g_act_policy_weights_table[1 + 2];


#endif    /* ACT_POLICY_DATA_PARAMS_H */
