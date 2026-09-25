/**
  ******************************************************************************
  * @file    act_policy.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-08-11T22:46:18+0800
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "act_policy.h"
#include "act_policy_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_act_policy
 
#undef AI_ACT_POLICY_MODEL_SIGNATURE
#define AI_ACT_POLICY_MODEL_SIGNATURE     "0x1b84c16dc8cb846b4e185e9e7b0e0d94"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2026-08-11T22:46:18+0800"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_ACT_POLICY_N_BATCHES
#define AI_ACT_POLICY_N_BATCHES         (1)

static ai_ptr g_act_policy_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_act_policy_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  state_history_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 36, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  state_history_Transpose_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 36, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_12_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  tof_history_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 13824, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_11_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_22_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_10_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_20_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_9_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_18_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_8_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_16_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_7_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_14_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_6_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_12_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_5_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_10_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_4_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_8_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_3_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_6_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_2_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_4_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_2_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  _Slice_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_12_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 36, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_11_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  _Div_11_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_11_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_11_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#35 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#36 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_11_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#37 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#38 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_10_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#39 */
AI_ARRAY_OBJ_DECLARE(
  _Div_10_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#40 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_10_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#41 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_10_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#42 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_10_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#43 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_10_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#44 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_10_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#45 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_10_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#46 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#47 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_9_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#48 */
AI_ARRAY_OBJ_DECLARE(
  _Div_9_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#49 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_9_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#50 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_9_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#51 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_9_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#52 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_9_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#53 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_9_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#54 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_9_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#55 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#56 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_8_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#57 */
AI_ARRAY_OBJ_DECLARE(
  _Div_8_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#58 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_8_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#59 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_8_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#60 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_8_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#61 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_8_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#62 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_8_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#63 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_8_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#64 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#65 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_7_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#66 */
AI_ARRAY_OBJ_DECLARE(
  _Div_7_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#67 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_7_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#68 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_7_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#69 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_7_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#70 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_7_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#71 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_7_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#72 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_7_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#73 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#74 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_6_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#75 */
AI_ARRAY_OBJ_DECLARE(
  _Div_6_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#76 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_6_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#77 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_6_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#78 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_6_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#79 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_6_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#80 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_6_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#81 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_6_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#82 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#83 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_5_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#84 */
AI_ARRAY_OBJ_DECLARE(
  _Div_5_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#85 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_5_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#86 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_5_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#87 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_5_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#88 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_5_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#89 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_5_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#90 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_5_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#91 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#92 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_4_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#93 */
AI_ARRAY_OBJ_DECLARE(
  _Div_4_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#94 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_4_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#95 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_4_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#96 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_4_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#97 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_4_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#98 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_4_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#99 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_4_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#100 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#101 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_3_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#102 */
AI_ARRAY_OBJ_DECLARE(
  _Div_3_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#103 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_3_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#104 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#105 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_3_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#106 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#107 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_3_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#108 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#109 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#110 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_2_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#111 */
AI_ARRAY_OBJ_DECLARE(
  _Div_2_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#112 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_2_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#113 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_2_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#114 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_2_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#115 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_2_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#116 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_2_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#117 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_2_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#118 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#119 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#120 */
AI_ARRAY_OBJ_DECLARE(
  _Div_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#121 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_1_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#122 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#123 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_1_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#124 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#125 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_1_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#126 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#127 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#128 */
AI_ARRAY_OBJ_DECLARE(
  _Sub_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#129 */
AI_ARRAY_OBJ_DECLARE(
  _Div_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1152, AI_STATIC)

/* Array#130 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#131 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#132 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#133 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#134 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#135 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_5_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#136 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_6_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#137 */
AI_ARRAY_OBJ_DECLARE(
  _Concat_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#138 */
AI_ARRAY_OBJ_DECLARE(
  _Div_12_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 36, AI_STATIC)

/* Array#139 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 36, AI_STATIC)

/* Array#140 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#141 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#142 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_0_Add_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#143 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#144 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#145 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#146 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#147 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_2_Add_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#148 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#149 */
AI_ARRAY_OBJ_DECLARE(
  _Concat_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#150 */
AI_ARRAY_OBJ_DECLARE(
  _Transpose_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 960, AI_STATIC)

/* Array#151 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#152 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#153 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#154 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#155 */
AI_ARRAY_OBJ_DECLARE(
  _Flatten_output_0_to_chlast_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#156 */
AI_ARRAY_OBJ_DECLARE(
  _action_head_action_head_0_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#157 */
AI_ARRAY_OBJ_DECLARE(
  _action_head_action_head_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#158 */
AI_ARRAY_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 30, AI_STATIC)

/* Array#159 */
AI_ARRAY_OBJ_DECLARE(
  _Reshape_24_output_0_to_chfirst_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 30, AI_STATIC)

/* Array#160 */
AI_ARRAY_OBJ_DECLARE(
  _Mul_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 30, AI_STATIC)

/* Array#161 */
AI_ARRAY_OBJ_DECLARE(
  action_q_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 30, AI_STATIC)

/* Array#162 */
AI_ARRAY_OBJ_DECLARE(
  action_q_Transpose_0_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 30, AI_STATIC)

/* Array#163 */
AI_ARRAY_OBJ_DECLARE(
  state_encoder_0_bias_3D_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#164 */
AI_ARRAY_OBJ_DECLARE(
  state_encoder_2_bias_3D_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#165 */
AI_ARRAY_OBJ_DECLARE(
  action_std_const_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3, AI_STATIC)

/* Array#166 */
AI_ARRAY_OBJ_DECLARE(
  state_mean_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3, AI_STATIC)

/* Array#167 */
AI_ARRAY_OBJ_DECLARE(
  tof_std_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 18, AI_STATIC)

/* Array#168 */
AI_ARRAY_OBJ_DECLARE(
  tof_mean_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 18, AI_STATIC)

/* Array#169 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 5184, AI_STATIC)

/* Array#170 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#171 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 13824, AI_STATIC)

/* Array#172 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#173 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 27648, AI_STATIC)

/* Array#174 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#175 */
AI_ARRAY_OBJ_DECLARE(
  state_std_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3, AI_STATIC)

/* Array#176 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#177 */
AI_ARRAY_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#178 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 15360, AI_STATIC)

/* Array#179 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#180 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#181 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#182 */
AI_ARRAY_OBJ_DECLARE(
  _action_head_action_head_0_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 98304, AI_STATIC)

/* Array#183 */
AI_ARRAY_OBJ_DECLARE(
  _action_head_action_head_0_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#184 */
AI_ARRAY_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 3840, AI_STATIC)

/* Array#185 */
AI_ARRAY_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 30, AI_STATIC)

/* Array#186 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#187 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#188 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#189 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_10_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#190 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_10_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#191 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_10_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#192 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_9_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#193 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_9_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#194 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_9_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#195 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_8_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#196 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_8_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#197 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_8_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#198 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_7_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#199 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_7_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#200 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_7_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#201 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_6_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#202 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_6_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#203 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_6_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#204 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_5_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#205 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_5_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#206 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_5_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#207 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_4_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#208 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_4_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#209 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_4_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#210 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_3_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#211 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_3_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#212 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_3_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#213 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_2_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#214 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_2_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#215 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_2_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#216 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_1_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#217 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_1_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#218 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_1_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#219 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_0_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 162, AI_STATIC)

/* Array#220 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_2_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/* Array#221 */
AI_ARRAY_OBJ_DECLARE(
  _tof_encoder_network_network_4_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 432, AI_STATIC)

/* Array#222 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#223 */
AI_ARRAY_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _Concat_1_output_0_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 80), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_Concat_1_output_0_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _Concat_output_0_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 64), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_Concat_output_0_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  _Div_10_output_0_output, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_10_output_0_output_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  _Div_11_output_0_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_11_output_0_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  _Div_12_output_0_output, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 3), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_Div_12_output_0_output_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  _Div_1_output_0_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_1_output_0_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  _Div_2_output_0_output, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_2_output_0_output_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _Div_3_output_0_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_3_output_0_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _Div_4_output_0_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_4_output_0_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _Div_5_output_0_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_5_output_0_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _Div_6_output_0_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_6_output_0_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  _Div_7_output_0_output, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_7_output_0_output_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  _Div_8_output_0_output, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_8_output_0_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  _Div_9_output_0_output, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_9_output_0_output_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _Div_output_0_output, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Div_output_0_output_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  _Flatten_output_0_to_chlast_output, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 12, 64), AI_STRIDE_INIT(4, 4, 4, 4, 48),
  1, &_Flatten_output_0_to_chlast_output_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  _Flatten_output_0_to_chlast_output0, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 768, 1, 1), AI_STRIDE_INIT(4, 4, 4, 3072, 3072),
  1, &_Flatten_output_0_to_chlast_output_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  _Mul_output_0_output, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 10, 1, 3), AI_STRIDE_INIT(4, 4, 4, 40, 40),
  1, &_Mul_output_0_output_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_10_output_0_to_chfirst_output, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_10_output_0_to_chfirst_output_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_12_output_0_to_chfirst_output, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_12_output_0_to_chfirst_output_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_14_output_0_to_chfirst_output, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_14_output_0_to_chfirst_output_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_16_output_0_to_chfirst_output, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_16_output_0_to_chfirst_output_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_18_output_0_to_chfirst_output, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_18_output_0_to_chfirst_output_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_20_output_0_to_chfirst_output, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_20_output_0_to_chfirst_output_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_22_output_0_to_chfirst_output, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_22_output_0_to_chfirst_output_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_24_output_0_to_chfirst_output, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 10, 1, 3), AI_STRIDE_INIT(4, 4, 4, 40, 40),
  1, &_Reshape_24_output_0_to_chfirst_output_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_2_output_0_to_chfirst_output, AI_STATIC,
  26, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_2_output_0_to_chfirst_output_array, NULL)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_4_output_0_to_chfirst_output, AI_STATIC,
  27, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_4_output_0_to_chfirst_output_array, NULL)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_6_output_0_to_chfirst_output, AI_STATIC,
  28, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_6_output_0_to_chfirst_output_array, NULL)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_8_output_0_to_chfirst_output, AI_STATIC,
  29, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_8_output_0_to_chfirst_output_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  _Reshape_output_0_to_chfirst_output, AI_STATIC,
  30, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Reshape_output_0_to_chfirst_output_array, NULL)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_10_output_0_output, AI_STATIC,
  31, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_10_output_0_output_array, NULL)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_10_output_0_output0, AI_STATIC,
  32, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_10_output_0_output_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_11_output_0_output, AI_STATIC,
  33, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_11_output_0_output_array, NULL)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_11_output_0_output0, AI_STATIC,
  34, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_11_output_0_output_array, NULL)

/* Tensor #35 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_12_output_0_output, AI_STATIC,
  35, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 3), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_Slice_12_output_0_output_array, NULL)

/* Tensor #36 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_1_output_0_output, AI_STATIC,
  36, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_1_output_0_output_array, NULL)

/* Tensor #37 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_1_output_0_output0, AI_STATIC,
  37, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_1_output_0_output_array, NULL)

/* Tensor #38 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_2_output_0_output, AI_STATIC,
  38, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_2_output_0_output_array, NULL)

/* Tensor #39 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_2_output_0_output0, AI_STATIC,
  39, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_2_output_0_output_array, NULL)

/* Tensor #40 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_3_output_0_output, AI_STATIC,
  40, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_3_output_0_output_array, NULL)

/* Tensor #41 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_3_output_0_output0, AI_STATIC,
  41, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_3_output_0_output_array, NULL)

/* Tensor #42 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_4_output_0_output, AI_STATIC,
  42, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_4_output_0_output_array, NULL)

/* Tensor #43 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_4_output_0_output0, AI_STATIC,
  43, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_4_output_0_output_array, NULL)

/* Tensor #44 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_5_output_0_output, AI_STATIC,
  44, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_5_output_0_output_array, NULL)

/* Tensor #45 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_5_output_0_output0, AI_STATIC,
  45, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_5_output_0_output_array, NULL)

/* Tensor #46 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_6_output_0_output, AI_STATIC,
  46, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_6_output_0_output_array, NULL)

/* Tensor #47 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_6_output_0_output0, AI_STATIC,
  47, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_6_output_0_output_array, NULL)

/* Tensor #48 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_7_output_0_output, AI_STATIC,
  48, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_7_output_0_output_array, NULL)

/* Tensor #49 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_7_output_0_output0, AI_STATIC,
  49, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_7_output_0_output_array, NULL)

/* Tensor #50 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_8_output_0_output, AI_STATIC,
  50, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_8_output_0_output_array, NULL)

/* Tensor #51 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_8_output_0_output0, AI_STATIC,
  51, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_8_output_0_output_array, NULL)

/* Tensor #52 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_9_output_0_output, AI_STATIC,
  52, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_9_output_0_output_array, NULL)

/* Tensor #53 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_9_output_0_output0, AI_STATIC,
  53, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_9_output_0_output_array, NULL)

/* Tensor #54 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_output_0_output, AI_STATIC,
  54, 0x0,
  AI_SHAPE_INIT(4, 1, 1152, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4608, 4608),
  1, &_Slice_output_0_output_array, NULL)

/* Tensor #55 */
AI_TENSOR_OBJ_DECLARE(
  _Slice_output_0_output0, AI_STATIC,
  55, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 18), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &_Slice_output_0_output_array, NULL)

/* Tensor #56 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_10_output_0_output, AI_STATIC,
  56, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_10_output_0_output_array, NULL)

/* Tensor #57 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_11_output_0_output, AI_STATIC,
  57, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_11_output_0_output_array, NULL)

/* Tensor #58 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_12_output_0_output, AI_STATIC,
  58, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 3), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_Sub_12_output_0_output_array, NULL)

/* Tensor #59 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_1_output_0_output, AI_STATIC,
  59, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_1_output_0_output_array, NULL)

/* Tensor #60 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_2_output_0_output, AI_STATIC,
  60, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_2_output_0_output_array, NULL)

/* Tensor #61 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_3_output_0_output, AI_STATIC,
  61, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_3_output_0_output_array, NULL)

/* Tensor #62 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_4_output_0_output, AI_STATIC,
  62, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_4_output_0_output_array, NULL)

/* Tensor #63 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_5_output_0_output, AI_STATIC,
  63, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_5_output_0_output_array, NULL)

/* Tensor #64 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_6_output_0_output, AI_STATIC,
  64, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_6_output_0_output_array, NULL)

/* Tensor #65 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_7_output_0_output, AI_STATIC,
  65, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_7_output_0_output_array, NULL)

/* Tensor #66 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_8_output_0_output, AI_STATIC,
  66, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_8_output_0_output_array, NULL)

/* Tensor #67 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_9_output_0_output, AI_STATIC,
  67, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_9_output_0_output_array, NULL)

/* Tensor #68 */
AI_TENSOR_OBJ_DECLARE(
  _Sub_output_0_output, AI_STATIC,
  68, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 8, 8), AI_STRIDE_INIT(4, 4, 4, 72, 576),
  1, &_Sub_output_0_output_array, NULL)

/* Tensor #69 */
AI_TENSOR_OBJ_DECLARE(
  _Transpose_output_0_output, AI_STATIC,
  69, 0x0,
  AI_SHAPE_INIT(4, 1, 80, 1, 12), AI_STRIDE_INIT(4, 4, 4, 320, 320),
  1, &_Transpose_output_0_output_array, NULL)

/* Tensor #70 */
AI_TENSOR_OBJ_DECLARE(
  _action_head_action_head_0_Gemm_output_0_bias, AI_STATIC,
  70, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_action_head_action_head_0_Gemm_output_0_bias_array, NULL)

/* Tensor #71 */
AI_TENSOR_OBJ_DECLARE(
  _action_head_action_head_0_Gemm_output_0_output, AI_STATIC,
  71, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_action_head_action_head_0_Gemm_output_0_output_array, NULL)

/* Tensor #72 */
AI_TENSOR_OBJ_DECLARE(
  _action_head_action_head_0_Gemm_output_0_weights, AI_STATIC,
  72, 0x0,
  AI_SHAPE_INIT(4, 768, 128, 1, 1), AI_STRIDE_INIT(4, 4, 3072, 393216, 393216),
  1, &_action_head_action_head_0_Gemm_output_0_weights_array, NULL)

/* Tensor #73 */
AI_TENSOR_OBJ_DECLARE(
  _action_head_action_head_1_Relu_output_0_output, AI_STATIC,
  73, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_action_head_action_head_1_Relu_output_0_output_array, NULL)

/* Tensor #74 */
AI_TENSOR_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_bias, AI_STATIC,
  74, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 1), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &_action_head_action_head_2_Gemm_output_0_bias_array, NULL)

/* Tensor #75 */
AI_TENSOR_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_output, AI_STATIC,
  75, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 1), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &_action_head_action_head_2_Gemm_output_0_output_array, NULL)

/* Tensor #76 */
AI_TENSOR_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_output0, AI_STATIC,
  76, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 10), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &_action_head_action_head_2_Gemm_output_0_output_array, NULL)

/* Tensor #77 */
AI_TENSOR_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_weights, AI_STATIC,
  77, 0x0,
  AI_SHAPE_INIT(4, 128, 30, 1, 1), AI_STRIDE_INIT(4, 4, 512, 15360, 15360),
  1, &_action_head_action_head_2_Gemm_output_0_weights_array, NULL)

/* Tensor #78 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_0_Add_output_0_output, AI_STATIC,
  78, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 16), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_state_encoder_state_encoder_0_Add_output_0_output_array, NULL)

/* Tensor #79 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_output, AI_STATIC,
  79, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 12), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_output_array, NULL)

/* Tensor #80 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_output, AI_STATIC,
  80, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 16), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_output_array, NULL)

/* Tensor #81 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_output, AI_STATIC,
  81, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 12), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_output_array, NULL)

/* Tensor #82 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_weights, AI_STATIC,
  82, 0x0,
  AI_SHAPE_INIT(4, 3, 16, 1, 1), AI_STRIDE_INIT(4, 4, 12, 192, 192),
  1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_weights_array, NULL)

/* Tensor #83 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_1_Relu_output_0_output, AI_STATIC,
  83, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 16), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_state_encoder_state_encoder_1_Relu_output_0_output_array, NULL)

/* Tensor #84 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_2_Add_output_0_output, AI_STATIC,
  84, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 16), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_state_encoder_state_encoder_2_Add_output_0_output_array, NULL)

/* Tensor #85 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_output, AI_STATIC,
  85, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 12), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_output_array, NULL)

/* Tensor #86 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_output, AI_STATIC,
  86, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 16), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_output_array, NULL)

/* Tensor #87 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_output, AI_STATIC,
  87, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 12), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_output_array, NULL)

/* Tensor #88 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_weights, AI_STATIC,
  88, 0x0,
  AI_SHAPE_INIT(4, 16, 16, 1, 1), AI_STRIDE_INIT(4, 4, 64, 1024, 1024),
  1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_weights_array, NULL)

/* Tensor #89 */
AI_TENSOR_OBJ_DECLARE(
  _state_encoder_state_encoder_3_Relu_output_0_output, AI_STATIC,
  89, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 16), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &_state_encoder_state_encoder_3_Relu_output_0_output_array, NULL)

/* Tensor #90 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_bias, AI_STATIC,
  90, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_temporal_encoder_temporal_encoder_0_Conv_output_0_bias_array, NULL)

/* Tensor #91 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_output, AI_STATIC,
  91, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 12), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_temporal_encoder_temporal_encoder_0_Conv_output_0_output_array, NULL)

/* Tensor #92 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_scratch0, AI_STATIC,
  92, 0x0,
  AI_SHAPE_INIT(4, 1, 80, 1, 3), AI_STRIDE_INIT(4, 4, 4, 320, 320),
  1, &_temporal_encoder_temporal_encoder_0_Conv_output_0_scratch0_array, NULL)

/* Tensor #93 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_weights, AI_STATIC,
  93, 0x0,
  AI_SHAPE_INIT(4, 80, 1, 3, 64), AI_STRIDE_INIT(4, 4, 320, 20480, 20480),
  1, &_temporal_encoder_temporal_encoder_0_Conv_output_0_weights_array, NULL)

/* Tensor #94 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_1_Relu_output_0_output, AI_STATIC,
  94, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 12), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_temporal_encoder_temporal_encoder_1_Relu_output_0_output_array, NULL)

/* Tensor #95 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_bias, AI_STATIC,
  95, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_temporal_encoder_temporal_encoder_2_Conv_output_0_bias_array, NULL)

/* Tensor #96 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_output, AI_STATIC,
  96, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 12), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_temporal_encoder_temporal_encoder_2_Conv_output_0_output_array, NULL)

/* Tensor #97 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_scratch0, AI_STATIC,
  97, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 3), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_temporal_encoder_temporal_encoder_2_Conv_output_0_scratch0_array, NULL)

/* Tensor #98 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_weights, AI_STATIC,
  98, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 3, 64), AI_STRIDE_INIT(4, 4, 256, 16384, 16384),
  1, &_temporal_encoder_temporal_encoder_2_Conv_output_0_weights_array, NULL)

/* Tensor #99 */
AI_TENSOR_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_3_Relu_output_0_output, AI_STATIC,
  99, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 12), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_temporal_encoder_temporal_encoder_3_Relu_output_0_output_array, NULL)

/* Tensor #100 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_10_Conv_output_0_output, AI_STATIC,
  100, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_10_Conv_output_0_output_array, NULL)

/* Tensor #101 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_10_Conv_output_0_scratch0, AI_STATIC,
  101, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_10_Conv_output_0_scratch0_array, NULL)

/* Tensor #102 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_bias, AI_STATIC,
  102, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_tof_encoder_network_network_0_11_Conv_output_0_bias_array, NULL)

/* Tensor #103 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_output, AI_STATIC,
  103, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_11_Conv_output_0_output_array, NULL)

/* Tensor #104 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_scratch0, AI_STATIC,
  104, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_11_Conv_output_0_scratch0_array, NULL)

/* Tensor #105 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_weights, AI_STATIC,
  105, 0x0,
  AI_SHAPE_INIT(4, 18, 3, 3, 32), AI_STRIDE_INIT(4, 4, 72, 2304, 6912),
  1, &_tof_encoder_network_network_0_11_Conv_output_0_weights_array, NULL)

/* Tensor #106 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_1_Conv_output_0_output, AI_STATIC,
  106, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_1_Conv_output_0_output_array, NULL)

/* Tensor #107 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_1_Conv_output_0_scratch0, AI_STATIC,
  107, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_1_Conv_output_0_scratch0_array, NULL)

/* Tensor #108 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_2_Conv_output_0_output, AI_STATIC,
  108, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_2_Conv_output_0_output_array, NULL)

/* Tensor #109 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_2_Conv_output_0_scratch0, AI_STATIC,
  109, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_2_Conv_output_0_scratch0_array, NULL)

/* Tensor #110 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_3_Conv_output_0_output, AI_STATIC,
  110, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_3_Conv_output_0_output_array, NULL)

/* Tensor #111 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_3_Conv_output_0_scratch0, AI_STATIC,
  111, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_3_Conv_output_0_scratch0_array, NULL)

/* Tensor #112 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_4_Conv_output_0_output, AI_STATIC,
  112, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_4_Conv_output_0_output_array, NULL)

/* Tensor #113 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_4_Conv_output_0_scratch0, AI_STATIC,
  113, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_4_Conv_output_0_scratch0_array, NULL)

/* Tensor #114 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_5_Conv_output_0_output, AI_STATIC,
  114, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_5_Conv_output_0_output_array, NULL)

/* Tensor #115 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_5_Conv_output_0_scratch0, AI_STATIC,
  115, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_5_Conv_output_0_scratch0_array, NULL)

/* Tensor #116 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_6_Conv_output_0_output, AI_STATIC,
  116, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_6_Conv_output_0_output_array, NULL)

/* Tensor #117 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_6_Conv_output_0_scratch0, AI_STATIC,
  117, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_6_Conv_output_0_scratch0_array, NULL)

/* Tensor #118 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_7_Conv_output_0_output, AI_STATIC,
  118, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_7_Conv_output_0_output_array, NULL)

/* Tensor #119 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_7_Conv_output_0_scratch0, AI_STATIC,
  119, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_7_Conv_output_0_scratch0_array, NULL)

/* Tensor #120 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_8_Conv_output_0_output, AI_STATIC,
  120, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_8_Conv_output_0_output_array, NULL)

/* Tensor #121 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_8_Conv_output_0_scratch0, AI_STATIC,
  121, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_8_Conv_output_0_scratch0_array, NULL)

/* Tensor #122 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_9_Conv_output_0_output, AI_STATIC,
  122, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_9_Conv_output_0_output_array, NULL)

/* Tensor #123 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_9_Conv_output_0_scratch0, AI_STATIC,
  123, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_9_Conv_output_0_scratch0_array, NULL)

/* Tensor #124 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_Conv_output_0_output, AI_STATIC,
  124, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_0_Conv_output_0_output_array, NULL)

/* Tensor #125 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_0_Conv_output_0_scratch0, AI_STATIC,
  125, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 3, 3), AI_STRIDE_INIT(4, 4, 4, 72, 216),
  1, &_tof_encoder_network_network_0_Conv_output_0_scratch0_array, NULL)

/* Tensor #126 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_10_Relu_output_0_output, AI_STATIC,
  126, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_10_Relu_output_0_output_array, NULL)

/* Tensor #127 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_11_Relu_output_0_output, AI_STATIC,
  127, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_11_Relu_output_0_output_array, NULL)

/* Tensor #128 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_1_Relu_output_0_output, AI_STATIC,
  128, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_1_Relu_output_0_output_array, NULL)

/* Tensor #129 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_2_Relu_output_0_output, AI_STATIC,
  129, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_2_Relu_output_0_output_array, NULL)

/* Tensor #130 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_3_Relu_output_0_output, AI_STATIC,
  130, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_3_Relu_output_0_output_array, NULL)

/* Tensor #131 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_4_Relu_output_0_output, AI_STATIC,
  131, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_4_Relu_output_0_output_array, NULL)

/* Tensor #132 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_5_Relu_output_0_output, AI_STATIC,
  132, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_5_Relu_output_0_output_array, NULL)

/* Tensor #133 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_6_Relu_output_0_output, AI_STATIC,
  133, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_6_Relu_output_0_output_array, NULL)

/* Tensor #134 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_7_Relu_output_0_output, AI_STATIC,
  134, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_7_Relu_output_0_output_array, NULL)

/* Tensor #135 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_8_Relu_output_0_output, AI_STATIC,
  135, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_8_Relu_output_0_output_array, NULL)

/* Tensor #136 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_9_Relu_output_0_output, AI_STATIC,
  136, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_9_Relu_output_0_output_array, NULL)

/* Tensor #137 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_1_Relu_output_0_output, AI_STATIC,
  137, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_tof_encoder_network_network_1_Relu_output_0_output_array, NULL)

/* Tensor #138 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_10_Conv_output_0_output, AI_STATIC,
  138, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_10_Conv_output_0_output_array, NULL)

/* Tensor #139 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_10_Conv_output_0_scratch0, AI_STATIC,
  139, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_10_Conv_output_0_scratch0_array, NULL)

/* Tensor #140 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_bias, AI_STATIC,
  140, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &_tof_encoder_network_network_2_11_Conv_output_0_bias_array, NULL)

/* Tensor #141 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_output, AI_STATIC,
  141, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_11_Conv_output_0_output_array, NULL)

/* Tensor #142 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_scratch0, AI_STATIC,
  142, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_11_Conv_output_0_scratch0_array, NULL)

/* Tensor #143 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_weights, AI_STATIC,
  143, 0x0,
  AI_SHAPE_INIT(4, 32, 3, 3, 48), AI_STRIDE_INIT(4, 4, 128, 6144, 18432),
  1, &_tof_encoder_network_network_2_11_Conv_output_0_weights_array, NULL)

/* Tensor #144 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_1_Conv_output_0_output, AI_STATIC,
  144, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_1_Conv_output_0_output_array, NULL)

/* Tensor #145 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_1_Conv_output_0_scratch0, AI_STATIC,
  145, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_1_Conv_output_0_scratch0_array, NULL)

/* Tensor #146 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_2_Conv_output_0_output, AI_STATIC,
  146, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_2_Conv_output_0_output_array, NULL)

/* Tensor #147 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_2_Conv_output_0_scratch0, AI_STATIC,
  147, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_2_Conv_output_0_scratch0_array, NULL)

/* Tensor #148 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_3_Conv_output_0_output, AI_STATIC,
  148, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_3_Conv_output_0_output_array, NULL)

/* Tensor #149 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_3_Conv_output_0_scratch0, AI_STATIC,
  149, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_3_Conv_output_0_scratch0_array, NULL)

/* Tensor #150 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_4_Conv_output_0_output, AI_STATIC,
  150, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_4_Conv_output_0_output_array, NULL)

/* Tensor #151 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_4_Conv_output_0_scratch0, AI_STATIC,
  151, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_4_Conv_output_0_scratch0_array, NULL)

/* Tensor #152 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_5_Conv_output_0_output, AI_STATIC,
  152, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_5_Conv_output_0_output_array, NULL)

/* Tensor #153 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_5_Conv_output_0_scratch0, AI_STATIC,
  153, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_5_Conv_output_0_scratch0_array, NULL)

/* Tensor #154 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_6_Conv_output_0_output, AI_STATIC,
  154, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_6_Conv_output_0_output_array, NULL)

/* Tensor #155 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_6_Conv_output_0_scratch0, AI_STATIC,
  155, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_6_Conv_output_0_scratch0_array, NULL)

/* Tensor #156 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_7_Conv_output_0_output, AI_STATIC,
  156, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_7_Conv_output_0_output_array, NULL)

/* Tensor #157 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_7_Conv_output_0_scratch0, AI_STATIC,
  157, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_7_Conv_output_0_scratch0_array, NULL)

/* Tensor #158 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_8_Conv_output_0_output, AI_STATIC,
  158, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_8_Conv_output_0_output_array, NULL)

/* Tensor #159 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_8_Conv_output_0_scratch0, AI_STATIC,
  159, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_8_Conv_output_0_scratch0_array, NULL)

/* Tensor #160 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_9_Conv_output_0_output, AI_STATIC,
  160, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_9_Conv_output_0_output_array, NULL)

/* Tensor #161 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_9_Conv_output_0_scratch0, AI_STATIC,
  161, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_9_Conv_output_0_scratch0_array, NULL)

/* Tensor #162 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_Conv_output_0_output, AI_STATIC,
  162, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_2_Conv_output_0_output_array, NULL)

/* Tensor #163 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_2_Conv_output_0_scratch0, AI_STATIC,
  163, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_tof_encoder_network_network_2_Conv_output_0_scratch0_array, NULL)

/* Tensor #164 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_10_Relu_output_0_output, AI_STATIC,
  164, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_10_Relu_output_0_output_array, NULL)

/* Tensor #165 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_11_Relu_output_0_output, AI_STATIC,
  165, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_11_Relu_output_0_output_array, NULL)

/* Tensor #166 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_1_Relu_output_0_output, AI_STATIC,
  166, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_1_Relu_output_0_output_array, NULL)

/* Tensor #167 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_2_Relu_output_0_output, AI_STATIC,
  167, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_2_Relu_output_0_output_array, NULL)

/* Tensor #168 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_3_Relu_output_0_output, AI_STATIC,
  168, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_3_Relu_output_0_output_array, NULL)

/* Tensor #169 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_4_Relu_output_0_output, AI_STATIC,
  169, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_4_Relu_output_0_output_array, NULL)

/* Tensor #170 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_5_Relu_output_0_output, AI_STATIC,
  170, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_5_Relu_output_0_output_array, NULL)

/* Tensor #171 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_6_Relu_output_0_output, AI_STATIC,
  171, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_6_Relu_output_0_output_array, NULL)

/* Tensor #172 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_7_Relu_output_0_output, AI_STATIC,
  172, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_7_Relu_output_0_output_array, NULL)

/* Tensor #173 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_8_Relu_output_0_output, AI_STATIC,
  173, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_8_Relu_output_0_output_array, NULL)

/* Tensor #174 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_9_Relu_output_0_output, AI_STATIC,
  174, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_9_Relu_output_0_output_array, NULL)

/* Tensor #175 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_3_Relu_output_0_output, AI_STATIC,
  175, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 4, 4), AI_STRIDE_INIT(4, 4, 4, 192, 768),
  1, &_tof_encoder_network_network_3_Relu_output_0_output_array, NULL)

/* Tensor #176 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_10_Conv_output_0_output, AI_STATIC,
  176, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_10_Conv_output_0_output_array, NULL)

/* Tensor #177 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_10_Conv_output_0_scratch0, AI_STATIC,
  177, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_10_Conv_output_0_scratch0_array, NULL)

/* Tensor #178 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_bias, AI_STATIC,
  178, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_4_11_Conv_output_0_bias_array, NULL)

/* Tensor #179 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_output, AI_STATIC,
  179, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_11_Conv_output_0_output_array, NULL)

/* Tensor #180 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_scratch0, AI_STATIC,
  180, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_11_Conv_output_0_scratch0_array, NULL)

/* Tensor #181 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_weights, AI_STATIC,
  181, 0x0,
  AI_SHAPE_INIT(4, 48, 3, 3, 64), AI_STRIDE_INIT(4, 4, 192, 12288, 36864),
  1, &_tof_encoder_network_network_4_11_Conv_output_0_weights_array, NULL)

/* Tensor #182 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_1_Conv_output_0_output, AI_STATIC,
  182, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_1_Conv_output_0_output_array, NULL)

/* Tensor #183 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_1_Conv_output_0_scratch0, AI_STATIC,
  183, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_1_Conv_output_0_scratch0_array, NULL)

/* Tensor #184 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_2_Conv_output_0_output, AI_STATIC,
  184, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_2_Conv_output_0_output_array, NULL)

/* Tensor #185 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_2_Conv_output_0_scratch0, AI_STATIC,
  185, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_2_Conv_output_0_scratch0_array, NULL)

/* Tensor #186 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_3_Conv_output_0_output, AI_STATIC,
  186, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_3_Conv_output_0_output_array, NULL)

/* Tensor #187 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_3_Conv_output_0_scratch0, AI_STATIC,
  187, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_3_Conv_output_0_scratch0_array, NULL)

/* Tensor #188 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_4_Conv_output_0_output, AI_STATIC,
  188, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_4_Conv_output_0_output_array, NULL)

/* Tensor #189 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_4_Conv_output_0_scratch0, AI_STATIC,
  189, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_4_Conv_output_0_scratch0_array, NULL)

/* Tensor #190 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_5_Conv_output_0_output, AI_STATIC,
  190, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_5_Conv_output_0_output_array, NULL)

/* Tensor #191 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_5_Conv_output_0_scratch0, AI_STATIC,
  191, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_5_Conv_output_0_scratch0_array, NULL)

/* Tensor #192 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_6_Conv_output_0_output, AI_STATIC,
  192, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_6_Conv_output_0_output_array, NULL)

/* Tensor #193 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_6_Conv_output_0_scratch0, AI_STATIC,
  193, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_6_Conv_output_0_scratch0_array, NULL)

/* Tensor #194 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_7_Conv_output_0_output, AI_STATIC,
  194, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_7_Conv_output_0_output_array, NULL)

/* Tensor #195 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_7_Conv_output_0_scratch0, AI_STATIC,
  195, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_7_Conv_output_0_scratch0_array, NULL)

/* Tensor #196 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_8_Conv_output_0_output, AI_STATIC,
  196, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_8_Conv_output_0_output_array, NULL)

/* Tensor #197 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_8_Conv_output_0_scratch0, AI_STATIC,
  197, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_8_Conv_output_0_scratch0_array, NULL)

/* Tensor #198 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_9_Conv_output_0_output, AI_STATIC,
  198, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_9_Conv_output_0_output_array, NULL)

/* Tensor #199 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_9_Conv_output_0_scratch0, AI_STATIC,
  199, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_9_Conv_output_0_scratch0_array, NULL)

/* Tensor #200 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_Conv_output_0_output, AI_STATIC,
  200, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_4_Conv_output_0_output_array, NULL)

/* Tensor #201 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_4_Conv_output_0_scratch0, AI_STATIC,
  201, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 3, 3), AI_STRIDE_INIT(4, 4, 4, 192, 576),
  1, &_tof_encoder_network_network_4_Conv_output_0_scratch0_array, NULL)

/* Tensor #202 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_10_Relu_output_0_output, AI_STATIC,
  202, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_10_Relu_output_0_output_array, NULL)

/* Tensor #203 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_11_Relu_output_0_output, AI_STATIC,
  203, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_11_Relu_output_0_output_array, NULL)

/* Tensor #204 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_1_Relu_output_0_output, AI_STATIC,
  204, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_1_Relu_output_0_output_array, NULL)

/* Tensor #205 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_2_Relu_output_0_output, AI_STATIC,
  205, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_2_Relu_output_0_output_array, NULL)

/* Tensor #206 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_3_Relu_output_0_output, AI_STATIC,
  206, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_3_Relu_output_0_output_array, NULL)

/* Tensor #207 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_4_Relu_output_0_output, AI_STATIC,
  207, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_4_Relu_output_0_output_array, NULL)

/* Tensor #208 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_5_Relu_output_0_output, AI_STATIC,
  208, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_5_Relu_output_0_output_array, NULL)

/* Tensor #209 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_6_Relu_output_0_output, AI_STATIC,
  209, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_6_Relu_output_0_output_array, NULL)

/* Tensor #210 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_7_Relu_output_0_output, AI_STATIC,
  210, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_7_Relu_output_0_output_array, NULL)

/* Tensor #211 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_8_Relu_output_0_output, AI_STATIC,
  211, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_8_Relu_output_0_output_array, NULL)

/* Tensor #212 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_9_Relu_output_0_output, AI_STATIC,
  212, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_9_Relu_output_0_output_array, NULL)

/* Tensor #213 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_5_Relu_output_0_output, AI_STATIC,
  213, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 2, 2), AI_STRIDE_INIT(4, 4, 4, 256, 512),
  1, &_tof_encoder_network_network_5_Relu_output_0_output_array, NULL)

/* Tensor #214 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output, AI_STATIC,
  214, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #215 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output0, AI_STATIC,
  215, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #216 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output, AI_STATIC,
  216, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #217 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output0, AI_STATIC,
  217, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #218 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output, AI_STATIC,
  218, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #219 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output0, AI_STATIC,
  219, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #220 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output, AI_STATIC,
  220, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #221 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output0, AI_STATIC,
  221, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #222 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output, AI_STATIC,
  222, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #223 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output0, AI_STATIC,
  223, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #224 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output, AI_STATIC,
  224, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #225 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output0, AI_STATIC,
  225, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #226 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output, AI_STATIC,
  226, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #227 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output0, AI_STATIC,
  227, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #228 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output, AI_STATIC,
  228, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #229 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output0, AI_STATIC,
  229, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #230 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output, AI_STATIC,
  230, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #231 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output0, AI_STATIC,
  231, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #232 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output, AI_STATIC,
  232, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #233 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output0, AI_STATIC,
  233, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #234 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output, AI_STATIC,
  234, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #235 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output0, AI_STATIC,
  235, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #236 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_GlobalAveragePool_output_0_output, AI_STATIC,
  236, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_tof_encoder_network_network_6_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #237 */
AI_TENSOR_OBJ_DECLARE(
  _tof_encoder_network_network_6_GlobalAveragePool_output_0_output0, AI_STATIC,
  237, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 64), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_tof_encoder_network_network_6_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #238 */
AI_TENSOR_OBJ_DECLARE(
  action_q_Transpose_0_output, AI_STATIC,
  238, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 10), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &action_q_Transpose_0_output_array, NULL)

/* Tensor #239 */
AI_TENSOR_OBJ_DECLARE(
  action_q_output, AI_STATIC,
  239, 0x0,
  AI_SHAPE_INIT(4, 1, 10, 1, 3), AI_STRIDE_INIT(4, 4, 4, 40, 40),
  1, &action_q_output_array, NULL)

/* Tensor #240 */
AI_TENSOR_OBJ_DECLARE(
  action_std_const, AI_STATIC,
  240, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 3), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &action_std_const_array, NULL)

/* Tensor #241 */
AI_TENSOR_OBJ_DECLARE(
  state_encoder_0_bias_3D, AI_STATIC,
  241, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 16), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &state_encoder_0_bias_3D_array, NULL)

/* Tensor #242 */
AI_TENSOR_OBJ_DECLARE(
  state_encoder_2_bias_3D, AI_STATIC,
  242, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 16), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &state_encoder_2_bias_3D_array, NULL)

/* Tensor #243 */
AI_TENSOR_OBJ_DECLARE(
  state_history_Transpose_output, AI_STATIC,
  243, 0x0,
  AI_SHAPE_INIT(4, 1, 12, 1, 3), AI_STRIDE_INIT(4, 4, 4, 48, 48),
  1, &state_history_Transpose_output_array, NULL)

/* Tensor #244 */
AI_TENSOR_OBJ_DECLARE(
  state_history_output, AI_STATIC,
  244, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 12), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &state_history_output_array, NULL)

/* Tensor #245 */
AI_TENSOR_OBJ_DECLARE(
  state_mean, AI_STATIC,
  245, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 3), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &state_mean_array, NULL)

/* Tensor #246 */
AI_TENSOR_OBJ_DECLARE(
  state_std, AI_STATIC,
  246, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 3), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &state_std_array, NULL)

/* Tensor #247 */
AI_TENSOR_OBJ_DECLARE(
  tof_history_output, AI_STATIC,
  247, 0x0,
  AI_SHAPE_INIT(4, 1, 13824, 1, 1), AI_STRIDE_INIT(4, 4, 4, 55296, 55296),
  1, &tof_history_output_array, NULL)

/* Tensor #248 */
AI_TENSOR_OBJ_DECLARE(
  tof_mean, AI_STATIC,
  248, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 1, 1), AI_STRIDE_INIT(4, 4, 4, 72, 72),
  1, &tof_mean_array, NULL)

/* Tensor #249 */
AI_TENSOR_OBJ_DECLARE(
  tof_std, AI_STATIC,
  249, 0x0,
  AI_SHAPE_INIT(4, 1, 18, 1, 1), AI_STRIDE_INIT(4, 4, 4, 72, 72),
  1, &tof_std_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  action_q_Transpose_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &action_q_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &action_q_Transpose_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  action_q_Transpose_0_layer, 1,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &action_q_Transpose_0_chain,
  NULL, &action_q_Transpose_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  action_q_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Mul_output_0_output, &_Slice_12_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &action_q_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  action_q_layer, 259,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &action_q_chain,
  NULL, &action_q_Transpose_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_24_output_0_to_chfirst_output, &action_std_const),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Mul_output_0_layer, 258,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Mul_output_0_chain,
  NULL, &action_q_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_24_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_action_head_action_head_2_Gemm_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_24_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_24_output_0_to_chfirst_layer, 252,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_24_output_0_to_chfirst_chain,
  NULL, &_Mul_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_action_head_action_head_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_action_head_action_head_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_action_head_action_head_2_Gemm_output_0_weights, &_action_head_action_head_2_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _action_head_action_head_2_Gemm_output_0_layer, 250,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_action_head_action_head_2_Gemm_output_0_chain,
  NULL, &_Reshape_24_output_0_to_chfirst_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _action_head_action_head_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_action_head_action_head_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_action_head_action_head_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _action_head_action_head_1_Relu_output_0_layer, 249,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_action_head_action_head_1_Relu_output_0_chain,
  NULL, &_action_head_action_head_2_Gemm_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _action_head_action_head_0_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Flatten_output_0_to_chlast_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_action_head_action_head_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_action_head_action_head_0_Gemm_output_0_weights, &_action_head_action_head_0_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _action_head_action_head_0_Gemm_output_0_layer, 248,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_action_head_action_head_0_Gemm_output_0_chain,
  NULL, &_action_head_action_head_1_Relu_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Flatten_output_0_to_chlast_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_temporal_encoder_temporal_encoder_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Flatten_output_0_to_chlast_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Flatten_output_0_to_chlast_layer, 247,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Flatten_output_0_to_chlast_chain,
  NULL, &_action_head_action_head_0_Gemm_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_temporal_encoder_temporal_encoder_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_temporal_encoder_temporal_encoder_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_3_Relu_output_0_layer, 246,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_temporal_encoder_temporal_encoder_3_Relu_output_0_chain,
  NULL, &_Flatten_output_0_to_chlast_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_temporal_encoder_temporal_encoder_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_temporal_encoder_temporal_encoder_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_temporal_encoder_temporal_encoder_2_Conv_output_0_weights, &_temporal_encoder_temporal_encoder_2_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_temporal_encoder_temporal_encoder_2_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_2_Conv_output_0_layer, 245,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_temporal_encoder_temporal_encoder_2_Conv_output_0_chain,
  NULL, &_temporal_encoder_temporal_encoder_3_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_temporal_encoder_temporal_encoder_0_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_temporal_encoder_temporal_encoder_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_1_Relu_output_0_layer, 244,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_temporal_encoder_temporal_encoder_1_Relu_output_0_chain,
  NULL, &_temporal_encoder_temporal_encoder_2_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Transpose_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_temporal_encoder_temporal_encoder_0_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_temporal_encoder_temporal_encoder_0_Conv_output_0_weights, &_temporal_encoder_temporal_encoder_0_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_temporal_encoder_temporal_encoder_0_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _temporal_encoder_temporal_encoder_0_Conv_output_0_layer, 243,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_temporal_encoder_temporal_encoder_0_Conv_output_0_chain,
  NULL, &_temporal_encoder_temporal_encoder_1_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Transpose_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Concat_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Transpose_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Transpose_output_0_layer, 240,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Transpose_output_0_chain,
  NULL, &_temporal_encoder_temporal_encoder_0_Conv_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Concat_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Concat_output_0_output, &_state_encoder_state_encoder_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Concat_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Concat_1_output_0_layer, 239,
  CONCAT_TYPE, 0x0, NULL,
  concat, forward_concat,
  &_Concat_1_output_0_chain,
  NULL, &_Transpose_output_0_layer, AI_STATIC, 
  .axis = AI_SHAPE_HEIGHT, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_2_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_3_Relu_output_0_layer, 237,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_state_encoder_state_encoder_3_Relu_output_0_chain,
  NULL, &_Concat_1_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_2_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &state_encoder_2_bias_3D, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_2_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_2_Add_output_0_layer, 236,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_state_encoder_state_encoder_2_Add_output_0_chain,
  NULL, &_state_encoder_state_encoder_3_Relu_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_layer, 235,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_chain,
  NULL, &_state_encoder_state_encoder_2_Add_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_weights),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_layer, 235,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_chain,
  NULL, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_layer, 235,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_chain,
  NULL, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_0_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_1_Relu_output_0_layer, 234,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_state_encoder_state_encoder_1_Relu_output_0_chain,
  NULL, &_state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_0_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &state_encoder_0_bias_3D, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_0_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_0_Add_output_0_layer, 233,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_state_encoder_state_encoder_0_Add_output_0_chain,
  NULL, &_state_encoder_state_encoder_1_Relu_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_layer, 232,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_chain,
  NULL, &_state_encoder_state_encoder_0_Add_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_weights),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_layer, 232,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_chain,
  NULL, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_12_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_layer, 232,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_chain,
  NULL, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_12_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_12_output_0_output, &state_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_12_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_12_output_0_layer, 231,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_12_output_0_chain,
  NULL, &_state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Concat_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 12, &_tof_encoder_network_network_6_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output0, &_tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Concat_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Concat_output_0_layer, 238,
  CONCAT_TYPE, 0x0, NULL,
  concat, forward_concat,
  &_Concat_output_0_chain,
  NULL, &_Div_12_output_0_layer, AI_STATIC, 
  .axis = AI_SHAPE_CHANNEL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_GlobalAveragePool_output_0_layer, 17,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_GlobalAveragePool_output_0_chain,
  NULL, &_Concat_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_Relu_output_0_layer, 16,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_Conv_output_0_layer, 15,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_Relu_output_0_layer, 14,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_Conv_output_0_layer, 13,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_Relu_output_0_layer, 12,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_Conv_output_0_layer, 11,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_output_0_layer, 10,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_output_0_chain,
  NULL, &_tof_encoder_network_network_0_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_output_0_layer, 9,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_output_0_chain,
  NULL, &_Div_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_1_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_1_GlobalAveragePool_output_0_layer, 36,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_1_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_1_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_1_Relu_output_0_layer, 35,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_1_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_1_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_1_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_1_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_1_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_1_Conv_output_0_layer, 34,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_1_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_1_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_1_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_1_Relu_output_0_layer, 33,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_1_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_1_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_1_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_1_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_1_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_1_Conv_output_0_layer, 32,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_1_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_1_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_1_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_1_Relu_output_0_layer, 31,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_1_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_1_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_1_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_1_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_1_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_1_Conv_output_0_layer, 30,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_1_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_1_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_1_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_1_output_0_layer, 29,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_1_output_0_chain,
  NULL, &_tof_encoder_network_network_0_1_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_2_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_1_output_0_layer, 28,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_1_output_0_chain,
  NULL, &_Div_1_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_2_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_2_GlobalAveragePool_output_0_layer, 55,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_2_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_1_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_2_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_2_Relu_output_0_layer, 54,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_2_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_2_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_2_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_2_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_2_Conv_output_0_layer, 53,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_2_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_2_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_2_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_2_Relu_output_0_layer, 52,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_2_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_2_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_2_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_2_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_2_Conv_output_0_layer, 51,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_2_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_2_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_2_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_2_Relu_output_0_layer, 50,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_2_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_2_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_2_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_2_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_2_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_2_Conv_output_0_layer, 49,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_2_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_2_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_2_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_2_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_2_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_2_output_0_layer, 48,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_2_output_0_chain,
  NULL, &_tof_encoder_network_network_0_2_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_2_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_4_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_2_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_2_output_0_layer, 47,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_2_output_0_chain,
  NULL, &_Div_2_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_3_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_3_GlobalAveragePool_output_0_layer, 74,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_3_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_2_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_3_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_3_Relu_output_0_layer, 73,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_3_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_3_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_3_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_3_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_3_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_3_Conv_output_0_layer, 72,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_3_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_3_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_3_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_3_Relu_output_0_layer, 71,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_3_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_3_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_3_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_3_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_3_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_3_Conv_output_0_layer, 70,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_3_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_3_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_3_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_3_Relu_output_0_layer, 69,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_3_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_3_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_3_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_3_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_3_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_3_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_3_Conv_output_0_layer, 68,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_3_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_3_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_3_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_3_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_3_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_3_output_0_layer, 67,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_3_output_0_chain,
  NULL, &_tof_encoder_network_network_0_3_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_3_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_6_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_3_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_3_output_0_layer, 66,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_3_output_0_chain,
  NULL, &_Div_3_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_4_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_4_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_4_GlobalAveragePool_output_0_layer, 93,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_4_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_3_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_4_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_4_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_4_Relu_output_0_layer, 92,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_4_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_4_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_4_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_4_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_4_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_4_Conv_output_0_layer, 91,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_4_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_4_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_4_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_4_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_4_Relu_output_0_layer, 90,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_4_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_4_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_4_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_4_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_4_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_4_Conv_output_0_layer, 89,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_4_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_4_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_4_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_4_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_4_Relu_output_0_layer, 88,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_4_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_4_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_4_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_4_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_4_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_4_Conv_output_0_layer, 87,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_4_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_4_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_4_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_4_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_4_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_4_output_0_layer, 86,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_4_output_0_chain,
  NULL, &_tof_encoder_network_network_0_4_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_4_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_8_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_4_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_4_output_0_layer, 85,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_4_output_0_chain,
  NULL, &_Div_4_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_5_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_5_GlobalAveragePool_output_0_layer, 112,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_5_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_4_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_5_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_5_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_5_Relu_output_0_layer, 111,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_5_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_5_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_5_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_5_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_5_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_5_Conv_output_0_layer, 110,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_5_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_5_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_5_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_5_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_5_Relu_output_0_layer, 109,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_5_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_5_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_5_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_5_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_5_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_5_Conv_output_0_layer, 108,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_5_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_5_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_5_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_5_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_5_Relu_output_0_layer, 107,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_5_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_5_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_5_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_5_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_5_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_5_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_5_Conv_output_0_layer, 106,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_5_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_5_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_5_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_5_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_5_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_5_output_0_layer, 105,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_5_output_0_chain,
  NULL, &_tof_encoder_network_network_0_5_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_5_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_10_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_5_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_5_output_0_layer, 104,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_5_output_0_chain,
  NULL, &_Div_5_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_6_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_6_GlobalAveragePool_output_0_layer, 131,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_6_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_5_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_6_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_6_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_6_Relu_output_0_layer, 130,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_6_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_6_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_6_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_6_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_6_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_6_Conv_output_0_layer, 129,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_6_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_6_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_6_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_6_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_6_Relu_output_0_layer, 128,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_6_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_6_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_6_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_6_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_6_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_6_Conv_output_0_layer, 127,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_6_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_6_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_6_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_6_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_6_Relu_output_0_layer, 126,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_6_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_6_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_6_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_6_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_6_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_6_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_6_Conv_output_0_layer, 125,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_6_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_6_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_6_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_6_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_6_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_6_output_0_layer, 124,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_6_output_0_chain,
  NULL, &_tof_encoder_network_network_0_6_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_6_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_12_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_6_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_6_output_0_layer, 123,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_6_output_0_chain,
  NULL, &_Div_6_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_7_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_7_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_7_GlobalAveragePool_output_0_layer, 150,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_7_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_6_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_7_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_7_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_7_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_7_Relu_output_0_layer, 149,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_7_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_7_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_7_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_7_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_7_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_7_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_7_Conv_output_0_layer, 148,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_7_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_7_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_7_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_7_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_7_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_7_Relu_output_0_layer, 147,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_7_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_7_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_7_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_7_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_7_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_7_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_7_Conv_output_0_layer, 146,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_7_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_7_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_7_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_7_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_7_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_7_Relu_output_0_layer, 145,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_7_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_7_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_7_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_7_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_7_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_7_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_7_Conv_output_0_layer, 144,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_7_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_7_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_7_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_7_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_7_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_7_output_0_layer, 143,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_7_output_0_chain,
  NULL, &_tof_encoder_network_network_0_7_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_7_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_14_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_7_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_7_output_0_layer, 142,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_7_output_0_chain,
  NULL, &_Div_7_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_8_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_8_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_8_GlobalAveragePool_output_0_layer, 169,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_8_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_7_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_8_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_8_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_8_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_8_Relu_output_0_layer, 168,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_8_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_8_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_8_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_8_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_8_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_8_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_8_Conv_output_0_layer, 167,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_8_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_8_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_8_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_8_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_8_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_8_Relu_output_0_layer, 166,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_8_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_8_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_8_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_8_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_8_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_8_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_8_Conv_output_0_layer, 165,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_8_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_8_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_8_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_8_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_8_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_8_Relu_output_0_layer, 164,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_8_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_8_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_8_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_8_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_8_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_8_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_8_Conv_output_0_layer, 163,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_8_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_8_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_8_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_8_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_8_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_8_output_0_layer, 162,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_8_output_0_chain,
  NULL, &_tof_encoder_network_network_0_8_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_8_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_16_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_8_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_8_output_0_layer, 161,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_8_output_0_chain,
  NULL, &_Div_8_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_9_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_9_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_9_GlobalAveragePool_output_0_layer, 188,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_9_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_8_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_9_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_9_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_9_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_9_Relu_output_0_layer, 187,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_9_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_9_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_9_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_9_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_9_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_9_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_9_Conv_output_0_layer, 186,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_9_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_9_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_9_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_9_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_9_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_9_Relu_output_0_layer, 185,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_9_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_9_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_9_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_9_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_9_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_9_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_9_Conv_output_0_layer, 184,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_9_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_9_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_9_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_9_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_9_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_9_Relu_output_0_layer, 183,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_9_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_9_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_9_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_9_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_9_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_9_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_9_Conv_output_0_layer, 182,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_9_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_9_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_9_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_9_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_9_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_9_output_0_layer, 181,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_9_output_0_chain,
  NULL, &_tof_encoder_network_network_0_9_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_9_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_18_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_9_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_9_output_0_layer, 180,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_9_output_0_chain,
  NULL, &_Div_9_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_10_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_10_GlobalAveragePool_output_0_layer, 207,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_10_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_9_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_10_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_10_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_10_Relu_output_0_layer, 206,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_10_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_10_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_10_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_10_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_10_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_10_Conv_output_0_layer, 205,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_10_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_10_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_10_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_10_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_10_Relu_output_0_layer, 204,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_10_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_10_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_10_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_10_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_10_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_10_Conv_output_0_layer, 203,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_10_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_10_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_10_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_10_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_10_Relu_output_0_layer, 202,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_10_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_10_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_10_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_10_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_10_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_10_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_10_Conv_output_0_layer, 201,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_10_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_10_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_10_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_10_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_10_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_10_output_0_layer, 200,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_10_output_0_chain,
  NULL, &_tof_encoder_network_network_0_10_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_10_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_20_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_10_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_10_output_0_layer, 199,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_10_output_0_chain,
  NULL, &_Div_10_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_6_11_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_11_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_6_11_GlobalAveragePool_output_0_layer, 226,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_tof_encoder_network_network_6_11_GlobalAveragePool_output_0_chain,
  NULL, &_Sub_10_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(2, 2), 
  .pool_stride = AI_SHAPE_2D_INIT(2, 2), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_5_11_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_11_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_5_11_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_5_11_Relu_output_0_layer, 225,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_5_11_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_6_11_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_11_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_4_11_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_4_11_Conv_output_0_weights, &_tof_encoder_network_network_4_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_4_11_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_4_11_Conv_output_0_layer, 224,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_4_11_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_5_11_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_3_11_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_11_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_3_11_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_3_11_Relu_output_0_layer, 223,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_3_11_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_4_11_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_11_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_2_11_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_2_11_Conv_output_0_weights, &_tof_encoder_network_network_2_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_2_11_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_2_11_Conv_output_0_layer, 222,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_2_11_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_3_11_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(2, 2), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_1_11_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_11_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_1_11_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_1_11_Relu_output_0_layer, 221,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_tof_encoder_network_network_1_11_Relu_output_0_chain,
  NULL, &_tof_encoder_network_network_2_11_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_11_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_tof_encoder_network_network_0_11_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_tof_encoder_network_network_0_11_Conv_output_0_weights, &_tof_encoder_network_network_0_11_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_tof_encoder_network_network_0_11_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _tof_encoder_network_network_0_11_Conv_output_0_layer, 220,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_tof_encoder_network_network_0_11_Conv_output_0_chain,
  NULL, &_tof_encoder_network_network_1_11_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Div_11_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Sub_11_output_0_output, &tof_std),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Div_11_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Div_11_output_0_layer, 219,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Div_11_output_0_chain,
  NULL, &_tof_encoder_network_network_0_11_Conv_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_11_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_Reshape_22_output_0_to_chfirst_output, &tof_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_11_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_11_output_0_layer, 218,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_11_output_0_chain,
  NULL, &_Div_11_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Sub_12_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &state_history_Transpose_output, &state_mean),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Sub_12_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Sub_12_output_0_layer, 230,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_Sub_12_output_0_chain,
  NULL, &_Sub_11_output_0_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_output_0_to_chfirst_layer, 8,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_output_0_to_chfirst_chain,
  NULL, &_Sub_12_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_output_0_axes_data, _Slice_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_output_0_starts_data[] = { 0 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_output_0_starts_data, _Slice_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_output_0_ends_data[] = { 1152 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_output_0_ends_data, _Slice_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_output_0_layer, 6,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_output_0_chain,
  NULL, &_Reshape_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_output_0_axes, 
  .starts = &_Slice_output_0_starts, 
  .ends = &_Slice_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_2_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_1_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_2_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_2_output_0_to_chfirst_layer, 27,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_2_output_0_to_chfirst_chain,
  NULL, &_Slice_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_1_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_1_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_1_output_0_axes_data, _Slice_1_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_1_output_0_starts_data[] = { 1152 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_1_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_1_output_0_starts_data, _Slice_1_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_1_output_0_ends_data[] = { 2304 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_1_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_1_output_0_ends_data, _Slice_1_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_1_output_0_layer, 25,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_1_output_0_chain,
  NULL, &_Reshape_2_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_1_output_0_axes, 
  .starts = &_Slice_1_output_0_starts, 
  .ends = &_Slice_1_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_4_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_2_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_4_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_4_output_0_to_chfirst_layer, 46,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_4_output_0_to_chfirst_chain,
  NULL, &_Slice_1_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_2_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_2_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_2_output_0_axes_data, _Slice_2_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_2_output_0_starts_data[] = { 2304 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_2_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_2_output_0_starts_data, _Slice_2_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_2_output_0_ends_data[] = { 3456 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_2_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_2_output_0_ends_data, _Slice_2_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_2_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_2_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_2_output_0_layer, 44,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_2_output_0_chain,
  NULL, &_Reshape_4_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_2_output_0_axes, 
  .starts = &_Slice_2_output_0_starts, 
  .ends = &_Slice_2_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_6_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_3_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_6_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_6_output_0_to_chfirst_layer, 65,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_6_output_0_to_chfirst_chain,
  NULL, &_Slice_2_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_3_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_3_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_3_output_0_axes_data, _Slice_3_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_3_output_0_starts_data[] = { 3456 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_3_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_3_output_0_starts_data, _Slice_3_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_3_output_0_ends_data[] = { 4608 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_3_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_3_output_0_ends_data, _Slice_3_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_3_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_3_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_3_output_0_layer, 63,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_3_output_0_chain,
  NULL, &_Reshape_6_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_3_output_0_axes, 
  .starts = &_Slice_3_output_0_starts, 
  .ends = &_Slice_3_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_8_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_4_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_8_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_8_output_0_to_chfirst_layer, 84,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_8_output_0_to_chfirst_chain,
  NULL, &_Slice_3_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_4_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_4_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_4_output_0_axes_data, _Slice_4_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_4_output_0_starts_data[] = { 4608 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_4_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_4_output_0_starts_data, _Slice_4_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_4_output_0_ends_data[] = { 5760 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_4_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_4_output_0_ends_data, _Slice_4_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_4_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_4_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_4_output_0_layer, 82,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_4_output_0_chain,
  NULL, &_Reshape_8_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_4_output_0_axes, 
  .starts = &_Slice_4_output_0_starts, 
  .ends = &_Slice_4_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_10_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_5_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_10_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_10_output_0_to_chfirst_layer, 103,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_10_output_0_to_chfirst_chain,
  NULL, &_Slice_4_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_5_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_5_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_5_output_0_axes_data, _Slice_5_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_5_output_0_starts_data[] = { 5760 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_5_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_5_output_0_starts_data, _Slice_5_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_5_output_0_ends_data[] = { 6912 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_5_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_5_output_0_ends_data, _Slice_5_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_5_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_5_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_5_output_0_layer, 101,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_5_output_0_chain,
  NULL, &_Reshape_10_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_5_output_0_axes, 
  .starts = &_Slice_5_output_0_starts, 
  .ends = &_Slice_5_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_12_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_6_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_12_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_12_output_0_to_chfirst_layer, 122,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_12_output_0_to_chfirst_chain,
  NULL, &_Slice_5_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_6_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_6_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_6_output_0_axes_data, _Slice_6_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_6_output_0_starts_data[] = { 6912 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_6_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_6_output_0_starts_data, _Slice_6_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_6_output_0_ends_data[] = { 8064 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_6_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_6_output_0_ends_data, _Slice_6_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_6_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_6_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_6_output_0_layer, 120,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_6_output_0_chain,
  NULL, &_Reshape_12_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_6_output_0_axes, 
  .starts = &_Slice_6_output_0_starts, 
  .ends = &_Slice_6_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_14_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_7_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_14_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_14_output_0_to_chfirst_layer, 141,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_14_output_0_to_chfirst_chain,
  NULL, &_Slice_6_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_7_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_7_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_7_output_0_axes_data, _Slice_7_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_7_output_0_starts_data[] = { 8064 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_7_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_7_output_0_starts_data, _Slice_7_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_7_output_0_ends_data[] = { 9216 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_7_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_7_output_0_ends_data, _Slice_7_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_7_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_7_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_7_output_0_layer, 139,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_7_output_0_chain,
  NULL, &_Reshape_14_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_7_output_0_axes, 
  .starts = &_Slice_7_output_0_starts, 
  .ends = &_Slice_7_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_16_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_8_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_16_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_16_output_0_to_chfirst_layer, 160,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_16_output_0_to_chfirst_chain,
  NULL, &_Slice_7_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_8_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_8_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_8_output_0_axes_data, _Slice_8_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_8_output_0_starts_data[] = { 9216 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_8_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_8_output_0_starts_data, _Slice_8_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_8_output_0_ends_data[] = { 10368 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_8_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_8_output_0_ends_data, _Slice_8_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_8_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_8_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_8_output_0_layer, 158,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_8_output_0_chain,
  NULL, &_Reshape_16_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_8_output_0_axes, 
  .starts = &_Slice_8_output_0_starts, 
  .ends = &_Slice_8_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_18_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_9_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_18_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_18_output_0_to_chfirst_layer, 179,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_18_output_0_to_chfirst_chain,
  NULL, &_Slice_8_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_9_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_9_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_9_output_0_axes_data, _Slice_9_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_9_output_0_starts_data[] = { 10368 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_9_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_9_output_0_starts_data, _Slice_9_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_9_output_0_ends_data[] = { 11520 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_9_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_9_output_0_ends_data, _Slice_9_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_9_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_9_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_9_output_0_layer, 177,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_9_output_0_chain,
  NULL, &_Reshape_18_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_9_output_0_axes, 
  .starts = &_Slice_9_output_0_starts, 
  .ends = &_Slice_9_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_20_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_10_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_20_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_20_output_0_to_chfirst_layer, 198,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_20_output_0_to_chfirst_chain,
  NULL, &_Slice_9_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_10_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_10_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_10_output_0_axes_data, _Slice_10_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_10_output_0_starts_data[] = { 11520 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_10_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_10_output_0_starts_data, _Slice_10_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_10_output_0_ends_data[] = { 12672 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_10_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_10_output_0_ends_data, _Slice_10_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_10_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_10_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_10_output_0_layer, 196,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_10_output_0_chain,
  NULL, &_Reshape_20_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_10_output_0_axes, 
  .starts = &_Slice_10_output_0_starts, 
  .ends = &_Slice_10_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Reshape_22_output_0_to_chfirst_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_11_output_0_output0),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Reshape_22_output_0_to_chfirst_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Reshape_22_output_0_to_chfirst_layer, 217,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &_Reshape_22_output_0_to_chfirst_chain,
  NULL, &_Slice_10_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


AI_STATIC_CONST ai_u8 _Slice_11_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_11_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_11_output_0_axes_data, _Slice_11_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_11_output_0_starts_data[] = { 12672 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_11_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_11_output_0_starts_data, _Slice_11_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_11_output_0_ends_data[] = { 13824 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_11_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_11_output_0_ends_data, _Slice_11_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_11_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tof_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_11_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_11_output_0_layer, 215,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_11_output_0_chain,
  NULL, &_Reshape_22_output_0_to_chfirst_layer, AI_STATIC, 
  .axes = &_Slice_11_output_0_axes, 
  .starts = &_Slice_11_output_0_starts, 
  .ends = &_Slice_11_output_0_ends, 
)


AI_STATIC_CONST ai_u8 _Slice_12_output_0_axes_data[] = { 2 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_12_output_0_axes, AI_ARRAY_FORMAT_U8,
    _Slice_12_output_0_axes_data, _Slice_12_output_0_axes_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_12_output_0_starts_data[] = { 11 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_12_output_0_starts, AI_ARRAY_FORMAT_S16,
    _Slice_12_output_0_starts_data, _Slice_12_output_0_starts_data, 1, AI_STATIC_CONST)

AI_STATIC_CONST ai_i16 _Slice_12_output_0_ends_data[] = { 12 };
AI_ARRAY_OBJ_DECLARE(
    _Slice_12_output_0_ends, AI_ARRAY_FORMAT_S16,
    _Slice_12_output_0_ends_data, _Slice_12_output_0_ends_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _Slice_12_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &state_history_Transpose_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_Slice_12_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _Slice_12_output_0_layer, 257,
  SLICE_TYPE, 0x0, NULL,
  slice, forward_slice,
  &_Slice_12_output_0_chain,
  NULL, &_Slice_11_output_0_layer, AI_STATIC, 
  .axes = &_Slice_12_output_0_axes, 
  .starts = &_Slice_12_output_0_starts, 
  .ends = &_Slice_12_output_0_ends, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  state_history_Transpose_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &state_history_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &state_history_Transpose_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  state_history_Transpose_layer, 2,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &state_history_Transpose_chain,
  NULL, &_Slice_12_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_WIDTH, AI_SHAPE_CHANNEL, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 709036, 1, 1),
    709036, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 110748, 1, 1),
    110748, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_ACT_POLICY_IN_NUM, &tof_history_output, &state_history_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_ACT_POLICY_OUT_NUM, &action_q_Transpose_0_output),
  &state_history_Transpose_layer, 0xec07e996, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 709036, 1, 1),
      709036, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 110748, 1, 1),
      110748, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_ACT_POLICY_IN_NUM, &tof_history_output, &state_history_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_ACT_POLICY_OUT_NUM, &action_q_Transpose_0_output),
  &state_history_Transpose_layer, 0xec07e996, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool act_policy_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_act_policy_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    tof_history_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 18444);
    tof_history_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 18444);
    state_history_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 18300);
    state_history_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 18300);
    state_history_Transpose_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 73740);
    state_history_Transpose_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 73740);
    _Slice_12_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 18432);
    _Slice_12_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 18432);
    _Slice_11_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_11_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_22_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 73884);
    _Reshape_22_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 73884);
    _Slice_10_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_10_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_20_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 78492);
    _Reshape_20_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 78492);
    _Slice_9_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_9_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_18_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 83100);
    _Reshape_18_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 83100);
    _Slice_8_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_8_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_16_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 87708);
    _Reshape_16_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 87708);
    _Slice_7_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_7_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_14_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 92316);
    _Reshape_14_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 92316);
    _Slice_6_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_6_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_12_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 96924);
    _Reshape_12_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 96924);
    _Slice_5_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_5_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_10_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 101532);
    _Reshape_10_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 101532);
    _Slice_4_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_4_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_8_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _Reshape_8_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _Slice_3_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_3_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_6_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _Reshape_6_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _Slice_2_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_2_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_4_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Reshape_4_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Slice_1_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_1_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_2_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 106140);
    _Reshape_2_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 106140);
    _Slice_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Slice_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Reshape_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 18444);
    _Reshape_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 18444);
    _Sub_12_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Sub_12_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13824);
    _Sub_11_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_11_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_11_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _Div_11_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _tof_encoder_network_network_0_11_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_0_11_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_0_11_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_0_11_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_1_11_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_11_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_2_11_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_2_11_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_2_11_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15120);
    _tof_encoder_network_network_2_11_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15120);
    _tof_encoder_network_network_3_11_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_3_11_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_11_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_4_11_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_4_11_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15696);
    _tof_encoder_network_network_4_11_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15696);
    _tof_encoder_network_network_5_11_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_5_11_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 14992);
    _tof_encoder_network_network_6_11_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 14992);
    _Sub_10_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_10_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_10_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _Div_10_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _tof_encoder_network_network_0_10_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_0_10_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_0_10_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_0_10_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_1_10_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_10_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_2_10_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_10_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_10_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_10_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_3_10_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_3_10_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_4_10_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_10_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_10_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_4_10_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_5_10_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_5_10_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _tof_encoder_network_network_6_10_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 13968);
    _Sub_9_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_9_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_9_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _Div_9_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _tof_encoder_network_network_0_9_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 14224);
    _tof_encoder_network_network_0_9_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 14224);
    _tof_encoder_network_network_0_9_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_0_9_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_1_9_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_9_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_2_9_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_9_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_9_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_9_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_3_9_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_3_9_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_4_9_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_9_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_9_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 24780);
    _tof_encoder_network_network_4_9_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 24780);
    _tof_encoder_network_network_5_9_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_5_9_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 14224);
    _tof_encoder_network_network_6_9_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 14224);
    _Sub_8_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_8_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_8_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _Div_8_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _tof_encoder_network_network_0_8_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_0_8_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_0_8_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_0_8_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_1_8_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_8_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_2_8_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_8_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_8_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_8_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_3_8_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_3_8_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_4_8_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_8_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_8_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 24780);
    _tof_encoder_network_network_4_8_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 24780);
    _tof_encoder_network_network_5_8_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_5_8_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 14480);
    _tof_encoder_network_network_6_8_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 14480);
    _Sub_7_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_7_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_7_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _Div_7_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _tof_encoder_network_network_0_7_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_0_7_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_0_7_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_0_7_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_1_7_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_7_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_2_7_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_7_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_7_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_7_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_3_7_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_3_7_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_4_7_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_7_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_7_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 24780);
    _tof_encoder_network_network_4_7_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 24780);
    _tof_encoder_network_network_5_7_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_5_7_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 14736);
    _tof_encoder_network_network_6_7_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 14736);
    _Sub_6_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_6_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_6_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _Div_6_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _tof_encoder_network_network_0_6_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_0_6_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_0_6_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_0_6_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_1_6_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_6_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_2_6_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_6_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_2_6_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_6_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_3_6_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_3_6_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_4_6_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_6_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_6_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 24780);
    _tof_encoder_network_network_4_6_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 24780);
    _tof_encoder_network_network_5_6_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_5_6_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 16272);
    _tof_encoder_network_network_6_6_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 16272);
    _Sub_5_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_5_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_5_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _Div_5_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _tof_encoder_network_network_0_5_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_0_5_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_0_5_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_0_5_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_1_5_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_5_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_2_5_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 16528);
    _tof_encoder_network_network_2_5_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 16528);
    _tof_encoder_network_network_2_5_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_5_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_3_5_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_3_5_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_5_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 16528);
    _tof_encoder_network_network_4_5_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 16528);
    _tof_encoder_network_network_4_5_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_4_5_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_5_5_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 16528);
    _tof_encoder_network_network_5_5_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 16528);
    _tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _tof_encoder_network_network_6_5_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15248);
    _Sub_4_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_4_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_4_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _Div_4_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_0_4_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 15504);
    _tof_encoder_network_network_0_4_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 15504);
    _tof_encoder_network_network_0_4_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_0_4_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_4_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_1_4_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_4_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_2_4_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_2_4_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 10368);
    _tof_encoder_network_network_2_4_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 10368);
    _tof_encoder_network_network_3_4_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_3_4_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_4_4_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_4_4_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_4_4_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 10944);
    _tof_encoder_network_network_4_4_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 10944);
    _tof_encoder_network_network_5_4_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_5_4_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 10240);
    _tof_encoder_network_network_6_4_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 10240);
    _Sub_3_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_3_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_3_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _Div_3_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_0_3_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_0_3_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 9216);
    _tof_encoder_network_network_0_3_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_0_3_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_3_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_1_3_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_3_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_2_3_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_2_3_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 5760);
    _tof_encoder_network_network_2_3_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 5760);
    _tof_encoder_network_network_3_3_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 10496);
    _tof_encoder_network_network_3_3_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 10496);
    _tof_encoder_network_network_4_3_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_4_3_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_4_3_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 6336);
    _tof_encoder_network_network_4_3_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 6336);
    _tof_encoder_network_network_5_3_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_5_3_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 5632);
    _tof_encoder_network_network_6_3_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 5632);
    _Sub_2_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_2_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_2_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Div_2_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_0_2_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_0_2_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4608);
    _tof_encoder_network_network_0_2_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_0_2_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_2_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_1_2_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_2_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_2_2_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_2_2_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 1152);
    _tof_encoder_network_network_2_2_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1152);
    _tof_encoder_network_network_3_2_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 5888);
    _tof_encoder_network_network_3_2_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 5888);
    _tof_encoder_network_network_4_2_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_4_2_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_4_2_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 1728);
    _tof_encoder_network_network_4_2_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1728);
    _tof_encoder_network_network_5_2_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_5_2_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 1024);
    _tof_encoder_network_network_6_2_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1024);
    _Sub_1_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_1_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_1_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _Div_1_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 27660);
    _tof_encoder_network_network_0_1_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_0_1_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_0_1_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_0_1_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 32268);
    _tof_encoder_network_network_1_1_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_1_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_2_1_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_2_1_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_2_1_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 2432);
    _tof_encoder_network_network_2_1_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 2432);
    _tof_encoder_network_network_3_1_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 5888);
    _tof_encoder_network_network_3_1_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 5888);
    _tof_encoder_network_network_4_1_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_4_1_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_4_1_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_4_1_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_5_1_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_5_1_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _tof_encoder_network_network_6_1_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Sub_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Sub_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _Div_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 18444);
    _Div_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 18444);
    _tof_encoder_network_network_0_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 256);
    _tof_encoder_network_network_0_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 256);
    _tof_encoder_network_network_0_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_0_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 23052);
    _tof_encoder_network_network_1_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_1_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 31244);
    _tof_encoder_network_network_2_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_2_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_2_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 2432);
    _tof_encoder_network_network_2_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 2432);
    _tof_encoder_network_network_3_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 5888);
    _tof_encoder_network_network_3_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 5888);
    _tof_encoder_network_network_4_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_4_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_4_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 3008);
    _tof_encoder_network_network_4_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 3008);
    _tof_encoder_network_network_5_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_5_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _tof_encoder_network_network_6_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 256);
    _tof_encoder_network_network_6_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 256);
    _Concat_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _Concat_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 1280);
    _Div_12_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Div_12_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 144);
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_in_transpose_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 144);
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 288);
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 288);
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_out_transpose_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _state_encoder_state_encoder_0_Add_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _state_encoder_state_encoder_0_Add_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _state_encoder_state_encoder_1_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _state_encoder_state_encoder_1_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_in_transpose_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_out_transpose_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _state_encoder_state_encoder_2_Add_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _state_encoder_state_encoder_2_Add_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _state_encoder_state_encoder_3_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _state_encoder_state_encoder_3_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Concat_1_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _Concat_1_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4352);
    _Transpose_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Transpose_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _temporal_encoder_temporal_encoder_0_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 3840);
    _temporal_encoder_temporal_encoder_0_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 3840);
    _temporal_encoder_temporal_encoder_0_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 4800);
    _temporal_encoder_temporal_encoder_0_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 4800);
    _temporal_encoder_temporal_encoder_1_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _temporal_encoder_temporal_encoder_1_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _temporal_encoder_temporal_encoder_2_Conv_output_0_scratch0_array.data = AI_PTR(g_act_policy_activations_map[0] + 3072);
    _temporal_encoder_temporal_encoder_2_Conv_output_0_scratch0_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 3072);
    _temporal_encoder_temporal_encoder_2_Conv_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 3840);
    _temporal_encoder_temporal_encoder_2_Conv_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 3840);
    _temporal_encoder_temporal_encoder_3_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _temporal_encoder_temporal_encoder_3_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Flatten_output_0_to_chlast_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 3072);
    _Flatten_output_0_to_chlast_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 3072);
    _action_head_action_head_0_Gemm_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _action_head_action_head_0_Gemm_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _action_head_action_head_1_Relu_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 512);
    _action_head_action_head_1_Relu_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 512);
    _action_head_action_head_2_Gemm_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _action_head_action_head_2_Gemm_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Reshape_24_output_0_to_chfirst_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 120);
    _Reshape_24_output_0_to_chfirst_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 120);
    _Mul_output_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    _Mul_output_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    action_q_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 120);
    action_q_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 120);
    action_q_Transpose_0_output_array.data = AI_PTR(g_act_policy_activations_map[0] + 0);
    action_q_Transpose_0_output_array.data_start = AI_PTR(g_act_policy_activations_map[0] + 0);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool act_policy_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_act_policy_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    state_encoder_0_bias_3D_array.format |= AI_FMT_FLAG_CONST;
    state_encoder_0_bias_3D_array.data = AI_PTR(g_act_policy_weights_map[0] + 0);
    state_encoder_0_bias_3D_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 0);
    state_encoder_2_bias_3D_array.format |= AI_FMT_FLAG_CONST;
    state_encoder_2_bias_3D_array.data = AI_PTR(g_act_policy_weights_map[0] + 64);
    state_encoder_2_bias_3D_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 64);
    action_std_const_array.format |= AI_FMT_FLAG_CONST;
    action_std_const_array.data = AI_PTR(g_act_policy_weights_map[0] + 128);
    action_std_const_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 128);
    state_mean_array.format |= AI_FMT_FLAG_CONST;
    state_mean_array.data = AI_PTR(g_act_policy_weights_map[0] + 140);
    state_mean_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 140);
    tof_std_array.format |= AI_FMT_FLAG_CONST;
    tof_std_array.data = AI_PTR(g_act_policy_weights_map[0] + 152);
    tof_std_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 152);
    tof_mean_array.format |= AI_FMT_FLAG_CONST;
    tof_mean_array.data = AI_PTR(g_act_policy_weights_map[0] + 224);
    tof_mean_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 224);
    _tof_encoder_network_network_0_11_Conv_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _tof_encoder_network_network_0_11_Conv_output_0_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 296);
    _tof_encoder_network_network_0_11_Conv_output_0_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 296);
    _tof_encoder_network_network_0_11_Conv_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _tof_encoder_network_network_0_11_Conv_output_0_bias_array.data = AI_PTR(g_act_policy_weights_map[0] + 21032);
    _tof_encoder_network_network_0_11_Conv_output_0_bias_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 21032);
    _tof_encoder_network_network_2_11_Conv_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _tof_encoder_network_network_2_11_Conv_output_0_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 21160);
    _tof_encoder_network_network_2_11_Conv_output_0_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 21160);
    _tof_encoder_network_network_2_11_Conv_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _tof_encoder_network_network_2_11_Conv_output_0_bias_array.data = AI_PTR(g_act_policy_weights_map[0] + 76456);
    _tof_encoder_network_network_2_11_Conv_output_0_bias_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 76456);
    _tof_encoder_network_network_4_11_Conv_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _tof_encoder_network_network_4_11_Conv_output_0_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 76648);
    _tof_encoder_network_network_4_11_Conv_output_0_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 76648);
    _tof_encoder_network_network_4_11_Conv_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _tof_encoder_network_network_4_11_Conv_output_0_bias_array.data = AI_PTR(g_act_policy_weights_map[0] + 187240);
    _tof_encoder_network_network_4_11_Conv_output_0_bias_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 187240);
    state_std_array.format |= AI_FMT_FLAG_CONST;
    state_std_array.data = AI_PTR(g_act_policy_weights_map[0] + 187496);
    state_std_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 187496);
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 187508);
    _state_encoder_state_encoder_0_MatMul_output_0_gemm_to_dense_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 187508);
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 187700);
    _state_encoder_state_encoder_2_MatMul_output_0_gemm_to_dense_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 187700);
    _temporal_encoder_temporal_encoder_0_Conv_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _temporal_encoder_temporal_encoder_0_Conv_output_0_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 188724);
    _temporal_encoder_temporal_encoder_0_Conv_output_0_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 188724);
    _temporal_encoder_temporal_encoder_0_Conv_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _temporal_encoder_temporal_encoder_0_Conv_output_0_bias_array.data = AI_PTR(g_act_policy_weights_map[0] + 250164);
    _temporal_encoder_temporal_encoder_0_Conv_output_0_bias_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 250164);
    _temporal_encoder_temporal_encoder_2_Conv_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _temporal_encoder_temporal_encoder_2_Conv_output_0_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 250420);
    _temporal_encoder_temporal_encoder_2_Conv_output_0_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 250420);
    _temporal_encoder_temporal_encoder_2_Conv_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _temporal_encoder_temporal_encoder_2_Conv_output_0_bias_array.data = AI_PTR(g_act_policy_weights_map[0] + 299572);
    _temporal_encoder_temporal_encoder_2_Conv_output_0_bias_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 299572);
    _action_head_action_head_0_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _action_head_action_head_0_Gemm_output_0_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 299828);
    _action_head_action_head_0_Gemm_output_0_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 299828);
    _action_head_action_head_0_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _action_head_action_head_0_Gemm_output_0_bias_array.data = AI_PTR(g_act_policy_weights_map[0] + 693044);
    _action_head_action_head_0_Gemm_output_0_bias_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 693044);
    _action_head_action_head_2_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _action_head_action_head_2_Gemm_output_0_weights_array.data = AI_PTR(g_act_policy_weights_map[0] + 693556);
    _action_head_action_head_2_Gemm_output_0_weights_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 693556);
    _action_head_action_head_2_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _action_head_action_head_2_Gemm_output_0_bias_array.data = AI_PTR(g_act_policy_weights_map[0] + 708916);
    _action_head_action_head_2_Gemm_output_0_bias_array.data_start = AI_PTR(g_act_policy_weights_map[0] + 708916);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_act_policy_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_ACT_POLICY_MODEL_NAME,
      .model_signature   = AI_ACT_POLICY_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 8535572,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xec07e996,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_act_policy_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_ACT_POLICY_MODEL_NAME,
      .model_signature   = AI_ACT_POLICY_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 8535572,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xec07e996,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_act_policy_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_act_policy_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_act_policy_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_act_policy_create(network, AI_ACT_POLICY_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_act_policy_data_params_get(&params) != true) {
    err = ai_act_policy_get_error(*network);
    return err;
  }
#if defined(AI_ACT_POLICY_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_ACT_POLICY_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_act_policy_init(*network, &params) != true) {
    err = ai_act_policy_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_act_policy_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_act_policy_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_act_policy_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_act_policy_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= act_policy_configure_weights(net_ctx, params);
  ok &= act_policy_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_act_policy_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_act_policy_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_ACT_POLICY_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

