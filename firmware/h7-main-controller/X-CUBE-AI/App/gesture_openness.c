/**
  ******************************************************************************
  * @file    gesture_openness.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-08-11T22:45:51+0800
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


#include "gesture_openness.h"
#include "gesture_openness_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_gesture_openness
 
#undef AI_GESTURE_OPENNESS_MODEL_SIGNATURE
#define AI_GESTURE_OPENNESS_MODEL_SIGNATURE     "0x97b83df2c4de2117a9771dc7a7a6ad38"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2026-08-11T22:45:51+0800"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_GESTURE_OPENNESS_N_BATCHES
#define AI_GESTURE_OPENNESS_N_BATCHES         (1)

static ai_ptr g_gesture_openness_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_gesture_openness_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  features_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 1280, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  features_Transpose_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1280, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_0_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1536, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1536, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_2_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_4_Conv_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_5_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2048, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_6_GlobalAveragePool_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_8_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 24, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_9_Relu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 24, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_10_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  openness_norm_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 1, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_0_Conv_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4320, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_0_Conv_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 24, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_2_Conv_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6912, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_2_Conv_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_4_Conv_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 9216, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_4_Conv_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_8_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_8_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 24, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_10_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 24, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_10_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_0_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 180, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_2_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 216, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  _net_net_4_Conv_output_0_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 288, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_0_Conv_output_0_bias, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 24, 1, 1), AI_STRIDE_INIT(4, 4, 4, 96, 96),
  1, &_net_net_0_Conv_output_0_bias_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_0_Conv_output_0_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 24, 8, 8), AI_STRIDE_INIT(4, 4, 4, 96, 768),
  1, &_net_net_0_Conv_output_0_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_0_Conv_output_0_scratch0, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 3, 3), AI_STRIDE_INIT(4, 4, 4, 80, 240),
  1, &_net_net_0_Conv_output_0_scratch0_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_0_Conv_output_0_weights, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 20, 3, 3, 24), AI_STRIDE_INIT(4, 4, 80, 1920, 5760),
  1, &_net_net_0_Conv_output_0_weights_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_10_Gemm_output_0_bias, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_net_net_10_Gemm_output_0_bias_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_10_Gemm_output_0_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_net_net_10_Gemm_output_0_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_10_Gemm_output_0_weights, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 24, 1, 1, 1), AI_STRIDE_INIT(4, 4, 96, 96, 96),
  1, &_net_net_10_Gemm_output_0_weights_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_1_Relu_output_0_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 24, 8, 8), AI_STRIDE_INIT(4, 4, 4, 96, 768),
  1, &_net_net_1_Relu_output_0_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_2_Conv_output_0_bias, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_net_net_2_Conv_output_0_bias_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_2_Conv_output_0_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_net_net_2_Conv_output_0_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_2_Conv_output_0_scratch0, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 24, 3, 3), AI_STRIDE_INIT(4, 4, 4, 96, 288),
  1, &_net_net_2_Conv_output_0_scratch0_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_2_Conv_output_0_weights, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 24, 3, 3, 32), AI_STRIDE_INIT(4, 4, 96, 3072, 9216),
  1, &_net_net_2_Conv_output_0_weights_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_3_Relu_output_0_output, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_net_net_3_Relu_output_0_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_4_Conv_output_0_bias, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_net_net_4_Conv_output_0_bias_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_4_Conv_output_0_output, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_net_net_4_Conv_output_0_output_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_4_Conv_output_0_scratch0, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 3, 3), AI_STRIDE_INIT(4, 4, 4, 128, 384),
  1, &_net_net_4_Conv_output_0_scratch0_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_4_Conv_output_0_weights, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 32, 3, 3, 32), AI_STRIDE_INIT(4, 4, 128, 4096, 12288),
  1, &_net_net_4_Conv_output_0_weights_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_5_Relu_output_0_output, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 8, 8), AI_STRIDE_INIT(4, 4, 4, 128, 1024),
  1, &_net_net_5_Relu_output_0_output_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_6_GlobalAveragePool_output_0_output, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_net_net_6_GlobalAveragePool_output_0_output_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_8_Gemm_output_0_bias, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 24, 1, 1), AI_STRIDE_INIT(4, 4, 4, 96, 96),
  1, &_net_net_8_Gemm_output_0_bias_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_8_Gemm_output_0_output, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 24, 1, 1), AI_STRIDE_INIT(4, 4, 4, 96, 96),
  1, &_net_net_8_Gemm_output_0_output_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_8_Gemm_output_0_weights, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 32, 24, 1, 1), AI_STRIDE_INIT(4, 4, 128, 3072, 3072),
  1, &_net_net_8_Gemm_output_0_weights_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  _net_net_9_Relu_output_0_output, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 24, 1, 1), AI_STRIDE_INIT(4, 4, 4, 96, 96),
  1, &_net_net_9_Relu_output_0_output_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  features_Transpose_output, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 20, 8, 8), AI_STRIDE_INIT(4, 4, 4, 80, 640),
  1, &features_Transpose_output_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  features_output, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 8, 20), AI_STRIDE_INIT(4, 4, 4, 32, 256),
  1, &features_output_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  openness_norm_output, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &openness_norm_output_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  openness_norm_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_10_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &openness_norm_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  openness_norm_layer, 12,
  NL_TYPE, 0x0, NULL,
  nl, forward_sigmoid,
  &openness_norm_chain,
  NULL, &openness_norm_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_10_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_9_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_10_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_net_10_Gemm_output_0_weights, &_net_net_10_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_10_Gemm_output_0_layer, 11,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_net_net_10_Gemm_output_0_chain,
  NULL, &openness_norm_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_9_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_8_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_9_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_9_Relu_output_0_layer, 10,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_net_net_9_Relu_output_0_chain,
  NULL, &_net_net_10_Gemm_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_8_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_6_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_8_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_net_8_Gemm_output_0_weights, &_net_net_8_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_8_Gemm_output_0_layer, 9,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_net_net_8_Gemm_output_0_chain,
  NULL, &_net_net_9_Relu_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_6_GlobalAveragePool_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_6_GlobalAveragePool_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_6_GlobalAveragePool_output_0_layer, 7,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &_net_net_6_GlobalAveragePool_output_0_chain,
  NULL, &_net_net_8_Gemm_output_0_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(8, 8), 
  .pool_stride = AI_SHAPE_2D_INIT(8, 8), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_5_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_5_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_5_Relu_output_0_layer, 6,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_net_net_5_Relu_output_0_chain,
  NULL, &_net_net_6_GlobalAveragePool_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_4_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_4_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_net_net_4_Conv_output_0_weights, &_net_net_4_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_net_4_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _net_net_4_Conv_output_0_layer, 5,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_net_net_4_Conv_output_0_chain,
  NULL, &_net_net_5_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_3_Relu_output_0_layer, 4,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_net_net_3_Relu_output_0_chain,
  NULL, &_net_net_4_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_2_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_2_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_net_net_2_Conv_output_0_weights, &_net_net_2_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_net_2_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _net_net_2_Conv_output_0_layer, 3,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_net_net_2_Conv_output_0_chain,
  NULL, &_net_net_3_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_0_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _net_net_1_Relu_output_0_layer, 2,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &_net_net_1_Relu_output_0_chain,
  NULL, &_net_net_2_Conv_output_0_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_net_0_Conv_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &features_Transpose_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_net_0_Conv_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &_net_net_0_Conv_output_0_weights, &_net_net_0_Conv_output_0_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_net_0_Conv_output_0_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  _net_net_0_Conv_output_0_layer, 1,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &_net_net_0_Conv_output_0_chain,
  NULL, &_net_net_1_Relu_output_0_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 1, 1, 1), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  features_Transpose_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &features_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &features_Transpose_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  features_Transpose_layer, 2,
  TRANSPOSE_TYPE, 0x0, NULL,
  transpose, forward_transpose,
  &features_Transpose_chain,
  NULL, &_net_net_0_Conv_output_0_layer, AI_STATIC, 
  .out_mapping = AI_SHAPE_INIT(6, AI_SHAPE_IN_CHANNEL, AI_SHAPE_HEIGHT, AI_SHAPE_CHANNEL, AI_SHAPE_WIDTH, AI_SHAPE_DEPTH, AI_SHAPE_EXTENSION), 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 85412, 1, 1),
    85412, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 13312, 1, 1),
    13312, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_GESTURE_OPENNESS_IN_NUM, &features_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_GESTURE_OPENNESS_OUT_NUM, &openness_norm_output),
  &features_Transpose_layer, 0x30d0d3cc, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 85412, 1, 1),
      85412, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 13312, 1, 1),
      13312, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_GESTURE_OPENNESS_IN_NUM, &features_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_GESTURE_OPENNESS_OUT_NUM, &openness_norm_output),
  &features_Transpose_layer, 0x30d0d3cc, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool gesture_openness_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_gesture_openness_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    features_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 2048);
    features_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 2048);
    features_Transpose_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 7168);
    features_Transpose_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 7168);
    _net_net_0_Conv_output_0_scratch0_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 12288);
    _net_net_0_Conv_output_0_scratch0_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 12288);
    _net_net_0_Conv_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 1024);
    _net_net_0_Conv_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 1024);
    _net_net_1_Relu_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 7168);
    _net_net_1_Relu_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 7168);
    _net_net_2_Conv_output_0_scratch0_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 1024);
    _net_net_2_Conv_output_0_scratch0_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 1024);
    _net_net_2_Conv_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 2432);
    _net_net_2_Conv_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 2432);
    _net_net_3_Relu_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 2432);
    _net_net_3_Relu_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 2432);
    _net_net_4_Conv_output_0_scratch0_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 12160);
    _net_net_4_Conv_output_0_scratch0_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 12160);
    _net_net_4_Conv_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 0);
    _net_net_4_Conv_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 0);
    _net_net_5_Relu_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 0);
    _net_net_5_Relu_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 0);
    _net_net_6_GlobalAveragePool_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 8192);
    _net_net_6_GlobalAveragePool_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 8192);
    _net_net_8_Gemm_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 0);
    _net_net_8_Gemm_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 0);
    _net_net_9_Relu_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 96);
    _net_net_9_Relu_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 96);
    _net_net_10_Gemm_output_0_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 0);
    _net_net_10_Gemm_output_0_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 0);
    openness_norm_output_array.data = AI_PTR(g_gesture_openness_activations_map[0] + 4);
    openness_norm_output_array.data_start = AI_PTR(g_gesture_openness_activations_map[0] + 4);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool gesture_openness_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_gesture_openness_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    _net_net_0_Conv_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_net_0_Conv_output_0_weights_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 0);
    _net_net_0_Conv_output_0_weights_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 0);
    _net_net_0_Conv_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_net_0_Conv_output_0_bias_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 17280);
    _net_net_0_Conv_output_0_bias_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 17280);
    _net_net_2_Conv_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_net_2_Conv_output_0_weights_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 17376);
    _net_net_2_Conv_output_0_weights_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 17376);
    _net_net_2_Conv_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_net_2_Conv_output_0_bias_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 45024);
    _net_net_2_Conv_output_0_bias_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 45024);
    _net_net_4_Conv_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_net_4_Conv_output_0_weights_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 45152);
    _net_net_4_Conv_output_0_weights_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 45152);
    _net_net_4_Conv_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_net_4_Conv_output_0_bias_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 82016);
    _net_net_4_Conv_output_0_bias_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 82016);
    _net_net_8_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_net_8_Gemm_output_0_weights_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 82144);
    _net_net_8_Gemm_output_0_weights_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 82144);
    _net_net_8_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_net_8_Gemm_output_0_bias_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 85216);
    _net_net_8_Gemm_output_0_bias_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 85216);
    _net_net_10_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_net_10_Gemm_output_0_weights_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 85312);
    _net_net_10_Gemm_output_0_weights_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 85312);
    _net_net_10_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_net_10_Gemm_output_0_bias_array.data = AI_PTR(g_gesture_openness_weights_map[0] + 85408);
    _net_net_10_Gemm_output_0_bias_array.data_start = AI_PTR(g_gesture_openness_weights_map[0] + 85408);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_gesture_openness_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_GESTURE_OPENNESS_MODEL_NAME,
      .model_signature   = AI_GESTURE_OPENNESS_MODEL_SIGNATURE,
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
      
      .n_macc            = 1317931,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x30d0d3cc,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_gesture_openness_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_GESTURE_OPENNESS_MODEL_NAME,
      .model_signature   = AI_GESTURE_OPENNESS_MODEL_SIGNATURE,
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
      
      .n_macc            = 1317931,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x30d0d3cc,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_gesture_openness_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_gesture_openness_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_gesture_openness_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_gesture_openness_create(network, AI_GESTURE_OPENNESS_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_gesture_openness_data_params_get(&params) != true) {
    err = ai_gesture_openness_get_error(*network);
    return err;
  }
#if defined(AI_GESTURE_OPENNESS_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_GESTURE_OPENNESS_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_gesture_openness_init(*network, &params) != true) {
    err = ai_gesture_openness_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_gesture_openness_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_gesture_openness_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_gesture_openness_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_gesture_openness_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= gesture_openness_configure_weights(net_ctx, params);
  ok &= gesture_openness_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_gesture_openness_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_gesture_openness_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_GESTURE_OPENNESS_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

