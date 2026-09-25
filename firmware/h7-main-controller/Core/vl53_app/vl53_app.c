#include "vl53_app.h"
#include "vl53lmz_api.h"
#include "vl53lmz_plugin_cnh.h"
#include "app_x-cube-ai.h"
#include "csv_logger.h"
#include "gesture_cnn_norm_params.h"
#include "main.h"
#include "usart.h"
#include "vofa.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define VL53_APP_RESOLUTION             VL53LMZ_RESOLUTION_8X8
/* ACT单次推理约100ms，10Hz可减少主循环持续追赶新帧造成的跳帧。 */
#define VL53_APP_RANGING_FREQ_HZ        10U
#define VL53_APP_INTEGRATION_TIME_MS    20U
#define VL53_APP_READY_POLL_PERIOD_MS   1U  /* data_ready最多每1ms查询一次，避免持续占用I2C。 */
#define VL53_APP_LOG_PERIOD_MS          200U
#define VL53_APP_FRAME_LOG_ENABLE       0U
#define VL53_APP_BOOT_INIT_DELAY_MS     1000U
#define VL53_APP_INIT_RETRY_PERIOD_MS   1000U

#define VL53_APP_CNH_START_BIN          3
#define VL53_APP_CNH_SENSOR_BINS        11U
#define VL53_APP_CNH_GESTURE_OFFSET     3U  /* 仍使用原物理bin 6~13，兼容旧手势模型。 */
#define VL53_APP_CNH_GESTURE_BINS       8U
#define VL53_APP_CNH_SUB_SAMPLE         1

/* ARMCC V5不稳定支持源码中的UTF-8字符串，运行输出使用明确字节序列。 */
#define VL53_APP_TEXT_MAPPING_VALID                                      \
  "\xE5\xBD\x93\xE5\x89\x8D\xE6\x98\xA0\xE5\xB0\x84"             \
  "X\xEF\xBC\x9A%d Y\xEF\xBC\x9A%d "                               \
  "\xE8\xA7\x92\xE5\xBA\xA6\xEF\xBC\x9A%ddeg "                   \
  "\xE8\xB7\x9D\xE7\xA6\xBB\xEF\xBC\x9A%umm "                    \
  "\xE6\x89\x8B\xE5\xBC\xAF\xE6\x9B\xB2\xE7\xA8\x8B\xE5\xBA\xA6" \
  "\xEF\xBC\x9A%u raw=%u\r\n"
#define VL53_APP_TEXT_MAPPING_INVALID                                    \
  "\xE5\xBD\x93\xE5\x89\x8D\xE6\x98\xA0\xE5\xB0\x84"             \
  "\xEF\xBC\x9A\xE6\x97\xA0\xE6\x95\x88 st=%u\r\n"
#define VL53_APP_CNH_CENTER_ZONE        27
#define VL53_APP_SAMPLE_DELAY_MS        500U
#define VL53_APP_CSV_FILE               "0:/gesture.csv"
#define VL53_APP_AI_VALID_MIN_MM        160
#define VL53_APP_AI_VALID_MAX_MM        450
#define VL53_APP_AI_FOREGROUND_MARGIN   120
#define VL53_APP_AI_REL_SCALE_MM        200.0f
#define VL53_APP_AI_Z_CENTER_MM         300.0f
#define VL53_APP_AI_Z_SCALE_MM          150.0f
#define VL53_APP_AI_NB_TARGET_SCALE     3.0f
#define VL53_APP_AI_CNH_EPS             0.000001f
#define VL53_APP_AI_MIN_HAND_ZONES      3U
#define VL53_APP_AI_LOG_PERIOD_MS       100U
#define VL53_APP_AI_Z_NEAR_MM           220U
#define VL53_APP_AI_Z_FAR_MM            400U
#define VL53_APP_AI_FILTER_ALPHA        0.25f
#define VL53_APP_AI_STATUS_OK           0U
#define VL53_APP_RAD_TO_DEG             57.2957795f
#define VL53_APP_AI_STATUS_CNH_INVALID  1U
#define VL53_APP_AI_STATUS_VALID_FEW    2U
#define VL53_APP_AI_STATUS_HAND_FEW     3U
#define VL53_APP_AI_STATUS_CNH_ADDR     4U
#define VL53_APP_AI_STATUS_RUN_FAIL     20U
#define VL53_APP_GRID_SIZE              8U
#define VL53_APP_CNN_CHANNELS           20U
#define VL53_APP_ROI_TIP_IS_TOP         1U
#define VL53_APP_ROI_FOREGROUND_MARGIN  120
#define VL53_APP_ROI_LOG_PERIOD_MS      0U
#define VL53_APP_ROI_PALM_WIDTH_MM      150.0f
#define VL53_APP_ROI_PALM_HEIGHT_MM     170.0f
#define VL53_APP_ROI_TAN_HALF_FOV       0.41421356f
#define VL53_APP_ROI_MIN_SIZE           2U
#define VL53_APP_ROI_STATUS_OK          0U
#define VL53_APP_ROI_STATUS_VALID_FEW   1U
#define VL53_APP_ROI_STATUS_FG_FEW      2U
#define VL53_APP_ROI_STATUS_TIP_LOST    3U
#define VL53_APP_ROI_STATUS_ROI_FEW     4U
#define VL53_APP_CSV_COLUMNS            (5U + \
                                          VL53_APP_MAX_ZONES + \
                                          VL53_APP_MAX_ZONES + \
                                          VL53_APP_MAX_ZONES + \
                                          (VL53_APP_MAX_ZONES * VL53_APP_CNH_GESTURE_BINS))

#if (GESTURE_CNN_NORM_CHANNELS != VL53_APP_CNN_CHANNELS)
#error "CNN归一化通道数必须和VL53_APP_CNN_CHANNELS一致"
#endif

typedef struct
{
  uint8_t valid;                         /* ROI是否可信 */
  uint8_t status;                        /* 0成功，非0表示ROI提取失败原因 */
  uint8_t x_min;                         /* ROI左边界，0~7 */
  uint8_t x_max;                         /* ROI右边界，0~7 */
  uint8_t y_min;                         /* ROI上边界，0~7 */
  uint8_t y_max;                         /* ROI下边界，0~7 */
  uint8_t width;                         /* ROI宽度，单位zone */
  uint8_t height;                        /* ROI高度，单位zone */
  uint8_t zone_count;                    /* ROI内前景zone数量 */
  uint8_t tip_x;                         /* 手部前端中心X，0~7 */
  uint8_t tip_y;                         /* 手部前端Y，0~7 */
  uint16_t z_ref_mm;                     /* ROI尺度估计参考距离，单位mm */
} VL53_App_RoiDebug_t;

static VL53LMZ_Configuration g_vl53_config;
static VL53LMZ_ResultsData g_vl53_results;
static VL53LMZ_Motion_Configuration g_vl53_cnh_config;
static cnh_data_buffer_t g_vl53_cnh_buffer;
static uint32_t g_vl53_cnh_data_size;
static uint8_t g_vl53_cnh_valid;
static VL53_App_Mode_t g_vl53_mode;
static uint8_t g_vl53_sample_pending;
static uint8_t g_vl53_sample_result_pending;
static uint8_t g_vl53_last_sample_status;
static uint32_t g_vl53_sample_request_ms;
static uint8_t g_vl53_label_score;
static VL53_App_Frame_t g_vl53_frame;
static VL53_App_ActFrame_t g_vl53_act_frame;
static VL53_App_AiEstimate_t g_vl53_ai;
static VL53_App_RuntimeStats_t g_vl53_runtime_stats;
static uint32_t g_vl53_stats_last_frame_ms;
static uint8_t g_vl53_stats_last_stream;
static uint8_t g_vl53_stats_stream_valid;
static uint32_t g_vl53_stats_last_process_end_ms;
static uint32_t g_vl53_stats_ready_poll_count;
static uint32_t g_vl53_last_ready_poll_ms;
static float g_vl53_ai_input[GESTURE_AI_INPUT_SIZE];
static float g_vl53_ai_openness_filtered;
static uint8_t g_vl53_ai_filter_ready;
static VL53_App_RoiDebug_t g_vl53_roi;
/* 默认关闭手势推理，由UART7的GESTURE命令显式打开。 */
static uint8_t g_vl53_gesture_infer_enable = 0U;
static uint8_t g_vl53_ready;
static uint8_t g_vl53_new_frame;
static uint32_t g_vl53_next_init_retry_ms;
static uint8_t g_vl53_init_retry_count;
static uint32_t g_vl53_last_ai_log_ms;
static uint8_t g_vl53_ai_log_enable = 1U;
#if (VL53_APP_ROI_LOG_PERIOD_MS != 0U)
static uint32_t g_vl53_last_roi_log_ms;
#endif
#if (VL53_APP_FRAME_LOG_ENABLE != 0U)
static uint32_t g_vl53_last_log_ms;
#endif

/**
  * @brief 通过 USART1 输出一条简短调试信息。
  * @param text 以 '\0' 结尾的字符串。
  */
static void VL53_App_Log(const char *text)
{
  (void)Vofa_Write(text);
}

/**
  * @brief 输出 VL53 API 某一步失败的信息，便于上板定位初始化或测距问题。
  * @param step 失败步骤名称。
  * @param status VL53 ULD API 返回状态。
  */
static void VL53_App_LogStepError(const char *step, uint8_t status)
{
  char log_buffer[80];

  snprintf(log_buffer, sizeof(log_buffer), "VL53: %s failed, status=%u\r\n", step, status);
  VL53_App_Log(log_buffer);
}

/**
  * @brief  标记VL53初始化失败，并安排下一次自动重试。
  * @param  status 当前失败状态，0会被转成1，避免“失败但返回成功”。
  * @return 非0失败状态。
  */
static uint8_t VL53_App_MarkInitFailed(uint8_t status)
{
  g_vl53_next_init_retry_ms = HAL_GetTick() + VL53_APP_INIT_RETRY_PERIOD_MS;
  return (status != 0U) ? status : 1U;
}

/**
  * @brief  VL53未ready时按上电延时和重试周期重新初始化。
  * @param  无。
  *
  * 说明：用于处理传感器上电慢于MCU的情况；成功后恢复正常测距任务。
  */
static void VL53_App_TryInitRetry(void)
{
  uint32_t now_ms = HAL_GetTick();
  char log_buffer[80];

  if (g_vl53_next_init_retry_ms == 0U)
  {
    g_vl53_next_init_retry_ms = now_ms + VL53_APP_BOOT_INIT_DELAY_MS;
    return;
  }

  if ((int32_t)(now_ms - g_vl53_next_init_retry_ms) < 0)
  {
    return;
  }

  g_vl53_init_retry_count++;
  snprintf(log_buffer,
           sizeof(log_buffer),
           "VL53: init retry %u\r\n",
           (unsigned int)g_vl53_init_retry_count);
  VL53_App_Log(log_buffer);
  (void)VL53_App_Init();
}

/**
  * @brief 判断某个zone是否可作为AI前处理的有效测距点。
  * @param zone 8x8区域编号，范围0~63。
  * @return 1有效，0无效。
  */
static uint8_t VL53_App_IsAiValidZone(uint8_t zone)
{
  int16_t distance = g_vl53_frame.distance_mm[zone];
  uint8_t status = g_vl53_frame.target_status[zone];

  return ((distance >= VL53_APP_AI_VALID_MIN_MM) &&
          (distance <= VL53_APP_AI_VALID_MAX_MM) &&
          ((status == 5U) || (status == 9U)) &&
          (g_vl53_frame.nb_target_detected[zone] > 0U)) ? 1U : 0U;
}

/**
  * @brief 对少量距离样本求中位数。
  * @param data 距离数组，会被原地排序。
  * @param count 样本数量。
  * @return 中位数距离，单位mm。
  */
static uint16_t VL53_App_MedianU16(uint16_t *data, uint8_t count)
{
  uint8_t i;

  for (i = 1U; i < count; i++)
  {
    uint16_t key = data[i];
    uint8_t j = i;

    while ((j > 0U) && (data[j - 1U] > key))
    {
      data[j] = data[j - 1U];
      j--;
    }
    data[j] = key;
  }

  if ((count & 1U) != 0U)
  {
    return data[count / 2U];
  }

  return (uint16_t)(((uint32_t)data[(count / 2U) - 1U] + data[count / 2U]) / 2U);
}

/**
  * @brief 把float限制到0~100并四舍五入为uint8。
  * @param value 输入值。
  * @return 0~100整数。
  */
static uint8_t VL53_App_ClampU8FromFloat(float value)
{
  if (!(value > 0.0f))
  {
    return 0U;
  }

  if (value >= 100.0f)
  {
    return 100U;
  }

  return (uint8_t)(value + 0.5f);
}

/**
  * @brief 根据当前X/Y控制量计算显示角度。
  * @param x X方向控制量，范围约-100~100。
  * @param y Y方向控制量，范围约-100~100。
  * @return 角度，单位deg，范围约-180~180。
  * @note  该角度只用于VOFA观察，不参与实际控制。
  */
static int16_t VL53_App_CalcAngleDegFromXY(int16_t x, int16_t y)
{
  float angle_deg;

  if ((x == 0) && (y == 0))
  {
    return 0;
  }

  angle_deg = atan2f((float)y, (float)x) * VL53_APP_RAD_TO_DEG;
  if (angle_deg >= 0.0f)
  {
    return (int16_t)(angle_deg + 0.5f);
  }

  return (int16_t)(angle_deg - 0.5f);
}

/**
  * @brief 根据参考距离估算手掌实际尺寸覆盖多少个8x8 zone。
  * @param size_mm 手掌宽度或高度，单位mm。
  * @param z_ref_mm 手面参考距离，单位mm。
  * @return 需要覆盖的zone数量，限制在2~8。
  *
  * 说明：VL53L8CH水平/垂直FoV按45度估算，单个zone宽度约为
  * 2 * z * tan(22.5度) / 8。这里避免引入ceilf，手动做向上取整。
  */
static uint8_t VL53_App_CalcRoiCells(float size_mm, uint16_t z_ref_mm)
{
  float cell_mm;
  uint8_t cells;

  if (z_ref_mm == 0U)
  {
    return VL53_APP_GRID_SIZE;
  }

  cell_mm = (2.0f * (float)z_ref_mm * VL53_APP_ROI_TAN_HALF_FOV) /
            (float)VL53_APP_GRID_SIZE;
  if (!(cell_mm > 0.0f))
  {
    return VL53_APP_GRID_SIZE;
  }

  cells = (uint8_t)(size_mm / cell_mm);
  if (((float)cells * cell_mm) < size_mm)
  {
    cells++;
  }

  if (cells < VL53_APP_ROI_MIN_SIZE)
  {
    cells = VL53_APP_ROI_MIN_SIZE;
  }
  else if (cells > VL53_APP_GRID_SIZE)
  {
    cells = VL53_APP_GRID_SIZE;
  }

  return cells;
}

/**
  * @brief 计算手部ROI调试信息，不参与当前AI输入。
  * @param 无。
  *
  * 说明：ROI先按有效zone中的最近前景提取，再从最上方手部前端向下估算手掌区域。
  * 如果实际安装方向相反，可把 VL53_APP_ROI_TIP_IS_TOP 改为0。
  */
static void VL53_App_UpdateRoiDebug(void)
{
  uint8_t valid_mask[VL53_APP_MAX_ZONES];
  uint8_t fg_mask[VL53_APP_MAX_ZONES];
  uint16_t z_samples[24];
  uint8_t valid_count = 0U;
  uint8_t fg_count = 0U;
  uint8_t sample_count = 0U;
  uint8_t zone;
  uint8_t x;
  uint8_t y;
  uint8_t row;
  uint8_t row_found = 0U;
  uint8_t tip_y = 0U;
  uint8_t tip_x_min = 0U;
  uint8_t tip_x_max = 0U;
  uint8_t tip_x;
  uint8_t roi_w;
  uint8_t roi_h;
  uint8_t roi_count = 0U;
  uint16_t z_ref;
  int16_t d_min = 32767;
  int16_t start;
  int8_t row_step;
  int8_t yy;
  uint8_t row_offset;

  memset(&g_vl53_roi, 0, sizeof(g_vl53_roi));
  memset(valid_mask, 0, sizeof(valid_mask));
  memset(fg_mask, 0, sizeof(fg_mask));

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (VL53_App_IsAiValidZone(zone) != 0U)
    {
      valid_mask[zone] = 1U;
      valid_count++;
      if (g_vl53_frame.distance_mm[zone] < d_min)
      {
        d_min = g_vl53_frame.distance_mm[zone];
      }
    }
  }

  if (valid_count < VL53_APP_AI_MIN_HAND_ZONES)
  {
    g_vl53_roi.status = VL53_APP_ROI_STATUS_VALID_FEW;
    return;
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if ((valid_mask[zone] != 0U) &&
        (g_vl53_frame.distance_mm[zone] <= (d_min + VL53_APP_ROI_FOREGROUND_MARGIN)))
    {
      fg_mask[zone] = 1U;
      fg_count++;
    }
  }

  if (fg_count < VL53_APP_AI_MIN_HAND_ZONES)
  {
    g_vl53_roi.status = VL53_APP_ROI_STATUS_FG_FEW;
    return;
  }

#if (VL53_APP_ROI_TIP_IS_TOP != 0U)
  for (y = 0U; y < VL53_APP_GRID_SIZE; y++)
#else
  for (y = VL53_APP_GRID_SIZE; y > 0U; y--)
#endif
  {
#if (VL53_APP_ROI_TIP_IS_TOP != 0U)
    row = y;
#else
    row = (uint8_t)(y - 1U);
#endif

    for (x = 0U; x < VL53_APP_GRID_SIZE; x++)
    {
      zone = (uint8_t)((row * VL53_APP_GRID_SIZE) + x);
      if (fg_mask[zone] != 0U)
      {
        if (row_found == 0U)
        {
          tip_x_min = x;
          tip_x_max = x;
          tip_y = row;
          row_found = 1U;
        }
        else
        {
          tip_x_max = x;
        }
      }
    }

    if (row_found != 0U)
    {
      break;
    }
  }

  if (row_found == 0U)
  {
    g_vl53_roi.status = VL53_APP_ROI_STATUS_TIP_LOST;
    return;
  }

  tip_x = (uint8_t)((tip_x_min + tip_x_max) / 2U);
  row_step = (VL53_APP_ROI_TIP_IS_TOP != 0U) ? 1 : -1;

  for (row_offset = 0U; row_offset < 3U; row_offset++)
  {
    yy = (int8_t)tip_y + ((int8_t)row_offset * row_step);
    if ((yy < 0) || (yy >= (int8_t)VL53_APP_GRID_SIZE))
    {
      continue;
    }

    for (x = 0U; x < VL53_APP_GRID_SIZE; x++)
    {
      zone = (uint8_t)(((uint8_t)yy * VL53_APP_GRID_SIZE) + x);
      if ((fg_mask[zone] != 0U) && (sample_count < (uint8_t)(sizeof(z_samples) / sizeof(z_samples[0]))))
      {
        z_samples[sample_count] = (uint16_t)g_vl53_frame.distance_mm[zone];
        sample_count++;
      }
    }
  }

  if (sample_count == 0U)
  {
    g_vl53_roi.status = VL53_APP_ROI_STATUS_TIP_LOST;
    return;
  }

  z_ref = VL53_App_MedianU16(z_samples, sample_count);
  roi_w = VL53_App_CalcRoiCells(VL53_APP_ROI_PALM_WIDTH_MM, z_ref);
  roi_h = VL53_App_CalcRoiCells(VL53_APP_ROI_PALM_HEIGHT_MM, z_ref);

  start = (int16_t)tip_x - (int16_t)(roi_w / 2U);
  if (start < 0)
  {
    start = 0;
  }
  else if ((start + roi_w) > VL53_APP_GRID_SIZE)
  {
    start = (int16_t)(VL53_APP_GRID_SIZE - roi_w);
  }
  g_vl53_roi.x_min = (uint8_t)start;
  g_vl53_roi.x_max = (uint8_t)(start + roi_w - 1U);

  if (VL53_APP_ROI_TIP_IS_TOP != 0U)
  {
    start = (int16_t)tip_y;
    if ((start + roi_h) > VL53_APP_GRID_SIZE)
    {
      start = (int16_t)(VL53_APP_GRID_SIZE - roi_h);
    }
  }
  else
  {
    start = (int16_t)tip_y - (int16_t)roi_h + 1;
    if (start < 0)
    {
      start = 0;
    }
  }
  g_vl53_roi.y_min = (uint8_t)start;
  g_vl53_roi.y_max = (uint8_t)(start + roi_h - 1U);

  for (y = g_vl53_roi.y_min; y <= g_vl53_roi.y_max; y++)
  {
    for (x = g_vl53_roi.x_min; x <= g_vl53_roi.x_max; x++)
    {
      zone = (uint8_t)((y * VL53_APP_GRID_SIZE) + x);
      if (fg_mask[zone] != 0U)
      {
        roi_count++;
      }
    }
  }

  g_vl53_roi.width = roi_w;
  g_vl53_roi.height = roi_h;
  g_vl53_roi.zone_count = roi_count;
  g_vl53_roi.tip_x = tip_x;
  g_vl53_roi.tip_y = tip_y;
  g_vl53_roi.z_ref_mm = z_ref;

  if (roi_count < VL53_APP_AI_MIN_HAND_ZONES)
  {
    g_vl53_roi.status = VL53_APP_ROI_STATUS_ROI_FEW;
    return;
  }

  g_vl53_roi.valid = 1U;
  g_vl53_roi.status = VL53_APP_ROI_STATUS_OK;
}

/**
  * @brief 低频输出ROI调试结果，用于确认手部区域是否框在手掌上。
  * @param 无。
  */
static void VL53_App_LogRoiDebugIfDue(void)
{
#if (VL53_APP_ROI_LOG_PERIOD_MS == 0U)
  return;
#else
  uint32_t now_ms = HAL_GetTick();
  char log_buffer[128];

  if ((now_ms - g_vl53_last_roi_log_ms) < VL53_APP_ROI_LOG_PERIOD_MS)
  {
    return;
  }
  g_vl53_last_roi_log_ms = now_ms;

  if (g_vl53_roi.valid != 0U)
  {
    snprintf(log_buffer,
             sizeof(log_buffer),
             "ROI v=1 st=0 x=%u-%u y=%u-%u w=%u h=%u z=%u zones=%u tip=%u,%u\r\n",
             (unsigned int)g_vl53_roi.x_min,
             (unsigned int)g_vl53_roi.x_max,
             (unsigned int)g_vl53_roi.y_min,
             (unsigned int)g_vl53_roi.y_max,
             (unsigned int)g_vl53_roi.width,
             (unsigned int)g_vl53_roi.height,
             (unsigned int)g_vl53_roi.z_ref_mm,
             (unsigned int)g_vl53_roi.zone_count,
             (unsigned int)g_vl53_roi.tip_x,
             (unsigned int)g_vl53_roi.tip_y);
  }
  else
  {
    snprintf(log_buffer,
             sizeof(log_buffer),
             "ROI v=0 st=%u z=%u zones=%u\r\n",
             (unsigned int)g_vl53_roi.status,
             (unsigned int)g_vl53_roi.z_ref_mm,
             (unsigned int)g_vl53_roi.zone_count);
  }

  VL53_App_Log(log_buffer);
#endif
}

/**
  * @brief 把float限制到-100~100并四舍五入为int16。
  * @param value 输入值。
  * @return -100~100整数。
  */
static int16_t VL53_App_ClampI16FromFloat(float value)
{
  if (value <= -100.0f)
  {
    return -100;
  }

  if (value >= 100.0f)
  {
    return 100;
  }

  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

/**
  * @brief 把float限制到指定范围。
  * @param value 输入值。
  * @param min_value 最小值。
  * @param max_value 最大值。
  * @return 限幅后的值。
  */
static float VL53_App_ClipFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

/**
  * @brief 写入CNN输入张量的一个zone特征。
  * @param channel CNN通道编号，0~19。
  * @param zone 8x8区域编号，0~63。
  * @param value 写入值。
  * @note 训练脚本使用NCHW格式，部署端按 channel * 64 + zone 连续存放。
  */
static void VL53_App_SetCnnFeature(uint8_t channel, uint8_t zone, float value)
{
  if ((channel < VL53_APP_CNN_CHANNELS) && (zone < VL53_APP_MAX_ZONES))
  {
    uint16_t index = (uint16_t)(((uint16_t)channel * VL53_APP_MAX_ZONES) + zone);
    g_vl53_ai_input[index] = value;
  }
}

/**
  * @brief 对20通道CNN输入做训练集均值方差归一化。
  * @param 无。
  */
static void VL53_App_NormalizeCnnInput(void)
{
  uint8_t channel;
  uint8_t zone;

  for (channel = 0U; channel < VL53_APP_CNN_CHANNELS; channel++)
  {
    float mean = GESTURE_CNN_NORM_MEAN[channel];
    float std = GESTURE_CNN_NORM_STD[channel];

    if (std < VL53_APP_AI_CNH_EPS)
    {
      std = 1.0f;
    }

    for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
    {
      uint16_t index = (uint16_t)(((uint16_t)channel * VL53_APP_MAX_ZONES) + zone);
      g_vl53_ai_input[index] = (g_vl53_ai_input[index] - mean) / std;
    }
  }
}

/**
  * @brief 根据手部距离计算刚度控制量。
  * @param center_z_mm 手部中心距离，单位mm。
  * @return 刚度0~100，近处更硬，远处更软。
  */
static uint8_t VL53_App_CalcStiffness(uint16_t center_z_mm)
{
  if (center_z_mm <= VL53_APP_AI_Z_NEAR_MM)
  {
    return 100U;
  }

  if (center_z_mm >= VL53_APP_AI_Z_FAR_MM)
  {
    return 0U;
  }

  return (uint8_t)(((uint32_t)(VL53_APP_AI_Z_FAR_MM - center_z_mm) * 100U) /
                   (VL53_APP_AI_Z_FAR_MM - VL53_APP_AI_Z_NEAR_MM));
}

/**
  * @brief 对AI张开程度做一阶低通滤波。
  * @param openness 当前帧AI输出，范围0~100。
  * @return 滤波后的张开程度。
  *
  * 说明：先只滤波张握程度，x/y/z保持原始估计，方便继续观察位置方向。
  */
static float VL53_App_FilterOpenness(float openness)
{
  if (g_vl53_ai_filter_ready == 0U)
  {
    g_vl53_ai_openness_filtered = openness;
    g_vl53_ai_filter_ready = 1U;
  }
  else
  {
    g_vl53_ai_openness_filtered =
        (g_vl53_ai_openness_filtered * (1.0f - VL53_APP_AI_FILTER_ALPHA)) +
        (openness * VL53_APP_AI_FILTER_ALPHA);
  }

  return g_vl53_ai_openness_filtered;
}

/**
  * @brief 从当前VL53帧构造CubeAI输入特征。
  * @param estimate 输出本帧的中心位置和状态。
  * @return 0成功，非0表示前处理失败。
  *
  * 说明：这里复刻训练脚本的前景相对距离和CNH归一化逻辑。
  */
static uint8_t VL53_App_BuildAiInput(VL53_App_AiEstimate_t *estimate)
{
  uint8_t valid_mask[VL53_APP_MAX_ZONES];
  uint8_t hand_mask[VL53_APP_MAX_ZONES];
  uint16_t hand_dist[VL53_APP_MAX_ZONES];
  uint8_t zone;
  uint8_t bin;
  uint8_t valid_count = 0U;
  uint8_t hand_count = 0U;
  uint8_t bbox_x_min = VL53_APP_GRID_SIZE;
  uint8_t bbox_x_max = 0U;
  uint8_t bbox_y_min = VL53_APP_GRID_SIZE;
  uint8_t bbox_y_max = 0U;
  uint8_t bbox_width;
  uint8_t bbox_height;
  int16_t d_min = 32767;
  uint16_t center_z;
  float weight_sum = 0.0f;
  float center_x_sum = 0.0f;
  float center_y_sum = 0.0f;
  float center_x;
  float center_y;
  float global_center_x;
  float global_center_y;
  float global_center_z;
  float cnh_scale = 0.0f;
  int32_t *p_hist = NULL;
  int8_t *p_hist_scaler = NULL;
  int32_t *p_ambient = NULL;
  int8_t *p_ambient_scaler = NULL;

  if ((estimate == NULL) || (g_vl53_cnh_valid == 0U))
  {
    if (estimate != NULL)
    {
      estimate->status = VL53_APP_AI_STATUS_CNH_INVALID;
    }
    return VL53_APP_AI_STATUS_CNH_INVALID;
  }

  memset(g_vl53_ai_input, 0, sizeof(g_vl53_ai_input));
  memset(valid_mask, 0, sizeof(valid_mask));
  memset(hand_mask, 0, sizeof(hand_mask));

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    uint8_t status_ok = ((g_vl53_frame.target_status[zone] == 5U) ||
                         (g_vl53_frame.target_status[zone] == 9U)) ? 1U : 0U;
    float nb_norm = (float)g_vl53_frame.nb_target_detected[zone] /
                    VL53_APP_AI_NB_TARGET_SCALE;

    VL53_App_SetCnnFeature(4U, zone, (float)status_ok);
    VL53_App_SetCnnFeature(5U, zone, VL53_App_ClipFloat(nb_norm, 0.0f, 1.0f));

    if (VL53_App_IsAiValidZone(zone) != 0U)
    {
      valid_mask[zone] = 1U;
      valid_count++;
      VL53_App_SetCnnFeature(2U, zone, 1.0f);
      if (g_vl53_frame.distance_mm[zone] < d_min)
      {
        d_min = g_vl53_frame.distance_mm[zone];
      }
    }
  }

  if (valid_count < VL53_APP_AI_MIN_HAND_ZONES)
  {
    estimate->status = VL53_APP_AI_STATUS_VALID_FEW;
    return estimate->status;
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    int16_t distance = g_vl53_frame.distance_mm[zone];

    if ((valid_mask[zone] != 0U) &&
        (distance <= (d_min + VL53_APP_AI_FOREGROUND_MARGIN)))
    {
      uint8_t x = zone % VL53_APP_GRID_SIZE;
      uint8_t y = zone / VL53_APP_GRID_SIZE;

      hand_mask[zone] = 1U;
      hand_dist[hand_count] = (uint16_t)distance;
      hand_count++;

      if (x < bbox_x_min)
      {
        bbox_x_min = x;
      }
      if (x > bbox_x_max)
      {
        bbox_x_max = x;
      }
      if (y < bbox_y_min)
      {
        bbox_y_min = y;
      }
      if (y > bbox_y_max)
      {
        bbox_y_max = y;
      }
    }
  }

  if (hand_count < VL53_APP_AI_MIN_HAND_ZONES)
  {
    estimate->status = VL53_APP_AI_STATUS_HAND_FEW;
    return estimate->status;
  }

  center_z = VL53_App_MedianU16(hand_dist, hand_count);
  estimate->center_z_mm = center_z;
  estimate->hand_zone_count = hand_count;

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (hand_mask[zone] != 0U)
    {
      float distance = (float)g_vl53_frame.distance_mm[zone];
      float weight = 1.0f / distance;

      weight_sum += weight;
      center_x_sum += (float)(zone % 8U) * weight;
      center_y_sum += (float)(zone / 8U) * weight;
    }
  }

  if (!(weight_sum > 0.0f))
  {
    estimate->status = VL53_APP_AI_STATUS_HAND_FEW;
    return estimate->status;
  }

  center_x = center_x_sum / weight_sum;
  center_y = center_y_sum / weight_sum;
  bbox_width = (uint8_t)(bbox_x_max - bbox_x_min + 1U);
  bbox_height = (uint8_t)(bbox_y_max - bbox_y_min + 1U);

  estimate->center_x_q10 = (uint16_t)(center_x * 10.0f + 0.5f);
  estimate->center_y_q10 = (uint16_t)(center_y * 10.0f + 0.5f);
  estimate->x = VL53_App_ClampI16FromFloat((center_x - 3.5f) * 100.0f / 3.5f);
  estimate->y = VL53_App_ClampI16FromFloat((center_y - 3.5f) * 100.0f / 3.5f);

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (hand_mask[zone] != 0U)
    {
      float distance = (float)g_vl53_frame.distance_mm[zone];
      float rel = (distance - (float)center_z) / VL53_APP_AI_REL_SCALE_MM;
      float abs_norm = (distance - (float)VL53_APP_AI_VALID_MIN_MM) /
                       (float)(VL53_APP_AI_VALID_MAX_MM - VL53_APP_AI_VALID_MIN_MM);

      VL53_App_SetCnnFeature(0U, zone, VL53_App_ClipFloat(rel, -3.0f, 3.0f));
      VL53_App_SetCnnFeature(1U, zone, VL53_App_ClipFloat(abs_norm, 0.0f, 1.0f));
      VL53_App_SetCnnFeature(3U, zone, 1.0f);

      if ((vl53lmz_cnh_get_block_addresses(&g_vl53_cnh_config,
                                           zone,
                                           g_vl53_cnh_buffer,
                                           &p_hist,
                                           &p_hist_scaler,
                                           &p_ambient,
                                           &p_ambient_scaler) != 0U) ||
          (p_hist == NULL))
      {
        estimate->status = VL53_APP_AI_STATUS_CNH_ADDR;
        return estimate->status;
      }

      for (bin = 0U; bin < VL53_APP_CNH_GESTURE_BINS; bin++)
      {
        float value = (float)p_hist[VL53_APP_CNH_GESTURE_OFFSET + bin];
        float abs_value = (value >= 0.0f) ? value : -value;

        VL53_App_SetCnnFeature((uint8_t)(6U + bin), zone, value);
        if (abs_value > cnh_scale)
        {
          cnh_scale = abs_value;
        }
      }
    }
  }

  if (cnh_scale < VL53_APP_AI_CNH_EPS)
  {
    cnh_scale = 1.0f;
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    for (bin = 0U; bin < VL53_APP_CNH_GESTURE_BINS; bin++)
    {
      uint16_t index = (uint16_t)(((uint16_t)(6U + bin) * VL53_APP_MAX_ZONES) + zone);
      g_vl53_ai_input[index] /= cnh_scale;
    }
  }

  global_center_x = VL53_App_ClipFloat((center_x - 3.5f) / 3.5f, -1.0f, 1.0f);
  global_center_y = VL53_App_ClipFloat((center_y - 3.5f) / 3.5f, -1.0f, 1.0f);
  global_center_z = VL53_App_ClipFloat(((float)center_z - VL53_APP_AI_Z_CENTER_MM) /
                                       VL53_APP_AI_Z_SCALE_MM,
                                       -2.0f,
                                       2.0f);

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    VL53_App_SetCnnFeature(14U, zone, global_center_x);
    VL53_App_SetCnnFeature(15U, zone, global_center_y);
    VL53_App_SetCnnFeature(16U, zone, global_center_z);
    VL53_App_SetCnnFeature(17U, zone, (float)hand_count / (float)VL53_APP_MAX_ZONES);
    VL53_App_SetCnnFeature(18U, zone, (float)bbox_width / (float)VL53_APP_GRID_SIZE);
    VL53_App_SetCnnFeature(19U, zone, (float)bbox_height / (float)VL53_APP_GRID_SIZE);
  }

  VL53_App_NormalizeCnnInput();

  estimate->status = VL53_APP_AI_STATUS_OK;
  return VL53_APP_AI_STATUS_OK;
}

/**
  * @brief 更新当前帧AI估计结果。
  * @param 无。
  */
static void VL53_App_UpdateAiEstimate(void)
{
  VL53_App_AiEstimate_t estimate;
  uint32_t start_ms = HAL_GetTick();
  uint32_t elapsed_ms;
  float openness = 0.0f;
  uint8_t openness_raw;

  memset(&estimate, 0, sizeof(estimate));
  estimate.frame_count = g_vl53_frame.frame_count;

  if (VL53_App_BuildAiInput(&estimate) != VL53_APP_AI_STATUS_OK)
  {
    g_vl53_ai_filter_ready = 0U;
    elapsed_ms = HAL_GetTick() - start_ms;
    estimate.ai_time_ms = (elapsed_ms > 255U) ? 255U : (uint8_t)elapsed_ms;
    g_vl53_ai = estimate;
    return;
  }

  if (GestureAI_Run(g_vl53_ai_input, &openness) != 0)
  {
    g_vl53_ai_filter_ready = 0U;
    estimate.status = VL53_APP_AI_STATUS_RUN_FAIL;
    elapsed_ms = HAL_GetTick() - start_ms;
    estimate.ai_time_ms = (elapsed_ms > 255U) ? 255U : (uint8_t)elapsed_ms;
    g_vl53_ai = estimate;
    return;
  }

  elapsed_ms = HAL_GetTick() - start_ms;
  estimate.valid = 1U;
  estimate.status = VL53_APP_AI_STATUS_OK;
  estimate.ai_time_ms = (elapsed_ms > 255U) ? 255U : (uint8_t)elapsed_ms;
  openness_raw = VL53_App_ClampU8FromFloat(openness);
  estimate.bend_raw_0_100 = 100U - openness_raw;
  estimate.openness_score = VL53_App_ClampU8FromFloat(VL53_App_FilterOpenness(openness));
  estimate.bend_0_100 = 100U - estimate.openness_score;
  estimate.stiffness_0_100 = VL53_App_CalcStiffness(estimate.center_z_mm);
  g_vl53_ai = estimate;
}

/**
  * @brief 低频输出AI估计结果，便于先验证数值是否合理。
  * @param 无。
  */
static void VL53_App_LogAiEstimateIfDue(void)
{
  uint32_t now_ms = HAL_GetTick();
  char log_buffer[128];

  if (g_vl53_ai_log_enable == 0U)
  {
    return;
  }

  if ((now_ms - g_vl53_last_ai_log_ms) < VL53_APP_AI_LOG_PERIOD_MS)
  {
    return;
  }
  g_vl53_last_ai_log_ms = now_ms;

  if (g_vl53_ai.valid != 0U)
  {
      snprintf(log_buffer,
               sizeof(log_buffer),
               VL53_APP_TEXT_MAPPING_VALID,
               (int)g_vl53_ai.x,
               (int)g_vl53_ai.y,
               (int)VL53_App_CalcAngleDegFromXY(g_vl53_ai.x, g_vl53_ai.y),
               (unsigned int)g_vl53_ai.center_z_mm,
              (unsigned int)g_vl53_ai.bend_0_100,
              (unsigned int)g_vl53_ai.bend_raw_0_100);
  }
  else
  {
    snprintf(log_buffer,
             sizeof(log_buffer),
             VL53_APP_TEXT_MAPPING_INVALID,
             (unsigned int)g_vl53_ai.status);
  }

  VL53_App_Log(log_buffer);
}

/**
  * @brief 向 CSV 缓冲区追加一段格式化文本。
  * @param cursor 当前写入位置。
  * @param remain 剩余可写空间。
  * @param fmt printf 风格格式串。
  * @return 0 成功，非 0 失败。
  */
static uint8_t VL53_App_Append(char **cursor, size_t *remain, const char *fmt, ...)
{
  int n;
  va_list args;

  va_start(args, fmt);
  n = vsnprintf(*cursor, *remain, fmt, args);
  va_end(args);

  if ((n < 0) || ((size_t)n >= *remain))
  {
    return 1U;
  }

  *cursor += n;
  *remain -= (size_t)n;
  return 0U;
}

/**
  * @brief 统计一行 CSV 的列数，同时拒绝空字段。
  * @param line 以 '\0' 结尾的 CSV 行。
  * @return 有效列数；返回 0 表示存在空字段或非法输入。
  */
static uint16_t VL53_App_CountCsvColumns(const char *line)
{
  uint16_t columns = 1U;
  char prev = '\0';

  if ((line == NULL) || (line[0] == '\0') || (line[0] == ','))
  {
    return 0U;
  }

  while ((*line != '\0') && (*line != '\r') && (*line != '\n'))
  {
    if (*line == ',')
    {
      if (prev == ',')
      {
        return 0U;
      }
      columns++;
    }

    prev = *line;
    line++;
  }

  return (prev == ',') ? 0U : columns;
}

/**
  * @brief 生成 CSV 表头，确保训练脚本可以稳定解析列含义。
  * @param buffer 表头缓冲区。
  * @param buffer_size 表头缓冲区大小。
  * @return 0 成功，非 0 失败。
  */
static uint8_t VL53_App_BuildCsvHeader(char *buffer, size_t buffer_size)
{
  char *cursor = buffer;
  size_t remain = buffer_size;
  uint8_t zone;
  uint8_t bin;

  if (VL53_App_Append(&cursor, &remain, "timestamp_ms,label_score,frame_count,mode,valid") != 0U)
  {
    return 1U;
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (VL53_App_Append(&cursor, &remain, ",d%u", (unsigned int)zone) != 0U)
    {
      return 2U;
    }
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (VL53_App_Append(&cursor, &remain, ",target_status_z%u", (unsigned int)zone) != 0U)
    {
      return 3U;
    }
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (VL53_App_Append(&cursor, &remain, ",nb_target_z%u", (unsigned int)zone) != 0U)
    {
      return 4U;
    }
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    for (bin = 0U; bin < VL53_APP_CNH_GESTURE_BINS; bin++)
    {
      if (VL53_App_Append(&cursor, &remain,
                          ",cnh_z%u_b%u",
                          (unsigned int)zone,
                          (unsigned int)bin) != 0U)
      {
        return 5U;
      }
    }
  }

  if (VL53_App_Append(&cursor, &remain, "\r\n") != 0U)
  {
    return 6U;
  }

  return (VL53_App_CountCsvColumns(buffer) == VL53_APP_CSV_COLUMNS) ? 0U : 7U;
}

/**
  * @brief 将 ST ULD 原始测距结果整理到应用层帧缓存中。
  * @param results ST ULD 输出的原始测距结果。
  *
  * 说明：当前使用 8x8 分辨率，后续采集和位置估计统一按 64 个 zone 处理。
  */
static void VL53_App_CopyResultsToFrame(const VL53LMZ_ResultsData *results)
{
  uint8_t zone;

  g_vl53_frame.valid = 1U;
  g_vl53_frame.resolution = VL53_APP_RESOLUTION;
  g_vl53_frame.frame_count++;
  g_vl53_frame.timestamp_ms = HAL_GetTick();

  for (zone = 0U; zone < VL53_APP_RESOLUTION; zone++)
  {
    g_vl53_frame.distance_mm[zone] = results->distance_mm[zone];
    g_vl53_frame.target_status[zone] = results->target_status[zone];
    g_vl53_frame.nb_target_detected[zone] = results->nb_target_detected[zone];
  }

  for (; zone < VL53_APP_MAX_ZONES; zone++)
  {
    g_vl53_frame.distance_mm[zone] = 0;
    g_vl53_frame.target_status[zone] = 0U;
    g_vl53_frame.nb_target_detected[zone] = 0U;
  }

  g_vl53_new_frame = 1U;
}

/**
  * @brief 复制ACT训练需要的完整VL53帧。
  * @param results ST ULD返回的本次测距结果。
  * @return 0表示完整复制成功，非0表示CNH地址获取失败。
  * @note frame_seq最后更新，调用方不会读取到只写了一半的新帧。
  */
static uint8_t VL53_App_CopyResultsToActFrame(
  const VL53LMZ_ResultsData *results)
{
  uint8_t zone;
  uint8_t bin;
  int32_t *p_hist = NULL;
  int8_t *p_hist_scaler = NULL;
  int32_t *p_ambient = NULL;
  int8_t *p_ambient_scaler = NULL;

  if (results == NULL)
  {
    return 1U;
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    g_vl53_act_frame.distance_mm[zone] = results->distance_mm[zone];
    g_vl53_act_frame.target_status[zone] = results->target_status[zone];
    g_vl53_act_frame.target_count[zone] = results->nb_target_detected[zone];
    g_vl53_act_frame.signal_per_spad[zone] = results->signal_per_spad[zone];
    g_vl53_act_frame.ambient_per_spad[zone] = results->ambient_per_spad[zone];
    g_vl53_act_frame.reflectance[zone] = results->reflectance[zone];
    g_vl53_act_frame.range_sigma_mm[zone] = results->range_sigma_mm[zone];

    if (vl53lmz_cnh_get_block_addresses(&g_vl53_cnh_config,
                                        zone,
                                        g_vl53_cnh_buffer,
                                        &p_hist,
                                        &p_hist_scaler,
                                        &p_ambient,
                                        &p_ambient_scaler) != 0U)
    {
      return 2U;
    }
    if ((p_hist == NULL) || (p_hist_scaler == NULL))
    {
      return 3U;
    }

    for (bin = 0U; bin < VL53_APP_ACT_CNH_BINS; bin++)
    {
      g_vl53_act_frame.cnh_raw[bin][zone] = p_hist[bin];
      g_vl53_act_frame.cnh_scaler[bin][zone] = p_hist_scaler[bin];
    }
  }

  g_vl53_act_frame.timestamp_ms = g_vl53_frame.timestamp_ms;
  g_vl53_act_frame.frame_seq = g_vl53_frame.frame_count;
  return 0U;
}

/**
  * @brief 把当前帧整理成一行 CSV 并写入 SD。
  * @param 无。
  * @return 0 成功，非 0 失败。
  */
static uint8_t VL53_App_WriteCsvFrame(void)
{
  static char csv_line[12288];
  static char csv_header[12288];
  const VL53_App_Frame_t *frame;
  char *cursor;
  size_t remain;
  uint8_t zone;
  uint8_t agg_id;
  uint8_t bin;
  uint8_t status;
  int32_t *p_hist = NULL;
  int8_t *p_hist_scaler = NULL;
  int32_t *p_ambient = NULL;
  int8_t *p_ambient_scaler = NULL;

  if (VL53_App_BuildCsvHeader(csv_header, sizeof(csv_header)) != 0U)
  {
    return 7U;
  }

  frame = VL53_App_GetLatestFrame();
  if (frame->valid == 0U)
  {
    return 1U;
  }

  cursor = csv_line;
  remain = sizeof(csv_line);

  if (VL53_App_Append(&cursor, &remain,
                      "%lu,%u,%lu,%u,%u",
                      (unsigned long)frame->timestamp_ms,
                      (unsigned int)g_vl53_label_score,
                      (unsigned long)frame->frame_count,
                      (unsigned int)g_vl53_mode,
                      (unsigned int)frame->valid) != 0U)
  {
    return 2U;
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (VL53_App_Append(&cursor, &remain, ",%d", frame->distance_mm[zone]) != 0U)
    {
      return 3U;
    }
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (VL53_App_Append(&cursor, &remain, ",%u", (unsigned int)frame->target_status[zone]) != 0U)
    {
      return 4U;
    }
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (VL53_App_Append(&cursor, &remain, ",%u", (unsigned int)frame->nb_target_detected[zone]) != 0U)
    {
      return 5U;
    }
  }

  for (agg_id = 0U; agg_id < VL53_APP_MAX_ZONES; agg_id++)
  {
    status = vl53lmz_cnh_get_block_addresses(&g_vl53_cnh_config,
                                             agg_id,
                                             g_vl53_cnh_buffer,
                                             &p_hist,
                                             &p_hist_scaler,
                                             &p_ambient,
                                             &p_ambient_scaler);
    if ((status != 0U) || (p_hist == NULL) || (p_hist_scaler == NULL))
    {
      return 6U;
    }

    for (bin = 0U; bin < VL53_APP_CNH_GESTURE_BINS; bin++)
    {
      if (VL53_App_Append(&cursor,
                          &remain,
                          ",%ld",
                          (long)p_hist[VL53_APP_CNH_GESTURE_OFFSET + bin]) != 0U)
      {
        return 7U;
      }
    }
  }

  if (VL53_App_Append(&cursor, &remain, "\r\n") != 0U)
  {
    return 8U;
  }

  if (VL53_App_CountCsvColumns(csv_line) != VL53_APP_CSV_COLUMNS)
  {
    return 9U;
  }

  return CsvLogger_AppendLine(VL53_APP_CSV_FILE, csv_header, csv_line);
}

/**
  * @brief 处理一次采样请求。
  *
  * 说明：采样开关按一次只触发一次，等待 500ms 后把稳定帧写进 CSV。
  */
static void VL53_App_TryCommitSample(void)
{
  uint32_t now_ms;

  if ((g_vl53_mode != VL53_APP_MODE_SAMPLE) || (g_vl53_sample_pending == 0U))
  {
    return;
  }

  now_ms = HAL_GetTick();
  if ((now_ms - g_vl53_sample_request_ms) < VL53_APP_SAMPLE_DELAY_MS)
  {
    return;
  }

  g_vl53_sample_pending = 0U;
  g_vl53_last_sample_status = VL53_App_WriteCsvFrame();
  g_vl53_sample_result_pending = 1U;

  if (g_vl53_last_sample_status == 0U)
  {
    VL53_App_Log("CSV saved\r\n");
  }
  else
  {
    char log_buffer[48];
    snprintf(log_buffer,
             sizeof(log_buffer),
             "CSV failed, status=%u\r\n",
             g_vl53_last_sample_status);
    VL53_App_Log(log_buffer);
  }
}

/**
  * @brief 配置 CNH 输出块，并用自定义 output config 启动测距。
  *
  * 说明：CNH 属于额外输出块，不能再用普通 vl53lmz_start_ranging()；
  * 必须 create output config 后追加 CNH block，再统一 start。
  */
static uint8_t VL53_App_StartCnhRanging(void)
{
  uint8_t status;
  union Block_header cnh_block_header;
  char log_buffer[96];

  status = vl53lmz_cnh_init_config(&g_vl53_cnh_config,
                                   VL53_APP_CNH_START_BIN,
                                   VL53_APP_CNH_SENSOR_BINS,
                                   VL53_APP_CNH_SUB_SAMPLE);
  if (status != 0U)
  {
    VL53_App_LogStepError("cnh init config", status);
    return status;
  }

  status = vl53lmz_cnh_create_agg_map(&g_vl53_cnh_config,
                                      VL53_APP_RESOLUTION,
                                      0,
                                      0,
                                      1,
                                      1,
                                      8,
                                      8);
  if (status != 0U)
  {
    VL53_App_LogStepError("cnh create agg map", status);
    return status;
  }

  status = vl53lmz_cnh_calc_required_memory(&g_vl53_cnh_config, &g_vl53_cnh_data_size);
  if ((status != 0U) || (g_vl53_cnh_data_size > VL53LMZ_CNH_MAX_DATA_BYTES))
  {
    VL53_App_LogStepError("cnh memory size", status);
    return (status != 0U) ? status : 1U;
  }

  status = vl53lmz_cnh_send_config(&g_vl53_config, &g_vl53_cnh_config);
  if (status != 0U)
  {
    VL53_App_LogStepError("cnh send config", status);
    return status;
  }

  status = vl53lmz_create_output_config(&g_vl53_config);
  if (status != 0U)
  {
    VL53_App_LogStepError("create output config", status);
    return status;
  }

  cnh_block_header.bytes = 0U;
  cnh_block_header.idx = VL53LMZ_CNH_DATA_IDX;
  cnh_block_header.type = 4U;
  cnh_block_header.size = g_vl53_cnh_data_size / 4U;

  status = vl53lmz_add_output_block(&g_vl53_config, cnh_block_header.bytes);
  if (status != 0U)
  {
    VL53_App_LogStepError("add cnh output", status);
    return status;
  }

  status = vl53lmz_send_output_config_and_start(&g_vl53_config);
  if (status != 0U)
  {
    VL53_App_LogStepError("start cnh ranging", status);
    return status;
  }

  snprintf(log_buffer, sizeof(log_buffer), "VL53: CNH started, size=%lu bytes\r\n",
           (unsigned long)g_vl53_cnh_data_size);
  VL53_App_Log(log_buffer);
  return 0U;
}

/**
  * @brief 按低频节奏打印中心 zone 的距离和 CNH bin，用于确认 CNH 是否正常变化。
  *
  * 说明：串口打印是阻塞式的，这里只打印一个中心 zone，避免明显拖慢主循环。
  */
#if (VL53_APP_FRAME_LOG_ENABLE != 0U)
static void VL53_App_LogFrameIfDue(void)
{
  uint32_t now_ms;
  uint8_t col;
  uint8_t status;
  char log_buffer[128];
  int32_t *p_hist = NULL;
  int8_t *p_hist_scaler = NULL;
  int32_t *p_ambient = NULL;
  int8_t *p_ambient_scaler = NULL;

  now_ms = HAL_GetTick();
  if ((now_ms - g_vl53_last_log_ms) < VL53_APP_LOG_PERIOD_MS)
  {
    return;
  }

  g_vl53_last_log_ms = now_ms;

  snprintf(log_buffer,
           sizeof(log_buffer),
           "VL53 frame %lu, t=%lu ms, z%u d=%d st=%u cnh_valid=%u\r\n",
           (unsigned long)g_vl53_frame.frame_count,
           (unsigned long)g_vl53_frame.timestamp_ms,
           (unsigned int)VL53_APP_CNH_CENTER_ZONE,
           g_vl53_frame.distance_mm[VL53_APP_CNH_CENTER_ZONE],
           g_vl53_frame.target_status[VL53_APP_CNH_CENTER_ZONE],
           g_vl53_cnh_valid);
  VL53_App_Log(log_buffer);

  if (g_vl53_cnh_valid == 0U)
  {
    return;
  }

  status = vl53lmz_cnh_get_block_addresses(&g_vl53_cnh_config,
                                           VL53_APP_CNH_CENTER_ZONE,
                                           g_vl53_cnh_buffer,
                                           &p_hist,
                                           &p_hist_scaler,
                                           &p_ambient,
                                           &p_ambient_scaler);
  if ((status != 0U) || (p_hist == NULL) || (p_hist_scaler == NULL))
  {
    VL53_App_LogStepError("cnh get address", status);
    return;
  }

  log_buffer[0] = '\0';
  for (col = 0U; col < VL53_APP_CNH_GESTURE_BINS; col++)
  {
    snprintf(&log_buffer[strlen(log_buffer)],
             sizeof(log_buffer) - strlen(log_buffer),
             "%ld(%d) ",
             (long)p_hist[VL53_APP_CNH_GESTURE_OFFSET + col],
             (int)p_hist_scaler[VL53_APP_CNH_GESTURE_OFFSET + col]);
  }
  VL53_App_Log(log_buffer);
  VL53_App_Log("\r\n");
}
#endif

uint8_t VL53_App_Init(void)
{
  uint8_t status;
  uint8_t is_alive = 0U;
  char log_buffer[96];

  g_vl53_ready = 0U;
  g_vl53_new_frame = 0U;
  g_vl53_mode = VL53_APP_MODE_INFER;
  g_vl53_sample_pending = 0U;
  g_vl53_sample_request_ms = 0U;
  g_vl53_label_score = VL53_APP_LABEL_NONE;
  g_vl53_cnh_valid = 0U;
  g_vl53_cnh_data_size = 0U;
  g_vl53_last_ai_log_ms = 0U;
#if (VL53_APP_ROI_LOG_PERIOD_MS != 0U)
  g_vl53_last_roi_log_ms = 0U;
#endif
  g_vl53_ai_openness_filtered = 0.0f;
  g_vl53_ai_filter_ready = 0U;
  memset(&g_vl53_frame, 0, sizeof(g_vl53_frame));
  memset(&g_vl53_runtime_stats, 0, sizeof(g_vl53_runtime_stats));
  g_vl53_runtime_stats.requested_hz = VL53_APP_RANGING_FREQ_HZ;
  g_vl53_stats_last_frame_ms = 0U;
  g_vl53_stats_last_stream = 0U;
  g_vl53_stats_stream_valid = 0U;
  g_vl53_stats_last_process_end_ms = 0U;
  g_vl53_stats_ready_poll_count = 0U;
  g_vl53_last_ready_poll_ms = 0U;
  memset(&g_vl53_act_frame, 0, sizeof(g_vl53_act_frame));
  memset(&g_vl53_ai, 0, sizeof(g_vl53_ai));
  memset(&g_vl53_roi, 0, sizeof(g_vl53_roi));
  memset(g_vl53_ai_input, 0, sizeof(g_vl53_ai_input));
  memset(&g_vl53_results, 0, sizeof(g_vl53_results));
  memset(&g_vl53_cnh_config, 0, sizeof(g_vl53_cnh_config));
  memset(g_vl53_cnh_buffer, 0, sizeof(g_vl53_cnh_buffer));
  g_vl53_config.platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;

  status = vl53lmz_is_alive(&g_vl53_config, &is_alive);
  if ((status != 0U) || (is_alive == 0U))
  {
    VL53_App_Log("VL53: sensor not detected\r\n");
    return VL53_App_MarkInitFailed(status);
  }

  VL53_App_Log("VL53: loading firmware\r\n");

  status = vl53lmz_init(&g_vl53_config);
  if (status != 0U)
  {
    VL53_App_Log("VL53: init failed\r\n");
    return VL53_App_MarkInitFailed(status);
  }

  snprintf(log_buffer,
           sizeof(log_buffer),
           "VL53: init ok, module type 0x%02X\r\n",
           g_vl53_config.module_type);
  VL53_App_Log(log_buffer);

  status = vl53lmz_set_resolution(&g_vl53_config, VL53_APP_RESOLUTION);
  if (status != 0U)
  {
    VL53_App_LogStepError("set resolution", status);
    return VL53_App_MarkInitFailed(status);
  }

  status = vl53lmz_set_target_order(&g_vl53_config, VL53LMZ_TARGET_ORDER_CLOSEST);
  if (status != 0U)
  {
    VL53_App_LogStepError("set target order", status);
    return VL53_App_MarkInitFailed(status);
  }

  status = vl53lmz_set_ranging_mode(&g_vl53_config, VL53LMZ_RANGING_MODE_AUTONOMOUS);
  if (status != 0U)
  {
    VL53_App_LogStepError("set ranging mode", status);
    return VL53_App_MarkInitFailed(status);
  }

  status = vl53lmz_set_ranging_frequency_hz(&g_vl53_config, VL53_APP_RANGING_FREQ_HZ);
  if (status != 0U)
  {
    VL53_App_LogStepError("set ranging frequency", status);
    return VL53_App_MarkInitFailed(status);
  }

  status = vl53lmz_set_integration_time_ms(&g_vl53_config, VL53_APP_INTEGRATION_TIME_MS);
  if (status != 0U)
  {
    VL53_App_LogStepError("set integration time", status);
    return VL53_App_MarkInitFailed(status);
  }

  status = VL53_App_StartCnhRanging();
  if (status != 0U)
  {
    return VL53_App_MarkInitFailed(status);
  }

  /*
   * CNH自定义启动完成后重新读回配置，确认启动过程没有覆盖
   * 测距频率或积分时间。诊断读取失败时保留0，不阻止正常测距。
   */
  (void)vl53lmz_get_ranging_frequency_hz(
    &g_vl53_config, &g_vl53_runtime_stats.actual_hz);
  (void)vl53lmz_get_integration_time_ms(
    &g_vl53_config, &g_vl53_runtime_stats.actual_integration_ms);
  g_vl53_runtime_stats.read_bytes = g_vl53_config.data_read_size;
  g_vl53_runtime_stats.cnh_bytes = g_vl53_cnh_data_size;

  snprintf(log_buffer,
           sizeof(log_buffer),
           "VL53: ranging started, 8x8 %uHz CNH\r\n",
           (unsigned int)VL53_APP_RANGING_FREQ_HZ);
  VL53_App_Log(log_buffer);

  g_vl53_ready = 1U;
  g_vl53_next_init_retry_ms = 0U;
  g_vl53_init_retry_count = 0U;
  return 0U;
}

uint8_t VL53_App_SetMode(VL53_App_Mode_t mode)
{
  if (mode == g_vl53_mode)
  {
    if (mode == VL53_APP_MODE_INFER)
    {
      g_vl53_sample_pending = 0U;
      g_vl53_sample_result_pending = 0U;
      g_vl53_sample_request_ms = 0U;
      CsvLogger_Close();
    }
    return 0U;
  }

  g_vl53_mode = mode;
  g_vl53_sample_pending = 0U;
  g_vl53_sample_result_pending = 0U;
  g_vl53_sample_request_ms = 0U;
  CsvLogger_Close();
  return 0U;
}

VL53_App_Mode_t VL53_App_GetMode(void)
{
  return g_vl53_mode;
}

void VL53_App_SetAiLogEnable(uint8_t enable)
{
  g_vl53_ai_log_enable = (enable != 0U) ? 1U : 0U;
}

void VL53_App_SetLabelScore(uint8_t label_score)
{
  if ((label_score <= 100U) || (label_score == VL53_APP_LABEL_NONE))
  {
    g_vl53_label_score = label_score;
  }
}

uint8_t VL53_App_GetLabelScore(void)
{
  return g_vl53_label_score;
}

void VL53_App_RequestSample(void)
{
  if (g_vl53_mode != VL53_APP_MODE_SAMPLE)
  {
    return;
  }

  g_vl53_sample_pending = 1U;
  g_vl53_sample_request_ms = HAL_GetTick();
}

uint8_t VL53_App_ConsumeSampleResult(uint8_t *status)
{
  if (g_vl53_sample_result_pending == 0U)
  {
    return 0U;
  }

  if (status != NULL)
  {
    *status = g_vl53_last_sample_status;
  }

  g_vl53_sample_result_pending = 0U;
  return 1U;
}

/**
  * @brief  获取当前帧的轻量采样统计，用于VOFA均衡采样分桶。
  * @param  stats 输出统计结果。
  * @return 0成功，非0表示当前帧无有效前景。
  *
  * 说明：这里复用AI前处理的有效点和 d_min+120 前景逻辑，
  * 不跑AI、不写SD，只计算当前手部中心距离。
  */
uint8_t VL53_App_GetSampleStats(VL53_App_SampleStats_t *stats)
{
  uint16_t dist[VL53_APP_MAX_ZONES];
  uint8_t valid_mask[VL53_APP_MAX_ZONES];
  uint8_t zone;
  uint8_t valid_count = 0U;
  uint8_t foreground_count = 0U;
  int16_t d_min = 32767;

  if (stats == NULL)
  {
    return 1U;
  }

  memset(stats, 0, sizeof(*stats));
  if (g_vl53_frame.valid == 0U)
  {
    return 2U;
  }

  memset(valid_mask, 0, sizeof(valid_mask));
  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if (VL53_App_IsAiValidZone(zone) != 0U)
    {
      valid_mask[zone] = 1U;
      valid_count++;
      if (g_vl53_frame.distance_mm[zone] < d_min)
      {
        d_min = g_vl53_frame.distance_mm[zone];
      }
    }
  }

  if (valid_count < VL53_APP_AI_MIN_HAND_ZONES)
  {
    return 3U;
  }

  for (zone = 0U; zone < VL53_APP_MAX_ZONES; zone++)
  {
    if ((valid_mask[zone] != 0U) &&
        (g_vl53_frame.distance_mm[zone] <= (d_min + VL53_APP_AI_FOREGROUND_MARGIN)))
    {
      dist[foreground_count] = (uint16_t)g_vl53_frame.distance_mm[zone];
      foreground_count++;
    }
  }

  if (foreground_count < VL53_APP_AI_MIN_HAND_ZONES)
  {
    return 4U;
  }

  stats->valid = 1U;
  stats->zone_count = foreground_count;
  stats->center_z_mm = VL53_App_MedianU16(dist, foreground_count);
  stats->frame_count = g_vl53_frame.frame_count;
  return 0U;
}

/**
 * @brief 更新成功测距帧的周期和传感器序号统计。
 * @param timestamp_ms 当前帧读取完成的HAL毫秒时间戳。
 *
 * 首帧只建立基准，不计算周期和跳帧。uint8_t减法可以自然处理
 * streamcount从255回绕到0的情况。
 */
static void VL53_App_UpdateRuntimeStats(uint32_t timestamp_ms)
{
  uint8_t current_stream = g_vl53_config.streamcount;
  uint8_t delta;

  if (g_vl53_stats_last_frame_ms != 0U)
  {
    g_vl53_runtime_stats.frame_period_ms =
      timestamp_ms - g_vl53_stats_last_frame_ms;
  }
  g_vl53_stats_last_frame_ms = timestamp_ms;

  g_vl53_runtime_stats.sensor_streamcount = current_stream;
  if (g_vl53_stats_stream_valid != 0U)
  {
    delta = (uint8_t)(current_stream - g_vl53_stats_last_stream);
    g_vl53_runtime_stats.stream_delta = delta;
    if (delta > 1U)
    {
      g_vl53_runtime_stats.skipped_frame_count += (uint32_t)(delta - 1U);
    }
  }
  else
  {
    g_vl53_runtime_stats.stream_delta = 0U;
    g_vl53_stats_stream_valid = 1U;
  }
  g_vl53_stats_last_stream = current_stream;
}

void VL53_App_Task(void)
{
  uint8_t status;
  uint8_t data_ready = 0U;
  uint32_t get_start_ms;
  uint32_t get_end_ms;
  uint32_t process_end_ms;
  uint32_t now_ms;

  if (g_vl53_ready == 0U)
  {
    VL53_App_TryInitRetry();
    return;
  }

  now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - g_vl53_last_ready_poll_ms) <
      VL53_APP_READY_POLL_PERIOD_MS)
  {
    return;
  }
  g_vl53_last_ready_poll_ms = now_ms;

  if (g_vl53_stats_ready_poll_count < 0xFFFFFFFFUL)
  {
    g_vl53_stats_ready_poll_count++;
  }

  status = vl53lmz_check_data_ready(&g_vl53_config, &data_ready);
  if (status != 0U)
  {
    VL53_App_LogStepError("check data ready", status);
    return;
  }

  if (data_ready == 0U)
  {
    return;
  }

  get_start_ms = HAL_GetTick();
  if (g_vl53_stats_last_process_end_ms != 0U)
  {
    g_vl53_runtime_stats.ready_wait_ms =
      get_start_ms - g_vl53_stats_last_process_end_ms;
  }
  else
  {
    g_vl53_runtime_stats.ready_wait_ms = 0U;
  }
  g_vl53_runtime_stats.ready_poll_count =
    g_vl53_stats_ready_poll_count;

  status = vl53lmz_get_ranging_data(&g_vl53_config, &g_vl53_results);
  get_end_ms = HAL_GetTick();
  g_vl53_runtime_stats.get_data_ms = get_end_ms - get_start_ms;
  if (status != 0U)
  {
    g_vl53_runtime_stats.read_error_count++;
    VL53_App_LogStepError("get ranging data", status);
    return;
  }

  g_vl53_stats_ready_poll_count = 0U;

  /* 读取成功后立即记录，避免后续CNH处理结果影响I2C帧统计。 */
  VL53_App_UpdateRuntimeStats(get_end_ms);

  status = vl53lmz_results_extract_block(&g_vl53_config,
                                         VL53LMZ_CNH_DATA_IDX,
                                         (uint8_t *)g_vl53_cnh_buffer,
                                         (uint16_t)g_vl53_cnh_data_size);
  if (status != 0U)
  {
    g_vl53_cnh_valid = 0U;
    VL53_App_LogStepError("extract cnh block", status);
    return;
  }

  g_vl53_cnh_valid = 1U;

  VL53_App_CopyResultsToFrame(&g_vl53_results);
  status = VL53_App_CopyResultsToActFrame(&g_vl53_results);
  if (status != 0U)
  {
    g_vl53_cnh_valid = 0U;
    VL53_App_LogStepError("copy ACT frame", status);
    return;
  }
  if ((g_vl53_mode == VL53_APP_MODE_INFER) &&
      (g_vl53_gesture_infer_enable != 0U))
  {
    VL53_App_UpdateRoiDebug();
    VL53_App_UpdateAiEstimate();
    VL53_App_LogAiEstimateIfDue();
    VL53_App_LogRoiDebugIfDue();
  }
  VL53_App_TryCommitSample();
#if (VL53_APP_FRAME_LOG_ENABLE != 0U)
  VL53_App_LogFrameIfDue();
#endif

  process_end_ms = HAL_GetTick();
  g_vl53_runtime_stats.post_process_ms = process_end_ms - get_end_ms;
  g_vl53_stats_last_process_end_ms = process_end_ms;
}

uint8_t VL53_App_IsReady(void)
{
  return g_vl53_ready;
}

uint8_t VL53_App_HasNewFrame(void)
{
  return g_vl53_new_frame;
}

void VL53_App_ClearNewFrameFlag(void)
{
  g_vl53_new_frame = 0U;
}

const VL53_App_Frame_t *VL53_App_GetLatestFrame(void)
{
  return &g_vl53_frame;
}

const VL53_App_ActFrame_t *VL53_App_GetLatestActFrame(void)
{
  return &g_vl53_act_frame;
}

const VL53_App_RuntimeStats_t *VL53_App_GetRuntimeStats(void)
{
  return &g_vl53_runtime_stats;
}

const VL53_App_AiEstimate_t *VL53_App_GetAiEstimate(void)
{
  return &g_vl53_ai;
}

void VL53_App_SetGestureInferEnable(uint8_t enable)
{
  enable = (enable != 0U) ? 1U : 0U;
  if (enable == g_vl53_gesture_infer_enable)
  {
    return;
  }

  g_vl53_gesture_infer_enable = enable;

  /* 切换后清除旧结果，防止电机继续使用切换前的手势数据。 */
  memset(&g_vl53_ai, 0, sizeof(g_vl53_ai));
  g_vl53_ai_openness_filtered = 0.0f;
  g_vl53_ai_filter_ready = 0U;
}

uint8_t VL53_App_GetGestureInferEnable(void)
{
  return g_vl53_gesture_infer_enable;
}
