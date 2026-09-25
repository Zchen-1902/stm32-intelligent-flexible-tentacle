#ifndef VL53_APP_H
#define VL53_APP_H

#include <stdint.h>

#define VL53_APP_MAX_ZONES 64U
#define VL53_APP_LABEL_NONE 255U
#define VL53_APP_ACT_CNH_BINS 10U

typedef enum
{
  VL53_APP_MODE_INFER = 0U,   /* 推理模式，不写 SD */
  VL53_APP_MODE_SAMPLE = 1U    /* 采样模式，允许写 SD */
} VL53_App_Mode_t;

typedef struct
{
  uint8_t valid;                                      /* 当前帧是否有效 */
  uint8_t resolution;                                 /* 当前分辨率，16 表示 4x4，64 表示 8x8 */
  uint32_t frame_count;                               /* 成功读取到的测距帧计数 */
  uint32_t timestamp_ms;                              /* 该帧读取完成时的 HAL tick 时间戳 */
  int16_t distance_mm[VL53_APP_MAX_ZONES];            /* 每个区域的距离，单位 mm */
  uint8_t target_status[VL53_APP_MAX_ZONES];          /* 每个区域的目标状态，5 或 9 通常表示有效 */
  uint8_t nb_target_detected[VL53_APP_MAX_ZONES];     /* 每个区域检测到的目标数量 */
} VL53_App_Frame_t;

/**
 * @brief ACT采集使用的一帧原始VL53数据。
 *
 * 每个数组均按8x8区域的0~63顺序排列。CNH保存10个距离区间，
 * 每个区间包含64个区域；该结构直接作为ACT_FRAME中的传感器数据。
 */
typedef struct
{
  uint32_t frame_seq;                                      /* VL53有效帧序号。 */
  uint32_t timestamp_ms;                                   /* 本帧读取完成时间。 */
  int16_t distance_mm[VL53_APP_MAX_ZONES];                 /* 距离，单位mm。 */
  uint8_t target_status[VL53_APP_MAX_ZONES];               /* ST测距状态码。 */
  uint8_t target_count[VL53_APP_MAX_ZONES];                /* 每区目标数量。 */
  uint32_t signal_per_spad[VL53_APP_MAX_ZONES];            /* 目标回波强度。 */
  uint32_t ambient_per_spad[VL53_APP_MAX_ZONES];           /* 环境红外强度。 */
  uint8_t reflectance[VL53_APP_MAX_ZONES];                 /* 估计反射率。 */
  uint16_t range_sigma_mm[VL53_APP_MAX_ZONES];             /* 距离不确定度。 */
  int32_t cnh_raw[VL53_APP_ACT_CNH_BINS][VL53_APP_MAX_ZONES]; /* CNH原始直方图。 */
  int8_t cnh_scaler[VL53_APP_ACT_CNH_BINS][VL53_APP_MAX_ZONES]; /* CNH缩放指数。 */
} VL53_App_ActFrame_t;

/**
 * @brief VL53实时读取性能统计。
 *
 * 只保存统计结果，不改变测距、ACT采集或串口协议。
 */
typedef struct
{
  uint8_t requested_hz;             /* 软件请求的VL53测距频率。 */
  uint8_t actual_hz;                /* CNH启动后从传感器读回的实际频率。 */
  uint8_t sensor_streamcount;       /* 传感器自身8位帧序号。 */
  uint8_t stream_delta;             /* 相邻成功读取帧的传感器序号差。 */
  uint32_t actual_integration_ms;   /* CNH启动后读回的实际积分时间。 */
  uint32_t read_bytes;              /* ULD每帧通过I2C读取的字节数。 */
  uint32_t cnh_bytes;               /* 当前CNH数据块字节数。 */
  uint32_t ready_wait_ms;           /* 上一帧处理完成到本帧ready的等待时间。 */
  uint32_t ready_poll_count;        /* 获得本帧前查询data_ready的次数。 */
  uint32_t get_data_ms;             /* 整帧读取和ULD解析耗时。 */
  uint32_t post_process_ms;         /* 读取完成后的CNH提取和帧处理耗时。 */
  uint32_t frame_period_ms;         /* 两次成功帧之间的时间。 */
  uint32_t skipped_frame_count;     /* 根据传感器帧序号统计的累计跳帧数。 */
  uint32_t read_error_count;        /* get_ranging_data累计失败次数。 */
} VL53_App_RuntimeStats_t;

typedef struct
{
  uint8_t valid;                                      /* AI结果是否有效 */
  uint8_t status;                                     /* 0成功，非0表示前处理或推理失败 */
  uint8_t hand_zone_count;                            /* 前景手部zone数量 */
  uint8_t ai_time_ms;                                 /* 前处理+推理耗时，单位ms */
  uint8_t openness_score;                             /* 张开程度，0=握拳，100=张开 */
  uint8_t bend_0_100;                                 /* 弯曲程度，bend=100-openness */
  uint8_t bend_raw_0_100;                             /* 未滤波弯曲程度，仅用于快速虚拟按键识别。 */
  uint8_t stiffness_0_100;                            /* 根据center_z得到的刚度 */
  int16_t x;                                          /* 平面X控制量，-100~100 */
  int16_t y;                                          /* 平面Y控制量，-100~100 */
  uint16_t center_z_mm;                               /* 手部前景距离中位数 */
  uint16_t center_x_q10;                              /* 8x8平面中心X，放大10倍 */
  uint16_t center_y_q10;                              /* 8x8平面中心Y，放大10倍 */
  uint32_t frame_count;                               /* 对应的VL53帧计数 */
} VL53_App_AiEstimate_t;

typedef struct
{
  uint8_t valid;                                      /* 当前统计结果是否有效 */
  uint8_t zone_count;                                 /* 用于统计中心距离的前景zone数量 */
  uint16_t center_z_mm;                               /* 当前前景距离中位数，单位mm */
  uint32_t frame_count;                               /* 对应的VL53帧计数 */
} VL53_App_SampleStats_t;

uint8_t VL53_App_Init(void);
void VL53_App_Task(void);
uint8_t VL53_App_IsReady(void);
uint8_t VL53_App_HasNewFrame(void);
void VL53_App_ClearNewFrameFlag(void);
const VL53_App_Frame_t *VL53_App_GetLatestFrame(void);
/**
 * @brief 获取最近一帧ACT原始传感器数据。
 * @return 只读帧指针；frame_seq为0表示尚未获得有效帧。
 * @note 调用方若通过DMA异步发送，必须先复制到自己的DMA缓冲区。
 */
const VL53_App_ActFrame_t *VL53_App_GetLatestActFrame(void);

/**
 * @brief 获取VL53实时读取性能统计。
 * @return 只读统计指针，调用方不得修改其中内容。
 * @note 该接口不访问I2C，只读取H7已经累计的统计结果。
 */
const VL53_App_RuntimeStats_t *VL53_App_GetRuntimeStats(void);

const VL53_App_AiEstimate_t *VL53_App_GetAiEstimate(void);

/**
 * @brief 打开或关闭手势模型推理。
 * @param enable 0关闭，非0打开。
 * @note 只控制手势模型，不停止VL53测距，也不影响ACT数据采集。
 */
void VL53_App_SetGestureInferEnable(uint8_t enable);

/**
 * @brief 查询手势模型推理是否打开。
 * @return 0表示关闭，1表示打开。
 */
uint8_t VL53_App_GetGestureInferEnable(void);

uint8_t VL53_App_SetMode(VL53_App_Mode_t mode);
VL53_App_Mode_t VL53_App_GetMode(void);
void VL53_App_SetAiLogEnable(uint8_t enable);
void VL53_App_SetLabelScore(uint8_t label_score);
uint8_t VL53_App_GetLabelScore(void);
void VL53_App_RequestSample(void);
uint8_t VL53_App_ConsumeSampleResult(uint8_t *status);
uint8_t VL53_App_GetSampleStats(VL53_App_SampleStats_t *stats);

#endif
