#ifndef __SPI_ENCODER_H__
#define __SPI_ENCODER_H__

#include "main.h"
#include "spi.h"
#include "Config.h"
#include <stdint.h>

/************************************************
* MT6816 SPI 编码器参数
* 分辨率：14bit
* 单圈机械角范围：0 ~ 2PI
* 通信方式：SPI
************************************************ */

#define ENCODER_CPR         16384.0f                    // 14bit 一圈总码值
#define ENCODER_RAW_MAX     16383U                      // 原始角度最大值
#define ENCODER_TIMEOUT     10U                         // SPI 阻塞收发超时时间(ms)
#define ENCODER_RAD_STEP    (2.0f * M_PI_F / ENCODER_CPR) // 每个码值对应的机械角(rad)

/* 编码器编号 */
typedef enum
{
    encoder_id_1 = 0,     // CS -> PB7
    encoder_id_2 = 1,     // CS -> PB6
    encoder_id_3 = 2      // CS -> PD7
} encoder_id_t;

/* 编码器数据结构体 */
typedef struct
{
    uint16_t raw_angle;           // 当前原始角度值（14bit）
    uint16_t raw_angle_last;      // 上一次原始角度值（14bit）
    uint8_t last_rx[4];           // 最近一次SPI接收原始帧：rx0 rx1 rx2 rx3
    float angle;                  // 当前机械角度，单位 rad，范围 0 ~ 2PI
    float angle_last;             // 上一次机械角度，单位 rad
    float angle_offset;           // 机械零点偏移，单位 rad
    float angle_single;           // 扣除零点后的单圈机械角，单位 rad，范围 0 ~ 2PI
    float angle_total;            // 多圈累计机械角度，单位 rad
    int32_t turn_count;            // 显式多圈计数，正向跨零点+1，反向跨零点-1
    uint32_t angle_sample_cycles;   // 最近一次有效角度样本的DWT时间戳
    float speed_raw;               // SPI相邻有效样本差分得到的未滤波机械角速度，单位 rad/s
    float speed;                   // TIM7低通后的机械角速度，单位 rad/s
    float speed_avg_diff_sum;       // 速度环周期内累计机械角差，单位 rad
    uint32_t speed_avg_cycles_sum;  // 速度环周期内累计DWT周期数
    uint16_t speed_avg_sample_count;// 速度环周期内累计有效SPI样本数
    uint8_t initialized;           // 是否已经完成首次有效读取，1 已初始化，0 未初始化
    uint8_t no_mag_warning;        // 磁场不足报警，来自 MT6816 状态位
    uint8_t parity_error;          // 奇偶校验错误，1 表示最近一次角度帧校验失败
    uint8_t valid;                // 最近一次读取是否有效，1 有效，0 无效
} spi_encoder_t;

/* 快环编码器快照：仅给初始化阶段已绑定好的高速路径使用，不做id/pointer合法性检查 */
typedef struct
{
    float angle_total;             // 多圈累计机械角度，单位 rad
    float speed_raw;               // 未滤波机械角速度，单位 rad/s
    float speed;                   // 低通后的机械角速度，单位 rad/s
    uint32_t sample_cycles;         // 最近一次有效角度样本DWT时间戳
    uint8_t valid;                  // 最近一次角度数据是否有效
} spi_encoder_fast_snapshot_t;

/* 编码器数据 */
extern spi_encoder_t spi_encoder[ENCODER_NUM];

/* 基本接口 */
void SPI_Encoder_Init(void);                                      // 初始化编码器模块软件状态
HAL_StatusTypeDef SPI_Encoder_Read_Raw(encoder_id_t id, uint16_t *raw);   // 读取原始角度值
HAL_StatusTypeDef SPI_Encoder_Read_Angle(encoder_id_t id, float *angle_rad); // 读取机械角度(rad)
HAL_StatusTypeDef SPI_Encoder_Update(encoder_id_t id, float dt);  // 更新角度、累计角度和速度

/* DMA 更新接口 */
HAL_StatusTypeDef SPI_Encoder_Start_Update_All_DMA(uint32_t now_us); // 启动一次三路编码器DMA读取，now_us为本次采样启动时间
HAL_StatusTypeDef SPI_Encoder_Start_Update_One_DMA(encoder_id_t id, uint32_t now_us); // 启动一次单路编码器DMA读取，用于PWM事件同步刷新角度缓存
HAL_StatusTypeDef SPI_Encoder_Request_Update_DMA(encoder_id_t id, uint32_t now_cycles); // 提交一路编码器DMA刷新请求，只置pending不等待完成
HAL_StatusTypeDef SPI_Encoder_ServicePendingOnce(void);             // SPI空闲时最多启动一路pending编码器DMA
void SPI_Encoder_TxRxCpltCallback(SPI_HandleTypeDef *hspi);       // SPI DMA收发完成回调入口，在HAL_SPI_TxRxCpltCallback中调用
void SPI_Encoder_ErrorCallback(SPI_HandleTypeDef *hspi);          // SPI错误回调入口，在HAL_SPI_ErrorCallback中调用
uint8_t SPI_Encoder_Is_DMA_Busy(void);                            // 判断当前是否正在进行编码器DMA读取
uint8_t SPI_Encoder_Is_Update_Done(void);                         // 判断最近一次三路DMA更新是否完成
void SPI_Encoder_Clear_Update_Done(void);                         // 清除DMA更新完成标志
uint32_t SPI_Encoder_Get_DMA_Missed_Count(void);                  // 获取DMA忙导致跳过更新的次数

/* 零点与角度接口 */
void SPI_Encoder_Set_Zero(encoder_id_t id);                       // 将当前角度设为机械零点
void SPI_Encoder_Set_Offset(encoder_id_t id, float offset_rad);   // 手动设置机械零点偏移
uint8_t SPI_Encoder_Restore_Total_Angle(encoder_id_t id,
                                        float target_total_rad,
                                        float max_error_rad);      // 按当前单圈角恢复多圈累计角，只改turn_count/angle_total
float SPI_Encoder_Get_Angle(encoder_id_t id);                     // 获取当前单圈机械角
float SPI_Encoder_Get_Total_Angle(encoder_id_t id);               // 获取当前累计机械角
int32_t SPI_Encoder_Get_Turn_Count(encoder_id_t id);              // 获取当前显式多圈计数

/* 速度与状态接口 */
HAL_StatusTypeDef SPI_Encoder_Update_Speed_By_Cache(encoder_id_t id, float dt); // 消费SPI样本累计量计算平均速度，不触发SPI通信，建议由TIM7固定周期调用
float SPI_Encoder_Get_Speed(encoder_id_t id);                     // 获取当前机械角速度
void SPI_Encoder_Get_FastSnapshot_Unchecked(encoder_id_t id, spi_encoder_fast_snapshot_t *snapshot); // 快环unchecked快照，调用方保证id和指针有效
uint8_t SPI_Encoder_Is_Valid(encoder_id_t id);                    // 判断最近一次数据是否有效

/* 工具函数 */
float SPI_Encoder_Normalize_Angle(float angle);                   // 将角度归一化到 0 ~ 2PI
float SPI_Encoder_Cycle_Diff(float diff, float cycle);            // 计算周期量的最短差值

#endif
