#ifndef __FOC_DRIVE_H
#define __FOC_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "main.h"
#include "tim.h"
#include "Config.h"
#include "Motor_type.h"

//******************************************FOC数学常量*******************************************/
#define FOC_SQRT3_F                  1.7320508f              // sqrt(3)，用于Clarke变换和SVPWM计算
#define FOC_ONE_BY_SQRT3_F           (1.0f / FOC_SQRT3_F)    // 1/sqrt(3)
#define FOC_SQRT3_BY_2_F             (FOC_SQRT3_F / 2.0f)    // sqrt(3)/2
#define FOC_TWO_BY_THREE_F           (2.0f / 3.0f)           // 2/3，幅值保持Clarke变换系数
#define FOC_ONE_BY_TWO_F             0.5f                    // 1/2
#define FOC_TWO_PI_F                 (2.0f * M_PI_F)         // 2*pi，用于电角度归一化

//******************************************FOC调制参数*******************************************/
#define FOC_VOLTAGE_NORM_LIMIT       1.4      // 归一化电压矢量幅值限幅，第一版限制在线性区附近
#define FOC_DUTY_CENTER              0.5f                    // 零电压矢量时的PWM占空比中心值

//******************************************SVPWM扇区参数*******************************************/
#define FOC_SVPWM_SECTOR_INVALID     0U                      // 无效扇区或零矢量兜底
#define FOC_SVPWM_SECTOR_MIN         1U                      // SVPWM最小有效扇区编号
#define FOC_SVPWM_SECTOR_MAX         6U                      // SVPWM最大有效扇区编号

//******************************************FOC基础坐标结构*******************************************/
typedef struct
{
    float alpha;                    // alpha轴分量，静止坐标系
    float beta;                     // beta轴分量，静止坐标系
} foc_alpha_beta_t;

typedef struct
{
    float d;                        // d轴分量，旋转坐标系
    float q;                        // q轴分量，旋转坐标系
} foc_dq_t;

//******************************************SVPWM计算结果*******************************************/
typedef struct
{
    float duty_u;                   // U相PWM占空比，范围0~1，最终写入TIM CCR前还会按PWM_DUTY_MIN/MAX限幅
    float duty_v;                   // V相PWM占空比，范围0~1
    float duty_w;                   // W相PWM占空比，范围0~1

    float t1;                       // 当前扇区第一个有效矢量作用时间，归一化到一个PWM周期
    float t2;                       // 当前扇区第二个有效矢量作用时间，归一化到一个PWM周期
    float t0;                       // 零矢量作用时间，归一化到一个PWM周期

    float voltage_scale;            // 电压矢量限幅缩放系数，1表示未缩放，小于1表示发生限幅
    uint8_t sector;                 // SVPWM扇区编号，范围1~6；0表示输入异常或零矢量兜底
    uint8_t voltage_limited;        // 电压矢量是否被限幅：0未限幅，1已限幅
} foc_svpwm_t;                      // SVPWM完整计算结果，保留真实数据类型，供FOC内部和软件验证使用

//******************************************FOC-VOFA观测数据*******************************************/
typedef struct
{
    float vd_cmd;                    // d轴归一化电压原始输入指令
    float vq_cmd;                    // q轴归一化电压原始输入指令

    float vd_limited;                // d轴归一化电压限幅后指令，实际参与逆Park计算
    float vq_limited;                // q轴归一化电压限幅后指令，实际参与逆Park计算

    float v_alpha;                   // alpha轴归一化电压，逆Park输出
    float v_beta;                    // beta轴归一化电压，逆Park输出
    float elec_angle_rad;            // 本次FOC计算使用的电角度，单位rad

    float duty_u;                    // U相PWM占空比，VOFA直接观察用
    float duty_v;                    // V相PWM占空比，VOFA直接观察用
    float duty_w;                    // W相PWM占空比，VOFA直接观察用

    float t1;                        // SVPWM有效矢量T1，归一化到一个PWM周期
    float t2;                        // SVPWM有效矢量T2，归一化到一个PWM周期
    float t0;                        // SVPWM零矢量T0，归一化到一个PWM周期

    float sector;                    // SVPWM扇区，使用float便于直接送VOFA JustFloat显示
    float voltage_scale;             // 电压矢量限幅缩放系数，1表示未缩放，小于1表示发生限幅
    float voltage_limited;           // 电压是否限幅，0.0f未限幅，1.0f已限幅，便于VOFA显示
} foc_vofa_data_t;                   // FOC/SVPWM过程观测数据，只用于调试显示，不作为控制状态源

//******************************************PWM硬件输出映射*******************************************/
typedef struct
{
    TIM_HandleTypeDef *htim;        // PWM定时器句柄，例如&htim1、&htim8、&htim20
    uint32_t channel_u;             // U相对应的TIM通道，例如TIM_CHANNEL_1
    uint32_t channel_v;             // V相对应的TIM通道，例如TIM_CHANNEL_2
    uint32_t channel_w;             // W相对应的TIM通道；TIM20当前可配置为TIM_CHANNEL_4
} foc_pwm_output_t;

//******************************************FOC数学接口*******************************************/
float FOC_WrapAngle0To2Pi(float angle_rad);                                      // 将输入角度限制到0~2*pi，单位rad
void FOC_CalcSinCos(float angle_rad, float *sin_theta, float *cos_theta);         // 计算电角度sin/cos，供快环内Park/InvPark复用

foc_alpha_beta_t FOC_Clarke(float ia, float ib, float ic);                       // 三相电流ia/ib/ic -> alpha/beta，单位保持为A
foc_dq_t FOC_Park(float i_alpha, float i_beta, float elec_angle_rad);            // alpha/beta电流 -> d/q电流，elec_angle_rad为电角度rad
foc_dq_t FOC_ParkSinCos(float i_alpha, float i_beta, float sin_theta, float cos_theta); // 使用已计算sin/cos执行Park变换
foc_alpha_beta_t FOC_InvPark(float vd_norm, float vq_norm, float elec_angle_rad);// d/q归一化电压 -> alpha/beta归一化电压
foc_alpha_beta_t FOC_InvParkSinCos(float vd_norm, float vq_norm, float sin_theta, float cos_theta); // 使用已计算sin/cos执行逆Park变换

HAL_StatusTypeDef FOC_SVPWM(float u_alpha_norm,
                            float u_beta_norm,
                            foc_svpwm_t *svpwm_out);                             // alpha/beta归一化电压 -> SVPWM完整计算结果，写入svpwm_out

//******************************************FOC驱动接口*******************************************/
HAL_StatusTypeDef FOC_RunVoltageNorm(motor_t *motor,
                                     float vd_norm,
                                     float vq_norm,
                                     float elec_angle_rad,
                                     foc_vofa_data_t *vofa_out);                 // 执行一次归一化电压FOC计算，写入motor->control；vofa_out可传NULL
HAL_StatusTypeDef FOC_RunVoltageNormSinCos(motor_t *motor,
                                           float vd_norm,
                                           float vq_norm,
                                           float elec_angle_rad,
                                           float sin_theta,
                                           float cos_theta,
                                           foc_vofa_data_t *vofa_out);            // 使用已计算sin/cos执行归一化电压FOC计算

HAL_StatusTypeDef FOC_WritePwmDuty(const foc_pwm_output_t *pwm,
                                   float duty_u,
                                   float duty_v,
                                   float duty_w);                                // 将三相duty写入对应TIM CCR，不启动PWM输出，返回写入状态

HAL_StatusTypeDef FOC_StartPwmOutput(const foc_pwm_output_t *pwm);               // 启动三相PWM及互补PWM输出，需由外部安全流程显式调用
HAL_StatusTypeDef FOC_StopPwmOutput(const foc_pwm_output_t *pwm);                // 停止三相PWM及互补PWM输出，故障或停机时调用

#ifdef __cplusplus
}
#endif

#endif /* __FOC_DRIVE_H */
