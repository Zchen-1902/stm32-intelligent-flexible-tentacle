#ifndef __MOTOR_CORE_H
#define __MOTOR_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "Motor_type.h"

//******************************************单电机控制命令*******************************************/
typedef struct
{
    uint8_t motor_id;             // 目标电机编号，范围0 ~ MOTOR_COUNT-1
    motor_mode_t mode;            // 目标控制模式

    float id_ref;                 // d轴电流目标，单位A
    float iq_ref;                 // q轴电流目标，单位A
    float speed_ref_rad_s;        // 机械角速度目标，单位rad/s
    float position_ref_rad;       // 机械角位置目标，单位rad，多圈位置可传连续角度
} motor_core_command_t;

typedef struct
{
    uint32_t last_cycles;         // 最近一次快环调用耗时，单位DWT cycle
    uint32_t max_cycles;          // 启动以来最大快环调用耗时，单位DWT cycle
    uint32_t run_count;           // 快环调用次数，包含未真正执行FOC的快速返回
    uint32_t overrun_count;       // 快环耗时超过MOTOR_CURRENT_LOOP_DT_S的次数
    uint8_t last_result;          // 最近一次快环返回值：1完成控制，0未执行
} motor_core_fast_loop_profile_t;

/*
 * motor_core 电机核心层
 *
 * 本模块后续负责创建和管理 motor_t 对象，并逐步连接 ADC、编码器、PID 和 FOC。
 * 第一版只先建立文件与接口位置，具体控制链路后续按最小改动逐步补充。
 */

void Motor_Core_Init(void);                         // 初始化三电机核心对象
motor_t *Motor_Core_GetMotor(uint8_t motor_id);     // 根据电机编号获取motor_t对象指针
uint8_t Motor_Core_GetMotorCount(void);             // 获取当前工程配置的电机数量
uint8_t Motor_Core_GetBoundEncoderId(uint8_t motor_id); // 获取初始化时绑定到该电机的编码器ID
void Motor_Core_GetFastLoopProfile(uint8_t motor_id,
                                   motor_core_fast_loop_profile_t *profile); // 获取指定电机快环DWT耗时统计

void Motor_Core_UpdateMotor(motor_t *motor, float dt);          // 更新单个电机的一周期控制流程
void Motor_Core_UpdateAll(float dt);                            // 更新全部电机的一周期控制流程
void Motor_Core_ApplyCommand(const motor_core_command_t *cmd);   // 应用单电机控制命令：设置目标值并切换模式
void Motor_Core_SetMode(uint8_t motor_id, motor_mode_t mode);   // 设置指定电机控制模式
void Motor_Core_ClearRuntimeFault(uint8_t motor_id);            // 清除运行态保护和PID缓存，不修改位置目标/RESTORE映射
void Motor_Core_SetCurrentRef(uint8_t motor_id,
                              float id_ref,
                              float iq_ref);                    // 设置指定电机d/q轴电流目标，单位A
void Motor_Core_SetSpeedRef(uint8_t motor_id,
                            float speed_ref_rad_s);             // 设置指定电机机械角速度目标，单位rad/s
void Motor_Core_SetPositionSpeedLimit(uint8_t motor_id,
                                      float speed_limit_rad_s); // 设置指定电机位置运动速度上限，单位rad/s
void Motor_Core_SetPositionSpeedLimitAll(float speed_limit_rad_s); // 设置三路位置运动速度上限，单位rad/s
void Motor_Core_SetPositionRef(uint8_t motor_id,
                               float mech_angle_ref_rad);        // 设置指定电机机械角位置目标，单位rad
void Motor_Core_SetPositionAbsoluteAll(const float *target_abs_rad); // 批量设置三电机绝对位置目标，单位rad
void Motor_Core_SetPositionRelativeAll(const float *delta_rad);      // 批量设置三电机相对当前位置的位置增量，单位rad
float Motor_Core_GetPositionFeedbackRad(uint8_t motor_id);           // 获取H7坐标下的实际多圈位置，单位rad
uint8_t Motor_Core_RestorePositionFeedbackQ(const int32_t position_q[3]); // 建立H7保存位置和G4当前原始位置的映射，单位0.01rad
uint8_t Motor_Core_IsPositionRestoreValid(void);                     // 三路都收到H7 ACTUAL后返回1
void Motor_Core_ClearPositionRestoreOffset(uint8_t motor_id);        // 清除H7 RESTORE建立的位置映射
void Motor_Core_SetEncoderZeroOffset(uint8_t motor_id,
                                     float offset_rad);            // 设置指定电机编码器机械零点偏移，单位rad，并同步到spi_encoder
void Motor_Core_ApplyEncoderOffsetConfig(void);                    // 将Config.h中的编码器零点配置应用到spi_encoder
HAL_StatusTypeDef Motor_Core_CalibrateCurrentZero(void);            // 执行三路电机电流零漂校准，需在ADC DMA和采样触发定时器启动后调用
HAL_StatusTypeDef Motor_Core_TrimCurrentZeroResidual(void);          // CAL退出后按真实ADC采样链路微调三相电流零点残差
HAL_StatusTypeDef Motor_Core_StartPwmOutput(uint8_t motor_id);     // 启动指定电机三相PWM和互补PWM输出，需外部确认安全条件
HAL_StatusTypeDef Motor_Core_StopPwmOutput(uint8_t motor_id);      // 停止指定电机三相PWM和互补PWM输出
HAL_StatusTypeDef Motor_Core_OutputNeutralPwm(uint8_t motor_id);   // 输出三相50%中性PWM，不停止定时器，适合调试空闲态保持ADC触发
uint8_t Motor_Core_IsPwmStartAllowed(uint8_t motor_id);            // 判断指定电机当前是否允许启动PWM输出
uint8_t Motor_Core_OutputFocAlignVoltage(uint8_t motor_id,
                                          float vd_norm);            // 输出固定电角度0的d轴对齐电压，只写CCR，不自动启动PWM
uint8_t Motor_Core_OutputFocQVoltage(uint8_t motor_id,
                                     float vq_norm);                 // 按当前FOC电角度输出q轴电压，只写CCR，不自动启动PWM
uint8_t Motor_Core_CaptureFocElecOffset(uint8_t motor_id);          // 读取当前编码器角度并计算FOC电角度零偏
void Motor_Core_StartFocOffsetAutoCalib(uint8_t motor_id,
                                        float vd_norm);              // 启动FOC电角度零偏自动校准：吸附两个相邻电角度0点
void Motor_Core_FocOffsetAutoCalibTask(void);                        // FOC电角度零偏自动校准任务，放主循环周期调用
uint8_t Motor_Core_IsFocOffsetAutoCalibBusy(void);                   // 当前FOC零偏自动校准是否在运行
uint8_t Motor_Core_GetFocOffsetAutoCalibResult(void);                // 最近一次FOC零偏自动校准结果
float Motor_Core_GetFocElecAngle(uint8_t motor_id);                 // 获取带FOC电角度零偏修正后的电角度，单位rad
uint8_t Motor_Core_UpdateCurrentObserve(uint8_t motor_id);           // 只刷新电流观测量：raw/ia/ib/ic/id/iq，不执行PID和PWM输出
void Motor_Core_GetCurrentSampleDebug(uint8_t motor_id,
                                      uint8_t *reconstruct_enable,
                                      uint8_t *reconstruct_phase,
                                      uint16_t *adc_trigger_ccr);     // 获取电流采样重构调试信息

uint8_t Motor_Core_UpdateCurrentLoop(motor_t *motor, float dt); // 更新单电流环：编码器角度+电流采样+PID+FOC数学
uint8_t Motor_Core_UpdateSpeedLoop(motor_t *motor, float dt);   // 更新单速度环：编码器速度+速度PID，输出iq_ref
uint8_t Motor_Core_UpdatePositionLoop(motor_t *motor, float dt); // 更新单位置环：机械角目标+位置PID，输出speed_ref_rad_s
uint8_t Motor_Core_UpdatePositionSpeedLoop(motor_t *motor, float dt); // 更新位置速度双环：位置环输出speed_ref，再执行速度环输出iq_ref
uint8_t Motor_Core_UpdateSpeedCurrentLoop(motor_t *motor, float dt); // 更新速度电流双环：速度环输出iq_ref，再执行电流环
uint8_t Motor_Core_UpdatePositionSpeedCurrentLoop(motor_t *motor, float dt); // 更新位置速度电流三环：位置->速度->电流->FOC
uint8_t Motor_Core_TryUpdateFromPwmRequest(uint8_t motor_id, float dt); // PWM同步请求到达后尝试更新指定电机高频控制
uint8_t Motor_Core_RunCurrentFastLoopDirectFromAdc(uint8_t motor_id, float dt); // 已确认启动后执行电流快环，不检查mode
uint8_t Motor_Core_RunCurrentFastLoopFromAdc(uint8_t motor_id, float dt); // ADC完成后执行指定电机快环：ADC采样+编码器快照+电流PID+FOC+CCR
uint8_t Motor_Core_RunMotor2CurrentFastLoopFromAdc(float dt); // ADC1完成后执行motor2固定快环：motor2/ADC1/encoder3/TIM20
uint8_t Motor_Core_RunSpeedOuterLoopFromTim7(uint8_t motor_id, float dt); // TIM7速度外环：只更新iq_ref，不执行电流环/FOC/PWM写入
uint8_t Motor_Core_RunPositionOuterLoopFromTim7(uint8_t motor_id, float dt); // TIM7位置外环：只更新speed_ref，不执行速度环/电流环/FOC

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CORE_H */
