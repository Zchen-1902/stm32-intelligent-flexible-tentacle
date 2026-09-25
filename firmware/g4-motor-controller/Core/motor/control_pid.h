#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * PID 控制模块
 *
 * 当前文件先建立控制层接口位置，后续用于电流环、速度环和位置环。
 * 第一版目标是保持模块独立，不直接依赖 HAL 外设。
 */

typedef enum
{
    CONTROL_PID_OK = 0,              // PID计算成功
    CONTROL_PID_PARAM_ERROR          // 输入指针或参数非法
} control_pid_status_t;

typedef struct
{
    float kp;                        // 比例系数
    float ki;                        // 积分系数
    float kd;                        // 微分系数

    float out_min;                   // 输出下限
    float out_max;                   // 输出上限
    float integral_min;              // 积分项下限
    float integral_max;              // 积分项上限
    float deadband;                  // 误差死区，abs(error)小于该值时按0处理

    uint8_t enable_integral;         // 积分使能：1使能，0关闭
    uint8_t enable_derivative;       // 微分使能：1使能，0关闭
    uint8_t enable_anti_windup;      // 简单抗积分饱和使能：1使能，0关闭
} control_pid_param_t;

typedef struct
{
    control_pid_param_t param;        // PID配置参数

    float ref;                       // 最近一次目标值
    float feedback;                  // 最近一次反馈值
    float error;                     // 当前误差
    float last_error;                // 上一次误差

    float proportional;              // 比例项
    float integral;                  // 积分项
    float derivative;                // 微分项
    float output;                    // 限幅后的输出
    float output_raw;                // 限幅前的输出

    uint8_t saturated;               // 输出是否触发限幅
    uint8_t initialized;             // PID是否已经初始化
} control_pid_t;

void Control_PID_SetDefaultParam(control_pid_param_t *param);                     // 填充一组安全默认参数
control_pid_status_t Control_PID_Init(control_pid_t *pid, const control_pid_param_t *param); // 初始化PID对象
void Control_PID_Reset(control_pid_t *pid);                                       // 清除积分、误差和输出状态
control_pid_status_t Control_PID_SetParam(control_pid_t *pid, const control_pid_param_t *param); // 更新PID参数

control_pid_status_t Control_PID_Update(control_pid_t *pid,
                                        float ref,
                                        float feedback,
                                        float dt,
                                        float *output);                           // 执行一次PID计算，dt单位s

float Control_PID_GetOutput(const control_pid_t *pid);                            // 获取最近一次输出
float Control_PID_GetIntegral(const control_pid_t *pid);                          // 获取当前积分项
uint8_t Control_PID_IsSaturated(const control_pid_t *pid);                        // 判断最近一次输出是否限幅

float Control_Ramp_Update(float current,
                          float target,
                          float rate_limit,
                          float dt);                                             // 目标斜坡限制，rate_limit单位为目标值单位/s

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_PID_H */
