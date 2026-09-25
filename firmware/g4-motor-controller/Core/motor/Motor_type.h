#ifndef __MOTOR_TYPE_H
#define __MOTOR_TYPE_H

#include <stdint.h>
#include "control_pid.h"



//******************************************电机模式*******************************************/
typedef enum
{
    MOTOR_MODE_IDLE = 0,              // 空闲模式，不执行控制环
    MOTOR_MODE_CURRENT,               // 单电流环：id/iq -> vd/vq -> FOC
    MOTOR_MODE_SPEED,                 // 单速度环：speed_ref -> iq_ref，不执行电流环
    MOTOR_MODE_POSITION,              // 单位置环：position_ref -> speed_ref，不执行速度/电流环
    MOTOR_MODE_SPEED_CURRENT,         // 速度 + 电流双环：speed_ref -> iq_ref -> vd/vq -> FOC
    MOTOR_MODE_POSITION_SPEED,        // 位置 + 速度双环：position_ref -> speed_ref -> iq_ref
    MOTOR_MODE_POSITION_SPEED_CURRENT // 位置 + 速度 + 电流三环
} motor_mode_t;

//******************************************电机运行状态*******************************************/
typedef enum
{
    MOTOR_STATE_OFF = 0,          // 未使能
    MOTOR_STATE_INIT,             // 初始化
    MOTOR_STATE_CALIB,            // 校准
    MOTOR_STATE_READY,            // 就绪
    MOTOR_STATE_RUN,              // 运行中
    MOTOR_STATE_FAULT             // 故障
} motor_run_state_t;

//******************************************三相电流*******************************************/
typedef struct
{
    uint16_t ia_raw;              // A相电流ADC原始值
    uint16_t ib_raw;              // B相电流ADC原始值
    uint16_t ic_raw;              // C相电流ADC原始值
    uint32_t adc_update_count;    // 已消费的ADC稳定帧计数，用于判断是否有新三相采样

    float ia;                     // A相电流，单位A
    float ib;                     // B相电流，单位A
    float ic;                     // C相电流，单位A

    float i_sum;                  // 三相电流和，正常应接近0

    float i_alpha;                // Clarke变换 alpha 轴电流
    float i_beta;                 // Clarke变换 beta 轴电流

    float id;                     // d轴电流
    float iq;                     // q轴电流
} motor_current_t;

//******************************************编码器预留状态*******************************************/
/*
 * motor_encoder_t 为电机对象预留的编码器状态结构。
 * 当前阶段编码器主数据源为 spi_encoder 模块，控制代码可直接通过 SPI_Encoder_Get_xxx() 读取；
 * 后续如果需要统一状态上报、故障保护或 motor_core 封装，再把有效结果同步到本结构体。
 */
typedef struct
{
    uint16_t raw_count;           // 编码器原始计数
    uint16_t raw_count_last;      // 上一次原始计数

    int32_t turn_count;           // 多圈计数（跨零点时+1/-1）

    float mech_angle_raw;         // 原始机械角，单位rad
    float mech_angle;             // 校正后的机械角，单位rad
    float mech_angle_total_rad;   // 多圈机械角，单位rad（连续角）
    float mech_angle_last;        // 上一次机械角

    float elec_angle;             // 电角度，单位rad
    float elec_angle_last;        // 上一次电角度

    float delta_angle;            // 单次角度变化量，单位rad
    float speed_rad_s;            // 机械角速度，单位rad/s
    float speed_rpm;              // 机械转速，单位rpm

    int8_t dir;                   // 编码器方向：1或-1
    uint8_t valid;                // 当前数据是否有效
    uint16_t invalid_count;       // 连续异常计数
} motor_encoder_t;

//******************************************电机-线轮-绳长相关量*******************************************/
typedef struct
{
    float motor_angle;            // 电机机械角，单位rad
    float motor_speed;            // 电机机械角速度，单位rad/s

    float spool_angle;            // 线轮角度，单位rad
    float spool_speed;            // 线轮角速度，单位rad/s

    float cable_length;           // 绳长变化量，单位m
    float cable_speed;            // 绳长变化速度，单位m/s
} motor_motion_t;

//******************************************参考目标值*******************************************/
/*
 * motor_ref_t 保存外部命令或轨迹规划给出的最终目标值。
 * control_pid_t.ref 是某一次 PID 计算实际使用的目标值，可能已经经过 ramp、限幅或模式切换处理；
 * 因此 motor.ref.xxx 和 pid.ref 不是同一个层级，不建议混用。
 */
typedef struct
{
    float id_ref;                 // d轴电流目标
    float iq_ref;                 // q轴电流目标
    float iq_ref_ramp;            // 斜坡后的q轴电流目标，实际送入电流环

    float speed_ref_rad_s;        // 目标机械角速度，单位rad/s
    float speed_ref_ramp_rad_s;   // 斜坡后的机械角速度目标，实际送入速度环
    float speed_ref_rpm;          // 目标机械转速，单位rpm

    float mech_angle_ref;         // 目标机械角，单位rad
    float mech_angle_ref_ramp;    // 斜坡后的机械角目标，实际送入位置环或轨迹接口
    float spool_angle_ref;        // 目标线轮角，单位rad
    float cable_length_ref;       // 目标绳长，单位m
} motor_ref_t;

//******************************************控制器相关*******************************************/
typedef struct
{
    control_pid_t id_pid;         // d轴电流环PID对象
    control_pid_t iq_pid;         // q轴电流环PID对象
    control_pid_t speed_pid;      // 速度环PID对象
    control_pid_t pos_pid;        // 位置环PID对象

    float vd_norm;                // d轴归一化电压指令，电流环PID输出
    float vq_norm;                // q轴归一化电压指令，电流环PID输出

    float v_alpha_norm;           // alpha轴归一化电压，逆Park输出
    float v_beta_norm;            // beta轴归一化电压，逆Park输出

    float duty_u;                 // U相PWM占空比
    float duty_v;                 // V相PWM占空比
    float duty_w;                 // W相PWM占空比

    uint16_t speed_loop_div_cnt;   // 速度环分频计数器，用于低于电流环频率运行
    uint16_t pos_loop_div_cnt;     // 位置环分频计数器，用于低于电流环频率运行
} motor_control_t;

//******************************************校准与个体参数*******************************************/
typedef struct
{
    float ia_zero_offset_v;       // A相零漂校正电压
    float ib_zero_offset_v;       // B相零漂校正电压
    float ic_zero_offset_v;       // C相零漂校正电压

    float encoder_zero_offset;    // 编码器零位偏置，单位rad
    float foc_elec_offset_rad;     // FOC电角度零位偏置，单位rad，只用于电角度换相，不用于机械零点
    float current_limit;          // 当前电机单独限流值，单位A
    float temp_limit;             // 当前电机单独温度限值
} motor_calib_t;

//******************************************保护与故障*******************************************/
typedef struct
{
    uint8_t over_current;         // 过流故障
    uint8_t over_temp;            // 过温故障
    uint8_t encoder_fault;        // 编码器故障
    uint8_t drv_fault;            // 驱动器故障
    uint8_t stall_fault;          // 堵转故障
    uint8_t comm_fault;           // 通信故障

    uint8_t warning;              // 预警标志
    uint8_t fault_active;         // 总故障标志

    uint16_t over_current_count;  // 过流连续计数
    uint16_t encoder_err_count;   // 编码器异常计数
    uint16_t stall_count;         // 堵转计数
} motor_protect_t;

//******************************************单个电机总对象*******************************************/
typedef struct
{
    uint8_t motor_id;             // 电机编号：0/1/2

    uint8_t current_adc_id;       // 快环绑定的ADC组ID：motor0=ADC4，motor1=ADC3，motor2=ADC1
    uint8_t encoder_id;           // 快环绑定的编码器ID：motor0/1/2 -> encoder1/2/3

    uint32_t pwm_arr;             // 快环PWM定时器ARR缓存
    uint32_t adc_guard_ccr;       // 动态ADC触发保护边界，单位TIM计数
    volatile uint32_t *pwm_ccr_u; // U相PWM CCR寄存器指针
    volatile uint32_t *pwm_ccr_v; // V相PWM CCR寄存器指针
    volatile uint32_t *pwm_ccr_w; // W相PWM CCR寄存器指针
    volatile uint32_t *adc_trigger_ccr; // ADC动态触发CCR寄存器指针
    uint32_t pwm_ccr_u_last;      // 最近一次快环写入U相PWM的CCR值，供ADC采样点计算复用
    uint32_t pwm_ccr_v_last;      // 最近一次快环写入V相PWM的CCR值，供ADC采样点计算复用
    uint32_t pwm_ccr_w_last;      // 最近一次快环写入W相PWM的CCR值，供ADC采样点计算复用

    motor_mode_t mode;            // 当前控制模式
    motor_run_state_t run_state;  // 当前运行状态

    uint8_t enable_cmd;           // 使能命令
    uint8_t start_cmd;            // 启动命令

    motor_current_t current;      // 电流采样与换算结果
    motor_encoder_t encoder;      // 编码器预留状态；当前主数据源仍为spi_encoder模块
    motor_motion_t motion;        // 机械/线轮/绳长量
    motor_ref_t ref;              // 外部/轨迹规划目标值，不等同于pid.ref
    motor_control_t control;      // 控制器参数与状态
    motor_calib_t calib;          // 校准与个体参数
    motor_protect_t protect;      // 保护与故障
} motor_t;



#endif      
