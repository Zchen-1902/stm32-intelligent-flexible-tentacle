#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * 板级参数选择：
 * 1~4分别对应触手1~4驱动板。
 * 每个配置只区分CAN节点号、闭环反馈方向和FOC电角度微调。
 * PID、编码器、机械传动和保护参数由四块板共同使用。
 */
#ifndef G4_BOARD_PROFILE_ID
#define G4_BOARD_PROFILE_ID 4U
#endif


/*****************************************无刷电机参数*****************************************

电机名称：BM3520H
额定电压 (Nominal voltage)		   7.4-36 V
空载电流 (No-load current)		   0.3 A
额定扭矩 (Nominal torque)		   0.1 N.M
最大转速 (Max speed)	    	   3500 rpm
堵转扭矩 (Stall torque)	    	   0.1 N.M
堵转电流 (Stall current)		   3.3 A
相间电阻 (Interphase resistance)    7.8Ω
相间电感 (Interphase inductance)    3.6mH
转速常数 (Speed constant)	       125 rpm/v
极对数 (Number of pole pairs)	   7
工作温度范围 (Working temperature)  -40~100 ℃
最高退磁温度 (Max demagnetize temperature)  150 ℃   


**************************************************************************************************/

//******************************************电机参数*******************************************/
#define MOTOR_COUNT                      3           // 电机数量
#define PHASE_COUNT_PER_MOTOR            3           // 每个电机3相
#define TOTAL_PHASE_CURRENT_CHANNELS     9           // 总共9路相电流采样

#define POLE_PAIRS                      11              // 4010电机24N22P，22极对应11极对数

#define MOTOR_KV_RPM_PER_V              45.0f           // 4010按Kt=0.21N.m/A估算Kv约45rpm/V，后续以实测修正

#define MOTOR_VOLTAGE_NOMINAL_V         24.0f           // 标称工作电压
#define MOTOR_VOLTAGE_MAX_V             24.0f           // 4010电机电压范围12~24V
#define MOTOR_SPEED_RATED_RPM           800.0f          // 4010初期保守速度估算，后续以实测空载速度修正
#define MOTOR_SPEED_MAX_RPM             1100.0f         // 4010按24V和Kv估算的最大机械转速上限

#define MOTOR_CURRENT_RATED_A           1.0f            // 4010初期保守额定电流
#define MOTOR_CURRENT_MAX_A             1.5f            // 4010初期验证最大工作电流
#define MOTOR_CURRENT_PEAK_A            2.0f            // 4010参数表最大瞬间电流2A

#define MOTOR_KT_NM_PER_A               0.21f           // 4010转矩常数，单位N.m/A，用于反电动势磁链估算
#define MOTOR_FLUX_WB                   (MOTOR_KT_NM_PER_A / (1.5f * (float)POLE_PAIRS)) // 永磁磁链估算值，单位Wb
#define MOTOR_BEMF_FF_ENABLE            1U              // q轴反电动势前馈使能：先只补vq，不做Ld/Lq交叉解耦
#define MOTOR_BEMF_FF_GAIN              -0.63f           // 反电动势前馈增益，初期只补20%，方向不对可改成负值验证
#define MOTOR_BEMF_FF_LIMIT_NORM        0.20f           // 反电动势前馈归一化限幅，避免速度估计异常时直接推飞
#define MOTOR_BEMF_FF_SPEED_MIN_RAD_S   2.0f            // 低速时不补偿，避免静止噪声引入vq偏置


//******************************************ADC参数*******************************************/
#define ADC_VREF_V                     3.3f            // ADC参考电压
#define ADC_RESOLUTION_BITS            12              // ADC分辨率
#define ADC_MAX_COUNT                  4095.0f         // 12bit ADC最大计数值

#define CURRENT_SHUNT_OHM              0.01f           // 电流采样分流电阻
#define CURRENT_AMP_GAIN               20.0f           // 电流采样放大倍数
#define CURRENT_ADC_MID_V              1.65f           // 双向电流采样中点偏置电压

#define CURRENT_ZERO_CALIB_SAMPLES     500             // 上电零点校准采样次数
#define CURRENT_ZERO_VALID_RANGE_COUNT_F 100.0f        // 零点漂移允许范围（浮点计数阈值）

#define M1_IA_ZERO_CORRECT_V            0.000f      // 电机1 A相零电流校正电压
#define M1_IB_ZERO_CORRECT_V            0.000f      // 电机1 B相零电流校正电压
#define M1_IC_ZERO_CORRECT_V            0.000f      // 电机1 C相零电流校正电压

#define M2_IA_ZERO_CORRECT_V            0.000f      // 电机2 A相零电流校正电压
#define M2_IB_ZERO_CORRECT_V            0.000f      // 电机2 B相零电流校正电压
#define M2_IC_ZERO_CORRECT_V            0.000f      // 电机2 C相零电流校正电压

#define M3_IA_ZERO_CORRECT_V            0.000f      // 电机3 A相零电流校正电压
#define M3_IB_ZERO_CORRECT_V            0.000f      // 电机3 B相零电流校正电压
#define M3_IC_ZERO_CORRECT_V            0.000f      // 电机3 C相零电流校正电压

//******************************************电流采样相序映射*******************************************/
// 说明：这里配置“FOC算法U/V/W三相电流”分别使用ADC采样Rank A/B/C中的哪一路。
// 默认ABC表示：ia=ADC_A，ib=ADC_B，ic=ADC_C。
// 若电流环中iq跟踪异常、id明显串入，可尝试ACB/BAC/BCA/CAB/CBA。
#define MOTOR_CURRENT_U_ADC_PHASE        ADC_APP_PHASE_A // FOC U相电流使用的ADC采样相
#define MOTOR_CURRENT_V_ADC_PHASE        ADC_APP_PHASE_B // FOC V相电流使用的ADC采样相
#define MOTOR_CURRENT_W_ADC_PHASE        ADC_APP_PHASE_C // FOC W相电流使用的ADC采样相

#define IDX_M1_IA                        0           // 电机1 A相电流索引
#define IDX_M1_IB                        1           // 电机1 B相电流索引
#define IDX_M1_IC                        2           // 电机1 C相电流索引

#define IDX_M2_IA                        3           // 电机2 A相电流索引
#define IDX_M2_IB                        4           // 电机2 B相电流索引
#define IDX_M2_IC                        5           // 电机2 C相电流索引

#define IDX_M3_IA                        6           // 电机3 A相电流索引
#define IDX_M3_IB                        7           // 电机3 B相电流索引
#define IDX_M3_IC                        8           // 电机3 C相电流索引
//******************************************数学基础宏定义*******************************************/

#define M_PI_F                  3.1415926f      // 圆周率
#define DEG2RAD(x)              ((x) * M_PI_F / 180.0f) // 度 -> 弧度
#define RAD2DEG(x)              ((x) * 180.0f / M_PI_F) // 弧度 -> 度

//*******************************************数学函数定义*****************************************/

#define MAX(x, y)                    (((x) > (y)) ? (x) : (y))   // 取x和y中的较大值
#define MIN(x, y)                    (((x) < (y)) ? (x) : (y))   // 取x和y中的较小值
#define LIMIT(x, low, high)          (((x) < (low)) ? (low) : (((x) > (high)) ? (high) : (x)))   // 把x限制在[low, high]范围内


//******************************************电机与线轮参数转换*******************************************/
#define REDUCTION_RATIO              10.0f       // 减速比：电机转10圈，线轮转1圈
#define GEARBOX_EFFICIENCY           0.80f       // 减速器效率：用于扭矩传递估算
#define SPOOL_RADIUS_M               0.012f      // 线轮半径：12mm = 0.012m
#define SPOOL_CIRCUMFERENCE_M        (2.0f * M_PI_F * SPOOL_RADIUS_M)   // 线轮一圈对应的绳长

#define MOTOR_TO_SPOOL_ANGLE(x)      ((x) / REDUCTION_RATIO)             // 电机侧角度 -> 线轮侧角度
#define SPOOL_TO_MOTOR_ANGLE(x)      ((x) * REDUCTION_RATIO)             // 线轮侧角度 -> 电机侧角度

#define MOTOR_TO_SPOOL_SPEED(x)      ((x) / REDUCTION_RATIO)             // 电机侧角速度/转速 -> 线轮侧角速度/转速
#define SPOOL_TO_MOTOR_SPEED(x)      ((x) * REDUCTION_RATIO)             // 线轮侧角速度/转速 -> 电机侧角速度/转速

#define SPOOL_ANGLE_TO_CABLE_LEN(x)  ((x) * SPOOL_RADIUS_M)              // 线轮转过x弧度 -> 绳长变化，x必须是弧度
#define CABLE_LEN_TO_SPOOL_ANGLE(x)  ((x) / SPOOL_RADIUS_M)              // 绳长变化 -> 线轮转角，结果是弧度

#define MOTOR_ANGLE_TO_CABLE_LEN(x)  (((x) / REDUCTION_RATIO) * SPOOL_RADIUS_M)  // 电机转角 -> 绳长变化，x必须是弧度
#define CABLE_LEN_TO_MOTOR_ANGLE(x)  (((x) / SPOOL_RADIUS_M) * REDUCTION_RATIO)  // 绳长变化 -> 电机转角，结果是弧度

#define MOTOR_TO_SPOOL_TORQUE(x)     ((x) * REDUCTION_RATIO * GEARBOX_EFFICIENCY) // 电机扭矩 -> 线轮扭矩，考虑减速器效率
#define SPOOL_TO_CABLE_FORCE(x)      ((x) / SPOOL_RADIUS_M)                       // 线轮扭矩 -> 绳子拉力
#define MOTOR_TO_CABLE_FORCE(x)      (((x) * REDUCTION_RATIO * GEARBOX_EFFICIENCY) / SPOOL_RADIUS_M) // 电机扭矩 -> 绳子拉力

//******************************************转速单位转换*******************************************/
#define RPM_TO_RAD_S(x)                 ((x) * 2.0f * M_PI_F / 60.0f)        // rpm -> rad/s
#define RAD_S_TO_RPM(x)                 ((x) * 60.0f / (2.0f * M_PI_F))      // rad/s -> rpm

//******************************************机械角度与电角度转换*******************************************/
#define MECH_TO_ELEC_ANGLE(x)           ((x) * POLE_PAIRS)             // 机械角度 -> 电角度
#define ELEC_TO_MECH_ANGLE(x)           ((x) / POLE_PAIRS)             // 电角度 -> 机械角度


//******************************************系统规模参数*******************************************/
#define MOTOR_NUM               3
#define TENTACLE_NUM            1
#define ENCODER_NUM             3
#define DRV_NUM                 3


// 卡尔曼滤波参数，要平滑 → 增大r、减小q；要响应 → 减小r、增大q；详见kalman_filter.c
#define KF_R 1.0f                // 测量噪声方差
#define KF_Q 0.001f                // 过程噪声方差


//******************************************环配置参数*******************************************/
#define PWM_FREQ 10000          //pwm1、2、3频率，Hz
#define SPEED_CALCU_FREQ 100	// TIM7频率配置值；CubeMX当前公式为100000/SPEED_CALCU_FREQ-1，实际中断频率为该值的10倍，当前为1kHz
#define TIMER_APP_CONTROL_TIM_CLK_HZ     100000U       // TIM3预分频后的计数频率，需与TIM3 Prescaler=1699保持一致
#define MOTOR_CONTROL_TASK_FREQ_HZ       1000U         // 电机控制任务调度频率，第一版默认1kHz
#define MOTOR_CONTROL_TASK_TIM_PERIOD    (TIMER_APP_CONTROL_TIM_CLK_HZ / MOTOR_CONTROL_TASK_FREQ_HZ - 1U) // TIM3自动重装载值
#define MOTOR_CONTROL_TASK_DT_S          (1.0f / (float)MOTOR_CONTROL_TASK_FREQ_HZ) // 控制任务周期，单位s
#define MOTOR_CONTROL_DT_MIN_S           (MOTOR_CONTROL_TASK_DT_S * 0.2f) // 控制实际dt下限，防止异常小dt影响微分/斜坡
#define MOTOR_CONTROL_DT_MAX_S           (MOTOR_CONTROL_TASK_DT_S * 5.0f) // 控制实际dt上限，防止长时间阻塞后积分突变

#define MOTOR_CURRENT_LOOP_FREQ_HZ      PWM_FREQ      // 电流环目标频率，默认跟随PWM频率
#define MOTOR_SPEED_LOOP_FREQ_HZ        1000U         // 速度环目标频率，当前TIM7实际为1kHz
#define MOTOR_POSITION_LOOP_FREQ_HZ     400U          // 位置环目标频率，默认200Hz

#define MOTOR_CURRENT_LOOP_DT_S         (1.0f / (float)MOTOR_CURRENT_LOOP_FREQ_HZ)  // 电流环周期，单位s
#define MOTOR_SPEED_LOOP_DT_S           (1.0f / (float)MOTOR_SPEED_LOOP_FREQ_HZ)    // 速度环周期，单位s
#define MOTOR_POSITION_LOOP_DT_S        (1.0f / (float)MOTOR_POSITION_LOOP_FREQ_HZ) // 位置环周期，单位s
#define MOTOR_CURRENT_LOOP_DT_MIN_S     (MOTOR_CURRENT_LOOP_DT_S * 0.2f) // 电流环实际dt下限，防止异常小dt影响PID
#define MOTOR_CURRENT_LOOP_DT_MAX_S     (MOTOR_CURRENT_LOOP_DT_S * 5.0f) // 电流环实际dt上限，防止偶发阻塞后积分突变

#define MOTOR_SPEED_LOOP_DIV            (MOTOR_CURRENT_LOOP_FREQ_HZ / MOTOR_SPEED_LOOP_FREQ_HZ)      // 速度环相对电流环分频
#define MOTOR_POSITION_LOOP_DIV         (MOTOR_CURRENT_LOOP_FREQ_HZ / MOTOR_POSITION_LOOP_FREQ_HZ)   // 位置环相对电流环分频

/* ===================== PID默认参数 ===================== */
#define ID_PID_KP_DEFAULT                 0.10f      // d轴电流环默认Kp
#define ID_PID_KI_DEFAULT                 10.018f      // d轴电流环默认Ki
#define ID_PID_KD_DEFAULT                 0.0000f       // d轴电流环默认Kd
#define ID_PID_INT_LIM_DEFAULT            0.476f       // d轴电流环积分限幅
#define ID_PID_OUT_LIM_NORM_DEFAULT       0.75f       // d轴电流环输出限幅（归一化）

#define IQ_PID_KP_DEFAULT                 0.30f      // q轴电流环默认Kp
#define IQ_PID_KI_DEFAULT                 24.0f      // q轴电流环默认Ki
#define IQ_PID_KD_DEFAULT                 0.000000f       // q轴电流环默认Kd
#define IQ_PID_INT_LIM_DEFAULT            1.20f       // q轴电流环积分限幅
#define IQ_PID_OUT_LIM_NORM_DEFAULT       1.75f       // q轴电流环输出限幅（归一化）

/* 电机3电流环临时独立参数：初始复制当前全局具体数值，后续只调M3 */
#define M3_ID_PID_KP_DEFAULT              0.10f
#define M3_ID_PID_KI_DEFAULT              10.018f
#define M3_ID_PID_KD_DEFAULT              0.0000f
#define M3_ID_PID_INT_LIM_DEFAULT         0.476f
#define M3_ID_PID_OUT_LIM_NORM_DEFAULT    0.75f

#define M3_IQ_PID_KP_DEFAULT              0.30f
#define M3_IQ_PID_KI_DEFAULT              24.0f
#define M3_IQ_PID_KD_DEFAULT              0.000000f
#define M3_IQ_PID_INT_LIM_DEFAULT         1.20f
#define M3_IQ_PID_OUT_LIM_NORM_DEFAULT    1.75f
#define CURRENT_PID_INTEGRAL_ENABLE_ERROR_A 0.12f    // 电流环积分分离阈值：误差超过该值时清积分并只用P/D快速拉回
#define CURRENT_PID_INTEGRAL_LEAK_FACTOR    0.998f    // 电流环积分分离泄放系数：误差较大时每周期保留90%已有积分

#define SPEED_PID_KP_DEFAULT              0.0696f      // 速度环默认Kp
#define SPEED_PID_KI_DEFAULT              0.224f      // 速度环默认Ki
#define SPEED_PID_KD_DEFAULT              0U       // 速度环默认Kd
#define SPEED_PID_INT_LIM_DEFAULT         1.37f       // 速度环积分限幅
#define SPEED_PID_OUT_LIM_A_DEFAULT       1.75f      // 速度环输出限幅（输出到iq_ref，单位A，当前空载联调先保守）
#define SPEED_IQ_REF_LPF_ALPHA            0.10f      // 速度环输出iq_ref一阶低通系数，越小越平滑但响应越慢
#define SPEED_PID_INTEGRAL_ENABLE_ERROR_RAD_S 5.4f   // 速度误差小于该值才允许积分，避免大误差时积分堆积
#define SPEED_PID_LARGE_ERROR_KI_SCALE    0.20f      // 速度误差大于积分阈值时Ki降额比例，避免积分关闭后负载区卡死
#define SPEED_PID_INTEGRAL_REVERSE_LEAK_FACTOR 0.985f // 速度误差与积分方向相反时的积分泄放系数，越小泄放越快
#define SPEED_PID_INTEGRAL_REVERSE_MIN_A  0.25f      // 积分绝对值小于该值时不做反向泄放，避免过零附近抖动
#define SPEED_PID_KP_MIN_SCALE            0.25f      // 速度误差接近0时P项最小比例，降低目标附近P项来回拉扯
#define SPEED_PID_KP_FULL_ERROR_RAD_S     6.0f       // 速度误差达到该值后使用完整Kp，保证远离目标时响应速度

#define POS_PID_KP_DEFAULT                12.532f       // 位置环默认Kp
#define POS_PID_KI_DEFAULT                0.320f       // 位置环默认Ki
#define POS_PID_KD_DEFAULT                0.00f       // 位置环默认Kd
#define POS_PID_INT_LIM_DEFAULT           26.0f       // 位置环积分限幅
#define POS_PID_OUT_LIM_RAD_S_DEFAULT     78.0f       // 位置环输出限幅（输出到speed_ref，单位rad/s）
#define POS_APPROACH_ENTER_RAD            2.0f        // 位置误差小于该值时退出远距离巡航，进入近距离位置环
#define POS_APPROACH_EXIT_RAD             2.5f        // 位置误差大于该值时进入远距离巡航，形成滞回
#define POS_APPROACH_SPEED_RAD_S          25.0f       // 远距离位置移动阶段的恒定速度目标
#define POS_SPEED_REF_ACCEL_LIMIT_RAD_S2  120.0f      // 非锁定区位置环输出到速度环的速度目标斜坡加速度限制
#define POS_LOCK_SPEED_REF_ACCEL_LIMIT_RAD_S2 330.0f  // 锁定区速度目标斜坡加速度限制，避免锁定区与非锁定区切换突变
#define POS_VEL_DAMPING_GAIN_DEFAULT      0.8f       // 位置环速度阻尼系数：speed_ref -= gain * measured_speed
#define POS_LOCK_ENTER_RAD                0.16f       // 进入位置锁定区阈值，进入后不清iq_ref，而是保持伺服刚度
#define POS_LOCK_EXIT_RAD                 0.24f       // 退出位置锁定区阈值，必须大于进入阈值形成滞回
#define POS_LOCK_KP_DEFAULT               8.0f        // 锁定区位置刚度：小误差转换为速度目标
#define POS_LOCK_KD_DEFAULT               0.8f        // 锁定区速度阻尼：抑制到位附近被推开或来回晃动
#define POS_LOCK_SPEED_LIM_RAD_S          6.0f        // 锁定区最大速度目标，避免小范围保持时输出过猛
#define POS_LOCK_SPEED_I_REVERSE_LEAK_FACTOR 0.90f   // 锁定区速度积分方向错误时的泄放系数
#define POS_LOCK_SPEED_I_CENTER_LEAK_RAD  0.02f       // 接近目标中心时轻微泄放速度积分的误差阈值
#define POS_LOCK_SPEED_I_CENTER_LEAK_FACTOR 0.98f    // 接近目标中心时速度积分轻微泄放系数

/* ===================== 参考值与输出限幅 ===================== */
#define MOTOR_ID_REF_MAX_A                1.0f       // d轴电流目标限幅
#define MOTOR_IQ_REF_MAX_A                1.5f       // 4010初期q轴电流目标限幅，先低于2A瞬时上限

#define MOTOR_SPEED_REF_MAX_RAD_S         RPM_TO_RAD_S(1000.0f) // 4010初期最大目标机械角速度

#define MOTOR_CABLE_LEN_REF_MAX_M         0.20f      // 最大目标绳长
#define MOTOR_CABLE_LEN_REF_MIN_M         0.0f       // 最小目标绳长

#define PWM_DUTY_MAX                      0.92f      // PWM占空比上限，调试阶段收窄以保证低侧采样窗口
#define PWM_DUTY_MIN                      0.08f      // PWM占空比下限，调试阶段收窄以保证低侧采样窗口

/* ===================== 误差死区（防抖/防震荡） ===================== */
#define CURRENT_DEADBAND_A                0.008f      // 电流误差死区，小于该值可认为已到位
#define SPEED_DEADBAND_RAD_S              0.6f       // 速度误差死区，单位rad/s
#define POS_DEADBAND_RAD                  0.0f       // 位置环不再使用控制死区；到位附近改用POS_LOCK_*锁定区
#define CABLE_LEN_DEADBAND_M              0.001f     // 绳长误差死区，约1mm

/* ===================== 启动与模式切换限制 ===================== */
#define MOTOR_ENABLE_DELAY_MS             50         // 电机使能后延时
#define MOTOR_STARTUP_RAMP_TIME_MS        300        // 启动斜坡时间
#define MOTOR_SPEED_START_LIMIT_RAD_S     RPM_TO_RAD_S(500.0f) // 启动阶段最大角速度限制（rad/s）
#define MOTOR_IQ_START_LIMIT_A            0.8f       // 启动阶段最大q轴电流限制

/* ===================== 电流保护参数 ===================== */
#define MOTOR_OVER_CURRENT_A              1.8f       // 4010初期软件过流阈值，低于2A瞬时上限
#define MOTOR_OVER_CURRENT_COUNT_LIMIT    5          // 连续过流计数阈值
#define PHASE_CURRENT_SUM_ERROR_MAX_A     0.3f       // 三相电流和允许误差，用于检查采样异常

/* ===================== 温度保护参数 ===================== */
#define TEMP_PROTECT_C                    75.0f      // 过温预警阈值
#define TEMP_SHUTDOWN_C                   85.0f      // 过温关断阈值
#define TEMP_RECOVER_C                    65.0f      // 过温恢复阈值

/* ===================== 编码器与通信保护参数 ===================== */
#define ENCODER_TIMEOUT_MS                20         // 编码器读取超时阈值
#define ENCODER_ANGLE_JUMP_MAX_RAD        1.5f       // 单次角度跳变最大允许值
#define ENCODER_INVALID_COUNT_LIMIT       10         // 编码器异常连续计数阈值

#define CAN_CMD_TIMEOUT_MS                100        // CAN命令超时阈值
#define CAN_HEARTBEAT_TIMEOUT_MS          300        // 心跳超时阈值

/* ===================== 堵转/失速保护参数 ===================== */
#define STALL_SPEED_MIN_RAD_S             1.0f       // 低于该速度可认为接近堵转
#define STALL_CURRENT_THRESHOLD_A         1.7f       // 堵转保护电流阈值，接近速度环1.75A限幅时才判定堵转
#define STALL_POS_ERROR_MIN_RAD           2.0f       // 位置误差大于该值才允许判定堵转
#define STALL_MOVE_MIN_RAD                0.35f      // 检测窗口内实际位移小于该值才认为没有有效运动
#define STALL_CHECK_COUNT_LIMIT           160U       // 堵转检测位置环累计次数，400Hz下约400ms

/* ===================== 采样与滤波参数 ===================== */
#define CURRENT_FILTER_ALPHA              0.7f       // 电流一阶滤波系数
#define SPEED_FILTER_ALPHA                0.2f       // 速度一阶滤波系数
#define POSITION_FILTER_ALPHA             0.2f       // 位置一阶滤波系数
#define TEMP_FILTER_ALPHA                 0.1f       // 温度一阶滤波系数

/* ===================== 故障恢复参数 ===================== */
#define FAULT_RECOVER_DELAY_MS            1000       // 故障后最短恢复等待时间
#define FAULT_AUTO_RECOVER_ENABLE         0          // 是否允许自动恢复：0关闭，1开启


//******************************************编码器参数*******************************************/
#define ENCODER_RESOLUTION_BITS        14                 // 编码器分辨率
#define ENCODER_COUNT_PER_TURN         16384.0f           // 单圈计数
#define ENCODER_MAX_COUNT              16383.0f           // 单圈最大计数
#define ENCODER_DIR_M1                 1                  // 电机1编码器方向
#define ENCODER_DIR_M2                 1                  // 电机2编码器方向
#define ENCODER_DIR_M3                 -1                  // 电机3编码器方向
#define ENCODER_ZERO_OFFSET_M1_RAD     0.0f               // 电机1零位偏置
#define ENCODER_ZERO_OFFSET_M2_RAD     0.0f               // 电机2零位偏置
#define ENCODER_ZERO_OFFSET_M3_RAD     0.0f               // 电机3零位偏置
#define ENCODER_MULTI_TURN_ENABLE      1                  // 编码器多圈计数使能：1使能，0关闭
#define ENCODER_MULTI_TURN_MAX_ABS     1000               // 多圈计数绝对值上限（圈）
#define ENCODER_SPEED_LPF_ALPHA        0.80f              // 速度环平均速度一阶低通系数；平均速度已降噪，可比单点差分更轻
#define ENCODER_FAST_SPEED_RAW_LIMIT_RAD_S  2000.0f       // SPI相邻有效样本差分速度限幅，防止单帧毛刺污染速度
#define FOC_ELEC_ANGLE_PREDICT_ENABLE        1U           // 电流快环使用编码器样本时间戳和speed_raw预测当前电角度
#define FOC_ELEC_ANGLE_PREDICT_MAX_AGE_S     0.0415f      // 编码器样本最大可预测时间，超出认为缓存过旧
#define FOC_ELEC_ANGLE_PREDICT_EXTRA_DELAY_S 0.0f         // 额外固定相位补偿，初始为0，后续按id抖动方向微调


/*
 * 板级配置放在文件末尾，使其能够覆盖本文件和motor_core.c中的默认值。
 * 四块板必须选择不同的G4_BOARD_PROFILE_ID，避免CAN地址冲突。
 */
#include "board_profiles.h"












#endif
