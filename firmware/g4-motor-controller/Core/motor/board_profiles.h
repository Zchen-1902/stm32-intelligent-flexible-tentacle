#ifndef BOARD_PROFILES_H
#define BOARD_PROFILES_H

/*
 * G4驱动板独立参数。
 *
 * CAN_APP_BOARD_ID：
 *   决定当前G4在CAN总线中的命令和状态帧地址。
 *
 * MOTORx_MECH_FB_SIGN：
 *   三环位置和速度反馈方向。若方向错误，位置闭环会形成正反馈。
 *
 * MOTORx_FOC_ELEC_TRIM_RAD：
 *   每台电机FOC电角度微调，单位rad。
 *   方向正常但电机吸附、发热、扭矩异常时重点检查该参数。
 */

#if (G4_BOARD_PROFILE_ID == 1U)

/* 触手1：命令帧0x120，状态帧0x180。 */
#define CAN_APP_BOARD_ID                 (0U)

/* 触手1三台电机闭环反馈方向，使用当前板1已调通参数。 */
#define MOTOR1_MECH_FB_SIGN              ( 1.0f)
#define MOTOR2_MECH_FB_SIGN              (-1.0f)
#define MOTOR3_MECH_FB_SIGN              ( 1.0f)

/* 触手1三台电机FOC电角度微调。 */
#define MOTOR1_FOC_ELEC_TRIM_RAD         ( 0.5f * M_PI_F)
#define MOTOR2_FOC_ELEC_TRIM_RAD         ( 0.0f * M_PI_F)
#define MOTOR3_FOC_ELEC_TRIM_RAD         ( 0.5f * M_PI_F)

#elif (G4_BOARD_PROFILE_ID == 2U)

/* 触手2：命令帧0x121，状态帧0x181。 */
#define CAN_APP_BOARD_ID                 (1U)

/* 触手2三台电机闭环反馈方向，保留当前板2实测参数。 */
#define MOTOR1_MECH_FB_SIGN              ( 1.0f)
#define MOTOR2_MECH_FB_SIGN              (-1.0f)
#define MOTOR3_MECH_FB_SIGN              (-1.0f)

/* 触手2三台电机FOC电角度微调，保留当前板2实测参数。 */
#define MOTOR1_FOC_ELEC_TRIM_RAD         ( 0.5f * M_PI_F)
#define MOTOR2_FOC_ELEC_TRIM_RAD         (-0.5f * M_PI_F)
#define MOTOR3_FOC_ELEC_TRIM_RAD         ( 1.0f * M_PI_F)

#elif (G4_BOARD_PROFILE_ID == 3U)

/* 触手3：命令帧0x122，状态帧0x182。 */
#define CAN_APP_BOARD_ID                 (2U)

/* 板3电机3编码器安装方向与旧板相反，仅覆盖本板配置。 */
#undef ENCODER_DIR_M3
#define ENCODER_DIR_M3                   (-1)

/* 触手3尚未实物标定，暂时复制触手1参数。 */
#define MOTOR1_MECH_FB_SIGN              ( -1.0f)
#define MOTOR2_MECH_FB_SIGN              (-1.0f)
#define MOTOR3_MECH_FB_SIGN              ( -1.0f)

#define MOTOR1_FOC_ELEC_TRIM_RAD         ( 0.0f * M_PI_F)
#define MOTOR2_FOC_ELEC_TRIM_RAD         ( 0.0f * M_PI_F)
#define MOTOR3_FOC_ELEC_TRIM_RAD         ( 0.5f * M_PI_F)

#elif (G4_BOARD_PROFILE_ID == 4U)

/* 触手4：命令帧0x123，状态帧0x183。 */
#define CAN_APP_BOARD_ID                 (3U)

/* 触手4尚未实物标定，暂时复制触手1参数。 */
#define MOTOR1_MECH_FB_SIGN              ( 1.0f)
#define MOTOR2_MECH_FB_SIGN              ( 1.0f)
#define MOTOR3_MECH_FB_SIGN              ( -1.0f)

#define MOTOR1_FOC_ELEC_TRIM_RAD         ( 0.0f * M_PI_F)
#define MOTOR2_FOC_ELEC_TRIM_RAD         ( 0.5f * M_PI_F)
#define MOTOR3_FOC_ELEC_TRIM_RAD         ( 0.5f * M_PI_F)

#else
#error "G4_BOARD_PROFILE_ID must be 1, 2, 3 or 4"
#endif

#endif
