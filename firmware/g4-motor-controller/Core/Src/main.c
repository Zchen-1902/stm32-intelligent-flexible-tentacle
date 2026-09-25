/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Vofa_debug/Vofa_debug.h"
  #include "../timer_app/timer_app.h"
  #include "../adc_app/adc_app.h"
  #include "../spi_encoder/spi_encoder.h"
  #include "../drv8323S_drive/drv_drive.h"
  #include "../motor/motor_core.h"
  #include "../motor/Config.h"
  #include "../can_app/can_app.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_FOC_ALIGN_VD_NORM        0.13f  // FOC电角度对齐d轴电压，首次真实测试建议0.01~0.02
#define MAIN_FOC_ROTATE_VQ_NORM       0.1f  // FOC开环旋转q轴电压，当前降低力度用于观察震动
#define MAIN_CURRENT_TEST_ID_REF_A    0.0f   // 电流环测试d轴电流目标，表贴电机第一版固定为0
#define MAIN_CURRENT_TEST_IQ_REF_A    0.2f   // 4010新电机相序验证用q轴电流目标，先低电流确认方向和采样相序
#define MAIN_SPEED_TEST_REF_RAD_S     20.0f  // KEY2速度环测试低速目标，单位rad/s
#define MAIN_SPEED_TEST_REF_HIGH_RAD_S 30.0f // KEY2速度环测试高速目标，单位rad/s
#define MAIN_POSITION_TEST_DELTA_RAD  5.0f  // KEY2位置环测试目标：当前位置基础上增加的机械角，单位rad
#define MAIN_KEY2_ADD_CURRENT_TEST_ENABLE 1U    // KEY2逐步追加电流环测试：motor0->+motor1->+motor2->全停
#define MAIN_KEY2_SINGLE_TEST_MOTOR_ID 2U       // KEY2单电机测试选择：0/1/2对应电机1/2/3
#define MAIN_KEY2_POSITION_TEST_ENABLE 1U    // KEY2测试模式：换4010后先关闭位置环，只验证电流环/相序
#define MAIN_KEY2_SPEED_TEST_ENABLE   1U     // KEY2测试模式：换4010后先关闭速度环，只验证单电流环
#define MAIN_AUTO_FOC_CALIB_ROUNDS    2U     // 上电后三电机FOC零偏自动顺序校正轮数
#define MAIN_ENCODER_DMA_ADC_DIV      2U     // 各电机ADC回调中编码器SPI DMA请求分频：10k/2=5kHz
#define MAIN_TIM7_ACTUAL_FREQ_HZ      ((uint32_t)SPEED_CALCU_FREQ * 10U) // TIM7实际中断频率，需与Config.h注释保持一致
#define MAIN_SPEED_LOOP_TIM7_DIV      (MAIN_TIM7_ACTUAL_FREQ_HZ / MOTOR_SPEED_LOOP_FREQ_HZ) // TIM7到速度环的分频
#define MAIN_POSITION_LOOP_TIM7_DIV   (MAIN_TIM7_ACTUAL_FREQ_HZ / MOTOR_POSITION_LOOP_FREQ_HZ) // TIM7到位置环的分频
#define MAIN_CAN_STATUS_FREQ_HZ       20U     // 运动期间G4状态帧上报频率
#define MAIN_CAN_STATUS_TIM7_DIV      (MAIN_TIM7_ACTUAL_FREQ_HZ / MAIN_CAN_STATUS_FREQ_HZ)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t s_last_motor_control_cycles[MOTOR_COUNT] = {0U}; // 每个电机上一次实际控制执行时的DWT周期计数
static uint8_t s_motor_control_dt_valid[MOTOR_COUNT] = {0U};     // 每个电机控制dt首次运行标志
static uint32_t s_current_fast_loop_last_cycles[MOTOR_COUNT] = {0U}; // 每个电机ADC快环上一次进入时的DWT周期计数
static uint8_t s_current_fast_loop_dt_valid[MOTOR_COUNT] = {0U};     // 每个电机ADC快环真实dt首次运行标志
static uint8_t s_selected_motor_id = 2U;                         // 当前按键选中的电机编号，当前调试TIM20/DRV3链路
static uint8_t s_selected_motor_pwm_hw_started = 0U;             // 当前选中电机PWM硬件启动状态，1表示已调用Start
static uint8_t s_selected_motor_align_active = 0U;               // 当前选中电机是否处于FOC电角度校准对齐状态
static uint8_t s_selected_motor_rotate_active = 0U;              // 当前选中电机是否处于q轴开环旋转状态
static uint8_t s_selected_motor_current_active = 0U;             // 当前选中电机是否处于单电流环测试状态
static uint8_t s_key2_add_current_next_motor_index = 0U;         // KEY2逐步追加测试下一路待启动电机序号
static uint8_t s_key2_add_current_active_mask = 0U;              // KEY2逐步追加测试已启动电机位图，bit0/1/2对应motor0/1/2
static uint8_t s_key1_calib_seq_active = 0U;                     // KEY1三电机顺序校正是否进行中
static uint8_t s_key1_calib_seq_next_motor = 0U;                 // KEY1三电机顺序校正下一台电机序号
static uint8_t s_key1_calib_seq_started = 0U;                    // KEY1顺序校正是否已经成功启动过至少一台
static uint8_t s_key1_calib_seq_round = 0U;                      // 三电机顺序校正当前轮次
static uint8_t s_key2_speed_test_step = 1U;                      // KEY2速度环测试步进：0停机/待启动，1下次35，2下次20，3下次停机

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float Main_GetCurrentFastLoopDt(uint8_t motor_id); // 计算指定电机ADC快环真实dt
static uint8_t Main_StartNextMotorCurrentTest(void);      // KEY2逐步追加电流环测试启动下一路
static void Main_StopAllMotorCurrentTest(void);           // KEY2逐步追加电流环测试停止全部电机
static uint8_t Main_StartKey1CalibOne(uint8_t motor_id);   // KEY1顺序校正启动单台电机
static void Main_ServiceKey1CalibSequence(void);           // KEY1顺序校正推进器

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM20_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  VOFA_Debug_Init();
  VOFA_Debug_SetTopic(VOFA_TOPIC_ENCODER);//默认查看电机2编码器主题，按KEY3可切换到FOC_TEST
  VOFA_Debug_SetPeriodMs(5U);             // VOFA发送周期5ms，约200Hz；USART1当前设置为115200
  if (DRV_Drive_Init() != HAL_OK)
  {
    Error_Handler();
  }
  Motor_Core_Init();                      // 初始化三电机软件对象和默认PID/校准参数
  Timer_App_Init();
  SPI_Encoder_Init();
  Motor_Core_ApplyEncoderOffsetConfig();  // 将Config.h中的编码器零点偏移同步到spi_encoder，避免motor_core重复扣零点
  if (CAN_App_Init() != HAL_OK)
  {
    Error_Handler();
  }
  Timer_App_Start();
  ADC_App_Init();
  (void)ADC_App_CalibrateMotorAdc();       // STM32 ADC内部自校准，必须在启动ADC DMA之前执行
  (void)ADC_App_StartMotorCurrentDma();
  if (Timer_App_StartMotorControlTimers() != HAL_OK)
  {
    Error_Handler();
  }
  (void)Motor_Core_CalibrateCurrentZero(); // 临时排查阶段：校准失败也继续运行，避免串口/VOFA被Error_Handler阻断
  (void)Motor_Core_TrimCurrentZeroResidual(); // CAL退出后再按真实采样链路微调零点残差

  s_key1_calib_seq_active = 1U;
  s_key1_calib_seq_next_motor = 0U;
  s_key1_calib_seq_started = 0U;
  s_key1_calib_seq_round = 0U;
  if (Main_StartKey1CalibOne(0U) == 0U)
  {
    s_key1_calib_seq_active = 0U;
    s_key1_calib_seq_started = 0U;
    s_key1_calib_seq_round = 0U;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //    uint8_t motor_id;

    VOFA_Debug_Task(); // VOFA内部按period_ms和DMA busy限速，放主循环可突破TIM4 5ms调用限制
    CAN_App_Task();
    Motor_Core_FocOffsetAutoCalibTask(); // KEY1启动后自动完成FOC零偏校准
    Main_ServiceKey1CalibSequence();      // KEY1三电机顺序校正推进器

      if (Timer_App_IsTaskReady(TIMER_APP_TASK_5MS) != 0U)
      {
        Timer_App_ClearTaskFlag(TIMER_APP_TASK_5MS);
      }

    if (Timer_App_IsTaskReady(TIMER_APP_TASK_SPEED_CALC) != 0U)
    {
      Timer_App_ClearTaskFlag(TIMER_APP_TASK_SPEED_CALC);
    }
/*
    for (motor_id = 0U; motor_id < Motor_Core_GetMotorCount(); motor_id++)
    {
      if (Timer_App_ConsumeMotorControlRequest(motor_id) != 0U)
      {
        uint32_t now_cycles;
        uint32_t delta_cycles;
        float control_dt_s;

          now_cycles = Timer_App_GetRuntimeCycles();
          if (s_motor_control_dt_valid[motor_id] == 0U)
          {
            control_dt_s = MOTOR_CURRENT_LOOP_DT_S;
            s_motor_control_dt_valid[motor_id] = 1U;
          }
          else
          {
            delta_cycles = now_cycles - s_last_motor_control_cycles[motor_id];
            control_dt_s = (float)delta_cycles / (float)SystemCoreClock;

            if (control_dt_s < MOTOR_CURRENT_LOOP_DT_MIN_S)
            {
              control_dt_s = MOTOR_CURRENT_LOOP_DT_MIN_S;
            }
            else if (control_dt_s > MOTOR_CURRENT_LOOP_DT_MAX_S)
            {
              control_dt_s = MOTOR_CURRENT_LOOP_DT_MAX_S;
            }
          }

        s_last_motor_control_cycles[motor_id] = now_cycles;

        if ((s_selected_motor_rotate_active != 0U) && (motor_id == s_selected_motor_id))
        {
          (void)Motor_Core_OutputFocQVoltage(s_selected_motor_id, MAIN_FOC_ROTATE_VQ_NORM);
          (void)Motor_Core_UpdateCurrentObserve(s_selected_motor_id); // 开环旋转时同步刷新电流观测量，供PHASE_CURRENT主题显示
        }
        else
        {
          (void)Motor_Core_TryUpdateFromPwmRequest(motor_id, control_dt_s);
        }
      }
    }
	*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  计算指定电机ADC快环真实dt。
  * @param  motor_id: 电机编号，范围0~MOTOR_COUNT-1。
  * @retval 当前ADC快环dt，单位s。
  * @note   dt来源为DWT cycle差值，每个电机独立记录last cycle。
  *         第一次进入时使用默认MOTOR_CURRENT_LOOP_DT_S，后续做上下限保护。
  *         该dt表示两次ADC回调中实际进入快环入口的时间差，不等同于ADC硬件采样瞬间。
  */
static float Main_GetCurrentFastLoopDt(uint8_t motor_id)
{
  uint32_t now_cycles;
  uint32_t delta_cycles;
  float dt_s;

  if (motor_id >= MOTOR_COUNT)
  {
    return MOTOR_CURRENT_LOOP_DT_S;
  }

  now_cycles = Timer_App_GetRuntimeCycles();
  if (s_current_fast_loop_dt_valid[motor_id] == 0U)
  {
    dt_s = MOTOR_CURRENT_LOOP_DT_S;
    s_current_fast_loop_dt_valid[motor_id] = 1U;
  }
  else
  {
    delta_cycles = now_cycles - s_current_fast_loop_last_cycles[motor_id];
    dt_s = (float)delta_cycles / (float)SystemCoreClock;

    if (dt_s < MOTOR_CURRENT_LOOP_DT_MIN_S)
    {
      dt_s = MOTOR_CURRENT_LOOP_DT_MIN_S;
    }
    else if (dt_s > MOTOR_CURRENT_LOOP_DT_MAX_S)
    {
      dt_s = MOTOR_CURRENT_LOOP_DT_MAX_S;
    }
  }

  s_current_fast_loop_last_cycles[motor_id] = now_cycles;
  return dt_s;
}

/**
  * @brief  KEY2启动指定电机的位置-速度-电流三环测试。
  * @retval 1表示本次成功启动一路，0表示启动失败。
  * @note   目标为当前反馈位置增加MAIN_POSITION_TEST_DELTA_RAD，只启动选中电机。
  */
static uint8_t Main_StartNextMotorCurrentTest(void)
{
  uint8_t motor_id = MAIN_KEY2_SINGLE_TEST_MOTOR_ID;

  if (motor_id >= MOTOR_COUNT)
  {
    return 0U;
  }

  (void)Motor_Core_OutputNeutralPwm(motor_id);

  if (Motor_Core_IsPwmStartAllowed(motor_id) == 0U)
  {
    Main_StopAllMotorCurrentTest();
    return 0U;
  }

  if (Motor_Core_StartPwmOutput(motor_id) != HAL_OK)
  {
    Main_StopAllMotorCurrentTest();
    return 0U;
  }

  Motor_Core_SetCurrentRef(motor_id, MAIN_CURRENT_TEST_ID_REF_A, 0.0f);
  Motor_Core_SetSpeedRef(motor_id, 0.0f);
  Motor_Core_SetPositionRef(motor_id,
                            Motor_Core_GetPositionFeedbackRad(motor_id)
                            + ((motor_id == 0U) ? -MAIN_POSITION_TEST_DELTA_RAD
                                               : MAIN_POSITION_TEST_DELTA_RAD));
  Motor_Core_SetMode(motor_id, MOTOR_MODE_POSITION_SPEED_CURRENT);

  s_selected_motor_id = motor_id;
  s_selected_motor_pwm_hw_started = 1U;
  s_selected_motor_align_active = 0U;
  s_selected_motor_rotate_active = 0U;
  s_selected_motor_current_active = 1U;
  s_key2_add_current_active_mask = (uint8_t)(1U << motor_id);
  s_key2_speed_test_step = 0U;

  return 1U;
}

/**
  * @brief  停止KEY2逐步追加测试中的全部电机。
  * @retval None
  * @note   三路全部回到IDLE并关闭PWM输出，下一次KEY2重新从motor0开始。
  */
static void Main_StopAllMotorCurrentTest(void)
{
  uint8_t motor_id;

  for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
  {
    Motor_Core_SetCurrentRef(motor_id, 0.0f, 0.0f);
    Motor_Core_SetSpeedRef(motor_id, 0.0f);
    Motor_Core_SetMode(motor_id, MOTOR_MODE_IDLE);
    (void)Motor_Core_StopPwmOutput(motor_id);
  }

  s_selected_motor_pwm_hw_started = 0U;
  s_selected_motor_align_active = 0U;
  s_selected_motor_rotate_active = 0U;
  s_selected_motor_current_active = 0U;
  s_key2_add_current_active_mask = 0U;
  s_key2_add_current_next_motor_index = 0U;
  s_key2_speed_test_step = 0U;
}

/**
  * @brief  KEY1顺序校正启动单台电机。
  * @param  motor_id: 电机编号。
  * @retval 1表示启动成功，0表示失败。
  * @note   仅用于顺序校正内部，不改KEY2路径。
  */
static uint8_t Main_StartKey1CalibOne(uint8_t motor_id)
{
  uint8_t pwm_ready;
  uint8_t key1_encoder_valid;
  uint8_t key1_start_allowed;
  uint8_t key1_start_called;
  uint8_t key1_start_ok;
  uint32_t key1_ccer_before;

  if (motor_id >= MOTOR_COUNT)
  {
    return 0U;
  }

  key1_encoder_valid = SPI_Encoder_Is_Valid((encoder_id_t)Motor_Core_GetBoundEncoderId(motor_id));
  key1_ccer_before = (motor_id == 0U) ? TIM1->CCER : ((motor_id == 1U) ? TIM8->CCER : TIM20->CCER);
  pwm_ready = 0U;
  key1_start_allowed = 0U;
  key1_start_called = 0U;
  key1_start_ok = 0U;

  (void)Motor_Core_OutputNeutralPwm(motor_id);

  key1_start_allowed = Motor_Core_IsPwmStartAllowed(motor_id);
  if (key1_start_allowed != 0U)
  {
    key1_start_called = 1U;
    if (Motor_Core_StartPwmOutput(motor_id) == HAL_OK)
    {
      key1_start_ok = 1U;
      pwm_ready = 1U;
    }
  }

  VOFA_Debug_RecordKey1PwmStart(key1_encoder_valid,
                                key1_start_allowed,
                                key1_start_called,
                                key1_start_ok,
                                pwm_ready,
                                key1_ccer_before,
                                (motor_id == 0U) ? TIM1->CCER : ((motor_id == 1U) ? TIM8->CCER : TIM20->CCER));

  if (pwm_ready == 0U)
  {
    return 0U;
  }

  s_selected_motor_id = motor_id;
  s_selected_motor_pwm_hw_started = 1U;
  s_selected_motor_align_active = 0U;
  s_selected_motor_rotate_active = 0U;
  s_selected_motor_current_active = 0U;
  s_key2_speed_test_step = 0U;

  Motor_Core_SetMode(motor_id, MOTOR_MODE_IDLE);
  Motor_Core_StartFocOffsetAutoCalib(motor_id, MAIN_FOC_ALIGN_VD_NORM);

  return (Motor_Core_IsFocOffsetAutoCalibBusy() != 0U) ? 1U : 0U;
}

/**
  * @brief  推进KEY1三电机顺序校正。
  * @retval None
  */
static void Main_ServiceKey1CalibSequence(void)
{
  if (s_key1_calib_seq_active == 0U)
  {
    return;
  }

  if (Motor_Core_IsFocOffsetAutoCalibBusy() != 0U)
  {
    s_key1_calib_seq_started = 1U;
    return;
  }

  if (s_key1_calib_seq_started == 0U)
  {
    s_key1_calib_seq_active = 0U;
    return;
  }

  s_key1_calib_seq_started = 0U;
  if (Motor_Core_GetFocOffsetAutoCalibResult() == 0U)
  {
    s_key1_calib_seq_active = 0U;
    return;
  }

  s_key1_calib_seq_next_motor++;
  if (s_key1_calib_seq_next_motor >= MOTOR_COUNT)
  {
    s_key1_calib_seq_round++;
    if (s_key1_calib_seq_round >= MAIN_AUTO_FOC_CALIB_ROUNDS)
    {
      s_key1_calib_seq_active = 0U;
      return;
    }

    s_key1_calib_seq_next_motor = 0U;
    if (Main_StartKey1CalibOne(0U) == 0U)
    {
      s_key1_calib_seq_active = 0U;
    }
    return;
  }

  if (Main_StartKey1CalibOne(s_key1_calib_seq_next_motor) == 0U)
  {
    s_key1_calib_seq_active = 0U;
  }
}

/**
  * @brief  将KEY1逐台校正序号清零。
  * @retval None
  */
/**
  * @brief  UART发送完成回调
  * @param  huart: 触发发送完成的串口句柄
  * @retval None
  * @note   该回调由HAL库在DMA发送完成后自动调用。
  *         这里将USART1发送完成事件转发给VOFA调试模块处理。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  VOFA_Debug_TxCpltCallback(huart);
}

/**
  * @brief  UART错误回调
  * @param  huart: 发生错误的串口句柄
  * @retval None
  * @note   该回调由HAL库在串口或DMA发送异常时自动调用。
  *         这里将USART1错误事件转发给VOFA调试模块处理。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  VOFA_Debug_ErrorCallback(huart);
}

/**
  * @brief  TIM周期到达回调
  * @param  htim: 触发周期回调的定时器句柄
  * @retval None
  * @note   当前用于把TIM4 5ms基础节拍转发给timer_app，由timer_app产生软件任务标志。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  Timer_App_PeriodElapsedCallback(htim);

  if ((htim != NULL) && (htim->Instance == TIM7))
  {
    static uint16_t speed_loop_div[MOTOR_COUNT] = {0U};
    static uint16_t position_loop_div[MOTOR_COUNT] = {0U};
    static uint16_t can_status_div = 0U;
    static uint32_t last_speed_cycles[MOTOR_COUNT] = {0U};
    static uint8_t speed_dt_valid[MOTOR_COUNT] = {0U};
    uint32_t now_cycles;
    uint8_t motor_id;
    encoder_id_t encoder_id;
    HAL_StatusTypeDef speed_status;
    float speed_dt_s;
    uint8_t active_motor_mask;

    active_motor_mask = (uint8_t)(s_key2_add_current_active_mask | CAN_App_GetActiveMotorMask());
    if (active_motor_mask == 0U)
    {
      return;
    }

    for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
    {
      if ((active_motor_mask & (1U << motor_id)) == 0U)
      {
        continue;
      }

      position_loop_div[motor_id]++;
      if (position_loop_div[motor_id] >= MAIN_POSITION_LOOP_TIM7_DIV)
      {
        position_loop_div[motor_id] = 0U;
        (void)Motor_Core_RunPositionOuterLoopFromTim7(motor_id, MOTOR_POSITION_LOOP_DT_S);
      }

      speed_loop_div[motor_id]++;
      if (speed_loop_div[motor_id] >= MAIN_SPEED_LOOP_TIM7_DIV)
      {
        speed_loop_div[motor_id] = 0U;

        /*
         * TIM7分频后作为速度外环节拍：
         * 1. 只根据已有编码器角度缓存计算速度，不启动SPI传输；
         * 2. 速度环只更新iq_ref，不跑电流环、不跑FOC、不写CCR；
         * 3. 电流快环仍由各自ADC DMA完成回调触发。
         */
        now_cycles = Timer_App_GetRuntimeCycles();
        if (speed_dt_valid[motor_id] == 0U)
        {
          speed_dt_s = MOTOR_SPEED_LOOP_DT_S;
          speed_dt_valid[motor_id] = 1U;
        }
        else
        {
          speed_dt_s = (float)(now_cycles - last_speed_cycles[motor_id]) / (float)SystemCoreClock;
          if (speed_dt_s < (MOTOR_SPEED_LOOP_DT_S * 0.2f))
          {
            speed_dt_s = MOTOR_SPEED_LOOP_DT_S * 0.2f;
          }
          else if (speed_dt_s > (MOTOR_SPEED_LOOP_DT_S * 5.0f))
          {
            speed_dt_s = MOTOR_SPEED_LOOP_DT_S * 5.0f;
          }
        }
        last_speed_cycles[motor_id] = now_cycles;

        encoder_id = (encoder_id_t)Motor_Core_GetBoundEncoderId(motor_id);
        speed_status = SPI_Encoder_Update_Speed_By_Cache(encoder_id, speed_dt_s);

        if (speed_status == HAL_OK)
        {
          (void)Motor_Core_RunSpeedOuterLoopFromTim7(motor_id, speed_dt_s);
        }
      }
    }

    /*
     * 状态上报使用TIM7固定分频，不依赖最低优先级的HAL Tick。
     * 放在三电机循环外，避免某一路位置环提前返回后停止上报。
     */
    can_status_div++;
    if (can_status_div >= MAIN_CAN_STATUS_TIM7_DIV)
    {
      can_status_div = 0U;
      CAN_App_RequestStatusTx();
    }
  }
}

/**
  * @brief  SPI DMA收发完成回调
  * @param  hspi: 触发DMA完成的SPI句柄
  * @retval None
  * @note   当前用于把SPI3编码器DMA完成事件转发给spi_encoder模块。
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SPI_Encoder_TxRxCpltCallback(hspi);
}

/**
  * @brief  SPI错误回调
  * @param  hspi: 触发错误的SPI句柄
  * @retval None
  * @note   当前用于把SPI3编码器DMA错误事件转发给spi_encoder模块，
  *         防止编码器DMA busy状态卡死。
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  SPI_Encoder_ErrorCallback(hspi);
}

/**
  * @brief  ADC转换完成回调
  * @param  hadc: 触发转换完成回调的ADC句柄
  * @retval None
  * @note   当前用于把ADC DMA完成事件转发给adc_app，由adc_app维护raw更新标志。
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint8_t active_motor_mask;

  ADC_App_ConvCpltCallback(hadc);

  if (hadc == NULL)
  {
    return;
  }

  active_motor_mask = (uint8_t)(s_key2_add_current_active_mask | CAN_App_GetActiveMotorMask());

  if (hadc->Instance == ADC4)
  {
    static uint8_t encoder1_dma_div = 0U;

    if ((active_motor_mask & (1U << 0U)) != 0U)
    {
      (void)Motor_Core_RunCurrentFastLoopDirectFromAdc(0U, Main_GetCurrentFastLoopDt(0U));
    }

    if ((s_selected_motor_id == 0U) && (s_selected_motor_current_active == 0U))
    {
      (void)Motor_Core_UpdateCurrentObserve(0U);
    }

    encoder1_dma_div++;
    if (encoder1_dma_div >= MAIN_ENCODER_DMA_ADC_DIV)
    {
      encoder1_dma_div = 0U;
      (void)SPI_Encoder_Request_Update_DMA(encoder_id_1, Timer_App_GetRuntimeCycles());
    }
    (void)SPI_Encoder_ServicePendingOnce();
  }
  else if (hadc->Instance == ADC3)
  {
    static uint8_t encoder2_dma_div = 0U;

    if ((active_motor_mask & (1U << 1U)) != 0U)
    {
      (void)Motor_Core_RunCurrentFastLoopDirectFromAdc(1U, Main_GetCurrentFastLoopDt(1U));
    }

    if ((s_selected_motor_id == 1U) && (s_selected_motor_current_active == 0U))
    {
      (void)Motor_Core_UpdateCurrentObserve(1U);
    }

    encoder2_dma_div++;
    if (encoder2_dma_div >= MAIN_ENCODER_DMA_ADC_DIV)
    {
      encoder2_dma_div = 0U;
      (void)SPI_Encoder_Request_Update_DMA(encoder_id_2, Timer_App_GetRuntimeCycles());
    }
    (void)SPI_Encoder_ServicePendingOnce();
  }
  else if (hadc->Instance == ADC1)
  {
    static uint8_t encoder3_dma_div = 0U;

    if ((active_motor_mask & (1U << 2U)) != 0U)
    {
      (void)Motor_Core_RunCurrentFastLoopDirectFromAdc(2U, Main_GetCurrentFastLoopDt(2U));
    }

    if ((s_selected_motor_id == 2U) && (s_selected_motor_current_active == 0U))
    {
      (void)Motor_Core_UpdateCurrentObserve(2U);
    }

    encoder3_dma_div++;
    if (encoder3_dma_div >= MAIN_ENCODER_DMA_ADC_DIV)
    {
      encoder3_dma_div = 0U;
      (void)SPI_Encoder_Request_Update_DMA(encoder_id_3, Timer_App_GetRuntimeCycles());
    }
    (void)SPI_Encoder_ServicePendingOnce();
  }
}

/**
  * @brief  ADC错误回调
  * @param  hadc: 触发错误回调的ADC句柄
  * @retval None
  * @note   当前用于把ADC错误事件转发给adc_app，第一版adc_app只保留入口，不做复杂处理。
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  ADC_App_ErrorCallback(hadc);
}

/**
  * @brief  GPIO外部中断回调。
  * @param  GPIO_Pin: 触发外部中断的GPIO引脚。
  * @retval None
  * @note   KEY1：当前选中电机PWM开关；打开前先写入FOC电角度对齐电压。
  *         KEY2：PWM已开启并且转子吸合稳定后，捕获当前角度并修正FOC电角度零偏。
  *         KEY3：切换VOFA主题。
  *         当前按键已有硬件RC消抖，因此这里不做软件防抖。
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY1_Pin)
  {
    if ((s_key1_calib_seq_active != 0U) ||
        (Motor_Core_IsFocOffsetAutoCalibBusy() != 0U))
    {
      return;
    }

    s_key1_calib_seq_active = 1U;
    s_key1_calib_seq_next_motor = 0U;
    s_key1_calib_seq_started = 0U;
    s_key1_calib_seq_round = 0U;

    if (Main_StartKey1CalibOne(0U) == 0U)
    {
      s_key1_calib_seq_active = 0U;
      s_key1_calib_seq_started = 0U;
      s_key1_calib_seq_round = 0U;
    }
  }
    else if (GPIO_Pin == KEY2_Pin)
    {
#if (MAIN_KEY2_ADD_CURRENT_TEST_ENABLE != 0U)
      if (s_key2_add_current_active_mask == 0U)
      {
        (void)Main_StartNextMotorCurrentTest();
      }
      else
      {
        Main_StopAllMotorCurrentTest();
      }
      return;
#endif

      if (s_selected_motor_current_active == 0U)
      {
        uint8_t pwm_ready = s_selected_motor_pwm_hw_started;

        if (pwm_ready == 0U)
        {
          (void)Motor_Core_OutputNeutralPwm(s_selected_motor_id); // 启动PWM前先写安全中性占空比

          if (Motor_Core_IsPwmStartAllowed(s_selected_motor_id) != 0U)
          {
            if (Motor_Core_StartPwmOutput(s_selected_motor_id) == HAL_OK)
            {
              s_selected_motor_pwm_hw_started = 1U;
              pwm_ready = 1U;
            }
          }
        }

          if (pwm_ready != 0U)
          {
            s_selected_motor_align_active = 0U;
            s_selected_motor_rotate_active = 0U;
#if (MAIN_KEY2_POSITION_TEST_ENABLE != 0U)
            Motor_Core_SetCurrentRef(s_selected_motor_id, MAIN_CURRENT_TEST_ID_REF_A, 0.0f);
            Motor_Core_SetSpeedRef(s_selected_motor_id, 0.0f);
            Motor_Core_SetPositionRef(s_selected_motor_id,
                                      Motor_Core_GetPositionFeedbackRad(s_selected_motor_id)
                                      + ((s_selected_motor_id == 0U) ? -MAIN_POSITION_TEST_DELTA_RAD : MAIN_POSITION_TEST_DELTA_RAD));
            Motor_Core_SetMode(s_selected_motor_id, MOTOR_MODE_POSITION_SPEED_CURRENT);
#elif (MAIN_KEY2_SPEED_TEST_ENABLE != 0U)
            Motor_Core_SetCurrentRef(s_selected_motor_id, MAIN_CURRENT_TEST_ID_REF_A, 0.0f);
            Motor_Core_SetSpeedRef(s_selected_motor_id, MAIN_SPEED_TEST_REF_RAD_S);
            Motor_Core_SetMode(s_selected_motor_id, MOTOR_MODE_SPEED_CURRENT);
            s_key2_speed_test_step = 1U;
#else
            Motor_Core_SetCurrentRef(s_selected_motor_id,
                                     MAIN_CURRENT_TEST_ID_REF_A,
                                     MAIN_CURRENT_TEST_IQ_REF_A);
            Motor_Core_SetMode(s_selected_motor_id, MOTOR_MODE_CURRENT);
            s_key2_speed_test_step = 0U;
#endif
            s_selected_motor_current_active = 1U;
          }
      }
      else
      {
#if (MAIN_KEY2_SPEED_TEST_ENABLE != 0U)
        if (s_key2_speed_test_step == 1U)
        {
          Motor_Core_SetCurrentRef(s_selected_motor_id, MAIN_CURRENT_TEST_ID_REF_A, 0.0f);
          Motor_Core_SetSpeedRef(s_selected_motor_id, MAIN_SPEED_TEST_REF_HIGH_RAD_S);
          Motor_Core_SetMode(s_selected_motor_id, MOTOR_MODE_SPEED_CURRENT);
          s_key2_speed_test_step = 2U;
        }
        else if (s_key2_speed_test_step == 2U)
        {
          Motor_Core_SetCurrentRef(s_selected_motor_id, MAIN_CURRENT_TEST_ID_REF_A, 0.0f);
          Motor_Core_SetSpeedRef(s_selected_motor_id, MAIN_SPEED_TEST_REF_RAD_S);
          Motor_Core_SetMode(s_selected_motor_id, MOTOR_MODE_SPEED_CURRENT);
          s_key2_speed_test_step = 3U;
        }
        else
#endif
        {
          Motor_Core_SetCurrentRef(s_selected_motor_id, 0.0f, 0.0f);
          Motor_Core_SetSpeedRef(s_selected_motor_id, 0.0f);
          Motor_Core_SetMode(s_selected_motor_id, MOTOR_MODE_IDLE);
          (void)Motor_Core_StopPwmOutput(s_selected_motor_id); // 关闭PWM输出，让三相尽量进入自由滑行状态
          s_selected_motor_pwm_hw_started = 0U;
          s_selected_motor_align_active = 0U;
          s_selected_motor_rotate_active = 0U;
          s_selected_motor_current_active = 0U;
          s_key2_speed_test_step = 0U;
        }
      }
    }
  else if (GPIO_Pin == KEY3_Pin)
  {
    vofa_topic_t topic = VOFA_Debug_GetTopic();

    if (topic == VOFA_TOPIC_ENCODER)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_ENCODER_ALL);
    }
    else if (topic == VOFA_TOPIC_ENCODER_ALL)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_FOC_TEST);
    }
    else if (topic == VOFA_TOPIC_FOC_TEST)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_ADC_DIAG);
    }
    else if (topic == VOFA_TOPIC_ADC_DIAG)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_PHASE_CURRENT);
    }
    else if (topic == VOFA_TOPIC_PHASE_CURRENT)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_CURRENT_SAMPLE_DIAG);
    }
    else if (topic == VOFA_TOPIC_CURRENT_SAMPLE_DIAG)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_CURRENT_PID);
    }
    else if (topic == VOFA_TOPIC_CURRENT_PID)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_SPEED_PID);
    }
    else if (topic == VOFA_TOPIC_SPEED_PID)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_POSITION_PID);
    }
    else if (topic == VOFA_TOPIC_POSITION_PID)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_FAST_LOOP_PROFILE);
    }
    else if (topic == VOFA_TOPIC_FAST_LOOP_PROFILE)
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_DRV_CHECK);
    }
    else
    {
      VOFA_Debug_SetTopic(VOFA_TOPIC_ENCODER);
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
