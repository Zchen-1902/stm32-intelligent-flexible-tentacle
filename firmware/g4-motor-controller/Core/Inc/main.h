/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAL3_Pin GPIO_PIN_2
#define CAL3_GPIO_Port GPIOF
#define NSCS3_Pin GPIO_PIN_4
#define NSCS3_GPIO_Port GPIOA
#define NSCS1_Pin GPIO_PIN_4
#define NSCS1_GPIO_Port GPIOC
#define NSCS2_Pin GPIO_PIN_5
#define NSCS2_GPIO_Port GPIOC
#define CAL1_Pin GPIO_PIN_14
#define CAL1_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define CAL2_Pin GPIO_PIN_15
#define CAL2_GPIO_Port GPIOD
#define KEY3_Pin GPIO_PIN_9
#define KEY3_GPIO_Port GPIOC
#define KEY3_EXTI_IRQn EXTI9_5_IRQn
#define KEY2_Pin GPIO_PIN_8
#define KEY2_GPIO_Port GPIOA
#define KEY2_EXTI_IRQn EXTI9_5_IRQn
#define KEY1_Pin GPIO_PIN_11
#define KEY1_GPIO_Port GPIOA
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define CSN3_Pin GPIO_PIN_7
#define CSN3_GPIO_Port GPIOD
#define CSN2_Pin GPIO_PIN_6
#define CSN2_GPIO_Port GPIOB
#define CSN1_Pin GPIO_PIN_7
#define CSN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
