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
#include "stm32f4xx_hal.h"

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
#define TIM_CLK_MHz 168
#define PWM_FREQUENCY 10000
#define KEY0_Pin GPIO_PIN_2
#define KEY0_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOE
#define MAX_485_EN_Pin GPIO_PIN_10
#define MAX_485_EN_GPIO_Port GPIOI
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOF
#define PM1_CTRL_SD_Pin GPIO_PIN_10
#define PM1_CTRL_SD_GPIO_Port GPIOF
#define PM1_VTEMP_Pin GPIO_PIN_0
#define PM1_VTEMP_GPIO_Port GPIOA
#define PM1_AMPW_Pin GPIO_PIN_3
#define PM1_AMPW_GPIO_Port GPIOA
#define PM1_AMPV_Pin GPIO_PIN_6
#define PM1_AMPV_GPIO_Port GPIOA
#define PM1_AMPU_Pin GPIO_PIN_0
#define PM1_AMPU_GPIO_Port GPIOB
#define PM1_VBUS_Pin GPIO_PIN_1
#define PM1_VBUS_GPIO_Port GPIOB
#define PM1_PWM_UL_Pin GPIO_PIN_13
#define PM1_PWM_UL_GPIO_Port GPIOB
#define PM1_PWM_VL_Pin GPIO_PIN_14
#define PM1_PWM_VL_GPIO_Port GPIOB
#define PM1_PWM_WL_Pin GPIO_PIN_15
#define PM1_PWM_WL_GPIO_Port GPIOB
#define PM1_PWM_UH_Pin GPIO_PIN_8
#define PM1_PWM_UH_GPIO_Port GPIOA
#define PM1_PWM_VH_Pin GPIO_PIN_9
#define PM1_PWM_VH_GPIO_Port GPIOA
#define PM1_PWM_WH_Pin GPIO_PIN_10
#define PM1_PWM_WH_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
