/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TRIG1_Pin GPIO_PIN_5
#define TRIG1_GPIO_Port GPIOF
#define PWM_RRM_Pin GPIO_PIN_6
#define PWM_RRM_GPIO_Port GPIOA
#define D11_Pin GPIO_PIN_7
#define D11_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define D8_Pin GPIO_PIN_12
#define D8_GPIO_Port GPIOF
#define D7_Pin GPIO_PIN_13
#define D7_GPIO_Port GPIOF
#define D4_Pin GPIO_PIN_14
#define D4_GPIO_Port GPIOF
#define D2_Pin GPIO_PIN_15
#define D2_GPIO_Port GPIOF
#define PWM_FRM_Pin GPIO_PIN_9
#define PWM_FRM_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_11
#define D5_GPIO_Port GPIOE
#define PWM_FLM_Pin GPIO_PIN_13
#define PWM_FLM_GPIO_Port GPIOE
#define D10_Pin GPIO_PIN_14
#define D10_GPIO_Port GPIOD
#define PWM_RLM_Pin GPIO_PIN_15
#define PWM_RLM_GPIO_Port GPIOD
#define ECHO1_Pin GPIO_PIN_3
#define ECHO1_GPIO_Port GPIOD
#define D1_Pin GPIO_PIN_14
#define D1_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
