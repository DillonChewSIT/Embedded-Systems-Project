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
#include "stm32f3xx_hal.h"

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
#define Blue_B1_Pin GPIO_PIN_13
#define Blue_B1_GPIO_Port GPIOC
#define Blue_B1_EXTI_IRQn EXTI15_10_IRQn
#define left_vcw_Pin GPIO_PIN_0
#define left_vcw_GPIO_Port GPIOC
#define left_vcw_EXTI_IRQn EXTI0_IRQn
#define right_vcw_Pin GPIO_PIN_1
#define right_vcw_GPIO_Port GPIOC
#define right_vcw_EXTI_IRQn EXTI1_IRQn
#define Blue_LD_Pin GPIO_PIN_0
#define Blue_LD_GPIO_Port GPIOA
#define Red_LD_Pin GPIO_PIN_1
#define Red_LD_GPIO_Port GPIOA
#define tact_sw_Pin GPIO_PIN_4
#define tact_sw_GPIO_Port GPIOA
#define Green_LD2_Pin GPIO_PIN_5
#define Green_LD2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
