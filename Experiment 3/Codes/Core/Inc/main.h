/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define A_Pin GPIO_PIN_5
#define A_GPIO_Port GPIOE
#define A_EXTI_IRQn EXTI9_5_IRQn
#define dir_Pin GPIO_PIN_6
#define dir_GPIO_Port GPIOE
#define Push_botton_Pin GPIO_PIN_2
#define Push_botton_GPIO_Port GPIOF
#define Push_botton_EXTI_IRQn EXTI2_IRQn
#define Push_bottonF3_Pin GPIO_PIN_3
#define Push_bottonF3_GPIO_Port GPIOF
#define Push_bottonF3_EXTI_IRQn EXTI3_IRQn
#define B_Pin GPIO_PIN_6
#define B_GPIO_Port GPIOF
#define PWM_output_Pin GPIO_PIN_7
#define PWM_output_GPIO_Port GPIOF
#define Z_Pin GPIO_PIN_0
#define Z_GPIO_Port GPIOA
#define Z_EXTI_IRQn EXTI0_IRQn
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOG
#define LEDG5_Pin GPIO_PIN_5
#define LEDG5_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
