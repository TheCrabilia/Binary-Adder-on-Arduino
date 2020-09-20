/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define modeBtn_Pin GPIO_PIN_13
#define modeBtn_GPIO_Port GPIOC
#define modeBtn_EXTI_IRQn EXTI15_10_IRQn
#define modeLed_Pin GPIO_PIN_5
#define modeLed_GPIO_Port GPIOA
#define led1_Pin GPIO_PIN_7
#define led1_GPIO_Port GPIOA
#define n0d1_Pin GPIO_PIN_10
#define n0d1_GPIO_Port GPIOB
#define led4_Pin GPIO_PIN_7
#define led4_GPIO_Port GPIOC
#define n0d0_Pin GPIO_PIN_8
#define n0d0_GPIO_Port GPIOA
#define led8_Pin GPIO_PIN_9
#define led8_GPIO_Port GPIOA
#define n1d2_Pin GPIO_PIN_10
#define n1d2_GPIO_Port GPIOA
#define n1d1_Pin GPIO_PIN_3
#define n1d1_GPIO_Port GPIOB
#define n0d2_Pin GPIO_PIN_4
#define n0d2_GPIO_Port GPIOB
#define n1d0_Pin GPIO_PIN_5
#define n1d0_GPIO_Port GPIOB
#define led2_Pin GPIO_PIN_6
#define led2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
