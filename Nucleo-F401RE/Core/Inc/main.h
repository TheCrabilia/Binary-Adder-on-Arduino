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
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define led3_2_Pin GPIO_PIN_5
#define led3_2_GPIO_Port GPIOA
#define led2_32_Pin GPIO_PIN_6
#define led2_32_GPIO_Port GPIOA
#define led2_8_Pin GPIO_PIN_7
#define led2_8_GPIO_Port GPIOA
#define led3_4_Pin GPIO_PIN_5
#define led3_4_GPIO_Port GPIOC
#define led1_4_Pin GPIO_PIN_1
#define led1_4_GPIO_Port GPIOB
#define led1_16_Pin GPIO_PIN_2
#define led1_16_GPIO_Port GPIOB
#define led1_2_Pin GPIO_PIN_10
#define led1_2_GPIO_Port GPIOB
#define led2_4_Pin GPIO_PIN_12
#define led2_4_GPIO_Port GPIOB
#define test2_Pin GPIO_PIN_15
#define test2_GPIO_Port GPIOB
#define led3_16_Pin GPIO_PIN_6
#define led3_16_GPIO_Port GPIOC
#define led2_1_Pin GPIO_PIN_7
#define led2_1_GPIO_Port GPIOC
#define test1_Pin GPIO_PIN_8
#define test1_GPIO_Port GPIOC
#define led3_0_Pin GPIO_PIN_9
#define led3_0_GPIO_Port GPIOC
#define led1_8_Pin GPIO_PIN_8
#define led1_8_GPIO_Port GPIOA
#define led1_32_Pin GPIO_PIN_9
#define led1_32_GPIO_Port GPIOA
#define led2_16_Pin GPIO_PIN_11
#define led2_16_GPIO_Port GPIOA
#define led3_1_Pin GPIO_PIN_12
#define led3_1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define led2_2_Pin GPIO_PIN_6
#define led2_2_GPIO_Port GPIOB
#define led3_32_Pin GPIO_PIN_8
#define led3_32_GPIO_Port GPIOB
#define led3_8_Pin GPIO_PIN_9
#define led3_8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
