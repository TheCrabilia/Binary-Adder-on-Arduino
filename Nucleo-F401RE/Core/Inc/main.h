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
#define B4_M_Pin GPIO_PIN_0
#define B4_M_GPIO_Port GPIOC
#define B2_Z_Pin GPIO_PIN_1
#define B2_Z_GPIO_Port GPIOC
#define B3_P_Pin GPIO_PIN_2
#define B3_P_GPIO_Port GPIOC
#define B5_E_Pin GPIO_PIN_3
#define B5_E_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LED1_32_Pin GPIO_PIN_5
#define LED1_32_GPIO_Port GPIOA
#define LED2_2_Pin GPIO_PIN_6
#define LED2_2_GPIO_Port GPIOA
#define LED2_8_Pin GPIO_PIN_7
#define LED2_8_GPIO_Port GPIOA
#define LED1_16_Pin GPIO_PIN_5
#define LED1_16_GPIO_Port GPIOC
#define LED3_16_Pin GPIO_PIN_1
#define LED3_16_GPIO_Port GPIOB
#define LED3_4_Pin GPIO_PIN_2
#define LED3_4_GPIO_Port GPIOB
#define LED3_32_Pin GPIO_PIN_10
#define LED3_32_GPIO_Port GPIOB
#define LED2_16_Pin GPIO_PIN_12
#define LED2_16_GPIO_Port GPIOB
#define LED1_4_Pin GPIO_PIN_6
#define LED1_4_GPIO_Port GPIOC
#define LED3_1_Pin GPIO_PIN_7
#define LED3_1_GPIO_Port GPIOC
#define LED1_1_Pin GPIO_PIN_9
#define LED1_1_GPIO_Port GPIOC
#define LED3_8_Pin GPIO_PIN_8
#define LED3_8_GPIO_Port GPIOA
#define LED3_2_Pin GPIO_PIN_9
#define LED3_2_GPIO_Port GPIOA
#define LED2_4_Pin GPIO_PIN_11
#define LED2_4_GPIO_Port GPIOA
#define LED2_1_Pin GPIO_PIN_12
#define LED2_1_GPIO_Port GPIOA
#define B1_O_Pin GPIO_PIN_15
#define B1_O_GPIO_Port GPIOA
#define LED3_64_Pin GPIO_PIN_4
#define LED3_64_GPIO_Port GPIOB
#define LED3_0_Pin GPIO_PIN_5
#define LED3_0_GPIO_Port GPIOB
#define LED2_32_Pin GPIO_PIN_6
#define LED2_32_GPIO_Port GPIOB
#define LED1_2_Pin GPIO_PIN_8
#define LED1_2_GPIO_Port GPIOB
#define LED1_8_Pin GPIO_PIN_9
#define LED1_8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
