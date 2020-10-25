/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
enum buttonName {zButton, oButton, pButton, mButton, eButton};
enum sign {notAssigned, minus, plus};
enum row {first, second, third};
enum bool {false, true};

short binaryNumber = 0b000000;
unsigned char operationSign = notAssigned;
unsigned char numberSign = plus;
unsigned char row = first;

// Button state definition
unsigned char oButtonPressed = false;	// Number 1 button
unsigned char zButtonPressed = false;	// Number 0 button
unsigned char pButtonPressed = false;	// Plus button
unsigned char mButtonPressed = false;	// Minus button
unsigned char eButtonPressed = false;	// Equal button
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// Binary adder mechanics
static void updateBinaryNumber(unsigned char button);
static void takeAction(unsigned char button);
static void writeOutput(short binaryNumber);

// Actions with LEDs
static void enableFirstRow(void);
static void enableSecondRow(void);
static void enableThirdRow(void);
static void enableAllLeds(void);
static void disableFirstRow(void);
static void disableSecondRow(void);
static void disableThirdRow(void);
static void disableAllLeds(void);
static void simpleLedAnimation(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  enableAllLeds();
  HAL_Delay(2000);
  disableAllLeds();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Events
	  if (oButtonPressed == true) {
		  oButtonPressed = false;
		  updateBinaryNumber(oButton);
	  }
	  if (zButtonPressed == true) {
		  zButtonPressed = false;
		  updateBinaryNumber(zButton);
	  }
	  if (pButtonPressed == true) {
		  pButtonPressed = false;
		  takeAction(pButton);
	  }
	  if (mButtonPressed == true) {
		  mButtonPressed = false;
		  takeAction(mButton);
	  }
	  if (eButtonPressed == true) {
		  eButtonPressed = false;
		  takeAction(eButton);
	  }

	  //
	  writeOutput(binaryNumber);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, INDICATOR_LED_Pin|LED2_1_Pin|LED2_4_Pin|LED3_4_Pin
                          |LED3_1_Pin|LED2_2_Pin|LED1_32_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_16_Pin|LED1_4_Pin|LED2_32_Pin|LED1_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_8_Pin|LED3_2_Pin|LED3_16_Pin|LED2_8_Pin
                          |LED3_0_Pin|LED3_32_Pin|LED3_64_Pin|LED2_16_Pin
                          |LED1_2_Pin|LED1_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B4_M_Pin B2_Z_Pin B3_P_Pin B5_E_Pin */
  GPIO_InitStruct.Pin = B4_M_Pin|B2_Z_Pin|B3_P_Pin|B5_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INDICATOR_LED_Pin LED2_1_Pin LED2_4_Pin LED3_4_Pin
                           LED3_1_Pin LED2_2_Pin LED1_32_Pin */
  GPIO_InitStruct.Pin = INDICATOR_LED_Pin|LED2_1_Pin|LED2_4_Pin|LED3_4_Pin
                          |LED3_1_Pin|LED2_2_Pin|LED1_32_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_16_Pin LED1_4_Pin LED2_32_Pin LED1_1_Pin */
  GPIO_InitStruct.Pin = LED1_16_Pin|LED1_4_Pin|LED2_32_Pin|LED1_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_8_Pin LED3_2_Pin LED3_16_Pin LED2_8_Pin
                           LED3_0_Pin LED3_32_Pin LED3_64_Pin LED2_16_Pin
                           LED1_2_Pin LED1_8_Pin */
  GPIO_InitStruct.Pin = LED3_8_Pin|LED3_2_Pin|LED3_16_Pin|LED2_8_Pin
                          |LED3_0_Pin|LED3_32_Pin|LED3_64_Pin|LED2_16_Pin
                          |LED1_2_Pin|LED1_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_O_Pin */
  GPIO_InitStruct.Pin = B1_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_O_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

static void updateBinaryNumber(unsigned char button) {
	if (button == oButton) {
		if (binaryNumber == 0b000000) {
			binaryNumber += 0b000001;
		}
		else if (binaryNumber < 0b100000) {
			binaryNumber = binaryNumber << 1;
			binaryNumber += 0b000001;
		}
		else {
			return;
		}
	}
	if (button == zButton) {
		if (binaryNumber == 0b000000 || binaryNumber >= 0b100000) {
			return;
		}
		else {
			binaryNumber = binaryNumber << 1;
		}
	}
}


static void takeAction(unsigned char button) {
	static short number1, number2;

	if (row == first) {
		if (button == eButton) {
			return;
		}
		row = second;
		if (button == pButton) {
			operationSign = plus;
		}
		if (button == mButton) {
			operationSign = minus;
		}
		number1 = binaryNumber;
		binaryNumber = 0b000000;
		// Enable indicator LED (onboard green LED)
		HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
	}
	if (row == second) {
		if (button == eButton) {
			row = third;
			numberSign = plus;
			number2 = binaryNumber;
			if (operationSign == plus) {
				binaryNumber = number1 + number2;
			}
			if (operationSign == minus) {
				binaryNumber = number1 - number2;
				if (binaryNumber < 0b000000) {
					numberSign = minus;
					binaryNumber = binaryNumber * (-1);
				}
			}
			writeOutput(binaryNumber);
			HAL_Delay(4000);
			binaryNumber = 0b000000;
			simpleLedAnimation();
			// Disable indicator LED (onboard green LED)
			HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
		}
	}
}


static void writeOutput(short binaryNumber) {
	if (row == first) {
		disableFirstRow();
		if (binaryNumber >= 32) {
			HAL_GPIO_WritePin(LED1_32_GPIO_Port, LED1_32_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 32;
		}
		if (binaryNumber >= 16) {
			HAL_GPIO_WritePin(LED1_16_GPIO_Port, LED1_16_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 16;
		}
		if (binaryNumber >= 8) {
			HAL_GPIO_WritePin(LED1_8_GPIO_Port, LED1_8_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 8;
		}
		if (binaryNumber >= 4) {
			HAL_GPIO_WritePin(LED1_4_GPIO_Port, LED1_4_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 4;
		}
		if (binaryNumber >= 2) {
			HAL_GPIO_WritePin(LED1_2_GPIO_Port, LED1_2_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 2;
		}
		if (binaryNumber >= 1) {
			HAL_GPIO_WritePin(LED1_1_GPIO_Port, LED1_1_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 1;
		}
	}
	if (row == second) {
		disableSecondRow();
		if (binaryNumber >= 32) {
			HAL_GPIO_WritePin(LED2_32_GPIO_Port, LED2_32_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 32;
		}
		if (binaryNumber >= 16) {
			HAL_GPIO_WritePin(LED2_16_GPIO_Port, LED2_16_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 16;
		}
		if (binaryNumber >= 8) {
			HAL_GPIO_WritePin(LED2_8_GPIO_Port, LED2_8_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 8;
		}
		if (binaryNumber >= 4) {
			HAL_GPIO_WritePin(LED2_4_GPIO_Port, LED2_4_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 4;
		}
		if (binaryNumber >= 2) {
			HAL_GPIO_WritePin(LED2_2_GPIO_Port, LED2_2_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 2;
		}
		if (binaryNumber >= 1) {
			HAL_GPIO_WritePin(LED2_1_GPIO_Port, LED2_1_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 1;
		}
	}
	if (row == third) {
		disableThirdRow();
		if (numberSign == minus) {
			HAL_GPIO_WritePin(LED3_0_GPIO_Port, LED3_0_Pin, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(LED3_0_GPIO_Port, LED3_0_Pin, GPIO_PIN_RESET);
		}

		if (binaryNumber >= 64) {
			HAL_GPIO_WritePin(LED3_64_GPIO_Port, LED3_64_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 64;
		}
		if (binaryNumber >= 32) {
			HAL_GPIO_WritePin(LED3_32_GPIO_Port, LED3_32_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 32;
		}
		if (binaryNumber >= 16) {
			HAL_GPIO_WritePin(LED3_16_GPIO_Port, LED3_16_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 16;
		}
		if (binaryNumber >= 8) {
			HAL_GPIO_WritePin(LED3_8_GPIO_Port, LED3_8_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 8;
		}
		if (binaryNumber >= 4) {
			HAL_GPIO_WritePin(LED3_4_GPIO_Port, LED3_4_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 4;
		}
		if (binaryNumber >= 2) {
			HAL_GPIO_WritePin(LED3_2_GPIO_Port, LED3_2_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 2;
		}
		if (binaryNumber >= 1) {
			HAL_GPIO_WritePin(LED3_1_GPIO_Port, LED3_1_Pin, GPIO_PIN_SET);
			binaryNumber = binaryNumber - 1;
		}
		row = first;
	}
}


static void enableFirstRow(void) {
	HAL_GPIO_WritePin(LED1_32_GPIO_Port, LED1_32_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_16_GPIO_Port, LED1_16_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_8_GPIO_Port, LED1_8_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_4_GPIO_Port, LED1_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_2_GPIO_Port, LED1_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_1_GPIO_Port, LED1_1_Pin, GPIO_PIN_SET);
}


static void enableSecondRow(void) {
	HAL_GPIO_WritePin(LED2_32_GPIO_Port, LED2_32_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_16_GPIO_Port, LED2_16_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_8_GPIO_Port, LED2_8_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_4_GPIO_Port, LED2_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_2_GPIO_Port, LED2_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_1_GPIO_Port, LED2_1_Pin, GPIO_PIN_SET);
}


static void enableThirdRow(void) {
	HAL_GPIO_WritePin(LED3_0_GPIO_Port, LED3_0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_64_GPIO_Port, LED3_64_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_32_GPIO_Port, LED3_32_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_16_GPIO_Port, LED3_16_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_8_GPIO_Port, LED3_8_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_4_GPIO_Port, LED3_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_2_GPIO_Port, LED3_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_1_GPIO_Port, LED3_1_Pin, GPIO_PIN_SET);
}


static void enableAllLeds(void) {
	enableFirstRow();
	enableSecondRow();
	enableThirdRow();
}


static void disableFirstRow(void) {
	HAL_GPIO_WritePin(LED1_32_GPIO_Port, LED1_32_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_16_GPIO_Port, LED1_16_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_8_GPIO_Port, LED1_8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_4_GPIO_Port, LED1_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_2_GPIO_Port, LED1_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_1_GPIO_Port, LED1_1_Pin, GPIO_PIN_RESET);
}


static void disableSecondRow(void) {
	HAL_GPIO_WritePin(LED2_32_GPIO_Port, LED2_32_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_16_GPIO_Port, LED2_16_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_8_GPIO_Port, LED2_8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_4_GPIO_Port, LED2_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_2_GPIO_Port, LED2_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_1_GPIO_Port, LED2_1_Pin, GPIO_PIN_RESET);
}


static void disableThirdRow(void) {
	HAL_GPIO_WritePin(LED3_0_GPIO_Port, LED3_0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_64_GPIO_Port, LED3_64_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_32_GPIO_Port, LED3_32_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_16_GPIO_Port, LED3_16_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_8_GPIO_Port, LED3_8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_4_GPIO_Port, LED3_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_2_GPIO_Port, LED3_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_1_GPIO_Port, LED3_1_Pin, GPIO_PIN_RESET);
}


static void disableAllLeds(void) {
	disableFirstRow();
	disableSecondRow();
	disableThirdRow();
}


static void simpleLedAnimation(void) {
	unsigned short delay = 250;

	disableAllLeds();
	HAL_Delay(delay);

	enableFirstRow();
	HAL_Delay(delay);
	disableFirstRow();
	enableSecondRow();
	HAL_Delay(delay);
	disableSecondRow();
	enableThirdRow();
	HAL_Delay(delay);
	disableThirdRow();

	enableAllLeds();
	HAL_Delay(delay);
	disableAllLeds();
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
