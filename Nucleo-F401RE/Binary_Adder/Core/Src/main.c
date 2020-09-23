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
#include <string.h>
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
int run = 1;
int dataArray[12] = {};
int readSwitchPinsFuncExecCounter = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void readSwitchPins(void);
void processData(int data);
void outputData(char data[4]);
void enableAllLeds(void);
void disableAllLeds(void);
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Enables green onboard led
	  HAL_GPIO_WritePin(modeLed_GPIO_Port, modeLed_Pin, GPIO_PIN_SET);

	  readSwitchPins();
	  readSwitchPins();

	  int firstNumber = (dataArray[0]*32) + (dataArray[1]*16) + (dataArray[2]*8) + (dataArray[3]*4) + (dataArray[4]*2) + (dataArray[5]*1);
	  int secondNumber = (dataArray[6]*32) + (dataArray[7]*16) + (dataArray[8]*8) + (dataArray[9]*4) + (dataArray[10]*2) + (dataArray[11]*1);
	  int total = firstNumber + secondNumber;

	  processData(total);

	  disableAllLeds();
	  HAL_Delay(400);

	  // Some cool LED animation :)
	  for (int i=0; i < 4; i++) {
		  switch (i) {
		  case 0:
			  HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
			  HAL_Delay(200);
			  break;
		  case 1:
			  HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
			  HAL_Delay(200);
			  break;
		  case 2:
			  HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
			  HAL_Delay(200);
			  break;
		  case 3:
			  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			  HAL_Delay(200);
			  break;
		  }
	  }

	  enableAllLeds();
	  HAL_Delay(400);
	  disableAllLeds();
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
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  HAL_GPIO_WritePin(GPIOA, modeLed_Pin|led1_Pin|led8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : modeBtn_Pin */
  GPIO_InitStruct.Pin = modeBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(modeBtn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : modeLed_Pin led1_Pin led8_Pin */
  GPIO_InitStruct.Pin = modeLed_Pin|led1_Pin|led8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : n0d1_Pin n1d1_Pin n0d2_Pin n1d0_Pin */
  GPIO_InitStruct.Pin = n0d1_Pin|n1d1_Pin|n0d2_Pin|n1d0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : led4_Pin */
  GPIO_InitStruct.Pin = led4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : n0d0_Pin n1d2_Pin */
  GPIO_InitStruct.Pin = n0d0_Pin|n1d2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : led2_Pin */
  GPIO_InitStruct.Pin = led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void readSwitchPins(void) {
	extern int dataArray[];
	extern int readSwitchPinsFuncExecCounter;

	// Do while blue button is not pressed
	if (readSwitchPinsFuncExecCounter == 1) {
		while (run == 1) {
			// Reads all input pins for first number
			dataArray[0] = HAL_GPIO_ReadPin(n1d2_GPIO_Port, n1d2_Pin);
			dataArray[1] = HAL_GPIO_ReadPin(n1d1_GPIO_Port, n1d1_Pin);
			dataArray[2] = HAL_GPIO_ReadPin(n1d0_GPIO_Port, n1d0_Pin);
			dataArray[3] = HAL_GPIO_ReadPin(n0d2_GPIO_Port, n0d2_Pin);
			dataArray[4] = HAL_GPIO_ReadPin(n0d1_GPIO_Port, n0d1_Pin);
			dataArray[5] = HAL_GPIO_ReadPin(n0d0_GPIO_Port, n0d0_Pin);
		}
		readSwitchPinsFuncExecCounter = 2;
	}
	else if (readSwitchPinsFuncExecCounter == 2) {
		while (run == 1) {
			// Reads all input pins for second number
			dataArray[6] = HAL_GPIO_ReadPin(n1d2_GPIO_Port, n1d2_Pin);
			dataArray[7] = HAL_GPIO_ReadPin(n1d1_GPIO_Port, n1d1_Pin);
			dataArray[8] = HAL_GPIO_ReadPin(n1d0_GPIO_Port, n1d0_Pin);
			dataArray[9] = HAL_GPIO_ReadPin(n0d2_GPIO_Port, n0d2_Pin);
			dataArray[10] = HAL_GPIO_ReadPin(n0d1_GPIO_Port, n0d1_Pin);
			dataArray[11] = HAL_GPIO_ReadPin(n0d0_GPIO_Port, n0d0_Pin);

			// Blinks green onboard led
			HAL_GPIO_TogglePin(modeLed_GPIO_Port, modeLed_Pin);
			HAL_Delay(100);
		}
		readSwitchPinsFuncExecCounter = 1;

		// Disables green onboard led
		HAL_GPIO_WritePin(modeLed_GPIO_Port, modeLed_Pin, GPIO_PIN_RESET);
	}
	run = 1;
}

void processData(int data) {
	int outputArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	// Converts decimal number to binary
	for (int i=0; data > 0; i++) {
		outputArray[i] = data % 2;
		data = data / 2;
	}

	{
		int startPos = 7;
		int endPos = 4;
		char halfOfOutputArray[4] = {};
		char tmp[1];

		// Processes first part of binary number and sends it to output
		for (; startPos >= endPos; startPos--) {
			strcat(halfOfOutputArray, itoa(outputArray[startPos], tmp, 10));
		}
		outputData(halfOfOutputArray);
	}

	HAL_Delay(3000);

	{
		int startPos = 3;
		int endPos = 0;
		char halfOfOutputArray[4] = {};
		char tmp[1];

		// Processes second part of binary number and send it to output
		for (; startPos >= endPos; startPos--) {
			strcat(halfOfOutputArray, itoa(outputArray[startPos], tmp, 10));
		}
		outputData(halfOfOutputArray);
	}

	HAL_Delay(3000);
}


void outputData(char data[4]) {
	if (strcmp(data, "0000") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	}
	else if (strcmp(data, "0001") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	}
	else if (strcmp(data, "0010") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	}
	else if (strcmp(data, "0011") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	}
	else if (strcmp(data, "0100") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	}
	else if (strcmp(data, "0101") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	}
	else if (strcmp(data, "0110") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	}
	else if (strcmp(data, "0111") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	}
	else if (strcmp(data, "1000") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	}
	else if (strcmp(data, "1001") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	}
	else if (strcmp(data, "1010") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	}
	else if (strcmp(data, "1011") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	}
	else if (strcmp(data, "1100") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	}
	else if (strcmp(data, "1101") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	}
	else if (strcmp(data, "1110") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	}
	else if (strcmp(data, "1111") == 0) {
		HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	}
}


void enableAllLeds(void) {
	HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
}


void disableAllLeds(void) {
	HAL_GPIO_WritePin(led8_GPIO_Port, led8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
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
