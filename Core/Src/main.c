/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "eeprom_emul.h"
#include "lcd_1602.h"
#include "4x4keypad.h"
#include "ds18b20_delay.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t flag_lcd = 0;		// Flag is set when need to update lcd
char lcd_buffer[17] = { ' ' };
uint8_t btn_cnt = 0;
uint8_t is_btn_pressed = 0;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint16_t temperatureDS = 0;
uint8_t mode = 0;
char buffer1[17];
char buffer2[17];
char keyPressed = ' ';
char startKeyPressed = ' ';
char keyPressedTimerRunning = 0;
uint16_t startPinPressed = 0;
char uart_buffer[150];
GPIO_InitTypeDef GPIO_InitStructPrivate = { 0 };
DS18B20_TypeDef ds = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//void _write(int file, char *ptr, int len);
void TOGGLE_GREEN_LED_PIN(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_SYSTICK_Callback(void)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	if (GPIO_Pin == ROW1_PIN || GPIO_Pin == ROW2_PIN || GPIO_Pin == ROW3_PIN
			|| GPIO_Pin == ROW4_PIN) {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		//is_btn_pressed = 1;
		btn_cnt++;
		//char cnt_buf[50];
		//sprintf(cnt_buf, "btn_cnt = %d\n\r", btn_cnt);
		//HAL_UART_Transmit(&huart1, (uint8_t*)cnt_buf, strlen(cnt_buf), 1000);
		currentMillis = HAL_GetTick();
		if (currentMillis - previousMillis > KEYPAD_DEBOUNCE) {
			//while (HAL_GetTick() - currentMillis < 5) continue;
//		if (keyPressedTimerRunning == 0) {
			is_btn_pressed = 1;
			//startKeyPressed = KEYPAD4x4_GetPressedKey(GPIO_Pin);
			keyPressed = KEYPAD4x4_GetPressedKey(GPIO_Pin);
			//startPinPressed = GPIO_Pin;
			//HAL_TIM_Base_Start_IT(&htim2);
			//keyPressedTimerRunning = 1;


			char temp_buf[50];
			//char *temp_buf_p = temp_buf;
//			keyPressed = KEYPAD4x4_GetPressedKey(GPIO_Pin);
			sprintf(temp_buf, "Pressed Key = %c\n\r", keyPressed);
//			printf(temp_buf);
			HAL_UART_Transmit(&huart1, (uint8_t*)temp_buf, strlen(temp_buf), 1000);

			previousMillis = currentMillis;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		//HAL_TIM_Base_Stop_IT(&htim2);
		//__HAL_TIM_SET_COUNTER(&htim2, 0);
		char keyPressedLocal = KEYPAD4x4_GetPressedKey(startPinPressed);
		if (keyPressedLocal == startKeyPressed) {
			keyPressed = keyPressedLocal;
			startKeyPressed = 99;
			is_btn_pressed = 1;

			char temp_buf[50];
			//char *temp_buf_p = temp_buf;
//			keyPressed = KEYPAD4x4_GetPressedKey(GPIO_Pin);
			sprintf(temp_buf, "Pressed Key = %c\n\r", keyPressed);
//			printf(temp_buf);
			HAL_UART_Transmit(&huart1, (uint8_t*) temp_buf, strlen(temp_buf),
					1000);
		}
		keyPressedTimerRunning = 0;
		btn_cnt++;
		HAL_TIM_Base_Stop_IT(&htim2);
		GPIO_InitTypeDef GPIO_InitStructPrivate = { 0 };

		GPIO_InitStructPrivate.Pin = ROW1_PIN | ROW2_PIN | ROW3_PIN | ROW4_PIN;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
		GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(ROW1_GPIO, &GPIO_InitStructPrivate);

		TOGGLE_GREEN_LED_PIN();
		//btn_state = new_btn_state;
	}
}

int _write(int file, char *data, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) data, strlen(data), 1000);
	return strlen(data);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	//setvbuf(stdout, NULL, _IONBF, 0);
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
	MX_TIM6_Init();
	MX_TIM1_Init();
	MX_TIM15_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
//  EEPROM_ErasePage();
//  EEPROM_Write(EEPROM_START_ADDRESS, 555);
	// Timer2 dual channel PWM
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	//HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Transmit(&huart1, (const uint8_t*) "HELLO\n", 7, 1000);
	HAL_TIM_Base_Start_IT(&htim6);
//  int pulse = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);

	// LCD TEST
	Lcd_PortType gpio_arr[] = { GPIOA, GPIOA, GPIOA, GPIOA };
	Lcd_PinType pin_arr[] = { GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7 };
	Lcd_HandleTypeDef lcd = Lcd_create(gpio_arr, pin_arr,
	LCD_RS_GPIO_Port, LCD_RS_Pin,
	LCD_E_GPIO_Port, LCD_E_Pin, LCD_4_BIT_MODE);
	char *hello_string = "----------------";

	int8_t ds_presence = 0;
	//uint8_t ds_read_data = 0;
	float t;
	Lcd_string(&lcd, hello_string);
	HAL_Delay(1500);
	Lcd_cursor(&lcd, 0, 0);
	ds.GPIO = DS_PIN_GPIO_Port;
	ds.GPIO_PIN = DS_PIN_Pin;
	ds.TIMER = htim6.Instance;

	strcpy(buffer2, "CO_2 = ----ppm");
//
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		ds_presence = DS18B20_start_t(ds);

		if (ds_presence == DS18B20_NO_ERROR) {
			HAL_Delay(1000);
			Lcd_clear(&lcd);
			if (keyPressed == '1') {
				t = DS18B20_read_t(ds);
				sprintf(buffer1, "T = %2.1foC", t);
			} else if (keyPressed == '2') {
				char buf[17] = { '\0' };
				char *buf_ptr = buf;
				DS18B20_read_rom(ds, buf_ptr);
				strcpy(buffer1, buf);
			} else if (keyPressed == '3') {
				char buf1[17] = { '\0' };
				char *buf_ptr1 = buf1;
				DS18B20_read_scrpad(ds, buf_ptr1);
				strcpy(buffer1, buf1);
			}
			Lcd_clear(&lcd);
			Lcd_string(&lcd, buffer1);
			Lcd_cursor(&lcd, 1, 0);
			Lcd_string(&lcd, buffer2);
		} else {
			sprintf(lcd_buffer, "ERROR %d", ds_presence);
			Lcd_string(&lcd, lcd_buffer);
		}
		//Lcd_clear(&lcd);
		//Lcd_string(&lcd, lcd_buffer);
		HAL_Delay(100);

//		if (is_btn_pressed == 1) {
//			is_btn_pressed = 0;
//			char temp_buf[50];
//			//char *temp_buf_p = temp_buf;
////			keyPressed = KEYPAD4x4_GetPressedKey(GPIO_Pin);
//			sprintf(temp_buf, "Pressed Key = %c\n\r", keyPressed);
////			printf(temp_buf);
//			HAL_UART_Transmit(&huart1, (uint8_t*)temp_buf, strlen(temp_buf), 1000);
//
//		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 24 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 23;
	htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	htim2.Init.Period = 10000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */
	//__HAL_RCC_TIM6_CLK_ENABLE();
	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 24 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void) {

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 24 - 1;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 65535;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			COL_1_Pin | COL_2_Pin | COL_3_Pin | COL_4_Pin | GPIO_PIN_4
					| GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DS_PIN_GPIO_Port, DS_PIN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 | GREEN_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LCD_E_Pin | LCD_RS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : ROW_1_Pin ROW_2_Pin ROW_3_Pin ROW_4_Pin */
	GPIO_InitStruct.Pin = ROW_1_Pin | ROW_2_Pin | ROW_3_Pin | ROW_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : COL_1_Pin COL_2_Pin COL_3_Pin COL_4_Pin
	 PA4 PA5 PA6 PA7 */
	GPIO_InitStruct.Pin = COL_1_Pin | COL_2_Pin | COL_3_Pin | COL_4_Pin
			| GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : DS_PIN_Pin */
	GPIO_InitStruct.Pin = DS_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DS_PIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC7 GREEN_LED_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GREEN_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_E_Pin LCD_RS_Pin */
	GPIO_InitStruct.Pin = LCD_E_Pin | LCD_RS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TOGGLE_GREEN_LED_PIN(void) {
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
