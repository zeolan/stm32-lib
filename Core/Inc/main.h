/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
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
#define ROW_1_Pin GPIO_PIN_0
#define ROW_1_GPIO_Port GPIOC
#define ROW_1_EXTI_IRQn EXTI0_IRQn
#define ROW_2_Pin GPIO_PIN_1
#define ROW_2_GPIO_Port GPIOC
#define ROW_2_EXTI_IRQn EXTI1_IRQn
#define ROW_3_Pin GPIO_PIN_2
#define ROW_3_GPIO_Port GPIOC
#define ROW_3_EXTI_IRQn EXTI2_IRQn
#define ROW_4_Pin GPIO_PIN_3
#define ROW_4_GPIO_Port GPIOC
#define ROW_4_EXTI_IRQn EXTI3_IRQn
#define COL_1_Pin GPIO_PIN_0
#define COL_1_GPIO_Port GPIOA
#define COL_2_Pin GPIO_PIN_1
#define COL_2_GPIO_Port GPIOA
#define COL_3_Pin GPIO_PIN_2
#define COL_3_GPIO_Port GPIOA
#define COL_4_Pin GPIO_PIN_3
#define COL_4_GPIO_Port GPIOA
#define DS_PIN_Pin GPIO_PIN_0
#define DS_PIN_GPIO_Port GPIOB
#define CO2_PWM_Pin GPIO_PIN_14
#define CO2_PWM_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_9
#define GREEN_LED_GPIO_Port GPIOC
#define UART1_TX_Pin GPIO_PIN_6
#define UART1_TX_GPIO_Port GPIOB
#define UART1_RX_Pin GPIO_PIN_7
#define UART1_RX_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_8
#define LCD_E_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
