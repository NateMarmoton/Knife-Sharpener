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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lvgl.h"
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
	typedef enum
	{
		MOTOR_MODE = 1,
		MOTOR_FAULTS,
		MOTOR_RPM,
		MOTOR_CURRENT,
		MOTOR_POSITION,
		USER_BUTTON_PRESS,    // Button pressed
		USER_BUTTON_RELEASE,  // Button released
	} TaskNotifications_t;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ST7789_SPI SPI1
#define ST7789_DMA DMA2_Channel1
#define PB_Pin GPIO_PIN_14
#define PB_GPIO_Port GPIOC
#define PB_EXTI_IRQn EXTI4_15_IRQn
#define LCD_RESET_Pin GPIO_PIN_0
#define LCD_RESET_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_1
#define LCD_SCK_GPIO_Port GPIOA
#define LCD_SDI_Pin GPIO_PIN_2
#define LCD_SDI_GPIO_Port GPIOA
#define LCD_DCX_Pin GPIO_PIN_3
#define LCD_DCX_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define BL_Pin GPIO_PIN_5
#define BL_GPIO_Port GPIOA
#define POT_Pin GPIO_PIN_0
#define POT_GPIO_Port GPIOB
#define CurrentSense_Pin GPIO_PIN_1
#define CurrentSense_GPIO_Port GPIOB
#define LPM3V3_Pin GPIO_PIN_8
#define LPM3V3_GPIO_Port GPIOA
#define MotorEn_Pin GPIO_PIN_9
#define MotorEn_GPIO_Port GPIOA
#define RTOS_IDLE_Pin GPIO_PIN_11
#define RTOS_IDLE_GPIO_Port GPIOA
#define CAN_STB_Pin GPIO_PIN_6
#define CAN_STB_GPIO_Port GPIOB
#define CAN_VIO_Pin GPIO_PIN_7
#define CAN_VIO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define USER_SHORT_PRESS_DEBOUNCE 10    // ms
#define USER_LONG_PRESS_TIME      2000  // ms
#define USER_DOUBLE_PRESS_TIME    100   // ms

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
