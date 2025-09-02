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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <FreeRTOS.h>
#include <task.h>
#include "semphr.h"
#include "stm32g0xx_hal_conf.h"

#include "canbus.h"
#include "m15_motor.h"

#include "lcd.h"

#include "lvgl.h"
#include "ui.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t POT;
	uint16_t CurrentSense;
	uint16_t Temp;
	uint16_t VREF;

} APP_ADC_BUF_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern objects_t objects;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t  LvglTaskHandle;
TaskHandle_t  UI_TaskHandle;
TaskHandle_t  CAN_TaskHandle;

APP_ADC_BUF_t ADC_Buffer;
float         POT;


#if configGENERATE_RUN_TIME_STATS
volatile unsigned long     ulHighFrequencyTimerTicks;
extern LPTIM_HandleTypeDef hlptim2;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void CAN_Task(void* argument);

void UI_Task(void* argument);

void LVGL_Task(void* argument);

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_FDCAN1_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	TIM2->CCR1 = 0;


	/* Create FreeRTOS tasks */
	xTaskCreate(LVGL_Task, "LVGL Task", 1024, NULL, osPriorityLow, &LvglTaskHandle);
	xTaskCreate(UI_Task, "UI Task", 256, NULL, osPriorityHigh, &UI_TaskHandle);

	xTaskCreate(CAN_Task, "CAN Task", 256, NULL, osPriorityHigh, &CAN_TaskHandle);

	HAL_GPIO_WritePin(RTOS_IDLE_GPIO_Port, RTOS_IDLE_Pin, GPIO_PIN_SET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	vTaskStartScheduler();

	while(1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN            = 8;
	RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV32;
	RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

typedef enum
{
	MOTOR_NO_CHANGE  = 0,
	MOTOR_CYCLE_MODE = 1,
} CAN_COMMANDS_t;

void CAN_Task(void* argument)
{
	uint32_t ulNotifiedValue;

	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Buffer, sizeof(ADC_Buffer) / sizeof(uint16_t));

	/* Configure CANBUS with TX messages and RX filters */
	FDCAN_Config();

	/* Enable the CAN transceiver */
	HAL_GPIO_WritePin(CAN_VIO_GPIO_Port, CAN_VIO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_RESET);

	/* Enable the Motor */
	HAL_GPIO_WritePin(MotorEn_GPIO_Port, MotorEn_Pin, GPIO_PIN_SET);
	vTaskDelay(500);
	M15_Motor_Set_Mode(VELOCITY_CONTROL);
	M15_Motor_Set_SetPoint(0x10);


	for(;;)
	{
		uint32_t Mode;
		xTaskNotifyWaitIndexed(MOTOR_MODE, 0x0000, 0x0000, &Mode, portMAX_DELAY);

		xTaskNotifyWait(0x0000, 0x0000, &ulNotifiedValue, portMAX_DELAY);

		switch(ulNotifiedValue)
		{
		case MOTOR_CYCLE_MODE:
			M15_Motor_Set_Mode(M15_NextMode(Mode));
			xTaskNotifyStateClear(NULL);
			break;
		default:
			break;
		}
	}
}



void UI_Task(void* argument)
{

	uint32_t ButtonPressTime   = 0;
	uint32_t ButtonReleaseTime = 0;

	for(;;)
	{
		xTaskNotifyWaitIndexed(USER_BUTTON_PRESS, 0x0000, 0xFFFF, &ButtonPressTime, portMAX_DELAY);
		xTaskNotifyWaitIndexed(USER_BUTTON_RELEASE, 0x0000, 0xFFFF, &ButtonReleaseTime, portMAX_DELAY);



		if(ButtonPressTime != 0 && ButtonReleaseTime != 0)
		{
			uint32_t ButtonPressDuration = ButtonReleaseTime - ButtonPressTime;

			if(ButtonPressDuration >= USER_LONG_PRESS_TIME)
			{
				// Long press detected

			} else if(ButtonPressDuration > USER_SHORT_PRESS_DEBOUNCE)
			{
				// Short press detected
				xTaskNotify(CAN_TaskHandle, MOTOR_CYCLE_MODE, eSetBits);
			}
			ButtonPressTime   = 0;  // Reset the press time
			ButtonReleaseTime = 0;  // Reset the release time
		}
	}
}

void LVGL_Task(void* argument)
{
	/* Initialize LVGL */
	lv_init();

	/* Initialize LCD I/O */
	if(lcd_io_init() != 0)
		return;

	/* Create the LVGL display object and the LCD display driver */
	lcd_init();

	ui_init();

	uint32_t MotorSpeed;
	uint32_t MotorCurrentmA;
	uint32_t MotorPosition_int_addr;
	uint32_t Fault;
	uint32_t Mode;

	uint8_t  startupflag = 1;

	for(;;)
	{
		float adc_vref = 0;

		/* UPDATE DISPLAYED MOTOR MODE */
		xTaskNotifyWaitIndexed(MOTOR_MODE, 0x0000, 0x0000, &Mode, 0);
		lv_label_set_text(objects.mode_display, M15_Motor_Get_Mode_String(Mode));


		/* UPDATE DISPLAYED MOTOR SPEED */
		xTaskNotifyWaitIndexed(MOTOR_VELOCITY, 0x0000, 0x0000, &MotorSpeed, 0);
		lv_arc_set_value(objects.gauge, (int32_t)(LV_ABS(MotorSpeed)));

		/* UPDATE DISPLAYED MOTOR CURRENT*/
		xTaskNotifyWaitIndexed(MOTOR_CURRENT, 0x0000, 0x0000, &MotorCurrentmA, 0);
		// adc_vref      = __LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC_Buffer.VREF, LL_ADC_RESOLUTION_12B);
		// float current = ((ADC_Buffer.CurrentSense * adc_vref / 4095) - 1650) * 1000 / 132;
		lv_label_set_text_fmt(objects.current_display, "%dmA", MotorCurrentmA);

		/* UPDATE DISPLAYED MOTOR FAULTS */
		xTaskNotifyWaitIndexed(MOTOR_FAULTS, 0x0000, 0x0000, &Fault, 0);
		lv_label_set_text(objects.fault_display, M15_Motor_Get_Fault_String(Fault));

		/* UPDATE DISPLAYED MOTOR POSITION */
		// xTaskNotifyWaitIndexed(MOTOR_POSITION, 0x0000, 0x0000, &MotorPosition_int_addr, 0);
		// float MotorPosition = (int16_t)MotorPosition_int_addr * 0.0109863;
		// lv_label_set_text_fmt(objects.position_display, "%.2f", (float)(int16_t)MotorPosition_int_addr * 0.01f);

		/* UPDATE DISPLAYED MCU TEMPERATURE */
		// adc_vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC_Buffer.VREF, LL_ADC_RESOLUTION_12B);
		// int temp = __LL_ADC_CALC_TEMPERATURE(adc_vref, ADC_Buffer.Temp, LL_ADC_RESOLUTION_12B);
		// lv_label_set_text_fmt(objects.temperature_display, "%d°C", temp);


		/* The task running lv_timer_handler should have lower priority than that running `lv_tick_inc` */
		HAL_GPIO_WritePin(RTOS_IDLE_GPIO_Port, RTOS_IDLE_Pin, GPIO_PIN_RESET);
		lv_timer_handler();
		lv_timer_handler();
		HAL_GPIO_WritePin(RTOS_IDLE_GPIO_Port, RTOS_IDLE_Pin, GPIO_PIN_SET);

		if(startupflag)
		{
			/* Put the 3V3 Buck chip into high performance mode before turning on the display backlight */
			HAL_GPIO_WritePin(LPM3V3_GPIO_Port, LPM3V3_Pin, GPIO_PIN_SET);
			vTaskDelay(10);
			TIM2->CCR1  = 1000;
			startupflag = 0;
		}
	}
}

void freeRTOS_TickHook()
{
	lv_tick_inc(portTICK_PERIOD_MS);
}

void freeRTOS_IdleHook()
{
	/* Code to execute when the RTOS is idle */
	HAL_GPIO_WritePin(RTOS_IDLE_GPIO_Port, RTOS_IDLE_Pin, GPIO_PIN_SET);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	called if a stack overflow is detected. */

	UNUSED(xTask);
	UNUSED(pcTaskName);
	__disable_irq();

	// BSP_GOTOSAFE();

	while(1)
	{
		/* Infinite loop */
	}
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM15 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if(htim->Instance == TIM15)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while(1)
	{ }
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
