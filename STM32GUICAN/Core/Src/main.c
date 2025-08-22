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
#include "crc.h"
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

#include "lvgl.h"
#include "./src/drivers/display/st7789/lv_st7789.h"
#include "ui.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t POT;
	uint16_t CurrentSense;
	uint16_t Temp;

} APP_ADC_BUF_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_H_RES             240
#define LCD_V_RES             320
#define BUS_SPI1_POLL_TIMEOUT 0x1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t  LvglTaskHandle;
lv_display_t* lcd_disp;
volatile int  lcd_bus_busy = 0;

TaskHandle_t  ADC_TaskHandle;
APP_ADC_BUF_t ADC_Buffer;
float         Temperature;
float         POT;
float         CurrentSense;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void          LVGL_Task(void* argument);

void          ADC_Task(void* argument);

void          configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void          configureTimerForRunTimeStats(void) { }

unsigned long getRunTimeCounterValue(void)
{
	return 0;
}




void GUITask(void* argument)
{
	UNUSED(argument);
	for(;;)
	{
		uint32_t time_till_next;
		time_till_next = lv_timer_handler(); /*lv_lock/lv_unlock is called internally*/
		vTaskDelay(time_till_next);          /* sleep for a while */
	}
}

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
	MX_CRC_Init();
	MX_ADC1_Init();
	MX_FDCAN1_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	// Initialise LVGL UI library

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	TIM2->CCR1 = 1000;


	/* Create FreeRTOS tasks */
	xTaskCreate(LVGL_Task, "LVGL Task", 1024, NULL, osPriorityNormal - 1, &LvglTaskHandle);
	xTaskCreate(ADC_Task, "ADC Task", 256, NULL, osPriorityNormal, &ADC_TaskHandle);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	vTaskStartScheduler();

	while(1)
	{
		/* USER CODE END WHILE */
		__NOP();
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
	RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
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


void lcd_color_transfer_ready_cb(SPI_HandleTypeDef* hspi)
{
	/* CS high */
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
	lcd_bus_busy = 0;
	lv_display_flush_ready(lcd_disp);
}

/* Initialize LCD I/O bus, reset LCD */
static int32_t lcd_io_init(void)
{
	/* Register SPI Tx Complete Callback */
	HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_TX_COMPLETE_CB_ID, lcd_color_transfer_ready_cb);

	/* reset LCD */
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);

	return HAL_OK;
}

/* Platform-specific implementation of the LCD send command function. In general this should use polling transfer. */
static void lcd_send_cmd(lv_display_t* disp, const uint8_t* cmd, size_t cmd_size, const uint8_t* param, size_t param_size)
{
	LV_UNUSED(disp);
	while(lcd_bus_busy)
		; /* wait until previous transfer is finished */
	/* Set the SPI in 8-bit mode */
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	HAL_SPI_Init(&hspi1);
	/* DCX low (command) */
	HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_RESET);
	/* CS low */
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	/* send command */
	if(HAL_SPI_Transmit(&hspi1, cmd, cmd_size, BUS_SPI1_POLL_TIMEOUT) == HAL_OK)
	{
		/* DCX high (data) */
		HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);
		/* for short data blocks we use polling transfer */
		HAL_SPI_Transmit(&hspi1, (uint8_t*)param, (uint16_t)param_size, BUS_SPI1_POLL_TIMEOUT);
		/* CS high */
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
	}
}

/* Platform-specific implementation of the LCD send color function. For better performance this should use DMA transfer.
 * In case of a DMA transfer a callback must be installed to notify LVGL about the end of the transfer.
 */
static void lcd_send_color(lv_display_t* disp, const uint8_t* cmd, size_t cmd_size, uint8_t* param, size_t param_size)
{
	LV_UNUSED(disp);
	while(lcd_bus_busy)
		; /* wait until previous transfer is finished */
	/* Set the SPI in 8-bit mode */
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	HAL_SPI_Init(&hspi1);
	/* DCX low (command) */
	HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_RESET);
	/* CS low */
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	/* send command */
	if(HAL_SPI_Transmit(&hspi1, cmd, cmd_size, BUS_SPI1_POLL_TIMEOUT) == HAL_OK)
	{
		/* DCX high (data) */
		HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);
		/* for color data use DMA transfer */
		/* Set the SPI in 16-bit mode to match endianness */
		hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
		HAL_SPI_Init(&hspi1);
		lcd_bus_busy = 1;
		HAL_SPI_Transmit_DMA(&hspi1, param, (uint16_t)param_size / 2);
		/* NOTE: CS will be reset in the transfer ready callback */
	}
}


void action_turn_on_disp(lv_event_t* e)
{
	LV_UNUSED(e);
	TIM2->CCR1 = 1000;
}

void LVGL_Task(void* argument)
{
	/* Initialize LVGL */
	lv_init();

	/* Initialize LCD I/O */
	if(lcd_io_init() != 0)
		return;

	/* Create the LVGL display object and the LCD display driver */
	lcd_disp = lv_st7789_create(LCD_H_RES, LCD_V_RES, LV_LCD_FLAG_NONE, lcd_send_cmd, lcd_send_color);
	lv_display_set_rotation(lcd_disp, LV_DISPLAY_ROTATION_270);



	/* Allocate draw buffers on the heap. In this example we use two partial buffers of 1/10th size of the screen */
	lv_color_t* buf1 = NULL;
	lv_color_t* buf2 = NULL;

	uint32_t    buf_size = LCD_H_RES * LCD_V_RES / 100 * lv_color_format_get_size(lv_display_get_color_format(lcd_disp));

	buf1 = lv_malloc(buf_size);
	if(buf1 == NULL)
	{
		LV_LOG_ERROR("display draw buffer malloc failed");
		return;
	}

	buf2 = lv_malloc(buf_size);
	if(buf2 == NULL)
	{
		LV_LOG_ERROR("display buffer malloc failed");
		lv_free(buf1);
		return;
	}
	lv_display_set_buffers(lcd_disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

	ui_init();

	for(;;)
	{
		/* The task running lv_timer_handler should have lower priority than that running `lv_tick_inc` */
		lv_timer_handler();
		/* raise the task priority of LVGL and/or reduce the handler period can improve the performance */
		vTaskDelay(10);
	}
}

void ADC_Task(void* argument)
{
	UNUSED(argument);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Buffer, sizeof(ADC_Buffer) / sizeof(uint16_t));

	/* Infinite loop */
	for(;;)
	{
		vTaskDelay(1000);
		Temperature  = __LL_ADC_CALC_TEMPERATURE(3312, ADC_Buffer.Temp, LL_ADC_RESOLUTION_12B);
		CurrentSense = ((ADC_Buffer.CurrentSense * 3312 / 4095) - 1650) * 1000 / 132;
		__NOP();
	}
}

void freeRTOS_TickHook()
{
	lv_tick_inc(portTICK_PERIOD_MS);
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
	if(hspi == &hspi1)
	{ }
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
 * @note   This function is called  when TIM17 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if(htim->Instance == TIM17)
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
