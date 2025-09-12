/***************************************************************************************************
 *  @file M15_Motor.c
 *  @author Nathaniel Martin
 *  @date 2025-08-31
 *
 *
 *
 ***************************************************************************************************/




#include "M15_Motor.h"
#include "canbus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

extern FDCAN_HandleTypeDef   hfdcan1;

extern FDCAN_TxHeaderTypeDef CalibrateTxHeader;
extern FDCAN_TxHeaderTypeDef ModeTxHeader;
extern FDCAN_TxHeaderTypeDef SetPointTxHeader;
extern FDCAN_TxHeaderTypeDef FeedbackFrequencyTxHeader;
extern uint8_t               TxData[8];


/*  */

void M15_Motor_Reset(void)
{
	HAL_GPIO_WritePin(MotorEn_GPIO_Port, MotorEn_Pin, GPIO_PIN_RESET);
	vTaskDelay(500);
	HAL_GPIO_WritePin(MotorEn_GPIO_Port, MotorEn_Pin, GPIO_PIN_SET);
	vTaskDelay(500);
}

const char* M15_Motor_Get_Fault_String(M15_Faults_t fault)
{
	switch(fault)
	{
	case NO_FAULT:
		return " ";
	case UNDER_VOLTAGE_18V:
		return "Under Voltage 18V";
	case UNDER_VOLTAGE_20V:
		return "Under Voltage 20V";
	case OVER_VOLTAGE:
		return "Over Voltage";
	case OVER_CURRENT:
		return "Over Current";
	case OVER_SPEED:
		return "Over Speed";
	case OVER_TEMP_120C:
		return "Over Temperature 120C";
	case OVER_TEMP_80C:
		return "Over Temperature 80C";
	case SAMPLING_RESISTOR_FAULT:
		return "Sampling Resistor Fault";
	case POSITION_SENSOR_FAULT:
		return "Position Sensor Fault";
	case POSITION_SENSOR_INTERFERENCE:
		return "Position Sensor Interference";
	case TEMPERATURE_SENSOR_FAULT:
		return "Temperature Sensor Fault";
	case COMMUNICATION_TIMEOUT:
		return "Communication Timeout";
	case STALL:
		return "Stall";
	default:
		return "Unknown Fault";
	}
}

void M15_Motor_Set_Mode(M15_Mode_t mode)
{
	TxData[0] = mode;
	TxData[1] = 0x00;
	TxData[2] = 0x00;
	TxData[3] = 0x00;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;

	/* Start the Transmission process */
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ModeTxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler();
	}
}

void M15_Motor_Set_SetPoint(uint16_t setpoint)
{
	TxData[0] = (setpoint >> 8) & 0xFF;
	TxData[1] = setpoint & 0xFF;
	TxData[2] = 0x00;
	TxData[3] = 0x00;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;

	/* Start the Transmission process */
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &SetPointTxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler();
	}
}

void M15_Motor_Set_Feedback_Frequency(uint8_t frequency)
{
	if(frequency == 0)
	{
		TxData[0] = 0x80;
		TxData[1] = 0x80;
		TxData[2] = 0x80;
		TxData[3] = 0x80;
		TxData[4] = 0x80;
		TxData[5] = 0x80;
		TxData[6] = 0x80;
		TxData[7] = 0x80;
	} else
	{
		// Limit to 7 bits
		TxData[0] = frequency & 0x7F;
		TxData[1] = frequency & 0x7F;
		TxData[2] = frequency & 0x7F;
		TxData[3] = frequency & 0x7F;
		TxData[4] = frequency & 0x7F;
		TxData[5] = frequency & 0x7F;
		TxData[6] = frequency & 0x7F;
		TxData[7] = frequency & 0x7F;
	}

	/* Start the Transmission process */
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FeedbackFrequencyTxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler();
	}
}

void M15_Motor_Calibrate(void)
{

	TxData[0] = 0x00;
	TxData[1] = 0x00;
	TxData[2] = 0x00;
	TxData[3] = 0x00;
	TxData[4] = 0x00;
	TxData[5] = 0x00;
	TxData[6] = 0x00;
	TxData[7] = 0x00;

	/* Start the Transmission process */
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CalibrateTxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler();
	}
}
