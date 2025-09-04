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
extern FDCAN_TxHeaderTypeDef ModeTxHeader;
extern FDCAN_TxHeaderTypeDef SetPointTxHeader;
extern uint8_t               TxData[8];


const char*                  M15_Motor_Get_Mode_String(M15_Mode_t mode)
{
	switch(mode)
	{
	case VOLTAGE_CONTROL:
		return "Voltage Control";
	case CURRENT_CONTROL:
		return "Current Control";
	case VELOCITY_CONTROL:
		return "Velocity Control";
	case POSITION_CONTROL:
		return "Position Control";
	case MOTOR_DISABLED:
		return "Motor Disabled";
	case MOTOR_ENABLED:
		return "Motor Enabled";
	default:
		return "Unknown Mode";
	}
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

M15_Mode_t M15_NextMode(M15_Mode_t mode)
{
	switch(mode)
	{
	case VOLTAGE_CONTROL:
		return CURRENT_CONTROL;
	case CURRENT_CONTROL:
		return VELOCITY_CONTROL;
	case VELOCITY_CONTROL:
		return POSITION_CONTROL;
	case POSITION_CONTROL:
		return VOLTAGE_CONTROL;

	/* Do nothing for MOTOR_ENABLED */
	case MOTOR_ENABLED:
		return MOTOR_ENABLED;
	default:
		return MOTOR_DISABLED;
	}
}

void M15_Motor_Set_Mode(M15_Mode_t mode)
{
	TxData[0] = 0x00;
	TxData[1] = mode;
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

void M15_Motor_Set_SetPoint(uint8_t setpoint)
{
	TxData[0] = 0x00;
	TxData[1] = 0x00;
	TxData[2] = setpoint;
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