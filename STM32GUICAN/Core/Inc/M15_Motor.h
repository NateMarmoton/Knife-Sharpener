/***************************************************************************************************
 *  @file M15 Motor.h
 *  @author Nathaniel Martin
 *  @date 2025-08-31
 *
 ***************************************************************************************************/


#ifndef M15_MOTOR_H
#define M15_MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32g0xx_hal.h"


	typedef enum
	{
		VOLTAGE_CONTROL  = 0,
		CURRENT_CONTROL  = 1,
		VELOCITY_CONTROL = 2,
		POSITION_CONTROL = 3,
		MOTOR_DISABLED   = 9,
		MOTOR_ENABLED    = 10,
	} M15_Mode_t;

	typedef enum
	{
		NO_FAULT                     = 0,
		UNDER_VOLTAGE_18V            = 1,
		UNDER_VOLTAGE_20V            = 2,
		OVER_VOLTAGE                 = 3,
		OVER_CURRENT                 = 10,
		OVER_SPEED                   = 20,
		OVER_TEMP_120C               = 31,
		OVER_TEMP_80C                = 32,
		SAMPLING_RESISTOR_FAULT      = 41,
		POSITION_SENSOR_FAULT        = 42,
		POSITION_SENSOR_INTERFERENCE = 43,
		TEMPERATURE_SENSOR_FAULT     = 44,
		COMMUNICATION_TIMEOUT        = 60,
		STALL                        = 98,
	} M15_Faults_t;

	const char* M15_Motor_Get_Mode_String(M15_Mode_t mode);

	const char* M15_Motor_Get_Fault_String(M15_Faults_t fault);

	M15_Mode_t  M15_NextMode(M15_Mode_t mode);

	void        M15_Motor_Set_Mode(M15_Mode_t mode);

	void        M15_Motor_Set_SetPoint(uint8_t setpoint);


#ifdef __cplusplus
}
#endif

#endif /* M15_MOTOR_H */
