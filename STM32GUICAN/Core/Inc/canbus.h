/***************************************************************************************************
 *  @file canbus.h
 *  @author Nathaniel Martin
 *  @date 2025-08-29
 *
 *
 *  ---------------------------------------------------------------------------------------------------
 *  @attention
 *  Copyright (c) 2025 CREATOR WAREHOUSE INC.
 *  All rights reserved.
 *
 *  This software is licensed under terms that can be found in the LICENSE file
 *  in the root directory of this software component.
 *  If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ***************************************************************************************************/

#ifndef __CANBUS_H
#define __CANBUS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32g0xx_hal.h"



typedef enum
{
  MOTOR_SPEED,
  MOTOR_CURRENT,
  MOTOR_POSITION,
  FAULT,
  MODE
} MotorFeedbackTaskNotificationIndex_t;


	void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs);

	void FDCAN_Config(void);

#ifdef __cplusplus
}
#endif

#endif
