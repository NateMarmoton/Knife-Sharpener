/***************************************************************************************************
 *  @file LCDController.h
 *  @author Nathaniel Martin
 *  @date 2025-08-18
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

#ifndef LCDCONTROLLER_H
#define LCDCONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/* Initialize low level display driver */
void lv_port_disp_init(void);

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void);

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LCDCONTROLLER_H*/

