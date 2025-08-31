/***************************************************************************************************
 *  @file lcd.h
 *  @author Nathaniel Martin
 *  @date 2025-08-30
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

#ifndef LCD_H
#define LCD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "lvgl.h"


#define LCD_H_RES             240
#define LCD_V_RES             320
#define BUS_SPI1_POLL_TIMEOUT 0x1000U


void lcd_init(void);
void lcd_color_transfer_ready_cb(SPI_HandleTypeDef* hspi);
int32_t lcd_io_init(void);
void lcd_send_cmd(lv_display_t* disp, const uint8_t* cmd, size_t cmd_size, const uint8_t* param, size_t param_size);
void lcd_send_color(lv_display_t* disp, const uint8_t* cmd, size_t cmd_size, uint8_t* param, size_t param_size);






#ifdef __cplusplus
}
#endif

#endif /* LCD_H */