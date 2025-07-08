#include <Arduino.h>
#include "FreeRTOS.h"


#ifndef MAIN_H
#define MAIN_H

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */


void BlinkyTask(void* pvParameters);
void debugSysTick();


#endif // MAIN_H