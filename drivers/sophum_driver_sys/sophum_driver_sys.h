#ifndef __SOPHUM_DRIVER_SYS_H__
#define __SOPHUM_DRIVER_SYS_H__

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

extern void SOPHUM_delayMs(int32_t nms);
extern void SOPHUM_delayUs(int16_t nus);

#endif
