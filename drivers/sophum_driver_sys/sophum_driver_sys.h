#ifndef __SOPHUM_DRIVER_SYS_H__
#define __SOPHUM_DRIVER_SYS_H__

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

extern void SOPHUM_delayMs(uint32_t nms);
extern void SOPHUM_delayUs(uint16_t nus);

#endif
