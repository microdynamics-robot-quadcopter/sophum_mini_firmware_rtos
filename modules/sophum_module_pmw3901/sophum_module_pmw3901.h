#ifndef __SOPHUM_MODULE_PMW3901_H__
#define __SOPHUM_MODULE_PMW3901_H__

#include <stdio.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "sophum_driver_spi.h"
#include "sophum_driver_sys.h"

#define PMW3901_CS_PIN        GPIO_NUM_33
#define PMW3901_CS_PIN_MASK   GPIO_SEL_33


extern bool PMW3901_Init(void);
extern void PMW3901_readMotionCount(int16_t *delta_x, int16_t *delta_y);

#endif