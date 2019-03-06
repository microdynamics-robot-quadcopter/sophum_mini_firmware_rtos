#ifndef __SOPHUM_MODULE_BNO055_H__
#define __SOPHUM_MODULE_BNO055_H__

#include "bno055.h"
#include "sophum_driver_i2c.h"

extern int8_t BNO055_readMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
extern int8_t BNO055_writeMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);

#endif