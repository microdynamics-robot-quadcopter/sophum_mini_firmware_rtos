#ifndef __SOPHUM_MODULE_BMX055_H__
#define __SOPHUM_MODULE_BMX055_H__

#include "bma2x2.h"
#include "bmg160.h"
#include "bmm050.h"
#include "sophum_driver_sys.h"
#include "sophum_driver_i2c.h"


extern int8_t BMX055_readMultiBytes(uint8_t dev_id,  uint8_t reg_addr, uint8_t *data, uint8_t len);
extern int8_t BMX055_writeMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);
extern void BMX055_Init(void);
extern void BMX055_updateData(void);
#endif