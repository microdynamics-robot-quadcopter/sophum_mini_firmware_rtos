#ifndef __SOPHUM_MODULE_BMP388_H__
#define __SOPHUM_MODULE_BMP388_H__

#include <math.h>
#include "bmp3.h"
#include "bmp3_defs.h"
#include "sophum_driver_sys.h"
#include "sophum_driver_i2c.h"



extern int8_t BMP388_readMultiBytes(uint8_t dev_id,  uint8_t reg_addr, uint8_t *data, uint16_t len);
extern int8_t BMP388_writeMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
extern double BMP388_calcAltitude(double pres);













#endif