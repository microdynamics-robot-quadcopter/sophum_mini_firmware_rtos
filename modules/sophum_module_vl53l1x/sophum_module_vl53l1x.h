#ifndef __SOPHUM_MODULE_VL53L1X_H__
#define __SOPHUM_MODULE_VL53L1X_H__

#include "vl53l1_def.h"
#include ""
#include "vl53l1_api.h"
#include "vl53l1_api_calibration.h"
#include "vl53l1_platform.h"

extern VL53L1X_Error VL53L1X_Init(VL53L1_Dev_t *pDevice);
extern VL53L1X_Error VL53L1X_doSingleMeasurement(VL53L1_Dev_t *pDevice, uint16_t *meas_data);

#endif