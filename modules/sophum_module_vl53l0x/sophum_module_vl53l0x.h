#ifndef __SOPHUM_MODULE_VL53L0X_H__
#define __SOPHUM_MODULE_VL53L0X_H__

#include "vl53l0x_def.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

extern VL53L0X_Error VL53L0X_Init(VL53L0X_Dev_t *pDevice);
extern VL53L0X_Error VL53L0X_doSingleMeasurement(VL53L0X_Dev_t *pDevice, VL53L0X_RangingMeasurementData_t *meas_data);

#endif