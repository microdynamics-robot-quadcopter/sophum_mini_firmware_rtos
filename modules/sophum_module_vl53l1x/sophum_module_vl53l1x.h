#ifndef __SOPHUM_MODULE_VL53L1X_H__
#define __SOPHUM_MODULE_VL53L1X_H__

#include "esp_err.h"
#include "esp_log.h"
#include "vl53l1_def.h"
#include "vl53l1_api.h"
#include "vl53l1_platform.h"

extern VL53L1_Error VL53L1X_Init(VL53L1_Dev_t *pDevice);
extern VL53L1_Error VL53L1X_doSingleMeasurement(VL53L1_Dev_t *pDevice, VL53L1_RangingMeasurementData_t *meas_data);

#endif