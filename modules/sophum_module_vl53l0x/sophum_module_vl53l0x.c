#include "sophum_module_vl53l0x.h"

static const char *TAG = "SOPHUM_MODULE_VL53L0X";

static void VL53L0X_printError(VL53L0X_Error status, const char *method)
{
    if(status == VL53L0X_ERROR_NONE) return;

    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(status, buf);
    ESP_LOGI(TAG, "%s API status: %i : %s\n", method, status, buf);
}

VL53L0X_Error VL53L0X_Init(VL53L0X_Dev_t *pDevice)
{
    VL53L0X_Error status;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    status = VL53L0X_DataInit(pDevice);
    VL53L0X_printError(status, "VL53L0X_DataInit");

    if(status == VL53L0X_ERROR_NONE)
    {
        status = VL53L0X_StaticInit(pDevice); // Device Initialization
        VL53L0X_printError(status, "VL53L0X_StaticInit");
    }

    if(status == VL53L0X_ERROR_NONE)
    {
        status = VL53L0X_PerformRefCalibration(pDevice,
                                               &VhvSettings, &PhaseCal); // Device Initialization
        VL53L0X_printError(status, "VL53L0X_PerformRefCalibration");
    }

    if(status == VL53L0X_ERROR_NONE)
    {
        status = VL53L0X_PerformRefSpadManagement(pDevice,
                                                  &refSpadCount, &isApertureSpads); // Device Initialization
        ESP_LOGI(TAG,"refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
        VL53L0X_printError(status, "VL53L0X_PerformRefSpadManagement");
    }

    if(status == VL53L0X_ERROR_NONE)
    {
        status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        VL53L0X_printError(status, "VL53L0X_SetDeviceMode");
    }

    // Enable/Disable Sigma and Signal check
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(pDevice,
                                             VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        VL53L0X_printError(status, "VL53L0X_SetLimiteCheckEnable");
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(pDevice,
                                             VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        VL53L0X_printError(status, "VL53L0X_SetLimiteCheckEnable");
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckEnable(pDevice,
                                             VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
        VL53L0X_printError(status, "VL53L0X_SetLimiteCheckEnable");
    }

    //if (status == VL53L0X_ERROR_NONE) {
    //    status = VL53L0X_SetLimitCheckValue(pDevice,
    //                                        VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
    //                                        (FixPoint1616_t)(1.5*0.023*65536));
    //    VL53L0X_printError(status, "VL53L0X_SetLimitCheckValue");
    //}

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckValue(pDevice,
                                            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                            (FixPoint1616_t)(0.25*65536));
        VL53L0X_printError(status, "VL53L0X_SetLimitCheckValue");
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetLimitCheckValue(pDevice,
                                            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                            (FixPoint1616_t)(18*65536));
        VL53L0X_printError(status, "VL53L0X_SetLimitCheckValue");
    }

    if (status != VL53L0X_ERROR_NONE)
    {
        status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice,200000);
        VL53L0X_printError(status, "VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
    }

    return status;
}

VL53L0X_Error Vl53L0x_doSingleMeasurement(VL53L0X_Dev_t *pDevice, VL53L0X_RangingMeasurementData_t *meas_data)
{
    VL53L0X_Error status = VL53L0X_PerformSingleRangingMeasurement(pDevice, meas_data);
    VL53L0X_printError(status, "VL53L0X_PerformSingleRangingMeasurement");
    return status;
}
