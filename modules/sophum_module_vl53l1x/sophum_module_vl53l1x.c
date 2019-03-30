#include "sophum_module_vl53l1x.h"

static const char *TAG = "SOPHUM_MODULE_VL53L1X";

static void VL53L1X_printError(VL53L1_Error status, const char *method)
{
    if(status == VL53L1_ERROR_NONE) return;

    char buf[VL53L1_MAX_STRING_LENGTH];
    VL53L1_GetPalErrorString(status, buf);
    ESP_LOGI(TAG, "%s API status: %i : %s\n", method, status, buf);
}

VL53L1_Error VL53L1X_Init(VL53L1_Dev_t *pDevice)
{
    uint8_t byteData;
    uint16_t wordData;
    VL53L1_RdByte(pDevice, 0x010F, &byteData);
    printf("VL53L1X Model_ID: %02X\n\r", byteData);
    VL53L1_RdByte(pDevice, 0x0110, &byteData);
    printf("VL53L1X Module_Type: %02X\n\r", byteData);
    VL53L1_RdWord(pDevice, 0x010F, &wordData);
    printf("VL53L1X: %02X\n\r", wordData);
    VL53L1_Error status;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    status = VL53L1_DataInit(pDevice);
    VL53L1X_printError(status, "VL53L1X_DataInit");

    if(status == VL53L1_ERROR_NONE)
    {
        status = VL53L1_StaticInit(pDevice); // Device Initialization
        VL53L1X_printError(status, "VL53L1X_StaticInit");
    }

    // if(status == VL53L1_ERROR_NONE)
    // {
    //     status = VL53L1_PerformRefCalibration(pDevice,
    //                                           &VhvSettings, &PhaseCal); // Device Initialization
    //     VL53L1X_printError(status, "VL53L1X_PerformRefCalibration");
    // }

    // if(status == VL53L1_ERROR_NONE)
    // {
    //     status = VL53L1_PerformRefSpadManagement(pDevice,
    //                                             &refSpadCount, &isApertureSpads); // Device Initialization
    //     ESP_LOGI(TAG,"refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
    //     VL53L1X_printError(status, "VL53L1X_PerformRefSpadManagement");
    // }

    // if(status == VL53L1_ERROR_NONE)
    // {
    //     status = VL53L1_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    //     VL53L1X_printError(status, "VL53L1X_SetDeviceMode");
    // }

    if(status == VL53L1_ERROR_NONE)
    {
        status = VL53L1_SetDistanceMode(pDevice, VL53L1_DISTANCEMODE_LONG);
        VL53L1X_printError(status, "VL53L1X_SetDistanceMode");
    }

    // // Enable/Disable Sigma and Signal check
    // if (status == VL53L1_ERROR_NONE) {
    //     status = VL53L1_SetLimitCheckEnable(pDevice,
    //                                         VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    //     VL53L1X_printError(status, "VL53L1X_SetLimiteCheckEnable");
    // }
    // if (status == VL53L1_ERROR_NONE) {
    //     status = VL53L1_SetLimitCheckEnable(pDevice,
    //                                         VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    //     VL53L1X_printError(status, "VL53L1X_SetLimiteCheckEnable");
    // }

    // if (status == VL53L1_ERROR_NONE) {
    //     status = VL53L1_SetLimitCheckEnable(pDevice,
    //                                         VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    //     VL53L1X_printError(status, "VL53L1X_SetLimiteCheckEnable");
    // }

    // //if (status == VL53L1_ERROR_NONE) {
    // //    status = VL53L0X_SetLimitCheckValue(pDevice,
    // //                                        VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
    // //                                        (FixPoint1616_t)(1.5*0.023*65536));
    // //    VL53L1X_printError(status, "VL53L1X_SetLimitCheckValue");
    // //}

    // if (status == VL53L1_ERROR_NONE) {
    //     status = VL53L1_SetLimitCheckValue(pDevice,
    //                                         VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
    //                                         (FixPoint1616_t)(0.25*65536));
    //     VL53L1X_printError(status, "VL53L1X_SetLimitCheckValue");
    // }

    // if (status == VL53L1_ERROR_NONE) {
    //     status = VL53L1_SetLimitCheckValue(pDevice,
    //                                         VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
    //                                         (FixPoint1616_t)(18*65536));
    //     VL53L1X_printError(status, "VL53L1X_SetLimitCheckValue");
    // }

    if (status != VL53L1_ERROR_NONE)
    {
        status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(pDevice, 50000);
        VL53L1X_printError(status, "VL53L1X_SetMeasurementTimingBudgetMicroSeconds");
    }

    if (status != VL53L1_ERROR_NONE)
    {
        status = VL53L1_SetInterMeasurementPeriodMilliSeconds(pDevice, 100);
        VL53L1X_printError(status, "VL53L1X_SetMeasurementTimingBudgetMicroSeconds");
    }

    status = VL53L1_StartMeasurement(pDevice);

    return status;
    // return 0;
}

VL53L1_Error VL53L1X_doSingleMeasurement(VL53L1_Dev_t *pDevice, VL53L1_RangingMeasurementData_t *meas_data)
{
    VL53L1_Error status = VL53L1_WaitMeasurementDataReady(pDevice);
    if(status == 0)
    {
        status = VL53L1_GetRangingMeasurementData(pDevice, meas_data);
        status = VL53L1_ClearInterruptAndStartMeasurement(pDevice);
    }
    return status;
}