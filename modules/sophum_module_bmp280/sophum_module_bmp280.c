#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "sophum_driver_i2c.h"
#include "sophum_module_bmp280.h"


typedef struct
{
    uint16_t dig_T1; /* calibration T1 data */
    int16_t  dig_T2; /* calibration T2 data */
    int16_t  dig_T3; /* calibration T3 data */
    uint16_t dig_P1; /* calibration P1 data */
    int16_t  dig_P2; /* calibration P2 data */
    int16_t  dig_P3; /* calibration P3 data */
    int16_t  dig_P4; /* calibration P4 data */
    int16_t  dig_P5; /* calibration P5 data */
    int16_t  dig_P6; /* calibration P6 data */
    int16_t  dig_P7; /* calibration P7 data */
    int16_t  dig_P8; /* calibration P8 data */
    int16_t  dig_P9; /* calibration P9 data */
    int32_t  t_fine; /* calibration t_fine data */
}BMP280_TrimParamType;


/* private variable */
static BMP280_TrimParamType g_bmp280_trim;

/* private operation */
static bool BMP280_testConnection(void);
static int32_t BMP280_getRawPressure(void);
static int32_t BMP280_getRawTemperature(void);
static uint32_t BMP280_getCompensateTemp(int32_t adc_pres);
static uint32_t BMP280_getCompensatePres(int32_t adc_temp);
static void BMP280_getPressureFilter(float* in, float* out);
// static void BMP280_calc


static bool BMP280_testConnection(void)
{
    uint8_t bmp280_id = 0x00;
    I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, BMP280_ADDR, BMP280_ID, &bmp280_id);

    return bmp280_id == BMP280_ID_CHIP_ID;
}

static int32_t BMP280_getRawPressure(void)
{
    uint8_t tmp_data[BMP280_DATA_FRAME_SIZE];

    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, BMP280_ADDR, BMP280_PRESS_MSB,
                       BMP280_DATA_FRAME_SIZE, tmp_data);

    return (int32_t)((((uint32_t)(tmp_data[0])) << 12) | (((uint32_t)(tmp_data[1])) << 4) |
                      ((uint32_t)tmp_data[2] >> 4));
}

static int32_t BMP280_getRawTemperature(void)
{
    uint8_t tmp_data[BMP280_DATA_FRAME_SIZE];

    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, BMP280_ADDR, BMP280_TEMP_MSB,
                       BMP280_DATA_FRAME_SIZE, tmp_data);

    return (int32_t)((((uint32_t)(tmp_data[0])) << 12) | (((uint32_t)(tmp_data[1])) << 4) |
                      ((uint32_t)tmp_data[2] >> 4));
}

/* returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC */
/* t_fine carries fine temperature as global value */
static uint32_t BMP280_getCompensateTemp(int32_t adc_temp)
{
    int32_t tmp_var1, tmp_var2, tmp_T;

    tmp_var1 = ((((adc_temp >> 3) - ((int32_t)g_bmp280_trim.dig_T1 << 1))) * ((int32_t)g_bmp280_trim.dig_T2)) >> 11;
    tmp_var2 = (((((adc_temp >> 4) - ((int32_t)g_bmp280_trim.dig_T1)) * ((adc_temp >> 4) -
               ((int32_t)g_bmp280_trim.dig_T1))) >> 12) * ((int32_t)g_bmp280_trim.dig_T3)) >> 14;

    g_bmp280_trim.t_fine = tmp_var1 + tmp_var2;
	
    tmp_T = (g_bmp280_trim.t_fine * 5 + 128) >> 8;

    return tmp_T;
}

/* returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits) */
/* Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa */
static uint32_t BMP280_getCompensatePres(int32_t adc_pres)
{
    int64_t tmp_var1, tmp_var2, tmp_pres;
    tmp_var1 = ((int64_t)g_bmp280_trim.t_fine) - 128000;
    tmp_var2 = tmp_var1 * tmp_var1 * (int64_t)g_bmp280_trim.dig_P6;
    tmp_var2 = tmp_var2 + ((tmp_var1 * (int64_t)g_bmp280_trim.dig_P5) << 17);
    tmp_var2 = tmp_var2 + (((int64_t)g_bmp280_trim.dig_P4) << 35);
    tmp_var1 = ((tmp_var1 * tmp_var1 * (int64_t)g_bmp280_trim.dig_P3) >> 8) + ((tmp_var1 * (int64_t)g_bmp280_trim.dig_P2) << 12);
    tmp_var1 = (((((int64_t)1) << 47) + tmp_var1)) * ((int64_t)g_bmp280_trim.dig_P1) >> 33;

    if (tmp_var1 == 0) return 0;

    tmp_pres = 1048576 - adc_pres;
    tmp_pres = (((tmp_pres << 31) - tmp_var2) * 3125) / tmp_var1;
    tmp_var1 = (((int64_t)g_bmp280_trim.dig_P9) * (tmp_pres >> 13) * (tmp_pres >> 13)) >> 25;
    tmp_var2 = (((int64_t)g_bmp280_trim.dig_P8) * tmp_pres) >> 19;
    tmp_pres = ((tmp_pres + tmp_var1 + tmp_var2) >> 8) + (((int64_t)g_bmp280_trim.dig_P7) << 4);
    return (uint32_t)tmp_pres;
}


/* limited filter method */
static void BMP280_getPressureFilter(float* in, float* out)
{
    static uint8_t i = 0;
    static float filter_buf[BMP280_FILTER_NUM] = {0.0F};

    float tmp_deta;
    double filter_sum = 0.0F;

    if(filter_buf[i] == 0.0F)
    {
        filter_buf[i] = *in;
        *out = *in;
        if(++i >= BMP280_FILTER_NUM) i = 0;
	}
    else
    {
        if(i) tmp_deta = *in - filter_buf[i-1];
        else tmp_deta =*in - filter_buf[BMP280_FILTER_NUM-1];

        if(fabs(tmp_deta) < BMP280_FILTER_A)
        {
            filter_buf[i] = *in;
            if(++i >= BMP280_FILTER_NUM) i = 0;
        }

        for(int cnt = 0; cnt < BMP280_FILTER_NUM; cnt++)
        {
            filter_sum += filter_buf[cnt];
        }
        *out = filter_sum / BMP280_FILTER_NUM;
    }
}

void BMP280_Init(void)
{
    if(BMP280_testConnection() == false)
    {
        printf("BMP280 test connection ERROR!!!\n");
        return;
    }

    printf("BMP280 test connection SUCCESS!!!\n");

    /* read cali params */
    I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, BMP280_ADDR, BMP280_TEMPE_CALIB_DIG_T1,
                       BMP280_CALIB_DATA_LEN, (uint8_t*) &g_bmp280_trim);

    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, BMP280_ADDR, BMP280_CTRL_MEAS,
                     BMP280_MODE);
    I2C_writeOneByte(I2C_NUM1_MASTER_PORT_GPIO, BMP280_ADDR, BMP280_CONFIG,
                     BMP280_FILTER_CONFIG);
}

void BMP280_updateData(float* temp, float* pres, float* alti)
{
    static float comp_temp, comp_pres;
    int32_t raw_pressure = BMP280_getRawPressure();
    int32_t raw_tmperature = BMP280_getRawTemperature();

    comp_temp = BMP280_getCompensateTemp(raw_tmperature) / 100.0F;
    comp_pres = BMP280_getCompensatePres(raw_pressure) / 25600.0F;

    BMP280_getPressureFilter(&comp_pres, pres);
    *temp = (float) comp_temp;

    if(*pres > 0)
    {
        *alti = 44330.0F * (powf((1015.7F / (*pres)), 0.190295F) - 1.0F);
    }
    else
    {
        *alti = 0.0F;
    }
}