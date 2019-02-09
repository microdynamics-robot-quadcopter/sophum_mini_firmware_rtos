#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "sophum_module_ms5611.h"


volatile float g_ms5611_altitude    = 0;
volatile float g_ms5611_pressure    = 0;
volatile float g_ms5611_temperature = 0;
uint16_t calibration_params[MS5611_PROM_REG_COUNT+1];

/* FIFO queue */
// static float temperature_buffer[MS5611_BUFFER_SIZE];
// static float pressure_buffer[MS5611_BUFFER_SIZE];
// static float altitude_buffer[MS5611_BUFFER_SIZE];


/* private variable */
static float    g_altitude_offset      = 0; /* save the altitude of 0m(relative) */
static float    g_pressure_offset      = 0; /* save the pressure of 0m(relative) */
static uint16_t g_pressure_offset_cnt  = 0;
static double   g_pressure_offset_val  = 0.0F;
       bool     g_pressure_offset_flag = false;

/* private operation */
static void MS5611_Reset(void);
static void MS5611_ReadPROM(void);
static uint32_t MS5611_GetConversion(uint8_t command);
static void MS5611_CalcTempAndPres(void);
static void MS5611_CalcAltitude(void);




/**
 * @brief test code to write esp-i2c-slave
 *
 * 1. set mode
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */

static void MS5611_Reset(void)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MS5611_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MS5611_RESET, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK)
    {
        printf("MS5611 reset ERROR!!!\n");
    }
    else 
    {
        printf("MS5611 reset SUCCESS!!!\n");
    }
    
}

static void MS5611_ReadPROM(void)
{
    uint8_t data_l;
    uint8_t data_h;

    for(int i = 1; i <= MS5611_PROM_REG_COUNT; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, MS5611_ADDR, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE), ACK_CHECK_EN);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        vTaskDelay(1 / portTICK_RATE_MS);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, MS5611_ADDR + 1, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &data_h, ACK_VAL);
        i2c_master_read_byte(cmd, &data_l, NACK_VAL);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        calibration_params[i] = (((uint16_t)data_h << 8) | data_l);
    }

}

void MS5611_Init(void)
{
    MS5611_Reset();
    vTaskDelay(100 / portTICK_RATE_MS);
    MS5611_ReadPROM();
}

static uint32_t MS5611_GetConversion(uint8_t command)
{
    int ret;
    uint8_t conver_res[MS5611_D1D2_SIZE];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MS5611_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK)
    {
        printf("MS5611 D1D2 config ERROR!!!\n");
    }
    else 
    {
        // if(command == MS5611_D1 + MS5611_OSR_PRES)
        //     printf("pressure config success!!! start conversion...\n");

        // if(command == MS5611_D2 + MS5611_OSR_TEMP)
        //     printf("temperature config success!!! start conversion...\n");
    }
    
    vTaskDelay(20 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MS5611_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MS5611_ADC_READ, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK)
    {
        printf("MS5611 ADC config ERROR!!!\n");
    }
    else
    {
        // printf("start adc read...\n");
    }
    

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MS5611_ADDR + 1, ACK_CHECK_EN);

    i2c_master_read_byte(cmd, conver_res, ACK_VAL);
    i2c_master_read_byte(cmd, conver_res + 1, ACK_VAL);
    i2c_master_read_byte(cmd, conver_res + 2, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK)
    {
        printf("MS5611 conver ERROR!!!\n");
    }

    else
    {
        // printf("res[0]:%u res[1]:%u res[2]:%u\n", conver_res[0], conver_res[1], conver_res[2]);
    }

    return ((uint32_t)conver_res[0] << 16) + ((uint32_t)conver_res[1] << 8) + (uint32_t)conver_res[2];
}

static void MS5611_CalcTempAndPres(void)
{
    uint32_t pres_raw = MS5611_GetConversion(MS5611_D1 + MS5611_OSR_PRES);
    uint32_t temp_raw = MS5611_GetConversion(MS5611_D2 + MS5611_OSR_TEMP);

    // printf("D1(Pressure):%u\n", pres_raw);
    // printf("D2(temperature):%u\n", temp_raw);

    int32_t dT = 0, TEMP = 0;
    dT = (int32_t)temp_raw - ((int32_t)calibration_params[5] << 8);
    TEMP = 2000 + ((dT * (int32_t)calibration_params[6]) >> 23);

    int64_t OFF = 0, SENS = 0;
    int32_t PRES = 0;
    OFF  = ((int64_t)calibration_params[2] << 16) + ((dT * (int64_t)calibration_params[4]) >> 7);
    SENS = ((int64_t)calibration_params[1] << 15) + ((dT * (int64_t)calibration_params[3]) >> 8);
    
    if(TEMP < 2000)
    {
        int32_t T2    = ((dT * dT) >> 31);
        int64_t OFF2  = ((5 * (TEMP - 2000) * (TEMP - 2000)) >> 1);
        int64_t SENS2 = ((5 * (TEMP - 2000) * (TEMP - 2000)) >> 2);
        
        if(TEMP < -1500)
        {
            OFF2  = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 = ((SENS2 + (11 * (TEMP + 1500) * (TEMP + 1500))) >> 1);
        }

        TEMP = TEMP - T2;
        OFF  = OFF  - OFF2;
        SENS = SENS - SENS2; 
    }

    PRES = (((SENS * (int64_t)pres_raw) >> 21) - OFF) >> 15;

    g_ms5611_pressure    = PRES / 100.0F;
    g_ms5611_temperature = TEMP / 100.0F;    
}

static void MS5611_CalcAltitude(void)
{
    float ans;
    if(g_pressure_offset_flag == false)
    {
        if(g_pressure_offset_cnt > MS5611_PRES_OFFSET_NUM)
        {
            g_pressure_offset = g_pressure_offset_val / g_pressure_offset_cnt;
            g_pressure_offset_flag = true;
        }
        else
        {
            g_pressure_offset_val += g_ms5611_pressure;
        }
        g_pressure_offset_cnt++;
        g_ms5611_altitude = 0.0F;
    }

    ans = 1 - powf((g_ms5611_pressure / g_pressure_offset), 0.1903F);
    g_ms5611_altitude = 4433000.0F * ans * 0.01F;
    g_ms5611_altitude += g_altitude_offset;
}

void MS5611_UpdateData()
{
    while(1)
    {
        MS5611_CalcTempAndPres();
        MS5611_CalcAltitude();
        // printf("temperature: %fC\n", g_ms5611_temperature);
        // printf("pressure: %fmbar\n", g_ms5611_pressure);
        printf("altitude: %fm\n", g_ms5611_altitude);
    }
}