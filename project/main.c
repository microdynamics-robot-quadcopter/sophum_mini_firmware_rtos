#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "sophum_driver_i2c.h"
#include "sophum_module_ms5611.h"
#include "sophum_module_mpu9250.h"

// #define I2C_EXAMPLE_MASTER_SCL_IO          18               /*!< gpio number for I2C master clock */
// #define I2C_EXAMPLE_MASTER_SDA_IO          19               /*!< gpio number for I2C master data  */
// #define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
// #define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
// #define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
// #define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

// void i2c_example_master_init()
// {
//     int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
//     i2c_config_t conf;
//     conf.mode = I2C_MODE_MASTER;
//     conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
//     i2c_param_config(i2c_master_port, &conf);
//     i2c_driver_install(i2c_master_port, conf.mode,
//                        I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
//                        I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
// }


void app_main()
{
    // i2c_example_master_init();
    I2C_MasterInit(I2C_NUM1_MASTER_PORT_GPIO, I2C_NUM1_MASTER_SCL_GPIO, I2C_NUM1_MASTER_SDA_GPIO,
                   I2C_NUM1_MASTER_FREQ_HZ);

    MS5611_Init();
    for (int i = 1; i <= MS5611_PROM_REG_COUNT; i++)
    {
        printf("C[%d]:%u\n", i, calibration_params[i]);
    }
    MS5611_UpdateData();
    // MPU9250_getID();
}

