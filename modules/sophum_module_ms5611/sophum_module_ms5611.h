#ifndef __SOPHUM_MODULE_MS5611_H__
#define __SOPHUM_MODULE_MS5611_H__

#define MS5611_ADDR                     0xEE /* slave address for MS5611 sensor */
/* registers of the device */
#define MS5611_D1                       0x40
#define MS5611_D2                       0x50
#define MS5611_ADC_READ                 0x00
#define MS5611_RESET                    0x1E
#define MS5611_D1D2_SIZE                3

/* OSR(Over Sampling Ratio)constants */
#define MS5611_OSR_256                  0x00 /* conversion time 0.6ms resolution 0.065mbar */
#define MS5611_OSR_512                  0x02 /* conversion time 1.2ms resolution 0.042mbar */
#define MS5611_OSR_1024                 0x04 /* conversion time 2.3ms resolution 0.027mbar */
#define MS5611_OSR_2048                 0x06 /* conversion time 4.6ms resolution 0.018mbar */
#define MS5611_OSR_4096                 0x08 /* conversion time 9.1ms resolution 0.012mbar */

/* by adding ints from 0 to 6 we can read all the PROM configuration values */
/* C1 will be at 0xA2 and all the subsequent are multiples of 2 */
#define MS5611_PROM_BASE_ADDR           0xA2
#define MS5611_PROM_REG_COUNT           6   /* number of registers in the PROM */
#define MS5611_PROM_REG_SIZE            2   /* size in bytes of a PROM registry */

/* sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar) */
#define MS5611_MSLP                     101325

/* the sampling precision of pressure */
#define MS5611_OSR_PRES                 MS5611_OSR_4096
// the sampling precision of temperature */
#define MS5611_OSR_TEMP                 MS5611_OSR_4096
#define MS5611_BUFFER_SIZE              10
#define MS5611_PRES_OFFSET_NUM          50


#define I2C_EXAMPLE_MASTER_SCL_IO          18               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          19               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */


extern uint16_t calibration_params[MS5611_PROM_REG_COUNT+1];

/* public operation */
extern void MS5611_Init(void);
extern void MS5611_UpdateData();

#endif