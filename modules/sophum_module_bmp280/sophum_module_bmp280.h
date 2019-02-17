#ifndef __SOPHUM_MODULE_BMP280_H__
#define __SOPHUM_MODULE_BMP280_H__

#define BMP280_ADDR                                0x76
#define BMP280_TEMPE_CALIB_DIG_T1                  0x88
#define BMP280_DATA_FRAME_SIZE                     3
#define BMP280_CALIB_DATA_LEN                      24
#define BMP280_FILTER_NUM                          5
#define BMP280_FILTER_A                            0.1F

/* BMP280 register address */
#define BMP280_ID                                  0xD0
#define BMP280_RESET                               0xE0
#define BMP280_STATUS                              0xF3
#define BMP280_CTRL_MEAS                           0xF4
#define BMP280_CONFIG                              0xF5
#define BMP280_PRESS_MSB                           0xF7
#define BMP280_PRESS_LSB                           0xF8
#define BMP280_PRESS_XLSB                          0xF9
#define BMP280_TEMP_MSB                            0xFA
#define BMP280_TEMP_LSB                            0xFB
#define BMP280_TEMP_XLSB                           0xFC

/* ID register(0xD0) option value */
#define BMP280_ID_CHIP_ID                          0x58

/* RESET register(0xE0) option value */
#define BMP280_RESET_VALUE                         0xB6

/* STATUS register(0xF3) bit */
#define BMP280_STATUS_MEASURING_BIT                3
#define BMP280_STATUS_IM_UPDATE_BIT                0

/* STATUS register(0xF3) bit[3] option value */
#define BMP280_STATUS_MEASURING_DONE               0x0
#define BMP280_STATUS_MEASURING_ONGOING            0x1

/* STATUS register(0xF3) bit[0] option value */
#define BMP280_STATUS_IM_UPDATE_DONE               0x0
#define BMP280_STATUS_IM_UPDATE_ONGOING            0x1

/* CTRL_MEAS register(0xF4) bit */
#define BMP280_CTRL_MEAS_OSRS_T_BIT                7
#define BMP280_CTRL_MEAS_OSRS_T_LEN                3
#define BMP280_CTRL_MEAS_OSRS_P_BIT                4
#define BMP280_CTRL_MEAS_OSRS_P_LEN                3
#define BMP280_CTRL_MEAS_MODE_BIT                  1
#define BMP280_CTRL_MEAS_MODE_LEN                  2

/* CTRL_MEAS register(0xF4) bit[7:5] and [4:2] option value */
#define BMP280_CTRL_MEAS_OSRS_OVERSAMPLING_SKIPPED 0x0
#define BMP280_CTRL_MEAS_OSRS_OVERSAMPLING_1X      0x1
#define BMP280_CTRL_MEAS_OSRS_OVERSAMPLING_2X      0x2
#define BMP280_CTRL_MEAS_OSRS_OVERSAMPLING_4X      0x3
#define BMP280_CTRL_MEAS_OSRS_OVERSAMPLING_8X      0x4
#define BMP280_CTRL_MEAS_OSRS_OVERSAMPLING_16X     0x5

/* CTRL_MEAS register(0xF4) bit[1:0] option value */
#define BMP280_CTRL_MEAS_SLEEP_MODE                0x0
#define BMP280_CTRL_MEAS_FORCED_MODE               0x1
#define BMP280_CTRL_MEAS_NORMAL_MODE               0x3

/* CONFIG register(0xF5) bit */
#define BMP280_CONFIG_T_SB_BIT                     7
#define BMP280_CONFIG_T_SB_LEN                     3
#define BMP280_CONFIG_FILTER_BIT                   4
#define BMP280_CONFIG_FILTER_LEN                   3
#define BMP280_CONFIG_SPI3W_EN_BIT                 0

/* CONFIG register(0xF5) bit[7:5] option value */
#define BMP280_CONFIG_T_SB_0P5MS                   0x0
#define BMP280_CONFIG_T_SB_62P5MS                  0x1
#define BMP280_CONFIG_T_SB_125MS                   0x2
#define BMP280_CONFIG_T_SB_250MS                   0x3
#define BMP280_CONFIG_T_SB_500MS                   0x4
#define BMP280_CONFIG_T_SB_1000MS                  0x5
#define BMP280_CONFIG_T_SB_2000MS                  0x6
#define BMP280_CONFIG_T_SB_4000MS                  0x7

/* CONFIG register(0xF5) bit[4:2] option value */
#define BMP280_CONFIG_FILTER_OFF                   0x0
#define BMP280_CONFIG_FILTER_COEFF_2               0x1
#define BMP280_CONFIG_FILTER_COEFF_4               0x2
#define BMP280_CONFIG_FILTER_COEFF_8               0x3
#define BMP280_CONFIG_FILTER_COEFF_16              0x4

/* CONFIG register(0xF5) bit[0] option value */
#define BMP280_CONFIG_SPI3W_EN_DISABLE             0x0
#define BMP280_CONFIG_SPI3W_EN_ENABLE              0x1


#define BMP280_FILTER_CONFIG    (5 << 2)
#define BMP280_PRESSURE_OSR     BMP280_CTRL_MEAS_OSRS_OVERSAMPLING_8X
#define BMP280_TEMPERATURE_OSR  BMP280_CTRL_MEAS_OSRS_OVERSAMPLING_8X
#define BMP280_MODE             ((BMP280_PRESSURE_OSR<<2) | (BMP280_TEMPERATURE_OSR<<5) | BMP280_CTRL_MEAS_NORMAL_MODE)



/* public operation */
extern void BMP280_Init(void);
extern void BMP280_updateData(float* temp, float* pres, float* alti);


#endif