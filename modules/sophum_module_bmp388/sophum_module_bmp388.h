#ifndef __SOPHUM_MODULE_BMP388_H__
#define __SOPHUM_MODULE_BMP388_H__


#define BMP388_ADDR            0x76

/* BMP388 register address */
#define BMP388_CHIP_ID         0x00
#define BMP388_ERR_REG         0x02
#define BMP388_STATUS          0x03
#define BMP388_DATA_0          0x04
#define BMP388_DATA_1          0x05
#define BMP388_DATA_2          0x06
#define BMP388_DATA_3          0x07
#define BMP388_DATA_4          0x08
#define BMP388_DATA_5          0x09
#define BMP388_SENSO_RTIME_0   0x0C
#define BMP388_SENSO_RTIME_1   0x0D
#define BMP388_SENSO_RTIME_2   0x0E
#define BMP388_SENSO_RTIME_3   0x0F
#define BMP388_EVENT           0x10
#define BMP388_INT_STATUS      0x11
#define BMP388_FIFO_LENGTH_0   0x12
#define BMP388_FIFO_LENGTH_1   0x13
#define BMP388_FIFO_DATA       0x14
#define BMP388_FIFO_WTM_0      0x15
#define BMP388_FIFO_WTM_1      0x16
#define BMP388_FIFO_CONFIG_1   0x17
#define BMP388_FIFO_CONFIG_2   0x18
#define BMP388_INT_CTRL        0x19
#define BMP388_IF_CONF         0x1A
#define BMP388_PWR_CTRL        0x1B
#define BMP388_OSR             0x1C
#define BMP388_ODR             0x1D
#define BMP388_CONFIG          0x1F
#define BMP388_CMD             0x7D

/* CHIP_ID register(0x00) option value */
#define BMP388_CHIP_ID_VALUE  0x50

/* ERR_REG register(0x01) bit */
#define BMP388_ERR_REG_CONF_ERR_BIT   2
#define BMP388_ERR_REG_CMD_ERR_BIT    1
#define BMP388_ERR_REG_FATAL_ERR_BIT  0

/* STATUS register(0x03) bit */
#define BMP388_STATUS_DYDY_TEMP_BIT   6
#define BMP388_STATUS_DYDY_PRESS_BIT  5
#define BMP388_STATUS_CMD_RDY_BIT     4

/* EVENT register(0x10) bit */
#define BMP388_EVENT_POR_DETECTED_BIT 0

/* INT_STATUS register(0x11) bit */
#define BMP388_INT_STATUS_DRDY_BIT       3
#define BMP388_INT_STATUS_FFULL_INT_BIT  1
#define BMP388_INT_STATUS_FWM_INT_BIT    0

/* FIFO_CONFIG_1 register(0x17) bit */
#define BMP388_FIFO_CONFIG_1_FIFO_TEMP_EN_BIT      4
#define BMP388_FIFO_CONFIG_1_FIFO_PRESS_EN_BIT     3
#define BMP388_FIFO_CONFIG_1_FIFO_TIME_EN_BIT      2
#define BMP388_FIFO_CONFIG_1_FIFO_STOP_ON_FULL_BIT 1
#define BMP388_FIFO_CONFIG_1_FIFO_MODE_BIT         0

/* FIFO_CONFIG_2 register(0x18) bit */
#define BMP388_FIFO_CONFIG_2_DATA_SELECT_BIT       4
#define BMP388_FIFO_CONFIG_2_DATA_SELECT_LEN       2
#define BMP388_FIFO_CONFIG_2_FIFO_SUBSAMPLING_BIT  2
#define BMP388_FIFO_CONFIG_2_FIFO_SUBSAMPLING_LEN  3

/* ??? FIFO_CONFIG_2 register(0x18) bit[4:3] option value */
#define BMP388_FIFO_CONFIG_2_DATA_SELECT_UNFILTERED_DATA 0x0
#define BMP388_FIFO_CONFIG_2_DATA_SELECT_FILTERED_DATA   0x1

/* INT_CTRL register(0x19) bit */
#define BMP388_INT_CTRL_DRDY_EN_BIT            6
#define BMP388_INT_CTRL_FFULL_EN_BIT           4
#define BMP388_INT_CTRL_FWTM_EN_BIT            3
#define BMP388_INT_CTRL_INT_LATCH_BIT          2
#define BMP388_INT_CTRL_INT_LEVEL_BIT          1
#define BMP388_INT_CTRL_INT_OD_BIT             0

/* IF_CONF register(0x1A) bit */
#define BMP388_IF_CONF_I2C_WDT_SEL_BIT 2
#define BMP388_IF_CONF_I2C_WDT_EN_BIT  1
#define BMP388_IF_CONF_SPI3_BIT        0

/* PWR_CTRL register(0x1B) bit*/
#define BMP388_PWR_CTRL_MODE_BIT       5
#define BMP388_PWR_CTRL_MODE_LEN       2
#define BMP388_PWR_CTRL_TMEP_EN_BIT    1
#define BMP388_PWR_CTRL_PRESS_EN_BIT   0

/* PWR_CTRL register(0x1B) bit[5:4] option value */
#define BMP388_PWR_CTRL_MODE_SLEEP_MODE  0x0
#define BMP388_PWR_CTRL_MODE_FORCED_MODE 0x1
#define BMP388_PWR_CTRL_MODE_NORMAL_MODE 0x3

/* OSR register(0x1C) bit */
#define BMP388_OSR_OSR_T_BIT  5
#define BMP388_OSR_OSR_T_LEN  3
#define BMP388_OSR_OSR_P_BIT  2
#define BMP388_OSR_OSR_P_LEN  3

/* OSR register(0x1C) bit[5:3] [2:0] option value */
#define BMP388_OSR_OVERSAMPLING_1X   0x0
#define BMP388_OSR_OVERSAMPLING_2X   0x1
#define BMP388_OSR_OVERSAMPLING_4X   0x2
#define BMP388_OSR_OVERSAMPLING_8X   0x3
#define BMP388_OSR_OVERSAMPLING_16X  0x4
#define BMP388_OSR_OVERSAMPLING_32X  0x5

/* ODR register(0x1D) bit */
#define BMP388_ODR_ODR_SEL_BIT  4
#define BMP388_ODR_ODR_SEL_LEN  5


/* ODR register(0x1D) bit[4:0] option value */
#define BP388_ODR_ODR_SEL_ODR_200HZ     0x0
#define BP388_ODR_ODR_SEL_ODR_100HZ     0x1
#define BP388_ODR_ODR_SEL_ODR_50HZ      0x2
#define BP388_ODR_ODR_SEL_ODR_25HZ      0x3
#define BP388_ODR_ODR_SEL_ODR_12P5HZ    0x4
#define BP388_ODR_ODR_SEL_ODR_6P25HZ    0x5
#define BP388_ODR_ODR_SEL_ODR_3P1HZ     0x6
#define BP388_ODR_ODR_SEL_ODR_1P5HZ     0x7
#define BP388_ODR_ODR_SEL_ODR_0P78HZ    0x8
#define BP388_ODR_ODR_SEL_ODR_0P39HZ    0x9
#define BP388_ODR_ODR_SEL_ODR_0P2HZ     0xA
#define BP388_ODR_ODR_SEL_ODR_0P1HZ     0xB
#define BP388_ODR_ODR_SEL_ODR_0P05HZ    0xC
#define BP388_ODR_ODR_SEL_ODR_0P02HZ    0xD
#define BP388_ODR_ODR_SEL_ODR_0P01HZ    0xE
#define BP388_ODR_ODR_SEL_ODR_0P006HZ   0xF
#define BP388_ODR_ODR_SEL_ODR_0P003HZ   0x10
#define BP388_ODR_ODR_SEL_ODR_0P0015HZ  0x11

/* CONFIG register(0x1F) bit */
#define BMP388_CONFIG_IIR_FILTER_BIT 3
#define BMP388_CONFIG_IIR_FILTER_LEN 3

/* CONFIG register(0x1F) bit[3:1] option value */
#define BMP388_CONFIG_IIR_FILTER_COEF_0   0x0
#define BMP388_CONFIG_IIR_FILTER_COEF_1   0x1
#define BMP388_CONFIG_IIR_FILTER_COEF_3   0x2
#define BMP388_CONFIG_IIR_FILTER_COEF_7   0x3
#define BMP388_CONFIG_IIR_FILTER_COEF_15  0x4
#define BMP388_CONFIG_IIR_FILTER_COEF_31  0x5
#define BMP388_CONFIG_IIR_FILTER_COEF_63  0x6
#define BMP388_CONFIG_IIR_FILTER_COEF_127 0x7


/* CMD register(0x7E) bit[7:0] option value */
#define BMP388_CMD_NOP               0x00
#define BMP388_CMD_EXTMODE_EN_MIDDLE 0x34
#define BMP388_CMD_FIFO_FLUSH        0xB0
#define BMP388_CMD_SOFTRESET         0xB6


/* public operation */
extern void BMP388_Init(void);
extern void BMP388_updateData(float* temp, float* pres, float* alti);
#endif