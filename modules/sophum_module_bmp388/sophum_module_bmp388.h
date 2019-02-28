#ifndef __SOPHUM_MODULE_BMP388_H__
#define __SOPHUM_MODULE_BMP388_H__


#define BMP388_ADDR            0x76

/* BMP388 pressure settling time (micro secs) */
#define BMP388_PRESS_SETTLE_TIME    392
/* BMP388 temperature settling time (micro secs) */
#define BMP388_TEMP_SETTLE_TIME     313
/* BMP388 adc conversion time (micro secs) */
#define BMP388_ADC_CONV_TIME        2000

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
#define BMP388_ODR               0x1D
#define BMP388_CONFIG            0x1F
#define BMP388_NVM_PAR_T1_7_0     0x31
#define BMP388_NVM_PAR_T1_15_8    0x32
#define BMP388_NVM_PAR_T2_7_0     0x33
#define BMP388_NVM_PAR_T2_15_8    0x34
#define BMP388_NVM_PAR_T3_7_0     0x35
#define BMP388_NVM_PAR_P1_7_0     0x36
#define BMP388_NVM_PAR_P1_15_8    0x37
#define BMP388_NVM_PAR_P2_7_0     0x38
#define BMP388_NVM_PAR_P2_15_8    0x39
#define BMP388_NVM_PAR_P3_7_0     0x3A
#define BMP388_NVM_PAR_P4_7_0     0x3B
#define BMP388_NVM_PAR_P5_7_0     0x3C
#define BMP388_NVM_PAR_P5_15_8    0x3D
#define BMP388_NVM_PAR_P6_7_0     0x3E
#define BMP388_NVM_PAR_P6_15_8    0x3F
#define BMP388_NVM_PAR_P7_7_0     0x40
#define BMP388_NVM_PAR_P8_7_0     0x41
#define BMP388_NVM_PAR_P9_7_0     0x42
#define BMP388_NVM_PAR_P9_15_8    0x43
#define BMP388_NVM_PAR_P10_7_0    0x44
#define BMP388_NVM_PAR_P11_7_0    0x45
#define BMP388_CMD                0x7E

/* CHIP_ID register(0x00) option value */
#define BMP388_CHIP_ID_VALUE      0x50

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
#define BMP388_FIFO_CONFIG_2_DATA_SELECT_UNFILTERED_DATA 0x00
#define BMP388_FIFO_CONFIG_2_DATA_SELECT_FILTERED_DATA   0x01

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
#define BMP388_PWR_CTRL_MODE_SLEEP_MODE  0x00
#define BMP388_PWR_CTRL_MODE_FORCED_MODE 0x01
#define BMP388_PWR_CTRL_MODE_NORMAL_MODE 0x03

/* OSR register(0x1C) bit */
#define BMP388_OSR_OSR_T_BIT  5
#define BMP388_OSR_OSR_T_LEN  3
#define BMP388_OSR_OSR_P_BIT  2
#define BMP388_OSR_OSR_P_LEN  3

/* OSR register(0x1C) bit[5:3] [2:0] option value */
#define BMP388_OSR_OVERSAMPLING_1X   0x00
#define BMP388_OSR_OVERSAMPLING_2X   0x01
#define BMP388_OSR_OVERSAMPLING_4X   0x02
#define BMP388_OSR_OVERSAMPLING_8X   0x03
#define BMP388_OSR_OVERSAMPLING_16X  0x04
#define BMP388_OSR_OVERSAMPLING_32X  0x05

/* ODR register(0x1D) bit */
#define BMP388_ODR_ODR_SEL_BIT  4
#define BMP388_ODR_ODR_SEL_LEN  5

/* ODR register(0x1D) bit[4:0] option value */
#define BP388_ODR_ODR_SEL_ODR_200HZ     0x00
#define BP388_ODR_ODR_SEL_ODR_100HZ     0x01
#define BP388_ODR_ODR_SEL_ODR_50HZ      0x02
#define BP388_ODR_ODR_SEL_ODR_25HZ      0x03
#define BP388_ODR_ODR_SEL_ODR_12P5HZ    0x04
#define BP388_ODR_ODR_SEL_ODR_6P25HZ    0x05
#define BP388_ODR_ODR_SEL_ODR_3P1HZ     0x06
#define BP388_ODR_ODR_SEL_ODR_1P5HZ     0x07
#define BP388_ODR_ODR_SEL_ODR_0P78HZ    0x08
#define BP388_ODR_ODR_SEL_ODR_0P39HZ    0x09
#define BP388_ODR_ODR_SEL_ODR_0P2HZ     0x0A
#define BP388_ODR_ODR_SEL_ODR_0P1HZ     0x0B
#define BP388_ODR_ODR_SEL_ODR_0P05HZ    0x0C
#define BP388_ODR_ODR_SEL_ODR_0P02HZ    0x0D
#define BP388_ODR_ODR_SEL_ODR_0P01HZ    0x0E
#define BP388_ODR_ODR_SEL_ODR_0P006HZ   0x0F
#define BP388_ODR_ODR_SEL_ODR_0P003HZ   0x10
#define BP388_ODR_ODR_SEL_ODR_0P0015HZ  0x11

/* CONFIG register(0x1F) bit */
#define BMP388_CONFIG_IIR_FILTER_BIT 3
#define BMP388_CONFIG_IIR_FILTER_LEN 3

/* CONFIG register(0x1F) bit[3:1] option value */
#define BMP388_CONFIG_IIR_FILTER_COEF_0   0x00
#define BMP388_CONFIG_IIR_FILTER_COEF_1   0x01
#define BMP388_CONFIG_IIR_FILTER_COEF_3   0x02
#define BMP388_CONFIG_IIR_FILTER_COEF_7   0x03
#define BMP388_CONFIG_IIR_FILTER_COEF_15  0x04
#define BMP388_CONFIG_IIR_FILTER_COEF_31  0x05
#define BMP388_CONFIG_IIR_FILTER_COEF_63  0x06
#define BMP388_CONFIG_IIR_FILTER_COEF_127 0x07

/* CMD register(0x7E) bit[7:0] option value */
#define BMP388_CMD_NOP               0x00
#define BMP388_CMD_EXTMODE_EN_MIDDLE 0x34
#define BMP388_CMD_FIFO_FLUSH        0xB0
#define BMP388_CMD_SOFTRESET         0xB6



/* API status macros */
#define BMP388_OK                                0
#define BMP388_WARN_SENSOR_NOT_ENABLED           1
#define BMP388_WARN_INVALID_FIFO_REQ_FRAME_CNT   2
#define BMP388_ERROR_NULL_PTR                   -1
#define BMP388_ERROR_DEV_NOT_FOUND              -2
#define BMP388_ERROR_INVALID_ODR_OSR_SETTINGS   -3
#define BMP388_ERROR_CMD_EXEC_FAILED            -4
#define BMP388_ERROR_CONFIGURATION_ERROR        -5
#define BMP388_ERROR_INVALID_LEN                -6
#define BMP388_ERROR_COMM_FAIL                  -7
#define BMP388_ERROR_FIFO_WATERMARK_NOT_REACHED -8

/* BMP388 trimming coefficients */
struct BMP388_trimming_coeff
{
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t   par_t3;
    int16_t  par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int8_t   par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t   par_p7;
    int8_t   par_p8;
    int16_t  par_p9;
    int8_t   par_p10;
    int8_t   par_p11;
    int64_t  t_lin;
};


/* BMP388 advance settings */
struct BMP388_adv_settings
{
    /* i2c watch dog enable */
    uint8_t i2c_wdt_en;
    /* i2c watch dog select */
    uint8_t i2c_wdt_sel;
};


/* BMP388 ODR and filter settings */
struct BMP388_ODR_filter_settings
{
    /* pressure oversampling */
    uint8_t press_os;
    /* temperature oversampling */
    uint8_t temp_os;
    /* IIR filter */
    uint8_t iir_filter;
    /* output data rate */
    uint8_t odr;
};

/* BMP388 sensor status flags */
struct BMP388_sens_status
{
    /* command ready status */
    uint8_t cmd_dry;
    /* data ready for pressure */
    uint8_t drdy_press;
    /* data ready for temperature */
    uint8_t drdy_temp;
};

/* BMP388 interrupt status flags */
struct BMP388_int_status
{
    /* FIFO watermark interrupt */
    uint8_t fifo_wm;
    /* FIFO full interrupt */
    uint8_t fifo_full;
    /* data ready interrupt */
    uint8_t drdy;
};

/* BMP388 error status flags */
struct BMP388_error_status
{
    /* fatal error */
    uint8_t fatal;
    /* command error */
    uint8_t cmd;
    /* configuration error */
    uint8_t conf;
};

/* BMP388 status flags */
struct BMP388_status
{
    /* interrupt status */
    struct BMP388_int_status intr;
    /* sensor status */
    struct BMP388_sens_status sensor;
    /* error status */
    struct BMP388_error_status err;
    /* power on reset status */
    uint8_t pwr_on_rst;
};

/* BMP388 interrupt pin settings */
struct BMP388_int_ctrl_settings
{
    /* output mode */
    uint8_t ouput_mode;
    /* active high/low */
    uint8_t level;
    /* latched or non-latched */
    uint8_t latch;
    /* data ready interrupt */
    uint8_t drdy_en;
};


/* BMP388 device settings */
struct BMP388_settings
{
    /* power mode which user wants to set */
    uint8_t op_mode;
    /* enable/disable pressure sensor */
    uint8_t press_en;
    /* enable/disable temperature sensor */
    uint8_t temp_en;
    /* ODR and filter configuration */
    struct BMP388_ODR_filter_settings odr_filter;
    /* interrupt configuration */
    struct BMP388_int_ctrl_settings int_settings;
    /* advance settings */
    struct BMP388_adv_settings adv_settings;
};

/* BMP388 FIFO frame */
struct BMP388_fifo_data
{
    /* data buffer of user defined length is to be mapped here 512 + 4 */
    uint8_t buffer[516];
    /* number of bytes of data read from the FIFO */
    uint16_t byte_count;
    /* number of frames to be read as specified by the user */
    uint8_t req_frames;
    /* will be equal to length when no more frames are there to parse */
    uint16_t start_idx;
    /* will contain the no of parsed data frames from FIFO */
    uint8_t parsed_frames;
    /* configuration error */
    uint8_t config_err;
    /* sensor time */
    uint32_t sensor_time;
    /* FIFO input configuration change */
    uint8_t config_changes;
    /* all available frames are parsed */
    uint8_t frame_not_available;
};

/* BMP388 FIFO configuration */
struct BMP388_fifo_settings
{
    /* enable/disable */
    uint8_t mode;
    /* stop on full enable/disable */
    uint8_t stop_on_full_en;
    /* time enable/disable */
    uint8_t time_en;
    /* pressure enable/disable */
    uint8_t press_en;
    /* temperature enable/disable */
    uint8_t temp_en;
    /* down sampling rate */
    uint8_t down_sampling;
    /* filter enable/disable */
    uint8_t filter_en;
    /* FIFO watermark enable/disable */
    uint8_t fwtm_en;
    /* FIFO full enable/disable */
    uint8_t ffull_en;
};


/* BMP388 FIFO */
struct BMP388_fifo
{
    /* FIFO frame structure */
    struct BMP388_fifo_data data;
    /* FIFO config structure */
    struct BMP388_fifo_settings settings;
};

/* BMP388 sensor structure which comprises of compensated data */
struct BMP388_data
{
    /* compensated temperature */
    int64_t temperature;
    /* compensated pressure */
    uint64_t pressure;
};

/* BMP388 sensor structure which comprises of uncompensated data */
struct BMP388_uncomp_data
{
    /* uncompensated temperature */
    uint32_t temperature;
    /* uncompensated pressure */
    uint32_t pressure;
};

/* BMP388 device structure */
struct BMP388_dev
{
    /* chip id */
    uint8_t chip_id;
    /* device id */
    uint8_t dev_id;
    /* trimming coefficients */
    struct BMP388_trimming_coeff trim_coeff;
    /* sensor settings */
    struct BMP388_settings settings;
    /* sensor and interrupt status flags */
    struct BMP388_status status;
    /* FIFO data and settings structure */
    struct BMP388_fifo *fifo;
};

/* public operation */


/** This API is the entry point.
 *
 * It reads the chip-id and trimming coefficients of the sensor.
 *
 * @param[in,out] dev: structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_Init(struct BMP388_dev *dev);

/** This API performs the soft reset of the sensor.
 *
 * @param[in] dev: structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_doSoftReset(const struct BMP388_dev *dev);

/** This API sets the power control(pressure enable and
 *  temperature enable), over sampling, odr and filter
 *  settings in the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 * @param[in] desired_settings : variable used to select the settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros		        |   Functionality
 * ---------------------|----------------------------------------------
 * BMP3_PRESS_EN_SEL    |   Enable/Disable pressure.
 * BMP3_TEMP_EN_SEL     |   Enable/Disable temperature.
 * BMP3_PRESS_OS_SEL    |   Set pressure oversampling.
 * BMP3_TEMP_OS_SEL     |   Set temperature oversampling.
 * BMP3_IIR_FILTER_SEL  |   Set IIR filter.
 * BMP3_ODR_SEL         |   Set ODR.
 * BMP3_OUTPUT_MODE_SEL |   Set either open drain or push pull
 * BMP3_LEVEL_SEL       |   Set interrupt pad to be active high or low
 * BMP3_LATCH_SEL       |   Set interrupt pad to be latched or nonlatched.
 * BMP3_DRDY_EN_SEL     |   Map/Unmap the drdy interrupt to interrupt pad.
 * BMP3_I2C_WDT_EN_SEL  |   Enable/Disable I2C internal watch dog.
 * BMP3_I2C_WDT_SEL_SEL |   Set I2C watch dog timeout delay.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_setSensorSettings(uint32_t desired_settings, struct BMP388_dev *dev);

/** This API gets the power control(power mode, pressure enable and
 *  temperature enable), over sampling, odr, filter, interrupt control and
 *  advance settings from the sensor.
 *
 * @param[in,out] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_getSensorSettings(struct BMP388_dev *dev);

/** This API sets the power mode of the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * dev->settings.op_mode |   Macros
 * ----------------------|-------------------
 *     0                 | BMP3_SLEEP_MODE
 *     1                 | BMP3_FORCED_MODE
 *     3                 | BMP3_NORMAL_MODE
 *
 *
 * @note : Before setting normal mode, valid odr and osr settings should be set
 * in the sensor by using 'bmp3_set_sensor_settings' function.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_setOpMode(struct BMP388_dev *dev);

/** This API gets the power mode of the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 * @param[out] op_mode : pointer variable to store the op-mode.
 *
 *   op_mode             |   Macros
 * ----------------------|-------------------
 *     0                 | BMP3_SLEEP_MODE
 *     1                 | BMP3_FORCED_MODE
 *     3                 | BMP3_NORMAL_MODE
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_getOpMode(uint8_t *op_mode, const struct BMP388_dev *dev);

/** This API reads the pressure, temperature or both data from the
 *  sensor, compensates the data and store it in the BMP388_data structure
 *  instance passed by the user.
 *
 * @param[in] sensor_comp : variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BMP3_PRESS
 *     2       | BMP3_TEMP
 *     3       | BMP3_ALL
 *
 * @param[out] data : structure instance of BMP388_data.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_getSensorData(uint8_t sensor_comp, struct BMP388_data *data, struct BMP388_dev *dev);


/** This API sets the fifo_config_1(fifo_mode, fifo_stop_on_full, fifo_time_en, 
 *  fifo_press_en, fifo_temp_en), fifo_config_2(fifo_subsampling, data_select)
 *  and int_ctrl(fwtm_en, ffull_en) settings in the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 * @param[in] desired_settings : variable used to select the FIFO settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros                        |  Functionality
 * ------------------------------|----------------------------
 * BMP3_FIFO_MODE_SEL            |  Enable/Disable FIFO
 * BMP3_FIFO_STOP_ON_FULL_EN_SEL |  Set FIFO stop on full interrupt
 * BMP3_FIFO_TIME_EN_SEL         |  Enable/Disable FIFO time
 * BMP3_FIFO_PRESS_EN_SEL        |  Enable/Disable pressure
 * BMP3_FIFO_TEMP_EN_SEL         |  Enable/Disable temperature
 * BMP3_FIFO_DOWN_SAMPLING_SEL   |  Set FIFO downsampling
 * BMP3_FIFO_FILTER_EN_SEL       |  Enable/Disable FIFO filter
 * BMP3_FIFO_FWTM_EN_SEL         |  Enable/Disable FIFO watermark interrupt
 * BMP3_FIFO_FFULL_EN_SEL        |  Enable/Disable FIFO full interrupt
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_setFIFOSettings(uint16_t desired_settings, const struct BMP388_dev *dev);

/** This API gets the fifo_config_1(fifo_mode, fifo_stop_on_full, fifo_time_en,
 *  fifo_press_en, fifo_temp_en), fifo_config_2(fifo_subsampling, data_select)
 *  and int_ctrl(fwtm_en, ffull_en) settings from the sensor.
 *
 * @param[in,out] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_getFIFOSettings(const struct BMP388_dev *dev);

/** This API gets the fifo data from the sensor.
 *
 * @param[in,out] dev : structure instance of bmp3 device, where the fifo
 * data will be stored in fifo buffer.
 *
 * @return result of API execution status.
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_getFIFOData(const struct BMP388_dev *dev);

/** This API gets the fifo length from the sensor.
 *
 * @param[out] fifo_length : variable used to store the fifo length.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status.
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
extern int8_t BMP388_getFIFOLength(uint16_t *fifo_length, const struct BMP388_dev *dev);

/** This API extracts the temperature and/or pressure data from the FIFO
 *  data which is already read from the fifo.
 *
 * @param[out] data : array of BMP388_data structures where the temperature
 * and pressure frames will be stored.
 * @param[in,out] dev : structure instance of BMP388_dev which contains the
 * fifo buffer to parse the temperature and pressure frames.
 *
 * @return result of API execution status.
 * @retval zero -> success / -ve value -> error.
 */
extern int8_t BMP388_extractFIFOData(struct BMP388_data *data, struct BMP388_dev *dev);

/** This API gets the command ready, data ready for pressure and
 * temperature and interrupt (fifo watermark, fifo full, data ready) and
 * error status from the sensor.
 *
 * @param[in,out] dev : structure instance of BMP388_dev
 *
 * @return result of API execution status.
 * @retval zero -> success / -ve value -> error
 */
extern int8_t BMP388_getAllStatus(struct BMP388_dev *dev);

/** This API sets the fifo watermark length according to the frames count
 *  set by the user in the device structure. Refer below for usage.
 *
 * @note: dev->fifo->data.req_frames = 50;
 *
 * @param[in] dev : structure instance of BMP388_dev
 *
 * @return result of API execution status.
 * @retval zero -> success / -ve value -> Error
 */
extern int8_t BMP388_setFIFOWatermark(const struct BMP388_dev *dev);


extern void BMP388_updateData(float *temp, float *pres, float *alti);

#endif