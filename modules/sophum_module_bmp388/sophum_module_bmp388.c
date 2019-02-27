#include <stdio.h>
#include "driver/i2c.h"
#include "sophum_driver_i2c.h"
#include "sophum_module_bmp388.h"


/* private operation */

/** This internal API reads the trimming coefficients from the sensor, parse
 *  it then compensates it and store in the device structure.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_getTrimCoefficients(struct BMP388_dev *dev);

/** This internal API is used to parse the trimming coefficients, compensates
 *  it and store it in device structure.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 * @param[out] reg_data : contains trimming coefficients to be parsed.
 *
 */
static void BMP388_parseTrimCoefficients(const uint8_t *reg_data, struct BMP388_dev *dev);

/** This internal API gets the over sampling, odr and filter settings
 *  from the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_getODRAndFilterSettings(struct BMP388_dev *dev);

/** This internal API is used to parse the pressure and temperature data
 *  and store it in the BMP388_uncomp_data structure instance.
 *
 * @param[in] reg_data : contains the register data which needs to be parsed.
 * @param[out] uncomp_data : contains the uncompensated press and temp data.
 *
 */
static void BMP388_parseSensorData(const uint8_t *reg_data, struct BMP388_uncomp_data *uncomp_data);

/** This internal API is used to compensate the pressure or temperature
 *  or both the data according to the component selected by the user.
 *
 * @param[in] sensor_comp : used to select pressure or temperature.
 * @param[in] uncomp_data : contains the uncompensated pressure and
 * temperature data.
 * @param[out] comp_data : contains the compensated pressure and
 * temperature data.
 * @param[in] calib_data : pointer to the trimming coefficients structure.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_compensateData(uint8_t sensor_comp, const struct BMP388_uncomp_data *uncomp_data,
			                        struct BMP388_data *comp_data, struct BMP388_trimming_coeff *calib_data);

/** This internal API is used to compensate the raw temperature data and
 *  return the compensated temperature data in integer data type.
 *
 * @param[in] uncomp_data : contains the uncompensated temperature data.
 * @param[in] calib_data : pointer to trimming coefficients structure.
 *
 * @return compensated temperature data.
 * @retval compensated temperature data in integer.
 */
static int64_t BMP388_compensateTemperature(const struct BMP388_uncomp_data *uncomp_data,
						                    struct BMP388_trimming_coeff *calib_data);

/** This internal API is used to compensate the pressure data and return
 *  the compensated pressure data in integer data type.
 *
 * @param[in] uncomp_data : contains the uncompensated pressure data.
 * @param[in] calib_data : pointer to the trimming coefficients structure.
 *
 * @return compensated pressure data.
 * @retval compensated pressure data in integer.
 */
static uint64_t BMP388_compensatePressure(const struct BMP388_uncomp_data *uncomp_data,
					                      const struct BMP388_trimming_coeff *calib_data);

/** This internal API is used to calculate the power functionality.
 *
 * @param[in] base : contains the base value.
 * @param[in] power : contains the power value.
 *
 * @return output of power function.
 * @retval calculated power function output in integer.
 */
static uint32_t BMP388_calcPow(uint8_t base, uint8_t power);


/** This internal API is used to identify the settings which the user
 *  wants to modify in the sensor.
 *
 * @param[in] sub_settings : contains the settings subset to identify particular
 * group of settings which the user is interested to change.
 * @param[in] settings : contains the user specified settings.
 *
 * @return indicates whether user is interested to modify the settings which
 * are related to sub_settings.
 * @retval true -> user wants to modify this group of settings
 * @retval false -> user does not want to modify this group of settings
 */
static uint8_t BMP388_areSettingsChanged(uint32_t sub_settings, uint32_t settings);

/** This internal API interleaves the register address between the
 *  register data buffer for burst write operation.
 *
 * @param[in] reg_addr : contains the register address array.
 * @param[out] temp_buff : contains the temporary buffer to store the
 * register data and register address.
 * @param[in] reg_data : contains the register data to be written in the
 * temporary buffer.
 * @param[in] len : number of bytes of data to be written for burst write.
 *
 */
static void BMP388_interleaveRegAddr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);

/** This internal API sets the pressure enable and
 *  temperature enable settings of the sensor.
 *
 * @param[in] desired_settings : contains the settings which user wants to
 * change.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_setPowerControlSettings(uint32_t desired_settings, const struct BMP388_dev *dev);

/** This internal API sets the over sampling, odr and filter settings of
 *  the sensor based on the settings selected by the user.
 *
 * @param[in] desired_settings : variable used to select the settings which
 * are to be set.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_setODRAndFilterSettings(uint32_t desired_settings, struct BMP388_dev *dev);

/** This internal API sets the interrupt control (output mode, level,
 *  latch and data ready) settings of the sensor based on the settings
 *  selected by the user.
 *
 * @param[in] desired_settings : variable used to select the settings which
 * are to be set.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_setInterruptControlSettings(uint32_t desired_settings, const struct BMP388_dev *dev);

/** This internal API sets the advance (i2c_wdt_en, i2c_wdt_sel)
 *  settings of the sensor based on the settings selected by the user.
 *
 * @param[in] desired_settings : variable used to select the settings which
 * are to be set.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_setAdvanceSettings(uint32_t desired_settings, const struct BMP388_dev *dev);

/** This internal API fills the register address and register data of the
 *  the over sampling settings for burst write operation.
 *
 * @param[in] desired_settings : variable which specifies the settings which
 * are to be set in the sensor.
 * @param[out] addr : to store the address to fill in register buffer.
 * @param[out] reg_data : to store the osr register data.
 * @param[out] len : to store the len for burst write.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 */
static void BMP388_fillOSRData(uint32_t desired_settings, uint8_t *addr, uint8_t *reg_data, uint8_t *len,
			                   const struct BMP388_dev *dev);

/** This internal API fills the register address and register data of the
 *  the odr settings for burst write operation.
 *
 * @param[out] addr : to store the address to fill in register buffer.
 * @param[out] reg_data : to store the register data to set the odr data.
 * @param[out] len : to store the len for burst write.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 */
static void BMP388_fillODRData(uint8_t *addr, uint8_t *reg_data, uint8_t *len, struct BMP388_dev *dev);

/** This internal API fills the register address and register data of the
 *  the filter settings for burst write operation.
 *
 * @param[out] addr : to store the address to fill in register buffer.
 * @param[out] reg_data : to store the register data to set the filter.
 * @param[out] len : to store the len for burst write.
 * @param[in] dev : structure instance of BMP388_dev.
 *
 */
static void BMP388_fillFilterData(uint8_t *addr, uint8_t *reg_data, uint8_t *len, const struct BMP388_dev *dev);

/** This internal API is used to validate the device pointer for
 *  null conditions.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_checkNullPointer(const struct BMP388_dev *dev);

/** This internal API parse the power control(power mode, pressure enable
 *  and temperature enable), over sampling, odr, filter and interrupt control
 *  settings and store in the device structure.
 *
 * @param[in] reg_data : register data to be parsed.
 * @param[out] dev : structure instance of BMP388_dev.
 */
static void BMP388_parseSettingsData(const uint8_t *reg_data, struct BMP388_dev *dev);

/** This internal API parse the power control(power mode, pressure enable
 *  and temperature enable) settings and store in the device structure.
 *
 * @param[in] reg_data : pointer variable which stores the register data to
 * parse.
 * @param[out] settings : structure instance of BMP388_settings.
 */
static void BMP388_parsePowerControlSettings(const uint8_t *reg_data, struct BMP388_settings *settings);

/** This internal API parse the over sampling, odr and filter
 *  settings and store in the device structure.
 *
 * @param[in] reg_data : pointer variable which stores the register data to
 * parse.
 * @param[out] settings : structure instance of BMP388_ODR_filter_settings.
 */
static void BMP388_parseODRAndFilterSettings(const uint8_t *reg_data, struct BMP388_ODR_filter_settings *settings);

/** This internal API parse the interrupt control(output mode, level,
 *  latch and data ready) settings and store in the device structure.
 *
 * @param[in] reg_data : pointer variable which stores the register data to
 * parse.
 * @param[out] settings : structure instance of BMP388_int_ctrl_settings.
 */
static void BMP388_parseInterruptControlSettings(const uint8_t *reg_data, struct BMP388_int_ctrl_settings *settings);

/** This internal API parse the advance (i2c_wdt_en, i2c_wdt_sel)
 *  settings and store in the device structure.
 *
 * @param[in] reg_data : pointer variable which stores the register data to
 * parse.
 * @param[out] settings : structure instance of BMP388_adv_settings.
 */
static void BMP388_parseAdvanceSettings(const uint8_t *reg_data, struct BMP388_adv_settings *settings);

/** This internal API validate the normal mode settings of the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_validNormalModeSettings(struct BMP388_dev *dev);

/** This internal API validate the over sampling, odr settings of the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return indicates whether odr and osr settings are valid or not.
 * @retval zero -> success / -ve value -> error
 */
static int8_t BMP388_validOSRAndODRSettings(const struct BMP388_dev *dev);

/** This internal API calculates the pressure measurement duration of the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return pressure measurement time
 * @retval pressure measurement time in milli secs
 */
static uint16_t BMP388_calcPresMeasTime(const struct BMP388_dev *dev);

/** This internal API calculates the temperature measurement duration of the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return temperature measurement time
 * @retval temperature measurement time in millisecs
 */
static uint16_t BMP388_calcTempMeasTime(const struct BMP388_dev *dev);

/** This internal API checks whether the measurement time and odr duration
 *  of the sensor are proper.
 *
 * @param[in] meas_t : pressure and temperature measurement time in millisecs.
 * @param[in] odr_duration : duration in millisecs corresponding to the odr
 * value.
 *
 * @return indicates whether odr and osr settings are valid or not.
 * @retval zero -> success / +ve value -> warning
 */
static int8_t BMP388_verifyMeasTimeAndODRDuration(uint16_t meas_t, uint32_t odr_duration);

/** This internal API puts the device to sleep mode.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status.
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_putDeviceToSleep(const struct BMP388_dev *dev);

/** This internal API sets the normal mode in the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status.
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_setNormalMode(struct BMP388_dev *dev);

/** This internal API writes the power mode in the sensor.
 *
 * @param[in] dev : structure instance of BMP388_dev.
 *
 * @return result of API execution status.
 * @retval zero -> success / +ve value -> warning / -ve value -> error
 */
static int8_t BMP388_writePowerMode(const struct BMP388_dev *dev);

/** This internal API fills the fifo_config_1(fifo_mode,
 *  fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en) settings in the
 *  reg_data variable so as to burst write in the sensor.
 *
 * @param[in] desired_settings : variable which specifies the settings which
 * are to be set in the sensor.
 * @param[out] reg_data : pointer variable where the fifo_config_1 register
 * data will be stored so as to burst write in the register.
 * @param[in] dev_fifo : structure instance of BMP388_fifo_settings which
 * contains the fifo_config_1 values set by the user.
 */
static void BMP388_fillFIFOConfig1(uint16_t desired_settings, uint8_t *reg_data,
			                       struct BMP388_fifo_settings *dev_fifo);

/** This internal API fills the fifo_config_2(fifo_sub_sampling,
 *  data_select) settings in the reg_data variable so as to burst write
 *  in the sensor.
 *
 * @param[in] desired_settings : variable which specifies the settings which
 * are to be set in the sensor.
 * @param[out] reg_data : pointer variable where the fifo_config_2 register
 * data will be stored so as to burst write in the register.
 * @param[in] dev_fifo : structure instance of BMP388_fifo_settings which
 * contains the fifo_config_2 values set by the user.
 */
static void BMP388_fillFIFOConfig2(uint16_t desired_settings, uint8_t *reg_data,
			                       const struct BMP388_fifo_settings *dev_fifo);

/** This internal API fills the fifo interrupt control(fwtm_en, ffull_en)
 *  settings in the reg_data variable so as to burst write in the sensor.
 *
 * @param[in] desired_settings : variable which specifies the settings which
 * are to be set in the sensor.
 * @param[out] reg_data : pointer variable where the fifo interrupt control
 * register data will be stored so as to burst write in the register.
 * @param[in] dev_fifo : structure instance of BMP388_fifo_settings which
 * contains the fifo interrupt control values set by the user.
 */
static void BMP388_fillFIFOInterruptControl(uint16_t desired_settings, uint8_t *reg_data,
			                                const struct BMP388_fifo_settings *dev_fifo);

/** This internal API is used to parse the fifo_config_1(fifo_mode,
 *  fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 *  fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 *  settings and store it in device structure
 *
 * @param[in] reg_data : pointer variable which stores the fifo settings data
 * read from the sensor.
 * @param[out] dev_fifo : structure instance of BMP388_fifo_settings which
 * contains the fifo settings after parsing.
 */
static void BMP388_parseFIFOSettings(const uint8_t *reg_data, struct BMP388_fifo_settings *dev_fifo);

/** This internal API parse the FIFO data frame from the fifo buffer and
 *  fills the byte count, uncompensated pressure and/or temperature data and no
 *  of parsed frames.
 *
 * @param[in] header : pointer variable which stores the fifo settings data
 * read from the sensor.
 * @param[in,out] fifo : structure instance of bmp3_fifo which stores the
 * read fifo data.
 * @param[out] byte_index : byte count which is incremented according to the
 * of data.
 * @param[out] uncomp_data : uncompensated pressure and/or temperature data
 * which is stored after parsing fifo buffer data.
 * @param[out] parsed_frames : total number of parsed frames.
 *
 * @return result of API execution status.
 * @retval zero -> success / -ve value -> error
 */
static uint8_t BMP388_parseFIFODataFrame(uint8_t header, struct bmp3_fifo *fifo, uint16_t *byte_index,
					                     struct BMP388_uncomp_data *uncomp_data, uint8_t *parsed_frames);

/** This internal API unpacks the FIFO data frame from the fifo buffer and
 *  fills the byte count, uncompensated pressure and/or temperature data.
 *
 * @param[out] byte_index : byte count of fifo buffer.
 * @param[in] fifo_buffer : fIFO buffer from where the temperature and pressure
 * frames are unpacked.
 * @param[out] uncomp_data : uncompensated temperature and pressure data after
 * unpacking from fifo buffer.
 */
static void BMP388_unpackTempAndPresFrame(uint16_t *byte_index, const uint8_t *fifo_buffer,
					                      struct BMP388_uncomp_data *uncomp_data);

/** This internal API unpacks the FIFO data frame from the fifo buffer and
 *  fills the byte count and uncompensated pressure data.
 *
 * @param[out] byte_index : byte count of fifo buffer.
 * @param[in] fifo_buffer : FIFO buffer from where the pressure frames are
 * unpacked.
 * @param[out] uncomp_data : uncompensated pressure data after unpacking from
 * fifo buffer.
 */
static void BMP388_unpackPressFrame(uint16_t *byte_index, const uint8_t *fifo_buffer,
				                    struct BMP388_uncomp_data *uncomp_data);

/** This internal API unpacks the FIFO data frame from the fifo buffer and
 *  fills the byte count and uncompensated temperature data.
 *
 * @param[out] byte_index : byte count of fifo buffer.
 * @param[in] fifo_buffer : FIFO buffer from where the temperature frames
 * are unpacked.
 * @param[out] uncomp_data : uncompensated temperature data after unpacking from
 * fifo buffer.
 */
static void BMP388_unpackTempFrame(uint16_t *byte_index, const uint8_t *fifo_buffer,
                                   struct BMP388_uncomp_data *uncomp_data);

/** This internal API unpacks the time frame from the fifo data buffer and
 *  fills the byte count and update the sensor time variable.
 *
 * @param[out] byte_index : byte count of fifo buffer.
 * @param[in] fifo_buffer : FIFO buffer from where the sensor time frames
 * are unpacked.
 * @param[out] sensor_time : variable used to store the sensor time.
 */
static void BMP388_unpackTimeFrame(uint16_t *byte_index, const uint8_t *fifo_buffer,
                                   uint32_t *sensor_time);

/** This internal API parses the FIFO buffer and gets the header info.
 *
 * @param[out] header : variable used to store the fifo header data.
 * @param[in] fifo_buffer : FIFO buffer from where the header data is retrieved.
 * @param[out] byte_index : byte count of fifo buffer.
 */
static void BMP388_getHeaderInfo(uint8_t *header, const uint8_t *fifo_buffer, uint16_t *byte_index);

/** This internal API parses the FIFO data frame from the fifo buffer and
 *  fills uncompensated temperature and/or pressure data.
 *
 * @param[in] sensor_comp : variable used to select either temperature or
 * pressure or both while parsing the fifo frames.
 * @param[in] fifo_buffer : FIFO buffer where the temperature or pressure or
 * both the data to be parsed.
 * @param[out] uncomp_data : uncompensated temperature or pressure or both the
 * data after unpacking from fifo buffer.
 */
static void BMP388_parseFIFOSensorData(uint8_t sensor_comp, const uint8_t *fifo_buffer,
					                   struct BMP388_uncomp_data *uncomp_data);

/** This internal API resets the FIFO buffer, start index,
 *  parsed frame count, configuration change, configuration error and
 *  frame_not_available variables.
 *
 * @param[out] fifo : FIFO structure instance where the fifo related variables
 * are reset.
 */
static void BMP388_resetFIFOIndex(struct bmp3_fifo *fifo);

/** This API gets the command ready, data ready for pressure and
 *  temperature, power on reset status from the sensor.
 *
 * @param[in,out] dev : structure instance of BMP388_dev
 *
 * @return result of API execution status.
 * @retval zero -> success / -ve value -> error.
 */
static int8_t BMP388_getSensorStatus(struct BMP388_dev *dev);

/** This API gets the interrupt (fifo watermark, fifo full, data ready)
 *  status from the sensor.
 *
 * @param[in,out] dev : structure instance of BMP388_dev
 *
 * @return result of API execution status.
 * @retval zero -> success / -ve value -> error.
 */
static int8_t BMP388_getInterruptStatus(struct BMP388_dev *dev);

/** This API gets the fatal, command and configuration error
 *  from the sensor.
 *
 * @param[in,out] dev : structure instance of BMP388_dev
 *
 * @return result of API execution status.
 * @retval zero -> success / -ve value -> error.
 */
static int8_t BMP388_getErrorStatus(struct BMP388_dev *dev);

/** This internal API converts the no. of frames required by the user to
 *  bytes so as to write in the watermark length register.
 *
 * @param[in] dev : structure instance of BMP388_dev
 * @param[out] watermark_len : pointer variable which contains the watermark
 * length.
 *
 * @return result of API execution status.
 * @retval zero -> success / -ve value -> error.
 */
static int8_t BMP388_convertFramesToBytes(uint16_t *watermark_len, const struct BMP388_dev *dev);
