#ifndef __SOPHUM_DRIVER_SPI_H__
#define __SOPHUM_DRIVER_SPI_H__

#include "driver/spi_master.h"
#include "esp_err.h"

/* SPI opeartion macro */
#define SPIBUS_READ                  (0x7F)  /* addr & SPIBUS_READ  */
#define SPIBUS_WRITE                 (0x80)  /* addr | SPIBUS_WRITE */

#define SPI_HSPI_MASTER_MOSI_GPIO     25
#define SPI_HSPI_MASTER_MISO_GPIO     26
#define SPI_HSPI_MASTER_SCK_GPIO      27
#define SPI_HSPI_MASTER_PORT_GPIO     HSPI_HOST
#define SPI_HSPI_MASTER_CLK_SPEED     1000000 /* up to 1MHz for all registers, and 20MHz for sensor data registers only */

#define SPI_VSPI_MASTER_MOSI_GPIO     34
#define SPI_VSPI_MASTER_MISO_GPIO     35
#define SPI_VSPI_MASTER_SCK_GPIO      32
#define SPI_VSPI_MASTER_PORT_GPIO     VSPI_HOST

#define SPI_MASTER_MODE_0             0
#define SPI_MASTER_MODE_1             1
#define SPI_MASTER_MODE_2             2
#define SPI_MASTER_MODE_3             3




/** config SPI bus and initialize
 *
 * @param   mosi_io_num     [GPIO number for Master-out Slave-in]
 * @param   mosi_io_num     [GPIO number for Master-out Slave-in]
 * @param   miso_io_num     [GPIO number for Master-in Slave-out]
 * @param   miso_io_num     [GPIO number for clock line]
 * @param   max_transfer_sz [Maximum transfer size, in bytes. Defaults to 4094 if 0.]
 * @return  - ESP_ERR_INVALID_ARG   if configuration is invalid
 *          - ESP_ERR_INVALID_STATE if host already is in use
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 */
esp_err_t SPI_MasterInit(uint8_t spi_port, int mosi_io_num, int miso_io_num,
                         int sclk_io_num, int max_transfer_sz);

/** free the SPI bus
 *
 * @warning In order for this to succeed, all devices have to be removed first.
 * @return  - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_INVALID_STATE if not all devices on the bus are freed
 *          - ESP_OK                on success
 */
esp_err_t SPI_closeBus(spi_host_device_t handle);

/** Allocate a device on a SPI bus. (Up to three devices per peripheral)
 *
 * @param   mode            [SPI mode (0-3)]
 * @param   clock_speed_hz  [Clock speed, in Hz]
 * @param   cs_io_num       [ChipSelect GPIO pin for this device, or -1 if not used]
 * @param   handle          [Pointer to variable to hold the device handle]
 * @param   dev_config      [SPI interface protocol config for the device (for more custom configs)]
 *                          @see driver/spi_master.h
 * @return  - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NOT_FOUND     if host doesn't have any free CS slots
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 */
esp_err_t SPI_addDevice(uint8_t spi_port, uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle);

esp_err_t SPI_removeDevice(spi_device_handle_t handle);

/** SPI commands for writing to a 8-bit slave device register.
 *
 *         All of them returns standard esp_err_t codes. So it can be used
 *         with ESP_ERROR_CHECK();
 * @param  handle    [SPI device handle]
 * @param  regAddr   [Register address to write to]
 * @param  bitNum    [Bit position number to write to (bit 7~0)]
 * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
 * @param  data      [Value(s) to be write to the register]
 * @param  length    [Number of bytes to write (should be within the data buffer size)]
 *                   [writeBits() -> Number of bits after bitStart (including)]
 * @return  - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_OK                on success
 */
esp_err_t SPI_writeOneBit(spi_device_handle_t handle, uint8_t reg_addr, uint8_t bit_id, uint8_t data);
esp_err_t SPI_writeMultiBits(spi_device_handle_t handle, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t data);
esp_err_t SPI_writeOneByte(spi_device_handle_t handle, uint8_t reg_addr, uint8_t data);
esp_err_t SPI_writeMultiBytes(spi_device_handle_t handle, uint8_t reg_addr, size_t bit_len, const uint8_t *data);

/** SPI commands for reading a 8-bit slave device register.
 *
 *         All of them returns standard esp_err_t codes.So it can be used
 *         with ESP_ERROR_CHECK();
 * @param  handle    [SPI device handle]
 * @param  regAddr   [Register address to read from]
 * @param  bitNum    [Bit position number to write to (bit 7~0)]
 * @param  bitStart  [Start bit number when writing a bit-sequence (MSB)]
 * @param  data      [Buffer to store the read value(s)]
 * @param  length    [Number of bytes to read (should be within the data buffer size)]
 * @return  - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_OK                on success
 */
esp_err_t SPI_readOneBit(spi_device_handle_t handle, uint8_t reg_addr, uint8_t bit_id, uint8_t *data);
esp_err_t SPI_readMultiBits(spi_device_handle_t handle, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t *data);
esp_err_t SPI_readOneByte(spi_device_handle_t handle, uint8_t reg_addr, uint8_t *data);
esp_err_t SPI_readMultiBytes(spi_device_handle_t handle, uint8_t reg_addr, size_t bit_len, uint8_t *data);

#endif