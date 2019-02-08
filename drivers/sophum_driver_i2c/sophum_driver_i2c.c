#include <stdio.h>
#include "driver/i2c.h"
#include "sophum_driver_i2c.h"

/**
 * Write one byte to an I2C peripheral
 * @param dev_addr: The device address to write to
 * @param reg_addr: The internal address to write from, I2CDEV_NO_MEM_ADDR if none
 * @param data    : The byte to write
 *
 * @return TRUE if write was successful, otherwise FALSE
 */
extern bool I2C_writeOneByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * Write mulit bytes to an I2C peripheral
 * @param dev_addr: The device address to write to
 * @param reg_addr: The internal address to write to, I2CDEV_NO_MEM_ADDR if none
 * @param byte_num: Number of bytes to read
 * @param data    : Pointer to a buffer to read the data from that will be written
 *
 * @return TRUE if write was successful, otherwise FALSE
 */
extern bool I2C_writeMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint16_t byte_num, uint8_t* data);

/**
 * Write one bit to an I2C peripheral
 * @param dev_addr: The device address to write to
 * @param reg_addr: The internal address to write to, I2CDEV_NO_MEM_ADDR if none
 * @param bit_id  : The bit id(0 - 7)to write
 * @param data    : The bit to write
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_writeOneBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_id, uint8_t data);

/**
 * Write up to 8 bits to an I2C peripheral
 * @param dev_addr : The device address to write to
 * @param reg_addr : The internal address to write to, I2CDEV_NO_MEM_ADDR if none
 * @param bit_start: The bit to start from(0 - 7)
 * @param bit_len  : The number of bits to write(1 - 8)
 * @param data     : The byte containing the bits to write
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_writeMultiBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t data);

/**
 * Read one byte from an I2C peripheral
 * @param dev_addr: The device address to read from
 * @param reg_addr: The internal address to read from, I2CDEV_NO_MEM_ADDR if none
 * @param data    : Pointer to a buffer to read the data to
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_readOneByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data);

/**
 * Read mulit bytes from an I2C peripheral
 * @param dev_addr: The device address to read from
 * @param reg_addr: The internal address to read from, I2CDEV_NO_MEM_ADDR if none
 * @param byte_num: Number of bytes to read
 * @param data    : Pointer to a buffer to read the data to
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_readMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint16_t byte_num, uint8_t* data);

/**
 * Read one bit from an I2C peripheral
 * @param dev_addr: The device address to read from
 * @param reg_addr: The internal address to read from, I2CDEV_NO_MEM_ADDR if none
 * @param bit_id  : The bit id(0 - 7) to read
 * @param data    : Pointer to a buffer to read the data to
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_readOneBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_id, uint8_t* data);

/**
 * Read up to 8 bits from an I2C peripheral
 * @param dev_addr : The device address to read from
 * @param reg_addr : The internal address to read from, I2CDEV_NO_MEM_ADDR if none
 * @param bit_start: The bit to start from(0 - 7)
 * @param bit_len  : The number of bits to read(1 - 8)
 * @param data     : Pointer to a buffer to read the data to
 *
 * @return TRUE if read was successful, otherwise FALSE
 */
extern bool I2C_readMultiBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t* data);