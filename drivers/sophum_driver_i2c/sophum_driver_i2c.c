#include <stdio.h>
#include "driver/i2c.h"
#include "sophum_driver_i2c.h"

bool I2C_writeOneByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return I2C_writeMultiBytes(dev_addr, reg_addr, 1, &data);
}

bool I2C_writeMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t* data);
bool I2C_writeOneBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data);
bool I2C_writeMultiBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t data);

bool I2C_readOneByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data);
bool I2C_readMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t* data);
bool I2C_readOneBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t* data);
bool I2C_readMultiBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t* data);