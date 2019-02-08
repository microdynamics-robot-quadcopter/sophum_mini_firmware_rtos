#include <stdio.h>
#include "driver/i2c.h"
#include "sophum_driver_i2c.h"

#define I2C_EXAMPLE_MASTER_SCL_IO          18               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          19               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define MASTER_EN_CHECK_ACK                0x1              /*!< I2C master will check ack from slave*/
#define MASTER_DIS_CHECK_ACK               0x0              /*!< I2C master will not check ack from slave */
#define MASTER_ACK                         0x0              /*!< I2C master ack value for each byte read*/
#define MASTER_NACK                        0x1              /*!< I2C master nack value for each byte read*/
#define MASTER_LAST_NACK                   0x2              /*!< I2C master nack value for the last byte*/


bool I2C_writeOneByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return I2C_writeMultiBytes(dev_addr, reg_addr, 1, &data);
}

bool I2C_writeMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint16_t byte_num, uint8_t* data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr, MASTER_EN_CHECK_ACK);
    i2c_master_write_byte(cmd, reg_addr, MASTER_EN_CHECK_ACK);
    i2c_master_write(cmd, data, byte_num, MASTER_EN_CHECK_ACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK)
    {
        printf("%u's %u register write ERROR!!!\n", dev_addr, reg_addr);
        return false;
    }
    else
    {
        return true;
    }
}

extern bool I2C_writeOneBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_id, uint8_t data)
{
    uint8_t tmp_byte;
    bool state;

    if((state = I2C_readOneByte(dev_addr, reg_addr, &tmp_byte)) == true)
    {
        tmp_byte = (data != 0) ? (tmp_byte | (1 << bit_id)) : (tmp_byte & (~(1 << bit_id)));
        state = I2C_writeOneByte(dev_addr, reg_addr, tmp_byte);
    }
    return state;
}

bool I2C_writeMultiBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t data)
{
    uint8_t tmp_byte, mask;
    bool state;

    if((state = I2C_readOneByte(dev_addr, reg_addr, &tmp_byte)) == true)
    {
        mask = (((1 << bit_len) - 1) << (bit_start - bit_len + 1));
        data <<= (bit_start - bit_len + 1);
        data &= mask;
        tmp_byte &= (~(mask));
        tmp_byte |= data;
        state = I2C_writeOneByte(dev_addr, reg_addr, tmp_byte);
    }
    return state;
}

bool I2C_readOneByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data)
{
    return I2C_readMultiBytes(dev_addr, reg_addr, 1, data);
}

bool I2C_readMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint16_t byte_num, uint8_t* data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr, MASTER_EN_CHECK_ACK);
    i2c_master_write_byte(cmd, reg_addr, MASTER_EN_CHECK_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr + 1, MASTER_EN_CHECK_ACK);
    i2c_master_read(cmd, data, byte_num, MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK)
    {
        printf("%u's %u register read ERROR!!!\n", dev_addr, reg_addr);
        return false;
    }
    else
    {
        return true;
    }
}

bool I2C_readOneBit(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_id, uint8_t* data)
{
    uint8_t tmp_byte;
    bool state;

    if((state = I2C_readOneByte(dev_addr, reg_addr, &tmp_byte)) == true)
    {
        *data = (tmp_byte & (1 << bit_id));
    }
    return state;
}

bool I2C_readMultiBits(uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t* data)
{
    uint8_t tmp_byte, mask;
    bool state;

    if((state = I2C_readOneByte(dev_addr, reg_addr, &tmp_byte)) == true)
    {
        mask = (((1 << bit_len) - 1) << (bit_start - bit_len + 1));
        tmp_byte &= mask;
        tmp_byte >>= (bit_start - bit_len + 1);
        *data = tmp_byte;
    }
    return state;
}