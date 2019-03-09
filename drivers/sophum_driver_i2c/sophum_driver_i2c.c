#include "sophum_driver_i2c.h"


bool I2C_MasterInit(uint8_t i2c_port, uint8_t scl_pin, uint8_t sda_pin, uint32_t clk_frq)
{
    int32_t ret = ESP_OK;
    i2c_config_t conf;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = sda_pin;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = scl_pin;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk_frq;
    i2c_param_config(i2c_port, &conf);

    if(i2c_port == I2C_NUM_0)
    {
        ret = i2c_driver_install(i2c_port, conf.mode,
                                 I2C_NUM0_MASTER_RX_BUF_DISABLE,
                                 I2C_NUM0_MASTER_TX_BUF_DISABLE, 0);
    }
    else if(i2c_port == I2C_NUM_1)
    {
        ret = i2c_driver_install(i2c_port, conf.mode,
                                 I2C_NUM1_MASTER_RX_BUF_DISABLE,
                                 I2C_NUM1_MASTER_TX_BUF_DISABLE, 0);
    }

    if(ret != ESP_OK)
    {
        printf("I2c port %u master mode init ERROR!!!\n", i2c_port);
        return false;
    }
    else
    {
        printf("I2C port %u master mode init SUCCESS!!!\n", i2c_port);
        return true;
    }
}


bool I2C_writeZeroByte(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr)
{
    uint8_t tmp_data;
    return I2C_writeMultiBytes(i2c_port, dev_addr, reg_addr, 0, &tmp_data);
}


bool I2C_writeOneByte(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return I2C_writeMultiBytes(i2c_port, dev_addr, reg_addr, 1, &data);
}


bool I2C_writeMultiBytes(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr, uint16_t byte_num,
                         uint8_t* data)
{
    int32_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr, MASTER_EN_CHECK_ACK);
    i2c_master_write_byte(cmd, reg_addr, MASTER_EN_CHECK_ACK);
    if(byte_num != 0)
    {
        i2c_master_write(cmd, data, byte_num, MASTER_EN_CHECK_ACK);
    }

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK)
    {
        printf("%#X's %u register write ERROR!!!\n", dev_addr >> 1, reg_addr);
        return false;
    }
    else
    {
        return true;
    }
}


bool I2C_writeOneBit(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_id,
                     uint8_t data)
{
    uint8_t tmp_byte;
    bool state;

    if((state = I2C_readOneByte(i2c_port, dev_addr, reg_addr, &tmp_byte)) == true)
    {
        tmp_byte = (data != 0) ? (tmp_byte | (1 << bit_id)) : (tmp_byte & (~(1 << bit_id)));
        state = I2C_writeOneByte(i2c_port, dev_addr, reg_addr, tmp_byte);
    }
    return state;
}


bool I2C_writeMultiBits(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start,
                        uint8_t bit_len, uint8_t data)
{
    uint8_t tmp_byte, mask;
    bool state;

    if((state = I2C_readOneByte(i2c_port, dev_addr, reg_addr, &tmp_byte)) == true)
    {
        mask = (((1 << bit_len) - 1) << (bit_start - bit_len + 1));
        data <<= (bit_start - bit_len + 1);
        data &= mask;
        tmp_byte &= (~(mask));
        tmp_byte |= data;
        state = I2C_writeOneByte(i2c_port, dev_addr, reg_addr, tmp_byte);
    }
    return state;
}


bool I2C_readOneByte(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data)
{
    return I2C_readMultiBytes(i2c_port, dev_addr, reg_addr, 1, data);
}


bool I2C_readMultiBytes(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr, uint16_t byte_num,
                        uint8_t* data)
{
    int32_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr, MASTER_EN_CHECK_ACK);
    i2c_master_write_byte(cmd, reg_addr, MASTER_EN_CHECK_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_addr + 1, MASTER_EN_CHECK_ACK);
    i2c_master_read(cmd, data, byte_num, MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK)
    {
        printf("%#X's %u register read ERROR!!!\n", dev_addr >> 1, reg_addr);
        return false;
    }
    else
    {
        return true;
    }
}


bool I2C_readOneBit(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_id,
                    uint8_t* data)
{
    uint8_t tmp_byte;
    bool state;

    if((state = I2C_readOneByte(i2c_port, dev_addr, reg_addr, &tmp_byte)) == true)
    {
        *data = (tmp_byte & (1 << bit_id));
    }
    return state;
}


bool I2C_readMultiBits(uint8_t i2c_port, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start,
                       uint8_t bit_len, uint8_t* data)
{
    uint8_t tmp_byte, mask;
    bool state;

    if((state = I2C_readOneByte(i2c_port, dev_addr, reg_addr, &tmp_byte)) == true)
    {
        mask = (((1 << bit_len) - 1) << (bit_start - bit_len + 1));
        tmp_byte &= mask;
        tmp_byte >>= (bit_start - bit_len + 1);
        *data = tmp_byte;
    }
    return state;
}