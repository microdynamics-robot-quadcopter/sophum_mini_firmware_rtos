#include "sophum_module_bmx055.h"


int8_t BMX055_readMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    int8_t ret = 0;
    ret = I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_id << 1, reg_addr, len, data);
    if(ret == true)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

int8_t BMX055_writeMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    int8_t ret = 0;
    ret = I2C_writeMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_id << 1, reg_addr, len, data);
    if(ret == true)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}