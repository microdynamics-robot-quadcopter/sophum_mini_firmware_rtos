#include "sophum_module_bno055.h"


int8_t BNO055_readMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int8_t ret = 0;
    ret = I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_addr << 1, reg_addr, cnt, reg_data);

    if(ret == true)
    {
        return BNO055_SUCCESS;
    }
    else
    {
        return BNO055_ERROR;
    }

}

int8_t BNO055_writeMultiBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int8_t ret = 0;
    ret = I2C_writeMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_addr << 1, reg_addr, cnt, reg_data);

    if(ret == true)
    {
        return BNO055_SUCCESS;
    }
    else
    {
        return BNO055_ERROR;
    }
}