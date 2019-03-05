#include "sophum_module_bmp388.h"


int8_t BMP388_readMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int8_t ret = 0;
    ret = I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_id << 1, reg_addr, len, data);
    if(ret == true)
    {
        return BMP3_OK;
    }
    else
    {
        return BMP3_E_COMM_FAIL;
    }
}

int8_t BMP388_writeMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int8_t ret = 0;
    ret = I2C_writeMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_id << 1, reg_addr, len, data);
    if(ret == true)
    {
        return BMP3_OK;
    }
    else
    {
        return BMP3_E_COMM_FAIL;
    }
}

extern double BMP388_calcAltitude(double pres)
{
    return (1.0F - powf(pres / 101325, 0.190284F)) * 287.15F / 0.0065F;
}