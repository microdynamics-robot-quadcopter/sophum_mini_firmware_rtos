#include "sophum_module_bmx055.h"

struct bma2x2_t bma2x2_dev;
struct bmg160_t bmg160_dev;
struct bmm050_t bmm050_dev;

int8_t BMX055_readMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    int8_t ret = 0;
    ret = I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_id << 1, reg_addr, len, data);
    if(ret == true) return 0;
    else return -1;
}

int8_t BMX055_writeMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    int8_t ret = 0;
    ret = I2C_writeMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_id << 1, reg_addr, len, data);
    if(ret == true) return 0;
    else return -1;
}

static  s32 com_rslt   = ERROR;
void BMX055_Init(void)
{
    /*===================BMA2X2 func init start=================================*/
    bma2x2_dev.bus_read   = BMX055_readMultiBytes;
    bma2x2_dev.bus_write  = BMX055_writeMultiBytes;
    bma2x2_dev.delay_msec = SOPHUM_delayMs;
    bma2x2_dev.dev_addr   = BMA2x2_I2C_ADDR1;

    u8 bma_bw_set_val     = BMA2x2_INIT_VALUE;
    u8 bma_bw_get_val     = BMA2x2_INIT_VALUE;
    u8 bma_rg_set_val     = BMA2x2_INIT_VALUE;
    u8 bma_rg_get_val     = BMA2x2_INIT_VALUE;

    com_rslt = bma2x2_init(&bma2x2_dev);
    printf("BMA2X2 CHIP id is %#X\n", bma2x2_dev.chip_id);

    com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

    bma_rg_set_val = BMA2x2_RANGE_8G;
    com_rslt += bma2x2_set_range(bma_rg_set_val);
    com_rslt += bma2x2_get_range(&bma_rg_get_val);
    printf("BMA2X2 range is %#X g\n", bma_rg_get_val);

    bma_bw_set_val = BMA2x2_BW_250HZ;
    com_rslt += bma2x2_set_bw(bma_bw_set_val);
    com_rslt += bma2x2_get_bw(&bma_bw_get_val);
    printf("BMA2X2 bandwidth setvalue is %#X\n", bma_bw_get_val);

    /*===================BMG160 func init start=================================*/
    bmg160_dev.bus_read   = BMX055_readMultiBytes;
    bmg160_dev.bus_write  = BMX055_writeMultiBytes;
    bmg160_dev.delay_msec = SOPHUM_delayMs;
    bmg160_dev.dev_addr   = BMG160_I2C_ADDR1;

    u8 bmg_bw_set_val     = BMG160_INIT_VALUE;
    u8 bmg_bw_get_val     = BMG160_INIT_VALUE;
    u8 bmg_rg_set_val     = BMG160_INIT_VALUE;
    u8 bmg_rg_get_val     = BMG160_INIT_VALUE;

    com_rslt = bmg160_init(&bmg160_dev);
    printf("BMG160 CHIP id is %#X\n", bmg160_dev.chip_id);

    com_rslt += bmg160_set_power_mode(BMG160_MODE_NORMAL);

    bmg_rg_set_val = BMG160_RANGE_2000;
    com_rslt += bmg160_set_range_reg(bmg_rg_set_val);
    com_rslt += bmg160_get_range_reg(&bmg_rg_get_val);
    printf("BMG160 range is %#X dps\n", bmg_rg_get_val);

    bmg_bw_set_val = BMG160_BW_230_HZ;
    com_rslt += bmg160_set_bw(bmg_bw_set_val);
    com_rslt += bmg160_get_bw(&bmg_bw_get_val);
    printf("BMG160 bandwidth setvalue is %#X\n", bmg_bw_get_val);

    /*===================BMM050 func init start=================================*/
    bmm050_dev.bus_read   = BMX055_readMultiBytes;
    bmm050_dev.bus_write  = BMX055_writeMultiBytes;
    bmm050_dev.delay_msec = SOPHUM_delayMs;
    bmm050_dev.dev_addr   = BMM050_I2C_ADDRESS;

    u8 bmm_dr_set_val     = BMM050_INIT_VALUE;
    u8 bmm_dr_get_val     = BMM050_INIT_VALUE;
    
    com_rslt = bmm050_init(&bmm050_dev);
    printf("BMM050 CHIP id is %#X\n", bmm050_dev.company_id);

    com_rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);

    bmm_dr_set_val = BMM050_DATA_RATE_10HZ;
    com_rslt += bmm050_set_data_rate(bmm_dr_set_val);
    com_rslt += bmm050_get_data_rate(&bmm_dr_get_val);
    printf("BMM050 data rate is %#X\n", bmm_dr_get_val);
}

void BMX055_updateData(void)
{
    // struct bma2x2_accel_data acce_data;
    // struct bmg160_data_t gyro_data;
    struct bmm050_mag_s32_data_t magn_data;
    while(1)
    {
        // com_rslt += bma2x2_read_accel_xyz(&acce_data);
        // printf("acce[x]: %d acce[y]: %d acce[z]: %d\n", acce_data.x, acce_data.y, acce_data.z);
        // com_rslt += bmg160_get_data_XYZ(&gyro_data);
        // printf("gyro[x]: %d gyro[y]: %d gyro[z]: %d\n", gyro_data.datax, gyro_data.datay, gyro_data.dataz);
        com_rslt += bmm050_read_mag_data_XYZ_s32(&magn_data);
        printf("magn[x]: %d magn[y]: %d magn[z]: %d\n", magn_data.datax, magn_data.datay, magn_data.dataz);
    }
}