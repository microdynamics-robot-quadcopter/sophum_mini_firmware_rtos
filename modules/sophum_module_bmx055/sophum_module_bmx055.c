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

    u8 bw_value_u8 = BMA2x2_INIT_VALUE;
    u8 banwid      = BMA2x2_INIT_VALUE;

    com_rslt = bma2x2_init(&bma2x2_dev);
    printf("BMA2X2 CHIP id is %#X\n", bma2x2_dev.chip_id);

    com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

    bw_value_u8 = 0x08; /* set bandwidth of 7.81Hz*/
    com_rslt += bma2x2_set_bw(bw_value_u8);
    /* This API used to read back the written value of bandwidth*/
    com_rslt += bma2x2_get_bw(&banwid);
    printf("BMA2X2 bandwidth setvalue is %#X\n", banwid);
    /*===================BMA2X2 func init end===================================*/

    /*===================BMG160 func init start=================================*/
    bmg160_dev.bus_read   = BMX055_readMultiBytes;
    bmg160_dev.bus_write  = BMX055_writeMultiBytes;
    bmg160_dev.delay_msec = SOPHUM_delayMs;
    bmg160_dev.dev_addr   = BMG160_I2C_ADDR1;

    u8	v_gyro_value_u8 = BMG160_INIT_VALUE;
    u8 v_bw_u8 = BMG160_INIT_VALUE;
    com_rslt = bmg160_init(&bmg160_dev);
    printf("BMG160 CHIP id is %#X\n", bmg160_dev.chip_id);

    com_rslt += bmg160_set_power_mode(BMG160_MODE_NORMAL);

    v_bw_u8 = C_BMG160_BW_230HZ_U8X;/* set gyro bandwidth of 230Hz*/
    com_rslt += bmg160_set_bw(v_bw_u8);
    com_rslt += bmg160_get_bw(&v_gyro_value_u8);
    printf("BMG160 bandwidth setvalue is %#X\n", v_gyro_value_u8);
    /*===================BMG160 func init end===================================*/

    /*===================BMM050 func init start=================================*/
    bmm050_dev.bus_read   = BMX055_readMultiBytes;
    bmm050_dev.bus_write  = BMX055_writeMultiBytes;
    bmm050_dev.delay_msec = SOPHUM_delayMs;
    bmm050_dev.dev_addr   = BMM050_I2C_ADDRESS;

    u8 v_data_rate_u8 = BMM050_INIT_VALUE;
    u8 v_data_rate_value_u8 = BMM050_INIT_VALUE;
    com_rslt = bmm050_init(&bmm050_dev);
    printf("BMM050 CHIP id is %#X\n", bmm050_dev.company_id);

    com_rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);

    v_data_rate_value_u8 = BMM050_DATA_RATE_30HZ;/* set data rate of 30Hz */
    com_rslt += bmm050_set_data_rate(v_data_rate_value_u8);
    /* This API used to read back the written value of data rate */
    com_rslt += bmm050_get_data_rate(&v_data_rate_u8);
    printf("BMM050 data rate is %#X\n", v_data_rate_u8);
    /*===================BMM050 func init end===================================*/
}

void BMX055_updateData(void)
{
    struct bma2x2_accel_data sample_xyz;
    struct bmg160_data_t data_gyro;
    struct bmm050_mag_s32_data_t data_s32;
    /* Read the acce XYZ data */
    /* Read the gyro XYZ data */
        /* Read the magn XYZ data */
    while(1)
    {
        com_rslt += bma2x2_read_accel_xyz(&sample_xyz);
        printf("acce[x]: %d acce[y]: %d acce[z]: %d\n", sample_xyz.x, sample_xyz.y, sample_xyz.z);
        // com_rslt += bmg160_get_data_XYZ(&data_gyro);
        // printf("gyro[x]: %d gyro[y]: %d gyro[z]: %d\n", data_gyro.datax, data_gyro.datay, data_gyro.dataz);
        // com_rslt += bmm050_read_mag_data_XYZ_s32(&data_s32);
        // printf("magn[x]: %d magn[y]: %d magn[z]: %d\n", data_s32.datax, data_s32.datay, data_s32.dataz);
    }
}