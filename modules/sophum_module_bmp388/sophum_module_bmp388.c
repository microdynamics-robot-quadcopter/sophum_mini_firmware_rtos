#include "sophum_module_bmp388.h"

static struct bmp3_dev bmp388_dev;

static double BMP388_calcAltitude(double pres)
{
    return (1.0F - powf(pres / 101325, 0.190284F)) * 287.15F / 0.0065F;
}

int8_t BMP388_readMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int8_t ret = 0;
    ret = I2C_readMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_id << 1, reg_addr, len, data);
    if(ret == true) return BMP3_OK;
    else return BMP3_E_COMM_FAIL;
}

int8_t BMP388_writeMultiBytes(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int8_t ret = 0;
    ret = I2C_writeMultiBytes(I2C_NUM1_MASTER_PORT_GPIO, dev_id << 1, reg_addr, len, data);
    if(ret == true) return BMP3_OK;
    else return BMP3_E_COMM_FAIL;
}

void BMP388_Init(void)
{
    int8_t ret = BMP3_OK;
    bmp388_dev.dev_id   = BMP3_I2C_ADDR_PRIM;
    bmp388_dev.intf     = BMP3_I2C_INTF;
    bmp388_dev.read     = BMP388_readMultiBytes;
    bmp388_dev.write    = BMP388_writeMultiBytes;
    bmp388_dev.delay_ms = SOPHUM_delayMs;
    ret = bmp3_init(&bmp388_dev);
    if(ret == BMP3_OK)
    {
        printf("chip ID %#X\n", bmp388_dev.chip_id);

        printf("par_t1: %u\n", bmp388_dev.calib_data.reg_calib_data.par_t1);
        printf("par_t2: %u\n", bmp388_dev.calib_data.reg_calib_data.par_t2);
        printf("par_t3: %d\n", bmp388_dev.calib_data.reg_calib_data.par_t3);
        printf("par_p1: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p1);
        printf("par_p2: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p2);
        printf("par_p3: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p3);
        printf("par_p4: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p4);
        printf("par_p5: %u\n", bmp388_dev.calib_data.reg_calib_data.par_p5);
        printf("par_p6: %u\n", bmp388_dev.calib_data.reg_calib_data.par_p6);
        printf("par_p7: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p7);
        printf("par_p8: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p8);
        printf("par_p9: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p9);
        printf("par_p10: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p10);
        printf("par_p11: %d\n", bmp388_dev.calib_data.reg_calib_data.par_p11);
        ret = bmp3_get_sensor_settings(&bmp388_dev);
        if(ret == BMP3_OK)
        {
            ret = bmp3_get_status(&bmp388_dev);
            if(ret == BMP3_OK)
            {
                printf("BMP388 soft reset defalut settings:\n");
                printf("power mode: %u\n", bmp388_dev.settings.op_mode);
                printf("press en  : %u\n", bmp388_dev.settings.press_en);
                printf("temp  en  : %u\n", bmp388_dev.settings.temp_en);
                printf("press os  : %u\n", bmp388_dev.settings.odr_filter.press_os);
                printf("temp  os  : %u\n", bmp388_dev.settings.odr_filter.temp_os);
                printf("iir_filter: %u\n", bmp388_dev.settings.odr_filter.iir_filter);
                printf("odr       : %u\n", bmp388_dev.settings.odr_filter.odr);
                printf("intr output mode  : %u\n", bmp388_dev.settings.int_settings.output_mode);
                printf("intr level        : %u\n", bmp388_dev.settings.int_settings.level);
                printf("intr latch        : %u\n", bmp388_dev.settings.int_settings.latch);
                printf("intr drdy_en      : %u\n", bmp388_dev.settings.int_settings.drdy_en);
                printf("adv watch dog en  : %u\n", bmp388_dev.settings.adv_settings.i2c_wdt_en);
                printf("adv watch dog sel : %u\n", bmp388_dev.settings.adv_settings.i2c_wdt_sel);

                bmp388_dev.settings.op_mode = BMP3_NORMAL_MODE;
                bmp388_dev.settings.press_en = 1;
                bmp388_dev.settings.temp_en  = 1;
                bmp388_dev.settings.odr_filter.press_os   = BMP3_OVERSAMPLING_16X;
                bmp388_dev.settings.odr_filter.temp_os    = BMP3_OVERSAMPLING_2X;
                bmp388_dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;
                bmp388_dev.settings.odr_filter.odr        = BMP3_ODR_25_HZ;
                bmp388_dev.settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
                bmp388_dev.settings.int_settings.level       = BMP3_INT_PIN_ACTIVE_LOW;
                bmp388_dev.settings.int_settings.latch       = BMP3_INT_PIN_NON_LATCH;
                bmp388_dev.settings.int_settings.drdy_en     = 0;
                bmp388_dev.settings.adv_settings.i2c_wdt_en  = 0;
                bmp388_dev.settings.adv_settings.i2c_wdt_sel = 0;

                ret = bmp3_set_sensor_settings(BMP3_ALL_SETTINGS, &bmp388_dev);
                ret = bmp3_set_op_mode(&bmp388_dev);
                if(ret == BMP3_OK)
                {
                    printf("BMP388 user settings:\n");
                    printf("power mode        : %u\n", bmp388_dev.settings.op_mode);
                    printf("press en          : %u\n", bmp388_dev.settings.press_en);
                    printf("temp  en          : %u\n", bmp388_dev.settings.temp_en);
                    printf("press os          : %u\n", bmp388_dev.settings.odr_filter.press_os);
                    printf("temp  os          : %u\n", bmp388_dev.settings.odr_filter.temp_os);
                    printf("iir_filter        : %u\n", bmp388_dev.settings.odr_filter.iir_filter);
                    printf("odr               : %u\n", bmp388_dev.settings.odr_filter.odr);
                    printf("intr output mode  : %u\n", bmp388_dev.settings.int_settings.output_mode);
                    printf("intr level        : %u\n", bmp388_dev.settings.int_settings.level);
                    printf("intr latch        : %u\n", bmp388_dev.settings.int_settings.latch);
                    printf("intr drdy_en      : %u\n", bmp388_dev.settings.int_settings.drdy_en);
                    printf("adv watch dog en  : %u\n", bmp388_dev.settings.adv_settings.i2c_wdt_en);
                    printf("adv watch dog sel : %u\n", bmp388_dev.settings.adv_settings.i2c_wdt_sel);

                    if(ret == BMP3_OK)
                    {
                        return;
                    }
                }
            }
            else
            {
                printf("bmp3 get status error!!!\n");
            }
        }
        else
        {
            printf("bmp3 get sensor settings error!!!\n");
        }
    }
    else
    {
        printf("bmp388 error\n");
    }
}

void BMP388_updateData(void)
{
    struct bmp3_data out_data;

    bmp3_get_sensor_data(BMP3_ALL, &out_data, &bmp388_dev);
    printf("temp %lld: pres: %llu\n", out_data.temperature, out_data.pressure);
    printf("alti %lf\n", BMP388_calcAltitude(out_data.pressure / 100.0F));
    SOPHUM_delayMs(5);
}
