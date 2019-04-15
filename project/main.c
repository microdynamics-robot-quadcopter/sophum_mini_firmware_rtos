#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "sophum_driver_i2c.h"
#include "sophum_driver_spi.h"
#include "sophum_driver_sys.h"
#include "sophum_driver_uart.h"
#include "sophum_module_esc.h"
#include "sophum_module_bmp388.h"
#include "sophum_module_bmx055.h"
#include "sophum_module_upixels.h"
#include "sophum_module_pmw3901.h"
#include "sophum_module_vl53l0x.h"
#include "sophum_module_vl53l1x.h"


// #define BMP388_ADDR 0x76
// #define BMP388_CHIP_ID 0x00


// void BMP388_test(void)
// {
//     int ret;
//     uint8_t tmp_data;
//     ret = I2C_readOneByte(I2C_NUM1_MASTER_PORT_GPIO, BMP388_ADDR << 1, BMP388_CHIP_ID, &tmp_data);
//     if(ret == false)
//     {
//         printf("the BMP388 read ID is ERROR!!!\n");
//     }
//     else
//     {
//         printf("the BMP388 read ID is SUCCESS!!!\n");
//         printf("the chip id is %u\n", tmp_data);
//     }
// }

// struct BMP388_dev bmp388_dev;

static xQueueHandle gpio_evt_queue = NULL;

void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void gpio_task(void *arg)
{
    printf("\r\n into gpio neg edge check task\r\n");
    uint32_t io_num;

    while(1)
    {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            printf("GPIO_%d get neg edge interrupt!!! current state: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

struct bmp3_dev bmp388_dev;
// static VL53L0X_Dev_t vl53l0x_dev;
// static VL53L1_Dev_t  vl53l1x_dev;

uint8_t tmp_data;

void app_main()
{
    /*===================gpio interrupt func test start===========================*/
/*    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_SEL_4;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(1);
    gpio_isr_handler_add(GPIO_NUM_4, gpio_isr_handler, (void*) GPIO_NUM_4);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
*/    
    /*===================gpio interrupt func test end===========================*/

    UART1_Init(UART1_BAUD_RATE, UART1_DATA_BITS, UART1_PARITY_EN, UART1_STOP_BITS, UART1_FLOW_CTRL_EN);
    
    // UPIXELS_Init();
    // SOPHUM_delayMs(1000);
    // while(1)
    // {
    //     UPIXELS_updateData();
    // }
    // while(1)
    // {
    //     UPIXELS_Init();
    //     SOPHUM_delayMs(1000);
    // }
    
    // uint8_t uart_rev_data[UART1_BUFF_SIZE];
    // while(1)
    // {
    //     // Read data from the UART
    //     int len = uart_read_bytes(UART_NUM_1, uart_rev_data, UART1_BUFF_SIZE, 20 / portTICK_RATE_MS);
    //     // Write data back to the UART
    //     uart_write_bytes(UART_NUM_1, (const char *) uart_rev_data, len);
    // }


    /* right */
    // ESC_Init(PWM_TYPE_DSHOT600);
    // ESC_updateOutput(PWM_TYPE_DSHOT600, 1088, 1088, 1088, 1088);
    // ESC_txTask(PWM_TYPE_DSHOT600);


    I2C_MasterInit(I2C_NUM1_MASTER_PORT_GPIO, I2C_NUM1_MASTER_SCL_GPIO, I2C_NUM1_MASTER_SDA_GPIO,
                   I2C_NUM1_MASTER_FREQ_HZ);

    BMX055_Init();
    BMX055_updateData();

    /* right */
    // vl53l0x_dev.i2c_port_num = I2C_NUM1_MASTER_PORT_GPIO;
    // vl53l0x_dev.i2c_address  = 0x29;
    // VL53L0X_Error vl_status = VL53L0X_Init(&vl53l0x_dev);
    // VL53L0X_RangingMeasurementData_t meas_data;
    // while(1)
    // {
    //     vl_status = VL53L0X_doSingleMeasurement(&vl53l0x_dev, &meas_data);
    //     if(vl_status == VL53L0X_ERROR_NONE)
    //     {
    //         printf("meas distance is %u mm\n", meas_data.RangeMilliMeter);
    //     }
    // }

    /* right */
    // i2c_port_t vl53l1x_i2c_port = I2C_NUM1_MASTER_PORT_GPIO;
    // vl53l1x_dev.I2cHandle  = &vl53l1x_i2c_port;
    // vl53l1x_dev.I2cDevAddr = 0x52;
    // VL53L1X_Init(&vl53l1x_dev);
    // VL53L1_RangingMeasurementData_t meas_data;
    // while(1)
    // {
    //     if(VL53L1X_doSingleMeasurement(&vl53l1x_dev, &meas_data) == VL53L1_ERROR_NONE)
    //     {
    //         printf("meas distance is %u mm\n", meas_data.RangeMilliMeter);
    //     }
        
    // }

    /*===================BMP388 func test start=================================*/
/*    int8_t ret = BMP3_OK;
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
                        struct bmp3_data out_data;
                        while(1)
                        {
                            bmp3_get_sensor_data(BMP3_ALL, &out_data, &bmp388_dev);
                            printf("temp %lld: pres: %llu\n", out_data.temperature, out_data.pressure);
                            printf("alti %lf\n", BMP388_calcAltitude(out_data.pressure / 100.0F));
                            SOPHUM_delayMs(5);
                        }
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
*/
    /*===================BMP388 func test end===================================*/


    /*===================timer func test start===========================*/
/*    int64_t tim1 = esp_timer_get_time();
    SOPHUM_delayUs(60);
    int64_t tim2 = esp_timer_get_time();
    printf("tim1:%lldus tim2:%lldus tim dis: %lldus\n", tim1, tim2, tim2 - tim1);

    tim1 = esp_timer_get_time();
    SOPHUM_delayMs(100);
    tim2 = esp_timer_get_time();
    printf("tim1:%lldus tim2:%lldus tim dis: %lldus\n", tim1, tim2, tim2 - tim1);
*/
    /*===================timer func test end===========================*/




    // uint8_t ret = 0;
    // ret = BMP388_Init(&bmp388_dev);

    // if(ret == BMP388_OK)
    // {
    //     printf("BMP388 init SUCCESS!!!\n");
    //     printf("t1:%u\n", bmp388_dev.trim_coeff.par_t1);
    //     printf("t2:%u\n", bmp388_dev.trim_coeff.par_t2);
    //     printf("t3:%d\n", bmp388_dev.trim_coeff.par_t3);
    //     printf("p1:%d\n", bmp388_dev.trim_coeff.par_p1);
    //     printf("p2:%d\n", bmp388_dev.trim_coeff.par_p2);
    //     printf("p3:%d\n", bmp388_dev.trim_coeff.par_p3);
    //     printf("p4:%d\n", bmp388_dev.trim_coeff.par_p4);
    //     printf("p5:%u\n", bmp388_dev.trim_coeff.par_p5);
    //     printf("p6:%u\n", bmp388_dev.trim_coeff.par_p6);
    //     printf("p7:%d\n", bmp388_dev.trim_coeff.par_p7);
    //     printf("p8:%d\n", bmp388_dev.trim_coeff.par_p8);
    //     printf("p9:%d\n", bmp388_dev.trim_coeff.par_p9);
    //     printf("p10:%d\n", bmp388_dev.trim_coeff.par_p10);
    //     printf("p11:%d\n", bmp388_dev.trim_coeff.par_p11);
    // }
    // else
    // {
    //     printf("BMP388 init ERROR!!!\n");
    // }
    // BMP388_test();


}

