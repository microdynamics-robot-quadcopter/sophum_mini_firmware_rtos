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
#include "sophum_driver_store.h"
#include "sophum_module_esc.h"
#include "sophum_module_bat.h"
// #include "sophum_module_bmp388.h"
// #include "sophum_module_bmx055.h"
// #include "sophum_module_upixels.h"
#include "sophum_module_pmw3901.h"
// #include "sophum_module_vl53l0x.h"
#include "sophum_module_vl53l1x.h"


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


// static VL53L0X_Dev_t vl53l0x_dev;
// static VL53L1_Dev_t  vl53l1x_dev;
extern spi_device_handle_t pmw3901_dev;

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


    // gpio_config_t pwm3901_cs_pin_conf;
    // pwm3901_cs_pin_conf.intr_type    = GPIO_PIN_INTR_DISABLE;
    // pwm3901_cs_pin_conf.mode         = GPIO_MODE_OUTPUT;
    // pwm3901_cs_pin_conf.pin_bit_mask = PMW3901_CS_PIN_MASK;
    // pwm3901_cs_pin_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    // pwm3901_cs_pin_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    
    // gpio_config(&pwm3901_cs_pin_conf);

    // while(1)
    // {
    //     /* make sure the SPI BUS is reset */
    //     gpio_set_level(PMW3901_CS_PIN, 1);
    //     SOPHUM_delayUs(1100);
    //     gpio_set_level(PMW3901_CS_PIN, 0);
    //     SOPHUM_delayUs(1100);
    //     gpio_set_level(PMW3901_CS_PIN, 1);
    //     SOPHUM_delayUs(1100);
    // }
    

    
    
    SPI_MasterInit(SPI_HSPI_MASTER_PORT_GPIO, SPI_HSPI_MASTER_MOSI_GPIO, SPI_HSPI_MASTER_MISO_GPIO,
                   SPI_HSPI_MASTER_SCK_GPIO, 4094);
    SPI_addDevice(SPI_HSPI_MASTER_PORT_GPIO, SPI_MASTER_MODE_3, SPI_HSPI_MASTER_CLK_SPEED, PMW3901_CS_PIN,
                 &pmw3901_dev);

    while(1)
    {
        if(PMW3901_Init() == true)
        {
            int16_t pmw_datax = 0, pmw_datay = 0;
            SOPHUM_delayMs(1000);
            while(1)
            {
                PMW3901_readMotionCount(&pmw_datax, &pmw_datay);
                printf("X: %d Y: %d\n", pmw_datax, pmw_datay);
                SOPHUM_delayMs(100);
            }
        }
    }
    if(PMW3901_Init() == true)
    {
        int16_t pmw_datax = 0, pmw_datay = 0;
        while(1)
        {
            PMW3901_readMotionCount(&pmw_datax, &pmw_datay);
            printf("X: %u Y: %u\n", pmw_datax, pmw_datay);
        }
    }
    else
    {   
        printf("pmw3901 init is error!\n");
        while(1);
    }
    

    UART1_Init(UART1_BAUD_RATE, UART1_DATA_BITS, UART1_PARITY_EN, UART1_STOP_BITS, UART1_FLOW_CTRL_EN);
    char uart_test[] = "maksyuki";
    uint8_t recv[66];
    int recv_len;
    int len = sizeof(uart_test);
    while(1)
    {
        uart_write_bytes(UART_NUM_1, (const char *) uart_test, len);
        recv_len = uart_read_bytes(UART_NUM_1, recv, UART1_BUFF_SIZE, 1000 / portTICK_RATE_MS);
        for(int i = 0; i < recv_len; i++) printf("%c", recv[i]);
        printf("\n");
        SOPHUM_delayMs(1000);
    }
    
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
    // ESC_Init(PWM_TYPE_STANDARD);
    // ESC_updateOutput(PWM_TYPE_STANDARD, 1500, 1500, 1500, 1500);
    // ESC_txTask(PWM_TYPE_STANDARD);
    // while(1);

    I2C_MasterInit(I2C_NUM1_MASTER_PORT_GPIO, I2C_NUM1_MASTER_SCL_GPIO, I2C_NUM1_MASTER_SDA_GPIO,
                   I2C_NUM1_MASTER_FREQ_HZ);

    /* right */
    // STORE_Init();
    // STORE_writeData();
    // while(1);

    /* right */
    // BAT_Init();
    // while(1)
    // {
    //     BAT_updateData();
    // }

    // BMX055_Init();
    // BMX055_updateData();

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
    
    /* right */
    // BMP388_Init();
    // while(1)
    // {
    //     BMP388_updateData();
    // }


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
}

