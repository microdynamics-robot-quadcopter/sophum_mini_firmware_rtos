#include "sophum_module_pmw3901.h"


spi_device_handle_t pmw3901_dev;

static uint8_t PMW3901_readRegister(uint8_t reg_addr);
static void    PMW3901_writeRegister(uint8_t reg_addr, uint8_t value);
static void    PMW3901_initRegister(void);

static uint8_t PMW3901_readRegister(uint8_t reg_addr)
{
    uint8_t tmp_data = 0x00;
    gpio_set_level(PMW3901_CS_PIN, 0);
    SOPHUM_delayUs(60);
    SPI_readOneByte(pmw3901_dev, reg_addr, &tmp_data);
    SOPHUM_delayUs(250);
    gpio_set_level(PMW3901_CS_PIN, 1);
    SOPHUM_delayUs(250);

    return tmp_data;
}

static void PMW3901_writeRegister(uint8_t reg_addr, uint8_t value)
{
    gpio_set_level(PMW3901_CS_PIN, 0);
    SOPHUM_delayUs(60);
    SPI_writeOneByte(pmw3901_dev, reg_addr, value);
    SOPHUM_delayUs(60);
    gpio_set_level(PMW3901_CS_PIN, 1);
    SOPHUM_delayUs(250);
}

static void PMW3901_initRegister(void)
{
    PMW3901_writeRegister(0x7F, 0x00);
    PMW3901_writeRegister(0x61, 0xAD);
    PMW3901_writeRegister(0x7F, 0x03);
    PMW3901_writeRegister(0x40, 0x00);
    PMW3901_writeRegister(0x7F, 0x05);
    PMW3901_writeRegister(0x41, 0xB3);
    PMW3901_writeRegister(0x43, 0xF1);
    PMW3901_writeRegister(0x45, 0x14);
    PMW3901_writeRegister(0x5B, 0x32);
    PMW3901_writeRegister(0x5F, 0x34);
    PMW3901_writeRegister(0x7B, 0x08);
    PMW3901_writeRegister(0x7F, 0x06);
    PMW3901_writeRegister(0x44, 0x1B);
    PMW3901_writeRegister(0x40, 0xBF);
    PMW3901_writeRegister(0x4E, 0x3F);
    PMW3901_writeRegister(0x7F, 0x08);
    PMW3901_writeRegister(0x65, 0x20);
    PMW3901_writeRegister(0x6A, 0x18);
    PMW3901_writeRegister(0x7F, 0x09);
    PMW3901_writeRegister(0x4F, 0xAF);
    PMW3901_writeRegister(0x5F, 0x40);
    PMW3901_writeRegister(0x48, 0x80);
    PMW3901_writeRegister(0x49, 0x80);
    PMW3901_writeRegister(0x57, 0x77);
    PMW3901_writeRegister(0x60, 0x78);
    PMW3901_writeRegister(0x61, 0x78);
    PMW3901_writeRegister(0x62, 0x08);
    PMW3901_writeRegister(0x63, 0x50);
    PMW3901_writeRegister(0x7F, 0x0A);
    PMW3901_writeRegister(0x45, 0x60);
    PMW3901_writeRegister(0x7F, 0x00);
    PMW3901_writeRegister(0x4D, 0x11);
    PMW3901_writeRegister(0x55, 0x80);
    PMW3901_writeRegister(0x74, 0x1F);
    PMW3901_writeRegister(0x75, 0x1F);
    PMW3901_writeRegister(0x4A, 0x78);
    PMW3901_writeRegister(0x4B, 0x78);
    PMW3901_writeRegister(0x44, 0x08);
    PMW3901_writeRegister(0x45, 0x50);
    PMW3901_writeRegister(0x64, 0xFF);
    PMW3901_writeRegister(0x65, 0x1F);
    PMW3901_writeRegister(0x7F, 0x14);
    PMW3901_writeRegister(0x65, 0x60);
    PMW3901_writeRegister(0x66, 0x08);
    PMW3901_writeRegister(0x63, 0x78);
    PMW3901_writeRegister(0x7F, 0x15);
    PMW3901_writeRegister(0x48, 0x58);
    PMW3901_writeRegister(0x7F, 0x07);
    PMW3901_writeRegister(0x41, 0x0D);
    PMW3901_writeRegister(0x43, 0x14);
    PMW3901_writeRegister(0x4B, 0x0E);
    PMW3901_writeRegister(0x45, 0x0F);
    PMW3901_writeRegister(0x44, 0x42);
    PMW3901_writeRegister(0x4C, 0x80);
    PMW3901_writeRegister(0x7F, 0x10);
    PMW3901_writeRegister(0x5B, 0x02);
    PMW3901_writeRegister(0x7F, 0x07);
    PMW3901_writeRegister(0x40, 0x41);
    PMW3901_writeRegister(0x70, 0x00);

    SOPHUM_delayMs(100);
    PMW3901_writeRegister(0x32, 0x44);
    PMW3901_writeRegister(0x7F, 0x07);
    PMW3901_writeRegister(0x40, 0x40);
    PMW3901_writeRegister(0x7F, 0x06);
    PMW3901_writeRegister(0x62, 0xf0);
    PMW3901_writeRegister(0x63, 0x00);
    PMW3901_writeRegister(0x7F, 0x0D);
    PMW3901_writeRegister(0x48, 0xC0);
    PMW3901_writeRegister(0x6F, 0xd5);
    PMW3901_writeRegister(0x7F, 0x00);
    PMW3901_writeRegister(0x5B, 0xa0);
    PMW3901_writeRegister(0x4E, 0xA8);
    PMW3901_writeRegister(0x5A, 0x50);
    PMW3901_writeRegister(0x40, 0x80);
}

bool PMW3901_Init(void)
{
    /* config PWM3901 SPI cs pin */
    gpio_config_t pwm3901_cs_pin_conf;
    pwm3901_cs_pin_conf.intr_type    = GPIO_PIN_INTR_DISABLE;
    pwm3901_cs_pin_conf.mode         = GPIO_MODE_OUTPUT;
    pwm3901_cs_pin_conf.pin_bit_mask = PMW3901_CS_PIN_MASK;
    pwm3901_cs_pin_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    pwm3901_cs_pin_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    
    gpio_config(&pwm3901_cs_pin_conf);

    /* make sure the SPI BUS is reset */
    gpio_set_level(PMW3901_CS_PIN, 1);
    SOPHUM_delayUs(1100);
    gpio_set_level(PMW3901_CS_PIN, 0);
    SOPHUM_delayUs(1100);
    gpio_set_level(PMW3901_CS_PIN, 1);
    SOPHUM_delayUs(1100);

    /* power on reset */
    PMW3901_writeRegister(0x3A, 0x5A);
    SOPHUM_delayUs(5100);

    /* test the SPI communciation, check chip ID and inverse chip ID */
    uint8_t pmw3901_id = PMW3901_readRegister(0x00);
    uint8_t pmw3901_inv_id = PMW3901_readRegister(0x5F);

    printf("pwm3901 id is %#X inv id is %#X\n", pmw3901_id, pmw3901_inv_id);

    if(pmw3901_id != 0x49 && pmw3901_inv_id != 0xB8)
    {
        return false;
    }

    /* reading the motion registers one time */
    PMW3901_readRegister(0x02);
    PMW3901_readRegister(0x03);
    PMW3901_readRegister(0x04);
    PMW3901_readRegister(0x05);
    PMW3901_readRegister(0x06);

    SOPHUM_delayUs(1100);
    PMW3901_initRegister();

    return true;
}


void PMW3901_readMotionCount(int16_t *delta_x, int16_t *delta_y)
{
    PMW3901_readRegister(0x02);
    *delta_x = ((int16_t)PMW3901_readRegister(0x04) << 8) | PMW3901_readRegister(0x03);
    *delta_y = ((int16_t)PMW3901_readRegister(0x06) << 8) | PMW3901_readRegister(0x05);
}