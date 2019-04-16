#include "sophum_module_bat.h"

static esp_adc_cal_characteristics_t *adc_chars;

static void BAT_checkeFuse(void)
{
    /* check TP is burned into eFuse */
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }

    /* check Vref is burned into eFuse */
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void BAT_printValType(esp_adc_cal_value_t val_type)
{
    if(val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if(val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

void BAT_Init(void)
{
    BAT_checkeFuse();

    /* configure ADC */
    adc1_config_width(BAT_ADC_WIDTH_BIT);
    adc1_config_channel_atten(BAT_ADC_CHANNEL, BAT_ADC_ATTEN);

    /* characterize ADC */
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, BAT_ADC_ATTEN,
                                   BAT_ADC_WIDTH_BIT, BAT_DEFAULT_VREF, adc_chars);
    BAT_printValType(val_type);
}

void BAT_updateData(void)
{
    static uint8_t  count    = 1;
    static uint32_t adc_data = 0;
    uint32_t adc_volt = 0;
    if(count == BAT_SAMPLES_NUM)
    {

        adc_data /= BAT_SAMPLES_NUM;
        adc_volt = esp_adc_cal_raw_to_voltage(adc_data, adc_chars);
        printf("Raw: %d\tVoltage: %dmV\n", adc_data, adc_volt); /* debug */
        count    = 1;
        adc_data = 0;
    }
    else
    {
        count++;
        adc_data += adc1_get_raw((adc1_channel_t)BAT_ADC_CHANNEL);
    }
}