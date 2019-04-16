#ifndef __SOPHUM_MODULE_BAT_H__
#define __SOPHUM_MODULE_BAT_H__

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define BAT_DEFAULT_VREF 1100
#define BAT_SAMPLES_NUM  64

#define BAT_ADC_CHANNEL   ADC_CHANNEL_6
#define BAT_ADC_ATTEN     ADC_ATTEN_DB_0
#define BAT_ADC_WIDTH_BIT ADC_WIDTH_BIT_12

extern void BAT_Init(void);
extern void BAT_updateData(void);

#endif