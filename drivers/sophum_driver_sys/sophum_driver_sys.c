#include "sophum_driver_sys.h"


void SOPHUM_delayMs(int32_t nms)
{
    vTaskDelay(nms / portTICK_RATE_MS);
}

void SOPHUM_delayUs(int16_t nus)
{
    int64_t tim1 = esp_timer_get_time();
    while(esp_timer_get_time() < (tim1 + nus))
    {

    }
}