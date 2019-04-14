#ifndef __SOPHUM_MODULE_ESC_H__
#define __SOPHUM_MODULE_ESC_H__

#include "driver/rmt.h"
#include "driver/ledc.h"
#include "sophum_driver_sys.h"

#define ESC_TX_GPIO0    15
#define ESC_TX_GPIO1    16
#define ESC_TX_GPIO2    17
#define ESC_TX_GPIO3    5

#define ESC_TX_CHANNEL0 RMT_CHANNEL_0
#define ESC_TX_CHANNEL1 RMT_CHANNEL_1
#define ESC_TX_CHANNEL2 RMT_CHANNEL_2
#define ESC_TX_CHANNEL3 RMT_CHANNEL_3
#define ESC_TX_CHANNEL_NUM 4

typedef enum {
    PWM_TYPE_STANDARD = 0,
    PWM_TYPE_DSHOT600
}ESC_MotorProtocolTypes;

extern void ESC_Init(ESC_MotorProtocolTypes protocol_type);
extern void ESC_updateOutput(ESC_MotorProtocolTypes protocol_type, uint16_t output_ch0,
                             uint16_t output_ch1, uint16_t output_ch2, uint16_t output_ch3);

extern void ESC_txTask(ESC_MotorProtocolTypes protocol_type);
#endif