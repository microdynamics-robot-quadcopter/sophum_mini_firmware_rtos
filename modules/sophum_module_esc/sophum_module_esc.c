#include "sophum_module_esc.h"

const rmt_item32_t esc_bit0 = {{{24, 1, 50, 0 }}};
const rmt_item32_t esc_bit1 = {{{50, 1, 25, 0 }}};

static uint8_t gs_tx_pack_len = 16;

static rmt_item32_t gs_tx_pack0[] = {
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}}
};

static rmt_item32_t gs_tx_pack1[] = {
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}}
};

static rmt_item32_t gs_tx_pack2[] = {
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}}
};

static rmt_item32_t gs_tx_pack3[] = {
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}},
    {{{24, 1, 50, 0 }}}
};

static uint8_t ESC_getChannel(uint8_t val)
{
    if(val == 0) return RMT_CHANNEL_0;
    else if(val == 1) return RMT_CHANNEL_1;
    else if(val == 2) return RMT_CHANNEL_2;
    else if(val == 3) return RMT_CHANNEL_3;
    else return RMT_CHANNEL_0;
}

static uint8_t ESC_getGPIO(uint8_t val)
{
    if(val == 0) return ESC_TX_GPIO0;
    else if(val == 1) return ESC_TX_GPIO1;
    else if(val == 2) return ESC_TX_GPIO2;
    else if(val == 3) return ESC_TX_GPIO3;
    else return ESC_TX_GPIO0;
}

static uint16_t ESC_limitRange(uint16_t val, uint16_t minval, uint16_t maxval)
{
    if(val < minval) return minval;
    else if(val > maxval) return maxval;
    else return val;
}

static void ESC_initPWMStandard(void)
{

}

/* val: [48~2048] */
static uint16_t ESC_prepareDshotPacket(uint16_t val)
{
    uint16_t packet = (val << 1) | 0;

    /* compute checksum */
    int csum = 0, csum_data = packet;
    for (int i = 0; i < 3; i++)
    {
        csum ^=  csum_data;   /* xor data by nibbles */
        csum_data >>= 4;
    }
    csum &= 0xF;
    packet = (packet << 4) | csum; /* append checksum */

    return packet;
}

static void ESC_transPacketToRmtItems(uint16_t val, uint8_t id)
{
    if(id == 0)
    {
        for(int i = gs_tx_pack_len - 1; i >= 0; i--)
        {
            if(val & (1 << i)) gs_tx_pack0[gs_tx_pack_len-1-i] = esc_bit1;
            else gs_tx_pack0[gs_tx_pack_len-1-i] = esc_bit0;
        }
    }
    else if(id == 1)
    {
        for(int i = gs_tx_pack_len - 1; i >= 0; i--)
        {
            if(val & (1 << i)) gs_tx_pack1[gs_tx_pack_len-1-i] = esc_bit1;
            else gs_tx_pack1[gs_tx_pack_len-1-i] = esc_bit0;
        }
    }
    else if(id == 2)
    {
        for(int i = gs_tx_pack_len - 1; i >= 0; i--)
        {
            if(val & (1 << i)) gs_tx_pack2[gs_tx_pack_len-1-i] = esc_bit1;
            else gs_tx_pack2[gs_tx_pack_len-1-i] = esc_bit0;
        }
    }
    else if(id == 3)
    {
        for(int i = gs_tx_pack_len - 1; i >= 0; i--)
        {
            if(val & (1 << i)) gs_tx_pack3[gs_tx_pack_len-1-i] = esc_bit1;
            else gs_tx_pack3[gs_tx_pack_len-1-i] = esc_bit0;
        }
    }
    else
    {
        for(int i = gs_tx_pack_len - 1; i >= 0; i--)
        {
            if(val & (1 << i)) gs_tx_pack0[gs_tx_pack_len-1-i] = esc_bit1;
            else gs_tx_pack0[gs_tx_pack_len-1-i] = esc_bit0;
        }
    }
}

static void ESC_initDSHOT600(void)
{
    rmt_config_t config;
    config.rmt_mode      = RMT_MODE_TX;
    config.clk_div       = 2; /* 1/(80MHz/2) = 25ns(one tick) */
    config.mem_block_num = 1;

    config.tx_config.loop_en = 0;
    config.tx_config.carrier_en = 0;
    config.tx_config.carrier_freq_hz = 611;
    config.tx_config.carrier_duty_percent = 50;
    config.tx_config.carrier_level = 1;
    config.tx_config.idle_output_en = 0;
    config.tx_config.idle_level = 0;

    for(int i = 0; i < ESC_TX_CHANNEL_NUM; i++)
    {
        config.channel  = ESC_getChannel(i);
        config.gpio_num = ESC_getGPIO(i);
        ESP_ERROR_CHECK(rmt_config(&config));
        ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    // ESP_ERROR_CHECK(rmt_translator_init(config.channel, u8_to_rmt));
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    for(int i = 1; i <= 3000; i++)
    {
        rmt_write_items(RMT_CHANNEL_0, gs_tx_pack0, gs_tx_pack_len, true);
        rmt_write_items(RMT_CHANNEL_1, gs_tx_pack1, gs_tx_pack_len, true);
        rmt_write_items(RMT_CHANNEL_2, gs_tx_pack2, gs_tx_pack_len, true);
        rmt_write_items(RMT_CHANNEL_3, gs_tx_pack3, gs_tx_pack_len, true);
        SOPHUM_delayUs(279);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

void ESC_Init(ESC_MotorProtocolTypes protocol_type)
{
    if(protocol_type == PWM_TYPE_STANDARD)
    {
        ESC_initPWMStandard();
    }
    else if(protocol_type == PWM_TYPE_DSHOT600)
    {
        ESC_initDSHOT600();
    }
}

/* output_ch*: [1000~2000] min: about 1088 */
void ESC_updateOutput(ESC_MotorProtocolTypes protocol_type, uint16_t output_ch0,
                      uint16_t output_ch1, uint16_t output_ch2, uint16_t output_ch3)
{
    ESC_limitRange(output_ch0, 1000, 2000);
    ESC_limitRange(output_ch1, 1000, 2000);
    ESC_limitRange(output_ch2, 1000, 2000);
    ESC_limitRange(output_ch3, 1000, 2000);

    if(protocol_type == PWM_TYPE_STANDARD)
    {

    }
    else if(protocol_type == PWM_TYPE_DSHOT600)
    {
        output_ch0 = (output_ch0 - 1000) * 2 + 48;
        output_ch1 = (output_ch1 - 1000) * 2 + 48;
        output_ch2 = (output_ch2 - 1000) * 2 + 48;
        output_ch3 = (output_ch3 - 1000) * 2 + 48;
        ESC_transPacketToRmtItems(ESC_prepareDshotPacket(output_ch0), 0);
        ESC_transPacketToRmtItems(ESC_prepareDshotPacket(output_ch1), 1);
        ESC_transPacketToRmtItems(ESC_prepareDshotPacket(output_ch2), 2);
        ESC_transPacketToRmtItems(ESC_prepareDshotPacket(output_ch3), 3);
    }
}

void ESC_txTask(ESC_MotorProtocolTypes protocol_type)
{
    if(protocol_type == PWM_TYPE_STANDARD){}
    else if(protocol_type == PWM_TYPE_DSHOT600)
    {
        while(1)
        {
            rmt_write_items(RMT_CHANNEL_0, gs_tx_pack0, gs_tx_pack_len, true);
            rmt_write_items(RMT_CHANNEL_1, gs_tx_pack1, gs_tx_pack_len, true);
            rmt_write_items(RMT_CHANNEL_2, gs_tx_pack2, gs_tx_pack_len, true);
            rmt_write_items(RMT_CHANNEL_3, gs_tx_pack3, gs_tx_pack_len, true);
            vTaskDelay(40 / portTICK_PERIOD_MS);
        }
    }
}