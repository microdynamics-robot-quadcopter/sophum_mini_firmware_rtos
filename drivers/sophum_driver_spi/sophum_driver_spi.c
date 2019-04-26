#include "sophum_driver_spi.h"

esp_err_t SPI_MasterInit(uint8_t spi_port, int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz)
{
    spi_bus_config_t conf;
    conf.mosi_io_num = mosi_io_num;
    conf.miso_io_num = miso_io_num;
    conf.sclk_io_num = sclk_io_num;
    conf.quadwp_io_num = -1; /* -1 means not used */
    conf.quadhd_io_num = -1; /* -1 means not used */
    conf.max_transfer_sz = max_transfer_sz;

    return spi_bus_initialize(spi_port, &conf, 0); /* 0 means DMA not used */    
}

esp_err_t SPI_closeBus(spi_host_device_t handle)
{
    return spi_bus_free(handle);
}

esp_err_t SPI_addDevice(uint8_t spi_port, uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle)
{
    spi_device_interface_config_t dev_conf;
    dev_conf.command_bits     = 0;
    dev_conf.address_bits     = 8;
    dev_conf.dummy_bits       = 0;
    dev_conf.mode             = mode;
    dev_conf.duty_cycle_pos   = 128;  /* default 128 = 50%/50% duty */
    dev_conf.cs_ena_pretrans  = 0;    /* 0 not used */
    dev_conf.cs_ena_posttrans = 0;    /* 0 not used */
    dev_conf.clock_speed_hz   = clock_speed_hz;
    dev_conf.spics_io_num     = -1;   /* not used */
    dev_conf.flags            = 0;    /* 0 not used */
    dev_conf.queue_size       = 1;
    dev_conf.pre_cb           = NULL;
    dev_conf.post_cb          = NULL;
    return spi_bus_add_device(spi_port, &dev_conf, handle);
}

esp_err_t SPI_removeDevice(spi_device_handle_t handle)
{
    return spi_bus_remove_device(handle);
}


esp_err_t SPI_writeOneBit(spi_device_handle_t handle, uint8_t reg_addr, uint8_t bit_id, uint8_t data)
{
    uint8_t tmp_buf;
    esp_err_t err = SPI_readOneByte(handle, reg_addr, &tmp_buf);
    if (err) return err;

    tmp_buf = data ? (tmp_buf | (1 << bit_id)) : (tmp_buf & ~(1 << bit_id));
    return SPI_writeOneByte(handle, reg_addr, tmp_buf);
}


esp_err_t SPI_writeMultiBits(spi_device_handle_t handle, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t data)
{
    uint8_t tmp_buf;
    esp_err_t err = SPI_readOneByte(handle, reg_addr, &tmp_buf);
    if (err) return err;
    uint8_t mask = ((1 << bit_len) - 1) << (bit_start - bit_len + 1);
    data <<= (bit_start - bit_len + 1);
    data &= mask;
    tmp_buf &= ~mask;
    tmp_buf |= data;
    return SPI_writeOneByte(handle, reg_addr, tmp_buf);
}


esp_err_t SPI_writeOneByte(spi_device_handle_t handle, uint8_t reg_addr, uint8_t data)
{
    return SPI_writeMultiBytes(handle, reg_addr, 1, &data);
}


esp_err_t SPI_writeMultiBytes(spi_device_handle_t handle, uint8_t reg_addr, size_t bit_len, const uint8_t *data)
{
    spi_transaction_t transaction;
    transaction.flags     = 0;
    transaction.cmd       = 0;
    transaction.addr      = reg_addr | SPIBUS_WRITE;
    // transaction.addr      = reg_addr | 0x80u;
    transaction.length    = bit_len * 8;
    transaction.rxlength  = 0;
    transaction.user      = NULL;
    transaction.tx_buffer = data;
    transaction.rx_buffer = NULL;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    return err;
}


esp_err_t SPI_readOneBit(spi_device_handle_t handle, uint8_t reg_addr, uint8_t bit_id, uint8_t *data)
{
    return SPI_readMultiBits(handle, reg_addr, bit_id, 1, data);
}


esp_err_t SPI_readMultiBits(spi_device_handle_t handle, uint8_t reg_addr, uint8_t bit_start, uint8_t bit_len, uint8_t *data)
{
    uint8_t tmp_buf;
    esp_err_t err = SPI_readOneByte(handle, reg_addr, &tmp_buf);

    if(!err)
    {
        uint8_t mask = ((1 << bit_len) - 1) << (bit_start - bit_len + 1);
        tmp_buf &= mask;
        tmp_buf >>= (bit_start - bit_len + 1);
        *data = tmp_buf;
    }
    return err;
}


esp_err_t SPI_readOneByte(spi_device_handle_t handle, uint8_t reg_addr, uint8_t *data)
{
    return SPI_readMultiBytes(handle, reg_addr, 1, data);
}


esp_err_t SPI_readMultiBytes(spi_device_handle_t handle, uint8_t reg_addr, size_t bit_len, uint8_t *data)
{
    if(bit_len == 0) return ESP_ERR_INVALID_SIZE;

    spi_transaction_t transaction;
    transaction.flags     = 0;
    transaction.cmd       = 0;
    transaction.addr      = reg_addr & SPIBUS_READ;
    // transaction.addr      = reg_addr & (~0x80u);
    transaction.length    = bit_len * 8;
    transaction.rxlength  = bit_len * 8;
    transaction.user      = NULL;
    transaction.tx_buffer = NULL;
    transaction.rx_buffer = data;

    esp_err_t err = spi_device_transmit(handle, &transaction);
    return err;
}