#ifndef __SOPHUM_DRIVER_STORE_H__
#define __SOPHUM_DRIVER_STORE_H__

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#define USE_SPI_MODE

/* When testing SD and SPI modes, keep in mind that once the card has been */
/* initialized in SPI mode, it can not be reinitialized in SD mode without */
/* toggling power to the card. */

#ifdef USE_SPI_MODE
/* Pin mapping when using SPI mode. */
/* With this mapping, SD card can be used both in SPI and 1-line SD mode. */
/* Note that a pull-up on CS line is required in SD mode. */
#define STORE_MISO_PIN 2
#define STORE_MOSI_PIN 15
#define STORE_CLK_PIN  14
#define STORE_CS_PIN   13

#endif /* USE_SPI_MODE */

extern void STORE_Init(void);
extern void STORE_writeData(void);
extern void STORE_readData(void);

#endif