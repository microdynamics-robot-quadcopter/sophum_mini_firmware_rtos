#include "sophum_driver_store.h"

static const char *TAG = "SOPHUM_DRIVER_STORE";

void STORE_Init(void)
{
#ifndef USE_SPI_MODE
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    /* To use 1-line SD mode, uncomment the following line: */
    // host.flags = SDMMC_HOST_FLAG_1BIT;

    /* This initializes the slot without card detect (CD) and write protect (WP) signals. */
    /* Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals. */
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    /* GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups. */
    /* Internal pull-ups are not sufficient. However, enabling internal pull-ups */
    /* does make a difference some boards, so we do that here. */
    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   /* CMD, needed in 4- and 1- line modes */
    gpio_set_pull_mode(2,  GPIO_PULLUP_ONLY);    /* D0, needed in 4- and 1-line modes */
    gpio_set_pull_mode(4,  GPIO_PULLUP_ONLY);    /* D1, needed in 4-line mode only */
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   /* D2, needed in 4-line mode only */
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   /* D3, needed in 4- and 1-line modes */

#else
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = STORE_MISO_PIN;
    slot_config.gpio_mosi = STORE_MOSI_PIN;
    slot_config.gpio_sck  = STORE_CLK_PIN;
    slot_config.gpio_cs   = STORE_CS_PIN;
    /* This initializes the slot without card detect (CD) and write protect (WP) signals. */
    /* Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals. */
#endif /* USE_SPI_MODE */

    /* Options for mounting the filesystem(FATFS). */
    /* If format_if_mount_failed is set to true, SD card will be partitioned and */
    /* formatted in case when mounting fails. */
    esp_vfs_fat_sdmmc_mount_config_t mount_config =
    {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    /* Use settings defined above to initialize SD card and mount FAT filesystem. */
    /* Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function. */
    /* Please check its source code and implement error recovery when developing */
    /* production applications. */
    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if(ret != ESP_OK)
    {
        if(ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    /* Card has been initialized, print its properties */
    sdmmc_card_print_info(stdout, card);
}

void STORE_writeData(void)
{
    /* Use POSIX and C standard library functions to work with files. */
    /* First create a file. */
    ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen("/sdcard/hello.txt", "w");
    if(f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "Hello %s!\n", "card->cid.name");
    fclose(f);
    ESP_LOGI(TAG, "File written");
}

void STORE_readData(void)
{

}