/**
 * @file spi_port.c
 * @author by mondraker (https://oshwhub.com/mondraker)(https://github.com/HwzLoveDz)
 * @brief spi port,easy component migration
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "spi_port.h"

/**
 * @brief spi master initialization
 */

esp_err_t spi_master_init()
{
    esp_err_t ret;
    // spi_device_handle_t spi;
    // ESP_LOGI(TAG, "Initializing bus SPI%d...", SPI_HOST+1);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI, // MOSI信号线
        .miso_io_num = PIN_NUM_MISO, // MISO信号线
        .sclk_io_num = PIN_NUM_CLK,  // SCLK信号线
        .quadwp_io_num = -1,         // WP信号线，专用于QSPI的D2
        .quadhd_io_num = -1,         // HD信号线，专用于QSPI的D3
        .max_transfer_sz = 64 * 8,   // 最大传输数据大小
    };

    spi_device_interface_config_t devcfg = {
        .mode = 0,                             // SPI mode 0
        .clock_speed_hz = SPI_MASTER_FREQ_10M, // Clock out at 10 MHz,
        /*
         * The timing requirements to read the busy signal from the EEPROM cannot be easily emulated
         * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
         */
        .spics_io_num = -1,
        .queue_size = 7, // 传输队列大小，决定了等待传输数据的数量
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI_HOST, &buscfg, DMA_CHAN);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi_dev);
    if (ret != ESP_OK)
    {
        // Failed to add device, deinitialize the bus again
        spi_bus_free(SPI_HOST);
        return ret;
    }
    
    // gpio_pad_select_gpio(PIN_NUM_CS);                // 选择一个GPIO
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT); // 把这个GPIO作为输出

    return ESP_OK;
}

uint32_t spi_write(spi_device_handle_t spi, uint8_t *data, uint8_t len)
{
    esp_err_t ret;
    spi_transaction_t t;
    // if (len == 0)
    //     return;               // no need to send anything
    memset(&t, 0, sizeof(t)); // Zero out the transaction

    gpio_set_level(PIN_NUM_CS, 0);

    t.length = len * 8;                         // Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;                         // Data
    t.user = (void *)1;                         // D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.

    gpio_set_level(PIN_NUM_CS, 1);
    return ret;
}

uint32_t spi_read(spi_device_handle_t spi, uint8_t *data, uint8_t len)
{
    spi_transaction_t t;

    gpio_set_level(PIN_NUM_CS, 0);

    memset(&t, 0, sizeof(t));
    t.length=len * 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );
    *data = t.rx_data[0];

    gpio_set_level(PIN_NUM_CS, 1);

    return ret;
}
// uint32_t spi_read(spi_device_handle_t spi, uint8_t *data, uint8_t len)
// {
//     spi_transaction_t t;
//     esp_err_t ret = ESP_OK;

//     gpio_set_level(PIN_NUM_CS, 0);

//     for (uint8_t i = 0; i < len; i++)
//     {
//         memset(&t, 0, sizeof(t));
//         t.length = 8;
//         t.flags = SPI_TRANS_USE_RXDATA;
//         t.user = (void *)1;

//         ret = spi_device_polling_transmit(spi, &t);

//         assert(ret == ESP_OK);

//         data[i] = t.rx_data[0];
//     }

//     gpio_set_level(PIN_NUM_CS, 1);

//     return ret;
// }

spi_device_handle_t spi_dev;
