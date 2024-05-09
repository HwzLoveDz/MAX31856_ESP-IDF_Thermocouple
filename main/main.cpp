// This example demonstrates doing a one-shot measurement "manually".
// Separate calls are made to trigger the conversion and then check
// for conversion complete. While this typically only takes a couple
// 100 milliseconds, that times is made available by separating these
// two steps.

#include <stdio.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"

#include "Adafruit_MAX31856.h"
#include "spi_port.h"

static const char *TAG = "mian";

// spi_device_handle_t handle;

Adafruit_MAX31856 maxthermo;
// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
// Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10);
// use hardware SPI, pass in the CS pin and using SPI1
// Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, &SPI1);

// extern esp_err_t pmu_init();
// extern esp_err_t spi_init(void);
static void temp_task(void *args);
// esp_err_t spi_init();
// extern spi_device_handle_t spi_dev;

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(spi_master_init());
    ESP_LOGI(TAG, "SPI initialized successfully");

    // ... configure the bus and device as needed ...
    // while (1)
    // {
    //     vTaskDelay(1000); // replace this with whatever
    // }

    ESP_ERROR_CHECK(maxthermo.begin(spi_read, spi_write, spi_dev));

    maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);

    maxthermo.setConversionMode(MAX31856_ONESHOT_NOWAIT);

    xTaskCreate(temp_task, "App/temp", 4 * 1024, NULL, 10, NULL);
}

static void temp_task(void *args)
{
    while (1)
    {
        // trigger a conversion, returns immediately
        maxthermo.triggerOneShot();

        //
        // here's where you can do other things
        //
        vTaskDelay(100); // replace this with whatever
        //
        //

        // check for conversion complete and read temperature
        if (maxthermo.conversionComplete())
        {
            ESP_LOGI(TAG, "Temp:%.2f", maxthermo.readCJTemperature());
        }
        else
        {
            ESP_LOGI(TAG, "Conversion not complete!");
        }
    }
}

// esp_err_t spi_init()
// {
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = mosi,
//         .miso_io_num = miso,
//         .sclk_io_num = sclk,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 0,
//         .flags = 0,
//         .intr_flags = 0};

//     // Initialize the SPI bus
//     esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
//     if (ret != ESP_OK)
//     {
//         return ret;
//     }

//     spi_device_interface_config_t devcfg = {
//         .command_bits = 0,
//         .address_bits = 0,
//         .dummy_bits = 0,
//         .mode = 0,
//         .duty_cycle_pos = 128, // 50% duty cycle
//         .cs_ena_pretrans = 0,
//         .cs_ena_posttrans = 0,
//         .clock_speed_hz = 10 * 1000 * 1000, // Clock out at 1 MHz
//         .input_delay_ns = 0,
//         .spics_io_num = cs,
//         .flags = 0,
//         .queue_size = 7,
//         .pre_cb = NULL,
//         .post_cb = NULL,
//     };

//     // Attach the device to the SPI bus
//     // spi_device_handle_t spi_dev;
//     ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_dev);
//     if (ret != ESP_OK)
//     {
//         // Failed to add device, deinitialize the bus again
//         spi_bus_free(SPI3_HOST);
//         return ret;
//     }

//     return ESP_OK;
// }
