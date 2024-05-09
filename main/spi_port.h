/**
 * @file spi_port.h
 * @author by mondraker (https://oshwhub.com/mondraker)(https://github.com/HwzLoveDz)
 * @brief 
 * @version 0.1
 * @date 2023-07-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"

#define SPI_HOST        SPI3_HOST
#define DMA_CHAN        SPI_DMA_CH_AUTO
#define PIN_NUM_MISO    GPIO_NUM_5
#define PIN_NUM_MOSI    GPIO_NUM_4
#define PIN_NUM_CLK     GPIO_NUM_6
#define PIN_NUM_CS      GPIO_NUM_7

esp_err_t spi_master_init(void);
uint32_t spi_write(spi_device_handle_t spi, uint8_t *data, uint8_t len);   //! 类型错误：uint16_t
uint32_t spi_read(spi_device_handle_t spi, uint8_t *data, uint8_t len);  //! 类型错误：uint16_t

extern spi_device_handle_t spi_dev;

#ifdef __cplusplus
}
#endif
