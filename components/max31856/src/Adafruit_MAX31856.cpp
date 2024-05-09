/*!
 * @file Adafruit_MAX31856.cpp
 *
 * @mainpage Adafruit MAX31856 thermocouple reader
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's MAX31856 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit MAX31856 breakout: https://www.adafruit.com/products/3263
 *
 * These sensors use SPI to communicate, 4 pins are required to
 *  interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit_Sensor"> Adafruit_Sensor</a> being
 * present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Written by ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_MAX31856.h"

Adafruit_MAX31856::Adafruit_MAX31856() {}

// Adafruit_MAX31856::Adafruit_MAX31856(gpio_num_t spi_cs, gpio_num_t spi_clk, gpio_num_t spi_mosi, gpio_num_t spi_miso)
//     {
//         // gpio_pad_select_gpio(cs);
//         gpio_set_direction(spi_cs, GPIO_MODE_OUTPUT);
//     }

/**************************************************************************/
/*!
    @brief  Instantiate MAX31856 object and use software SPI pins
    @param  spi_cs Bitbang SPI Chip Select
    @param  spi_mosi Bitbang SPI MOSI
    @param  spi_miso Bitbang SPI MISO
    @param  spi_clk Bitbang SPI Clock
*/
/**************************************************************************/
// Adafruit_MAX31856::Adafruit_MAX31856(int8_t spi_cs, int8_t spi_mosi,
//                                      int8_t spi_miso, int8_t spi_clk)
//     : spi_dev(spi_cs, spi_clk, spi_miso, spi_mosi, 1000000,
//               SPI_BITORDER_MSBFIRST, SPI_MODE1) {}

/**************************************************************************/
/*!
    @brief  Instantiate MAX31856 object and use hardware SPI
    @param  spi_cs Any pin for SPI Chip Select
    @param _spi which spi buss to use.
*/
/**************************************************************************/
// Adafruit_MAX31856::Adafruit_MAX31856(int8_t spi_cs, SPIClass *_spi)
//     : spi_dev(spi_cs, 1000000, SPI_BITORDER_MSBFIRST, SPI_MODE1, _spi) {}

/**************************************************************************/
/*!
    @brief  Initialize MAX31856 attach/set pins or SPI device, default to K
   thermocouple
    @returns Always returns true at this time (no known way of detecting chip
   ID)
*/
/**************************************************************************/
bool Adafruit_MAX31856::begin(max_com_fptr_t read_cb, max_com_fptr_t write_cb, spi_device_handle_t spi)
{
  // initialized = spi_dev.begin();

  // if (!initialized)
  // return false;
  if (read_cb == nullptr || write_cb == nullptr)
    return -1;
  _read_cb = read_cb;
  _write_cb = write_cb;
  _spi = spi;

  // assert on any fault
  writeRegister8(MAX31856_MASK_REG, 0x0);

  // enable open circuit fault detection
  writeRegister8(MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0);

  // set cold junction temperature offset to zero
  writeRegister8(MAX31856_CJTO_REG, 0x0);

  // set Type K by default
  setThermocoupleType(MAX31856_TCTYPE_K);

  // set One-Shot conversion mode
  setConversionMode(MAX31856_ONESHOT);

  return ESP_OK;
}

/**************************************************************************/
/*!
    @brief  Set temperature conversion mode
    @param mode The conversion mode
*/
/**************************************************************************/
void Adafruit_MAX31856::setConversionMode(max31856_conversion_mode_t mode)
{
  conversionMode = mode;
  uint8_t t = readRegister8(MAX31856_CR0_REG); // get current register value
  if (conversionMode == MAX31856_CONTINUOUS)
  {
    t |= MAX31856_CR0_AUTOCONVERT; // turn on automatic
    t &= ~MAX31856_CR0_1SHOT;      // turn off one-shot
  }
  else
  {
    t &= ~MAX31856_CR0_AUTOCONVERT; // turn off automatic
    t |= MAX31856_CR0_1SHOT;        // turn on one-shot
  }
  writeRegister8(MAX31856_CR0_REG, t); // write value back to register
}

/**************************************************************************/
/*!
    @brief  Get temperature conversion mode
    @returns The conversion mode
*/
/**************************************************************************/
max31856_conversion_mode_t Adafruit_MAX31856::getConversionMode(void)
{
  return conversionMode;
}

/**************************************************************************/
/*!
    @brief  Set which kind of Thermocouple (K, J, T, etc) to detect & decode
    @param type The enumeration type of the thermocouple
*/
/**************************************************************************/
void Adafruit_MAX31856::setThermocoupleType(max31856_thermocoupletype_t type)
{
  uint8_t t = readRegister8(MAX31856_CR1_REG);
  t &= 0xF0; // mask off bottom 4 bits
  t |= (uint8_t)type & 0x0F;
  writeRegister8(MAX31856_CR1_REG, t);
}

/**************************************************************************/
/*!
    @brief  Get which kind of Thermocouple (K, J, T, etc) we are using
    @returns The enumeration type of the thermocouple
*/
/**************************************************************************/
max31856_thermocoupletype_t Adafruit_MAX31856::getThermocoupleType(void)
{
  uint8_t t = readRegister8(MAX31856_CR1_REG);
  t &= 0x0F;

  return (max31856_thermocoupletype_t)(t);
}

/**************************************************************************/
/*!
    @brief  Read the fault register (8 bits)
    @returns 8 bits of fault register data
*/
/**************************************************************************/
uint8_t Adafruit_MAX31856::readFault(void)
{
  return readRegister8(MAX31856_SR_REG);
}

/**************************************************************************/
/*!
    @brief  Sets the threshhold for internal chip temperature range
    for fault detection. NOT the thermocouple temperature range!
    @param  low Low (min) temperature, signed 8 bit so -128 to 127 degrees C
    @param  high High (max) temperature, signed 8 bit so -128 to 127 degrees C
*/
/**************************************************************************/
void Adafruit_MAX31856::setColdJunctionFaultThreshholds(int8_t low,
                                                        int8_t high)
{
  writeRegister8(MAX31856_CJLF_REG, low);
  writeRegister8(MAX31856_CJHF_REG, high);
}

/**************************************************************************/
/*!
    @brief  Sets the mains noise filter. Can be set to 50 or 60hz.
    Defaults to 60hz. You need to call this if you live in a 50hz country.
    @param  noiseFilter One of MAX31856_NOISE_FILTER_50HZ or
   MAX31856_NOISE_FILTER_60HZ
*/
/**************************************************************************/
void Adafruit_MAX31856::setNoiseFilter(max31856_noise_filter_t noiseFilter)
{
  uint8_t t = readRegister8(MAX31856_CR0_REG);
  if (noiseFilter == MAX31856_NOISE_FILTER_50HZ)
  {
    t |= 0x01;
  }
  else
  {
    t &= 0xfe;
  }
  writeRegister8(MAX31856_CR0_REG, t);
}

/**************************************************************************/
/*!
    @brief  Sets the threshhold for thermocouple temperature range
    for fault detection. NOT the internal chip temperature range!
    @param  flow Low (min) temperature, floating point
    @param  fhigh High (max) temperature, floating point
*/
/**************************************************************************/
void Adafruit_MAX31856::setTempFaultThreshholds(float flow, float fhigh)
{
  int16_t low, high;

  flow *= 16;
  low = flow;

  fhigh *= 16;
  high = fhigh;

  writeRegister8(MAX31856_LTHFTH_REG, high >> 8);
  writeRegister8(MAX31856_LTHFTL_REG, high);

  writeRegister8(MAX31856_LTLFTH_REG, low >> 8);
  writeRegister8(MAX31856_LTLFTL_REG, low);
}

/**************************************************************************/
/*!
    @brief  Begin a one-shot (read temperature only upon request) measurement.
    Value must be read later, not returned here!
*/
/**************************************************************************/
void Adafruit_MAX31856::triggerOneShot(void)
{

  if (conversionMode == MAX31856_CONTINUOUS)
    return;

  uint8_t t = readRegister8(MAX31856_CR0_REG); // get current register value
  t &= ~MAX31856_CR0_AUTOCONVERT;              // turn off autoconvert
  t |= MAX31856_CR0_1SHOT;                     // turn on one-shot
  writeRegister8(MAX31856_CR0_REG, t);         // write value back to register
                                               // conversion starts when CS goes high
}

/**************************************************************************/
/*!
    @brief  Return status of temperature conversion.
    @returns true if conversion complete, otherwise false
*/
/**************************************************************************/
bool Adafruit_MAX31856::conversionComplete(void)
{

  if (conversionMode == MAX31856_CONTINUOUS)
    return true;
  return !(readRegister8(MAX31856_CR0_REG) & MAX31856_CR0_1SHOT);
}

/**************************************************************************/
/*!
    @brief  Return cold-junction (internal chip) temperature
    @returns Floating point temperature in Celsius
*/
/**************************************************************************/
float Adafruit_MAX31856::readCJTemperature(void)
{

  return readRegister16(MAX31856_CJTH_REG) / 256.0;
}

/**************************************************************************/
/*!
    @brief  Return hot-junction (thermocouple) temperature
    @returns Floating point temperature in Celsius
*/
/**************************************************************************/
float Adafruit_MAX31856::readThermocoupleTemperature(void)
{

  // for one-shot, make it happen
  // if (conversionMode == MAX31856_ONESHOT) {
  //   triggerOneShot();
  //   uint32_t start = millis();
  //   while (!conversionComplete()) {
  //     if (millis() - start > 250)
  //       return NAN;
  //     delay(10);
  //   }
  // }
  if (conversionMode == MAX31856_ONESHOT)
  {
    triggerOneShot();
    uint64_t start = esp_timer_get_time();
    while (!conversionComplete())
    {
      if ((esp_timer_get_time() - start) / 1000 > 250) // convert us to ms
        return 0;
      vTaskDelay(10); // delay for 10ms
    }
  }

  // read the thermocouple temperature registers (3 bytes)
  int32_t temp24 = readRegister24(MAX31856_LTCBH_REG);
  // and compute temperature
  if (temp24 & 0x800000)
  {
    temp24 |= 0xFF000000; // fix sign
  }

  temp24 >>= 5; // bottom 5 bits are unused

  return temp24 * 0.0078125;
}

/**********************************************/

uint8_t Adafruit_MAX31856::readRegister8(uint8_t addr)
{
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t Adafruit_MAX31856::readRegister16(uint8_t addr)
{
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

uint32_t Adafruit_MAX31856::readRegister24(uint8_t addr)
{
  uint8_t buffer[3] = {0, 0, 0};
  readRegisterN(addr, buffer, 3);

  uint32_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];
  ret <<= 8;
  ret |= buffer[2];

  return ret;
}


// void Adafruit_MAX31856::readRegisterN(uint8_t addr, uint8_t buffer[],
//                                       uint8_t n) {
//   addr &= 0x7F; // MSB=0 for read, make sure top bit is not set

//   spi_dev.write_then_read(&addr, 1, buffer, n);
// }
void Adafruit_MAX31856::readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n)
{
  uint8_t addr_byte = addr & 0x7F; // MSB=0 for read

  // Write the address
  _write_cb(_spi, &addr_byte, 1);

  // Now read the requested bytes
  _read_cb(_spi, buffer, n);
}

// void Adafruit_MAX31856::writeRegister8(uint8_t addr, uint8_t data) {
//   addr |= 0x80; // MSB=1 for write, make sure top bit is set

//   uint8_t buffer[2] = {addr, data};

//   spi_dev.write(buffer, 2);
// }
void Adafruit_MAX31856::writeRegister8(uint8_t addr, uint8_t data)
{
  uint8_t buffer[2] = {static_cast<uint8_t>(addr | 0x80), data}; // MSB=1 for write

  // Remember to handle errors in real application!
  _write_cb(_spi, buffer, 2);
}
