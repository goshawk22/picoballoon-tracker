/**
 *  BME280 Combined humidity and pressure sensor library
 *
 *  @author  Toyomasa Watarai
 *  @version 1.0
 *  @date    06-April-2015
 *
 *  Library for "BME280 temperature, humidity and pressure sensor module" from Switch Science
 *    https://www.switch-science.com/catalog/2236/
 *
 *  For more information about the BME280:
 *    http://ae-bst.resource.bosch.com/media/products/dokumente/bme280/BST-BME280_DS001-10.pdf
 */
 
#pragma once

#include "mbed.h"

// default address with SDO High 0x77
// address with SDO LOW 0x76
#define DEFAULT_SLAVE_ADDRESS (0x77)

/** Initialize a BME280 sensor
 *
 *  Configure sensor setting and read parameters for calibration
 *
 */
void bmp280_initialize(void);

/** Read the current temperature value (degree Celsius) from BME280 sensor
 *
 */
float bmp280_getTemperature(void);

/** Read the current pressure value (hectopascal)from BME280 sensor
 *
 */
float bmp280_getPressure(void);

