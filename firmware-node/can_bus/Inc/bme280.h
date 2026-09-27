/*
 * bme280.h
 *
 *  Created on: Sep 27, 2026
 *      Author: haizon
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>

#define BME280_I2C_ADDR 0x76U

typedef struct
{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
}BME280_CalibData;

typedef struct
{
	int32_t temperature_centiC;
	uint32_t pressure_Pa_q24_8;
	uint32_t humidity_q22_10;
}BME280_Data;

uint8_t bme280Init(void);
uint8_t bme280ReadMeasurements(BME280_Data *out);

#endif /* BME280_H_ */
