/*
 * bme280.c
 *
 *  Created on: Sep 27, 2026
 *      Author: haizon
 */

#include "bme280.h"
#include "i2c1_driver.h"

// BME280 Register map, from datasheet section 5.3
#define BME280_REG_CHIP_ID 0xD0U	// "id" register
#define BME280_REG_RESET 0xE0U	// "reset" register
/**
 * "ctrl_hum" register-Humidity oversampling setting
 * Changes to this register will only become effective after a write operation to "ctrl_meas"
 */
#define BME280_REG_CTRL_HUM 0xF2U
/*
 * "status" register - Has 2 bits
 *
 * Bit 3- named measuring[0] Automatically set to '1' when a conversion is running
 * 							 Back to '0' when the results have been transferred to data registers
 *
 * Bit 0- name im_update[0] Automatically set to '1' when the NVM data are being copied to image registers
 * 							Back to '0' when copying is done
 *
 * 							The data is copied at power-on-reset and before every conversion
 */
#define BME280_REG_STATUS 0xF3U

/**
 * "ctrl_meas" register-Sets the pressure and data acquisition options of the device.
 *  Write only after changing "ctrl_hum" for changes to become effective.
 *
 * 8 bit register, bits 7,6,5 handle temperature, bits 4,3,2 handles pressure, bits 1,0 controls
 * sensor mode of the device
 *
 * mode[1:0], 00 for sleep mode, 01 and 10 for forced mode, 11 for normal mode
 * Temperature oversampling
 * osrs_t[2:0], 000 is for skipped oversampling,
 * 				001 is oversampling x 1
 * 				010 is oversampling x 2
 * 				011 is oversampling x 4
 * 				100 is oversampling x 8
 * 				101, others is oversampling x 16
 *
 * Pressure oversampling osrs_t[2:0], same value set as temperature oversampling
 */
#define BME280_REG_CTRL_MEAS 0xF4U

/**
 * "config" register-Sets the rate, filter and interface options of the device.
 * Writes to this register is ignored in normal mode, but not in sleep mode.
 *
 * Bit 7,6,5-t_sb[2:0]- Inactive duration in normal mode
 * 		000 - 0.5 ms
 * 		001 - 62.5 ms
 * 		010 to 101 keeps doubling from previous value
 * 		110 - 10 ms
 * 		111 - 20 ms
 * Bit 4,3,2-filter[2:0]- Controls the time constant of the IIR filter.
 * 		000 - filter off
 * 		001 - 2
 * 		010 - 4
 * 		011 - 8
 * 		100, others - 16
 * Bit 0 spi3w_en[0]- Enables 3-wire SPI interface when set to '1'
 */
#define BME280_REG_CONFIG 0xF5U

// Contains MSB part up[19:12] of the raw pressure measurement output data
#define BME280_REG_PRESS_MSB 0xF7U

// Contains MSB part up[19:12] of the raw temperature measurement output data
#define BME280_REG_TEMP_MSB 0xFAU

// Contains MSB part up[15:8] of the raw humidity measurement output data
#define BME280_REG_HUM_MSB 0xFD

#define BME280_REG_CALIB00 0x88U
#define BME280_REG_CALIB26 0xE1U
#define BME280_REG_DIG_H1  0xA1U

#define BME280_CHIP_ID_READ_VALUE 0x60U
static BME280_CalibData s_calib;
static int32_t st_fine;

static uint16_t le16_u(const uint8_t *p) {
	return (uint16_t) (p[0] | (p[1] << 8));
}

static int16_t le16_s(const uint8_t *p) {
	return (int16_t) (p[0] | (p[1] << 8));
}

uint8_t bme280Init(void) {
	uint8_t chip_id = 0;
	uint8_t calib_a[26];
	uint8_t calib_b[7];
	uint8_t h1;

	// Sanity check for communication with BME280
	if (i2c1ReadReg(BME280_I2C_ADDR, BME280_REG_CHIP_ID, &chip_id) != I2C1_OK)
		return 0;

	// Wrong address check
	if(chip_id != BME280_CHIP_ID_READ_VALUE)
		return 0;

	// Following data is programmed by Bosch and is unique per chip.
	// Compensation formulas are meaningless with it
	if(i2c1ReadBytes(BME280_I2C_ADDR, BME280_REG_CALIB00, calib_a, sizeof(calib_a)) != I2C1_OK)
		return 0;

	if(i2c1ReadReg(BME280_I2C_ADDR, BME280_REG_DIG_H1, &h1) != I2C1_OK)
		return 0;

	if(i2c1ReadBytes(BME280_I2C_ADDR, BME280_REG_CALIB26, calib_b, sizeof(calib_b)) != I2C1_OK)
		return 0;

	s_calib.dig_T1 = le16_u(&calib_a[0]);
	s_calib.dig_T1 = le16_s(&calib_a[2]);
	s_calib.dig_T1 = le16_s(&calib_a[4]);

	s_calib.dig_P1 = le16_u(&calib_a[6]);
	s_calib.dig_P2 = le16_s(&calib_a[8]);
	s_calib.dig_P3 = le16_s(&calib_a[10]);
	s_calib.dig_P4 = le16_s(&calib_a[12]);
	s_calib.dig_P5 = le16_s(&calib_a[14]);
	s_calib.dig_P6 = le16_s(&calib_a[16]);
	s_calib.dig_P7 = le16_s(&calib_a[18]);
	s_calib.dig_P8 = le16_s(&calib_a[20]);
	s_calib.dig_P9 = le16_s(&calib_a[22]);

	s_calib.dig_H1 = h1;
	s_calib.dig_H2 = le16_s(&calib_b[0]);
	s_calib.dig_H3 = calib_b[2];
	s_calib.dig_H4 = (int16_t)(((int16_t)(int8_t)calib_b[3] << 4) | (calib_b[4] & 0x0F));
	s_calib.dig_H5 = (int16_t)(((int16_t)(int8_t)calib_b[5] << 4) | (calib_b[4] >> 4));
	s_calib.dig_H6 = (int8_t)calib_b[6];

	if(i2c1WriteReg(BME280_I2C_ADDR, BME280_REG_CTRL_HUM, 0x01U) != I2C1_OK)
		return 0;

	if(i2c1WriteReg(BME280_I2C_ADDR, BME280_REG_CONFIG, 0x00U) != I2C1_OK)
		return 0;

	if(i2c1WriteReg(BME280_I2C_ADDR, BME280_REG_CTRL_MEAS, 0x27U) != I2C1_OK)
		return 0;
	return 1;
}

static int32_t compensateTemp(int32_t adc_t) {
	int32_t var1, var2, t;
	var1 = ((((adc_t >> 3) - ((int32_t) s_calib.dig_T1 << 1))) * ((int32_t) s_calib.dig_T2))
			>> 11;
	var2 = (((((adc_t >> 4) - ((int32_t) s_calib.dig_T1))
			* ((adc_t >> 4) - ((int32_t) s_calib.dig_T1))) >> 12)
			* ((int32_t) s_calib.dig_T3)) >> 14;

	st_fine = var1 + var2;
	t = (st_fine * 5 + 128) >> 8;
	return t;
}

static uint32_t compensatePres(int32_t adc_p) {
	int64_t var1, var2, p;
	var1 = ((int64_t) st_fine) - 128000;
	var2 = var1 * var1 * (int64_t) s_calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t) s_calib.dig_P5) << 17);
	var2 = var2 + (((int64_t) s_calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) s_calib.dig_P3) >> 8)
			+ ((var1 * (int64_t) s_calib.dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) s_calib.dig_P1) >> 33;
	if (var1 == 0) {
		return 0;
	}
	p = 1048576 - adc_p;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) s_calib.dig_P9) * (p >> 13) * (p >> 13) >> 25);
	var2 = (((int64_t) s_calib.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t) s_calib.dig_P7) << 4);
	return (uint32_t) p;
}

static uint32_t compensateHumd(int32_t adc_h) {
	int32_t v_x1;

	v_x1 = (st_fine - ((int32_t) 76800));
	v_x1 = (((((adc_h << 14) - (((int32_t) s_calib.dig_H4) << 20)
			- (((int32_t) s_calib.dig_H5) * v_x1)) + ((int32_t) 16384)) >> 15)
			* (((((((v_x1 * ((int32_t) s_calib.dig_H6)) >> 10)
					* (((v_x1 * ((int32_t) s_calib.dig_H3)) >> 11)
							+ ((int32_t) 32768))) >> 10) + ((int32_t) 2097152))
					* ((int32_t) s_calib.dig_H2) + 8192) >> 14));

	v_x1 = (v_x1
			- (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7)
					* ((int32_t) s_calib.dig_H1)) >> 4));

	v_x1 = (v_x1 < 0) ? 0 : v_x1;
	v_x1 = (v_x1 > 419430400) ? 419430400 : v_x1;

	return (uint32_t) (v_x1 >> 12);
}

uint8_t bme280ReadMeasurements(BME280_Data *out) {
	uint8_t raw[8];
	int32_t adc_P, adc_T, adc_H;

	if (i2c1ReadBytes(BME280_I2C_ADDR, BME280_REG_PRESS_MSB, raw, sizeof(raw))
			!= I2C1_OK)
		return 0;

	adc_P = ((int32_t) raw[0] << 12) | ((int32_t) raw[1] << 4)
			| ((int32_t) raw[2] >> 4);
	adc_T = ((int32_t) raw[3] << 12) | ((int32_t) raw[4] << 4)
			| ((int32_t) raw[5] >> 4);

	// Humidity is only 16 bits, msb/lsb/no xlsb
	adc_H = ((int32_t) raw[6] << 8) | (int32_t) raw[7];

	out->temperature_centiC = compensateTemp(adc_T);
	out->pressure_Pa_q24_8 = compensatePres(adc_P);
	out->humidity_q22_10 = compensateHumd(adc_H);

	return 1;
}

