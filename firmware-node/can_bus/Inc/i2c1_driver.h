/*
 * i2c1_driver.h
 *
 *  Created on: Sep 27, 2026
 *      Author: haizon
 */

#ifndef I2C1_DRIVER_H_
#define I2C1_DRIVER_H_

#include <stdint.h>
#include "stm32g070xx.h"

typedef enum
{
	I2C1_OK = 0,
	I2C1_ERR_NACK,
	I2C1_ERR_TIMEOUT
}I2C1_Status;

void i2c1Init();
I2C1_Status i2c1WriteReg(uint8_t devAAddr, uint8_t regAddr, uint8_t data);
I2C1_Status i2c1ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len);
I2C1_Status i2c1ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t *data);

#endif /* I2C1_DRIVER_H_ */
