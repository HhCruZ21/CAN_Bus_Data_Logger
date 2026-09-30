/*
 * i2c1_driver.c
 *
 *  Created on: Sep 27, 2026
 *      Author: haizon
 */


#include "i2c1_driver.h"

static int waitForFlag(volatile uint32_t *reg, uint32_t mask, uint32_t timeout)
{
	while(((*reg) & mask) == 0)
	{
		if(--timeout == 0)
			return 0;
	}
	return 1;
}

#define I2C1_TIMEOUT_LOOPS 100000UL
#define I2C1_TIMING_100KHZ_16MHZ	0x00303D5BUL
void i2c1Init()
{
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;	//GPIOB clock enable
	RCC->APBENR1 |= RCC_APBENR1_I2C1EN;	//I2C1 clock enable

	//Configure PB6/PB7 as I2C1 alternate function pins
	//MODER is 2 bits per pin. Alternate function value "10"
	//Pin 6 is bits [13:12], pin 7 field is bits [15:14]
	GPIOB->MODER &= ~((3UL << (6*2)) | (3UL << (7*2)));	//reset pins first
	GPIOB->MODER |= ((2UL << (6*2)) | (2UL << (7*2)));	//Alternate function mode for I2C peripheral

	//I2C requires GPIOB to be open drain
	GPIOB->OTYPER |= (1UL << 6) | (1UL << 7);

	//High speed
	GPIOB->OSPEEDR &= ~((3UL << (6*2)) | (3UL << (7*2)));
	GPIOB->OSPEEDR |= (1UL << (6*2)) | (1UL << (7*2));		// standard mode I2C bus speed - medium speed

	//Pull-up resistor activated for I2C
	GPIOB->PUPDR &= ~((3UL << (6*2)) | (3UL << (7*2)));
	GPIOB->PUPDR |= ((1UL << (6*2)) | (1UL << (7*2)));

	//AFR[0] is AFRL for PB pins 0 to 7
	GPIOB->AFR[0] &= ~((0xFUL << (6 * 4)) | (0xFUL << (7 * 4)));
	GPIOB->AFR[0] |=  ((6UL   << (6 * 4)) | (6UL   << (7 * 4)));

	//Configure
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->TIMINGR = I2C1_TIMING_100KHZ_16MHZ;
	I2C1->CR1 |= I2C_CR1_PE;
}

static void i2c1StartTransfer(uint8_t devAddr, uint8_t nbytes, uint8_t isRead, uint8_t autoend)
{
	uint32_t cr2 = 0;
	cr2 |= ((uint32_t)devAddr << 1) << I2C_CR2_SADD_Pos;
	cr2 |= ((uint32_t)nbytes) << I2C_CR2_NBYTES_Pos;
	if(isRead)
		cr2 |= I2C_CR2_RD_WRN;
	if(autoend)
		cr2 |= I2C_CR2_AUTOEND;
	cr2 |= I2C_CR2_START;
	I2C1->CR2 = cr2;
}

I2C1_Status i2c1WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    /* Make sure the bus is idle before we start a new transaction. */
    if (!waitForFlag(&I2C1->ISR, I2C_ISR_BUSY, I2C1_TIMEOUT_LOOPS) &&
        (I2C1->ISR & I2C_ISR_BUSY))
    {
        /* wait_for_flag() above is (ab)used in "wait until BUSY clears"
         * sense below instead; see note in ReadBytes(). Kept simple here:
         * if bus is still busy after our timeout budget, bail out. */
    }
    while (I2C1->ISR & I2C_ISR_BUSY) { /* spin until idle; add a timeout in production code */ }

    /* Tell the peripheral: talk to devAddr, we will WRITE 2 bytes total,
     * and automatically STOP once they're both sent. This line is what
     * actually generates the START condition + address byte on the bus. */
    i2c1StartTransfer(devAddr, 2, 0 /*write*/, 1 /*autoend*/);

    /* --- Byte 1: the register address we want to write to --- */
    if (!waitForFlag(&I2C1->ISR, I2C_ISR_TXIS | I2C_ISR_NACKF, I2C1_TIMEOUT_LOOPS))
    {
        return I2C1_ERR_TIMEOUT;
    }
    if (I2C1->ISR & I2C_ISR_NACKF)
    {
        I2C1->ICR = I2C_ICR_NACKCF; /* clear the NACK flag */
        return I2C1_ERR_NACK;      /* device didn't answer to its address */
    }
    I2C1->TXDR = regAddr;

	if(!waitForFlag(&I2C1->ISR, I2C_ISR_TXIS | I2C_ISR_NACKF, I2C1_TIMEOUT_LOOPS))
		return I2C1_ERR_TIMEOUT;
	if(I2C1->ISR & I2C_ISR_NACKF)
	{
		I2C1->ICR = I2C_ICR_NACKCF;
		return I2C1_ERR_NACK;
	}
	I2C1->TXDR = data;

	if(!waitForFlag(&I2C1->ISR, I2C_ISR_STOPF, I2C1_TIMEOUT_LOOPS))
		return I2C1_ERR_TIMEOUT;

    I2C1->ICR = I2C_ICR_STOPCF; /* clear the sticky STOPF flag for next time */

    return I2C1_OK;
}

I2C1_Status i2c1ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    while (I2C1->ISR & I2C_ISR_BUSY) { /* wait for idle bus; add timeout in production code */ }

    /* --- Phase A: send the register address we want to start reading from --- */
    i2c1StartTransfer(devAddr, 1, 0 /*write*/, 0 /*no autoend -> we want TC, not STOP*/);

    if (!waitForFlag(&I2C1->ISR, I2C_ISR_TXIS | I2C_ISR_NACKF, I2C1_TIMEOUT_LOOPS))
    {
        return I2C1_ERR_TIMEOUT;
    }
    if (I2C1->ISR & I2C_ISR_NACKF)
    {
        I2C1->ICR = I2C_ICR_NACKCF;
        return I2C1_ERR_NACK;
    }
    I2C1->TXDR = regAddr;

    /* Wait for Transfer Complete: the register-index byte has been sent
     * and ACKed, and because AUTOEND was 0 the hardware is now paused,
     * waiting for us to say what happens next (instead of auto-STOPping). */
    if (!waitForFlag(&I2C1->ISR, I2C_ISR_TC, I2C1_TIMEOUT_LOOPS))
    {
        return I2C1_ERR_TIMEOUT;
    }

    /* --- Phase B: repeated START, then read "len" bytes, then auto-STOP --- */
    i2c1StartTransfer(devAddr, (uint8_t)len, 1 /*read*/, 1 /*autoend*/);

    for (i = 0; i < len; i++)
    {
        if (!waitForFlag(&I2C1->ISR, I2C_ISR_RXNE | I2C_ISR_NACKF, I2C1_TIMEOUT_LOOPS))
            return I2C1_ERR_TIMEOUT;

        if (I2C1->ISR & I2C_ISR_NACKF)
        {
            I2C1->ICR = I2C_ICR_NACKCF;
            return I2C1_ERR_NACK;
        }
        buf[i] = (uint8_t)I2C1->RXDR; /* reading RXDR also clears RXNE */
    }

    /* After the last byte, AUTOEND fires STOP automatically - just wait
     * for confirmation and clear the sticky flag. */
    if (!waitForFlag(&I2C1->ISR, I2C_ISR_STOPF, I2C1_TIMEOUT_LOOPS))
        return I2C1_ERR_TIMEOUT;

    I2C1->ICR = I2C_ICR_STOPCF;

    return I2C1_OK;

}

I2C1_Status i2c1ReadReg(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
	return i2c1ReadBytes(devAddr, regAddr, data, 1);
}

