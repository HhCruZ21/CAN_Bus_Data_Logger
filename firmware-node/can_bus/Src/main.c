/* =========================================================================
 * main.c
 *
 * Minimal example tying the bare-metal I2C1 driver and the BME280 driver
 * together on an STM32G070RB. No HAL is used anywhere in this project.
 *
 * What this does:
 *   1. Initialise I2C1 (GPIO + peripheral registers).
 *   2. Initialise the BME280 (checks chip ID, reads calibration, starts it
 *      in normal mode with x1 oversampling).
 *   3. Forever: wait a bit, read temperature/pressure/humidity, and stash
 *      the results in global variables you can inspect with a debugger, or
 *      hook up to a UART print routine of your own.
 *
 * Add this file plus i2c1_bare_metal.c/.h and bme280.c/.h to your project
 * (e.g. STM32CubeIDE "empty" project with no HAL drivers added), along
 * with your startup file / linker script from the chip's CMSIS package.
 * ========================================================================= */

#include <stdint.h>
#include "i2c1_driver.h"
#include "bme280.h"

/* Latest readings, kept as globals purely so they're easy to inspect in a
 * debugger's "Live Expressions"/watch window without needing a UART. */
volatile int32_t g_temperature_centiC = 0; /* e.g. 2534 -> 25.34 degC       */
volatile uint32_t g_pressure_Pa = 0; /* whole Pascals                  */
volatile uint32_t g_humidity_permille = 0; /* %RH * 10, e.g. 463 -> 46.3 %RH */


int main(void) {
	BME280_Data reading;

	/* NOTE: this example assumes the default reset clock configuration
	 * (HSI16 as SYSCLK, no PLL) as described in i2c1_bare_metal.h. If your
	 * startup code / SystemInit() reconfigures the clock tree, update the
	 * TIMINGR value in i2c1_bare_metal.c accordingly. */

	i2c1Init();

	/* Try to bring the sensor up. If this fails, most likely causes are:
	 *   - wrong I2C address (check BME280_I2C_ADDR vs your SDO wiring)
	 *   - SDA/SCL swapped, or missing pull-up resistors
	 *   - sensor not powered / not connected to PB6(SCL)/PB7(SDA) */
	if (!bme280Init()) {
		/* Initialisation failed - spin here so it's obvious in a debugger.
		 * Replace with an LED blink pattern or an error log in real code. */
		while (1) {
			/* trapped: BME280 not found/responding */
		}
	}

	while (1) {
		/* The sensor runs continuously in Normal mode, so we can just poll
		 * it periodically - no need to trigger each measurement manually. */
		if (bme280ReadMeasurements(&reading)) {
			g_temperature_centiC = reading.temperature_centiC;

			/* Convert Q24.8 -> whole Pascals (drop the 8 fractional bits). */
			g_pressure_Pa = reading.pressure_Pa_q24_8 >> 8;

			/* Convert Q22.10 %RH -> tenths of a percent (x10), so it fits
			 * nicely into an integer without losing the first decimal:
			 * (value * 10) / 1024 */
			g_humidity_permille = (reading.humidity_q22_10 * 10U) >> 10;
		}

		/* Roughly pace the loop; replace with a real timer/SysTick delay
		 * for anything timing-sensitive. */
		// write a delay function
	}
}
