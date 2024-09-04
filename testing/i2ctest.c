/******************************************************************************
* lib_swi2c - Software I2C Library for the Ch32V003
* Example usage program
*
* See GitHub Repo for more information: 
* https://github.com/ADBeta/CH32V003_lib_swi2c
*
* 04 Sep 2024    Ver 2.2
*
* Released under the MIT Licence
* Copyright ADBeta (c) 2024
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the 
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
* sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in 
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE 
* USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************/
#include "ch32v003fun.h"
#include <stdio.h>


#include "lib_swi2c.h"


// I2C Scan Callback example function. Prints the address which responded
void i2c_scan_callback(const uint8_t addr)
{
	printf("Address: 0x%02X Responded.\n", addr);
}

// Setup the I2C Device
// NOTE: Frequency is not configurable currently, this library will run as 
// fast as it can, which, on the default HWCLK settings is 300kHz 
i2c_device_t dev = {
	.pin_scl = GPIO_PD5,
	.pin_sda = GPIO_PD4,

	.address = 0x68,
};


int main()
{
	SystemInit();


	// Initialise the I2C Bus for the specified pins
	swi2c_init(&dev);

	// Scan the bus for any devices
	printf("----Scanning I2C Bus for Devices---\n");
	swi2c_scan(&dev, i2c_scan_callback);
	printf("----Done Scanning----\n\n");

	
	/*** Example ***/
	// This example is specifically for the DS3231 I2C RTC Module.
	// Use this as an example for generic devices, changing Address, speed etc
	i2c_err_t i2c_stat;

	// Write to the -Seconds- Register (Reg 0x00, 0x00 Seconds, one byte)
	i2c_stat = swi2c_master_transmit(&dev, 0x00, (uint8_t[]){0x00}, 1);
	if(i2c_stat != I2C_OK) { printf("Error Using the I2C Bus\n"); return -1; }

	// Example of writing an array to a register.
	uint8_t array[3] = {0x00, 0x01, 0x02};
	i2c_stat = swi2c_master_transmit(&dev, 0x00, array, 3);
	if(i2c_stat != I2C_OK) { printf("Error Using the I2C Bus\n"); return -1; }

	// Example to read from the I2C Device
	uint8_t seconds = 0;    // Just Seconds (Read as Hex instead od Decimal)
	uint8_t time[3] = {0};  // Time in Sec, Min, Hrs (Hex not Decimal)
	
	// Loop forever
	while(1)
	{
		// Example reading just one byte
		i2c_stat = swi2c_master_receive(&dev, 0x00, &seconds, 1);
		if(i2c_stat != I2C_OK) printf("Error Using the I2C Bus\n");
		// Print Seconds as a single hex byte
		printf("Seconds: %02X\n", seconds);

		
		// Example reading multiple bytes
		i2c_stat = swi2c_master_receive(&dev, 0x00, time, 3);
		if(i2c_stat != I2C_OK) printf("Error Using the I2C Bus\n");
		// Print Time as Hrs Min Sec
		printf("Time: %02X:%02X:%02X\n\n", time[2], time[1], time[0]);


		// Wait 1 Second
		Delay_Ms(1000);
	}}
