/******************************************************************************
* lib_swi2c demonstation and test ground
* 
* ADBeta (c) 2024
******************************************************************************/

// ch32v003fun is included for helpful features like printf etc.
// it is NOT Needed in order to use this library
#include "ch32v003fun.h"
#include <stdio.h>

#include "lib_swi2c.h"

i2c_device_t dev = {
	.pin_scl = GPIO_PC3,
	.pin_sda = GPIO_PC4,

	.address = 0xD0,
};

uint8_t data[12];

int main()
{
	SystemInit();

	swi2c_init(&dev);
	swi2c_scan(&dev);
	while (1)
	{
		swi2c_master_receive(&dev, 0x3B, data, 12);
	}
}
