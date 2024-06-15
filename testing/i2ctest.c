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

i2c_bus_t bus = {
	.pin_scl = GPIO_PC3,
	.pin_sda = GPIO_PC4
};

uint8_t data[4] = {0x69, 0x55, 0xaa, 0xee};

int main()
{
	SystemInit();

	swi2c_init(&bus);
	while (1)
	{
		swi2c_master_receive(&bus, 0xD0, 0x3B, data, 2);

		printf("0x%02X%02X\n", data[0], data[1]);
	}
}
