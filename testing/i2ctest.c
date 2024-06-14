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
	.pin_scl = GPIO_PC4,
	.pin_sda = GPIO_PC5
};

int main()
{
	SystemInit();

	swi2c_master_tx_byte(&bus, 0xF0);
}
