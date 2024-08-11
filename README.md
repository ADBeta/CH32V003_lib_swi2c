# CH32Vxxx lib_swi2C

**NOTE:** This library is currently only tested on the CH32V003.  
Software I2C Library for the CH32Vxxx Family of MCUs  
This library supports:  
* I2C Interface on ANY pins (Assuming no alternate purpose)
* I2C Master Mode
* (On CH32V003) 250KHz interface speed

This library does not have any external dependencies - so can be used in
conjunction with any of the Development Stacks / Libraries.

## How to use
Copy the `lib_swi2c.c` and `lib_swi2c.h` files in your project and #include them.


## TODO
* Optimise speed by cleaning up function calls
* Better documentation & how to use
* Add speed limiting code to keep interface in spec

**-- Long Term --**
* Slave mode
* Test on other MCU variants

----
Copyright (c) ADBeta 2024

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, 
or (at your option) any later version.

This program is distributed in the hope that it will be useful, but 
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  
See the GNU General Public License for more details.
