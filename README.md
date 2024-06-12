# CH32Vxxx lib_swi2C

**NOTE:** This library is currently only tested on the CH32V003.  
Software I2C Library for the CH32Vxxx Family of MCUs  
This library supports:  
* I2C Interface on ANY pins (Assuming no alternate purpose)
* I2C Master Mode
* Full Speed (TODO: MEASURE) Interface

This library does not have any external dependencies - so can be used in
conjunction with any of the Development Stacks / Libraries.

## How to use
Copy the `lib_swi2c.c` and `lib_swi2c.h` files in your project and #include them.


## TODO
* Add I2C Error states
* Add clock stretch to master, not to slave
* Better documentation & how to use
* Add speed limiting code to keep interface in spec
* Slave mode
* Test on other MCU variants

----
ADBeta (c)    2024
