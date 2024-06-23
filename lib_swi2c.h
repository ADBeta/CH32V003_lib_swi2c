/******************************************************************************
* 
* ADBeta (c)    Jun 2024    Ver 0.5.0
******************************************************************************/
#ifndef LIB_SWI2C_H
#define LIB_SWI2C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*** Copied from lib_GPIOCTRL ************************************************/
// https://github.com/ADBeta/CH32V003_lib_GPIOCTRL

/*** GPIO Pin Enumeration ****************************************************/
/// @breif This enum is used as binary data for pin and port addressing. 
/// 0x[PIN][PORT] - 0x 0603 6th pin of port 4 (PORTD)
/// NOTE: Little-Endian Architecture means value in memory is [PORT][PIN] order
typedef enum {
	GPIO_PA0      = 0x0000,
	GPIO_PA1      = 0x0100, 
	GPIO_PA2      = 0x0200, 
	GPIO_PA3      = 0x0300,
	GPIO_PA4      = 0x0400,
	GPIO_PA5      = 0x0500,
	GPIO_PA6      = 0x0600,
	GPIO_PA7      = 0x0700,

	GPIO_PB0      = 0x0001,
	GPIO_PB1      = 0x0101, 
	GPIO_PB2      = 0x0201, 
	GPIO_PB3      = 0x0301,
	GPIO_PB4      = 0x0401,
	GPIO_PB5      = 0x0501,
	GPIO_PB6      = 0x0601,
	GPIO_PB7      = 0x0701,

	GPIO_PC0      = 0x0002,
	GPIO_PC1      = 0x0102, 
	GPIO_PC2      = 0x0202, 
	GPIO_PC3      = 0x0302,
	GPIO_PC4      = 0x0402,
	GPIO_PC5      = 0x0502,
	GPIO_PC6      = 0x0602,
	GPIO_PC7      = 0x0702,

	GPIO_PD0      = 0x0003,
	GPIO_PD1      = 0x0103, 
	GPIO_PD2      = 0x0203, 
	GPIO_PD3      = 0x0303,
	GPIO_PD4      = 0x0403,
	GPIO_PD5      = 0x0503,
	GPIO_PD6      = 0x0603,
	GPIO_PD7      = 0x0703,
} gpio_pin_t;

/*** Software I2C Functions **************************************************/
#define I2C_ACK  0
#define I2C_NACK 1

// I2C Error / Return states
typedef enum {
	I2C_OK      = 0,
	I2C_ERR_TIMEOUT,
	I2C_ERR_NACK,
	I2C_ERR_INVALID_ARGS,
} i2c_err_t;


// I2C Device Variables. Multiple devices can share SCL and SDA Pins
typedef struct {
	gpio_pin_t pin_scl, pin_sda;  // SCL and SDA Pins. e.g GPIO_PD4
	uint8_t address;              // I2C Deivce Address
	uint32_t hz;                  // I2C Bus Speed (In Hz)
	
	// Private Variables
	bool _active;                 // Keep track of I2C bus state (repeat start)
} i2c_device_t;


/// @breif Initialises the I2C bus pins, and sends a STOP command
/// @param i2c_device_t i2c, I2C Device Struct
/// @param gpio_pin_t scl, SCL I2C Pin
/// @param gpio_pin_t sda, SDA I2C Pin
/// @return i2c_err_t return state
i2c_err_t swi2c_init(i2c_device_t *i2c);

/*** Hardware Control ********************************************************/
/// @breif Sends a START Command to the I2C Device
/// @param i2c_device_t i2c, I2C Device Struct
/// @return i2c_err_t return state
i2c_err_t swi2c_start(i2c_device_t *i2c);

/// @breif Sends a STOP Command to the I2C Device
/// @param i2c_device_t i2c, I2C Device Struct
/// @return i2c_err_t return state
i2c_err_t swi2c_stop(i2c_device_t *i2c);

/// @breif Transmits a Byte in Master Mode
/// @param i2c_device_t i2c, I2C Device Struct
/// @param uint8_t byte, data to be sent
/// @return i2c_err_t return state
i2c_err_t swi2c_master_tx_byte(i2c_device_t *i2c, uint8_t data);

/// @breif Receives a Byte in Master Mode
/// @param i2c_device_t i2c, I2C Device Struct
/// @param bool ack, ack bit. ACK (LOW): Read more  NACK (HIGH): Stop Reading
/// @return i2c_err_t return state
uint8_t swi2c_master_rx_byte(i2c_device_t *i2c, bool ack);



/// @breif Scans the Interface for devices that respond. Prints their address
/// @param i2c_device_t i2c, I2C Device Struct
/// @return none
void swi2c_scan(i2c_device_t *i2c);

/// @breif Transmits data to a given Address
/// @param i2c_device_t i2c, I2C Device Struct
/// @param uint8_t reg, register to write to
/// @param uint8_t *data, data array pointer
/// @param uint16_t size, number of bytes to transmit
/// @return i2c_err_t return state
i2c_err_t swi2c_master_transmit(i2c_device_t *i2c, 
                        const uint8_t reg, const uint8_t *data, uint16_t size);


/// @breif Receive data from a given Address
/// @param i2c_device_t i2c, I2C Device Struct
/// @param uint8_t reg, register to read from
/// @param uint8_t *data, data array pointer
/// @param uint16_t size, number of bytes to receive
/// @return i2c_err_t return state
i2c_err_t swi2c_master_receive(i2c_device_t *i2c, 
                              const uint8_t reg, uint8_t *data, uint16_t size);


#endif
