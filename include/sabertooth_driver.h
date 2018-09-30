/*
 * sabertooth_driver.h
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

#ifndef SABERTOOTH_DRIVER_H_
#define SABERTOOTH_DRIVER_H_

// Constant Definitions
#define SABERTOOTH_ADDRESS     	128
#define SABERTOOTH_PORT			"/dev/tty05"

#define SABERTOOTH_MIN_VOLTAGE 	34  // Cutoff voltage of roughly 10V
#define SABERTOOTH_DEADBAND    	3   // Deadband cutoff of 3 counts
#define SABERTOOTH_TIMEOUT		200 // Serial timeout of 200ms

#define CMD_MIN    				-127
#define CMD_MAX     			127

// Motion Constants
#define SPEED_MIN          		-100
#define SPEED_MAX           	100

#define DIR_MIN            		-100
#define DIR_MAX             	100

// Serial configuration constants
enum BAUD {
	BR_75		= B75,
	BR_110		= B110,
	BR_300		= B300,
	BR_1200		= B1200,
	BR_2400		= B2400,
	BR_4800		= B4800,
	BR_9600		= B9600,
	BR_19200	= B19200,
	BR_38400	= B38400,
	BR_57600	= B57600,
	BR_115200	= B115200
};

enum BYTE_SIZE {
	BS_5_BIT	= CS5,
	BS_6_BIT	= CS6,
	BS_7_BIT	= CS7,
	BS_8_BIT	= CS8
};

enum PARITY_BIT {
	PB_NONE		= 0x0000,
	PB_EVEN		= PARENB,
	PB_ODD		= PARENB | PARODD
};

enum STOP_BIT {
	SB_ONE		= 0x0000,
	SB_TWO		= CSTOPB
};

// Functions
int open_port(const char *dir, int flags);
int close_port(int fd);
void configure_port(int fd, BAUD baud_rate, PARITY_BIT parity_bit, BYTE_SIZE byte_size, STOP_BIT stop_bit);
void motor_callback(const sabertooth_driver::motor_cmd& cmd_msg);
void mixed_callback(const sabertooth_driver::mixed_cmd& cmd_msg);

#endif /* SABERTHOOTH_DRIVER_H_ */
