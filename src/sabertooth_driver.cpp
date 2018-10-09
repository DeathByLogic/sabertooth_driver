/*
 * sabertooth_driver.cpp
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <termios.h>
#include <errno.h>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// Other C Includes
#include "sabertooth.h"
#include "sabertooth_driver.h"

// Global Variables
Sabertooth *ST;

int main (int argc, char** argv) {
	int fd;

	std::string port;
	int baud, address, timeout, ramp, deadband;
	bool mixed_mode;
	std::string motor1_topic, motor2_topic;
	std::string drive_topic, turn_topic;
	float min_voltage, max_voltage;
	BAUD baud_rate;

	// Initialize ROS
	ros::init(argc, argv, "sabertooth_driver");

	// Create a new node
	ros::NodeHandle n;

	// Get TTY Port
	if (!n.getParam("sabertooth_driver/port", port)) {
		ROS_ERROR("Sabertooth: Failed to get param port");

		// Quit on error
		exit(EINVAL);
	}

	// Get Sabertooth Address
	if (!n.getParam("sabertooth_driver/address", address)) {
		ROS_ERROR("Sabertooth: Failed to get param address");

		// Quit on error
		exit(EINVAL);
	}

	// Get the baud rate for the serial port
	n.param<int>("sabertooth_driver/baud", baud, 9600);

	switch (baud) {
		case 2400:
			baud_rate = BR_2400;
			break;
		case 9600:
			baud_rate = BR_9600;
			break;
		case 19200:
			baud_rate = BR_19200;
			break;
		case 38400:
			baud_rate = BR_38400;
			break;
		default:
			baud_rate = BR_9600;
			break;
	}

	// Get the operating mode
	n.param<bool>("sabertooth_driver/mixed_mode", mixed_mode, false);

	if (mixed_mode == false) {
		// Direct drive mode	
		n.param<std::string>("sabertooth_driver/motor1_topic", motor1_topic, "motor1_command");
		n.param<std::string>("sabertooth_driver/motor2_topic", motor2_topic, "motor2_command");
	} else {
		// Mixed mode
		n.param<std::string>("sabertooth_driver/drive_topic", drive_topic, "drive_command");
		n.param<std::string>("sabertooth_driver/turn_topic", turn_topic, "turn_command");
	}

	// Open serial port
	if ((fd = open_port(port.data(), O_RDWR | O_NOCTTY | O_NDELAY)) <= 0) {
		ROS_ERROR("Sabertooth: Failed to open port");

		exit(errno);
	}

	// Configure serial port
	configure_port(fd, baud_rate, PB_NONE, BS_8_BIT, SB_ONE);

	// Create Sabertooth driver object
	ST = new Sabertooth(address, fd);

	//
	// Configure Sabertooth
	//

	// Set deadband for Sabertooth
	if (n.getParam("sabertooth_driver/deadband", deadband)) {
		ROS_DEBUG("Setting deadband to %i.", deadband);
		ST->setDeadband(deadband);
	}

	// Set low voltage cutoff
	if (n.getParam("sabertooth_driver/min_voltage", min_voltage)) {
		ROS_DEBUG("Setting minimum voltage to %fV.", min_voltage);
		ST->setMinVoltage((int)((min_voltage - 6) / 0.2));
	}

	// Set over voltage cutoff
	if (n.getParam("sabertooth_driver/max_voltage", max_voltage)) {
		ROS_DEBUG("Setting maximum voltage to %fV.", max_voltage);
		ST->setMaxVoltage((int)(max_voltage * 5.12));
	}

	// Set serial port timeout
	if (n.getParam("sabertooth_driver/timeout", timeout)) {
		ROS_DEBUG("Setting timeout to %i milliseconds.", timeout);
		ST->setTimeout(timeout);
	}

	// Set ramping value
	if (n.getParam("sabertooth_driver/ramping", ramp)) {
		ROS_DEBUG("Setting ramp to %i.", ramp);
		ST->setRamping(ramp);
	}

	ros::Subscriber motor1_sub;
	ros::Subscriber motor2_sub;
	ros::Subscriber drive_sub;
	ros::Subscriber turn_sub;

	// Subscribe to the motor commands
	if (mixed_mode == false) {
		motor1_sub = n.subscribe(motor1_topic, 50, motor1_callback);
		motor2_sub = n.subscribe(motor2_topic, 50, motor2_callback);
	} else {
		drive_sub = n.subscribe(drive_topic, 50, drive_callback);
		turn_sub = n.subscribe(turn_topic, 50, turn_callback);
	}

	// Spin until done
	ros::spin();

	// Delete Sabertooth object
	delete ST;

	// Close serial port
	close_port(fd);

	return 0;
}

// Open serial port
int open_port(const char *dir, int flags) {
	int	fd;

	// Open the file for write only
	if ((fd = open(dir, flags)) < 0) {
		perror("Sabertooth: Unable to open port");
	}

	return fd;
}

// Close serial port
int close_port(int fd) {
	int rc;

	// Close the file
	if ((rc = close(fd)) < 0) {
		perror("Sabertooth: Unable to close port");
	}

	return rc;
}

// Configure Port
void configure_port(int fd, BAUD baud_rate, PARITY_BIT parity_bit, BYTE_SIZE byte_size, STOP_BIT stop_bit) {
	struct termios options;

	// Configure port reading
	fcntl(fd, F_SETFL, FNDELAY);

	// Get the current options for the port
	tcgetattr(fd, &options);

	// Set the Tx and Rx baud rates
	cfsetispeed(&options, baud_rate);
	cfsetospeed(&options, baud_rate);

	// Enable the receiver and set local mode
	options.c_cflag |= (CLOCAL | CREAD);

	// Mask previous settings
	options.c_cflag &= ~(PARENB | PARODD | CSTOPB | CSIZE);

	// Configure new settings
	options.c_cflag |= byte_size;
	options.c_cflag |= parity_bit;
	options.c_cflag |= stop_bit;

	// Disable hardware flow control
	options.c_cflag &= ~CRTSCTS;

	// Enable data to be processed as raw input
	options.c_lflag &= ~(ICANON | ECHO | ISIG | ONLCR | OCRNL | ECHONL | IEXTEN);
	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	options.c_oflag &= ~OPOST;

	// Set the new for the port
	tcsetattr(fd, TCSANOW, &options);
}

void motor1_callback(const std_msgs::Float64& cmd_msg) {
	ST->motor(1, (uint8_t)round(cmd_msg.data));
}

void motor2_callback(const std_msgs::Float64& cmd_msg) {
	ST->motor(2, (uint8_t)round(cmd_msg.data));
}

void drive_callback(const std_msgs::Float64& cmd_msg) {
	ST->drive((uint8_t)round(cmd_msg.data));
}

void turn_callback(const std_msgs::Float64& cmd_msg) {
	ST->turn((uint8_t)round(cmd_msg.data));
}
