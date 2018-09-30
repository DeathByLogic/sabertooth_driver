/*
Arduino Library for SyRen/Sabertooth Packet Serial
Copyright (c) 2012-2013 Dimension Engineering LLC
http://www.dimensionengineering.com/arduino

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "sabertooth.h"

using namespace std;

Sabertooth::Sabertooth(uint8_t address, int port)
  : _address(address), _port(port)
{

}

void Sabertooth::autobaud(bool dontWait) const
{
  autobaud(port(), dontWait);
}

void Sabertooth::autobaud(int port, bool dontWait) const
{
	uint8_t ab = 0xAA;

	if (!dontWait) { usleep(1500000); }

	writePort(&ab, 1);

	if (!dontWait) { usleep(500000); }
}

void Sabertooth::command(uint8_t command, uint8_t value) const
{
	 uint8_t bytes[4];

	 bytes[0] = address();
	 bytes[1] = command;
	 bytes[2] = value;
	 bytes[3] = (address() + command + value) & 0x7F;

	 writePort(bytes, 4);
}

void Sabertooth::writePort(const uint8_t *data, unsigned int count) const
{
	int rc;

	// Write value to port
	if ((rc = write(_port, data, count)) < 0) {
		perror("Sabertooth: Unable to write to port ");
	}
}

void Sabertooth::throttleCommand(uint8_t command, int power) const
{
  power = CONSTRAIN(power, -126, 126);
  this->command(command, (uint8_t)abs(power));
}

void Sabertooth::motor(int power) const
{
  motor(1, power);
}

void Sabertooth::motor(uint8_t motor, int power) const
{
  if (motor < 1 || motor > 2) { return; }
  throttleCommand((motor == 2 ? 4 : 0) + (power < 0 ? 1 : 0), power);
}

void Sabertooth::drive(int power) const
{
  throttleCommand(power < 0 ? 9 : 8, power);
}

void Sabertooth::turn(int power) const
{
  throttleCommand(power < 0 ? 11 : 10, power);
}

void Sabertooth::stop() const
{
  motor(1, 0);
  motor(2, 0);
}

void Sabertooth::setMinVoltage(uint8_t value) const
{
  command(2, (uint8_t)MIN(value, 120));
}

void Sabertooth::setMaxVoltage(uint8_t value) const
{
  command(3, (uint8_t)MIN(value, 127));
}

void Sabertooth::setBaudRate(long baudRate) const
{
  uint8_t value;
  switch (baudRate)
  {
  case 2400:           value = 1; break;
  case 9600: default: value = 2; break;
  case 19200:          value = 3; break;
  case 38400:          value = 4; break;
  case 115200:         value = 5; break;
  }
  command(15, value);
  
  // (1) flush() does not seem to wait until transmission is complete.
  //     As a result, a Serial.end() directly after this appears to
  //     not always transmit completely. So, we manually add a delay.
  // (2) Sabertooth takes about 200 ms after setting the baud rate to
  //     respond to commands again (it restarts).
  // So, this 500 ms delay should deal with this.
  usleep(500000);
}

void Sabertooth::setDeadband(uint8_t value) const
{
  command(17, (uint8_t)MIN(value, 127));
}

void Sabertooth::setRamping(uint8_t value) const
{
  command(16, (uint8_t)CONSTRAIN(value, 0, 80));
}

void Sabertooth::setTimeout(int milliseconds) const
{
  command(14, (uint8_t)((CONSTRAIN(milliseconds, 0, 12700) + 99) / 100));
}
