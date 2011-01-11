/*
 * Chris Cole (clecol@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MODBUS_RTAI_H_
#define _MODBUS_RTAI_H_

#include <rtai_serial.h>
#include "modbus.h"

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5
 * RS232 / RS485 ADU = 253 bytes + slave (1 byte) + CRC (2 bytes) = 256 bytes
 */
#define MODBUS_RTAI_MAX_ADU_LENGTH  256

modbus_t* modbus_new_rtai(uint32_t tty, uint32_t baud, uint32_t parity, 
                          uint32_t data_bits, uint32_t stop_bits,
                          int mode, int fifotrig, RTIME delay);

#endif /* _MODBUS_RTAI_H_ */
