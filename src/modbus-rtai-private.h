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

#ifndef _MODBUS_RTAI_PRIVATE_H_
#define _MODBUS_RTAI_PRIVATE_H_

#include <stdint.h>

#define _MODBUS_RTAI_HEADER_LENGTH      1
#define _MODBUS_RTAI_PRESET_REQ_LENGTH  6
#define _MODBUS_RTAI_PRESET_RSP_LENGTH  2

#define _MODBUS_RTAI_CHECKSUM_LENGTH    2

typedef struct _modbus_rtai {
  uint32_t tty;
  uint32_t baud;
  uint32_t data_bits;
  uint32_t stop_bits;
  uint32_t parity;
  int mode;
  int fifotrig;
} modbus_rtai_t;

#endif /* _MODBUS_RTAI_PRIVATE_H_ */
