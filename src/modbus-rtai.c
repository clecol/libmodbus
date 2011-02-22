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

#include <rtai_serial.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

#include "modbus-private.h"

#include "modbus-rtai.h"
#include "modbus-rtai-private.h"

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/* Define the slave ID of the remote device to talk in master mode or set the
 * internal slave ID in slave mode */
static int _modbus_set_slave(modbus_t *ctx, int slave)
{
    /* Broadcast address is 0 (MODBUS_BROADCAST_ADDRESS) */
    if (slave >= 0 && slave <= 247) {
        ctx->slave = slave;
    } else {
        errno = EINVAL;
        return -1;
    }

    return 0;
}

/* Builds a RTAI RTU request header */
static int _modbus_rtai_build_request_basis(modbus_t *ctx, int function,
                                           int addr, int nb,
                                           uint8_t *req)
{
    assert(ctx->slave != -1);
    req[0] = ctx->slave;
    req[1] = function;
    req[2] = addr >> 8;
    req[3] = addr & 0x00ff;
    req[4] = nb >> 8;
    req[5] = nb & 0x00ff;

    return _MODBUS_RTAI_PRESET_REQ_LENGTH;
}

/* Builds a RTAI RTU response header */
static int _modbus_rtai_build_response_basis(sft_t *sft, uint8_t *rsp)
{
    /* In this case, the slave is certainly valid because a check is already
     * done in _modbus_rtu_listen */
    rsp[0] = sft->slave;
    rsp[1] = sft->function;

    return _MODBUS_RTAI_PRESET_RSP_LENGTH;
}

static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

int _modbus_rtai_prepare_response_tid(const uint8_t *req, int *req_length)
{
    (*req_length) -= _MODBUS_RTAI_CHECKSUM_LENGTH;
    /* No TID */
    return 0;
}

int _modbus_rtai_send_msg_pre(uint8_t *req, int req_length)
{
    uint16_t crc = crc16(req, req_length);
    req[req_length++] = crc >> 8;
    req[req_length++] = crc & 0x00FF;

    return req_length;
}

ssize_t _modbus_rtai_send(modbus_t *ctx, const uint8_t *req, int req_length)
{
  // since rtai returns number of bytes NOT written, we'll return our required
  // request length subtracted by our rtai response to get the number of bytes
  // actually written, consistent with our normal serial port return value.
  //
  // if the return value is negative, return that instead.
  modbus_rtai_t *ctx_rtai = ctx->backend_data;

  int ret = rt_spwrite_timed(ctx_rtai->tty, (char*)req, req_length, ctx_rtai->delay);
  //printf("on send: wanted to send %d bytes, return value = %d, returning bytes written as %d\n",
  //req_length, ret, req_length);

  if (ret == 0) // successful, return rsp_length
      return req_length;
  else
      return ret; // error 
}

ssize_t _modbus_rtai_recv(modbus_t *ctx, uint8_t *rsp, int rsp_length)
{
  // since rtai returns number of bytes NOT received, we'll return our required
  // response length subtracted by our rtai response to get the number of bytes
  // actually read, consistent with our normal serial port return value.
  //
  // if the return value is negative, return that instead
  modbus_rtai_t *ctx_rtai = ctx->backend_data;

  //printf("before read\n");
  int ret = rt_spread_timed(ctx_rtai->tty, (char*)rsp, rsp_length, ctx_rtai->delay);
  //printf("on recv: wanted to recv %d bytes, return value = %d, returning bytes read as %d\n",
  //     rsp_length, ret, rsp_length);

  if (ret == 0) // successful, return rsp_length
      return rsp_length;
  else
      return ret; // error 
}

int _modbus_rtai_flush(modbus_t *);


/* The check_crc16 function shall return the message length if the CRC is
   valid. Otherwise it shall return -1 and set errno to EMBADCRC. */
int _modbus_rtai_check_integrity(modbus_t *ctx, uint8_t *msg,
                                 const int msg_length)
{
    uint16_t crc_calculated;
    uint16_t crc_received;

    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of msg */
    if (crc_calculated == crc_received) {
        return msg_length;
    } else {
        if (ctx->debug) {
            fprintf(stderr, "ERROR CRC received %0X != CRC calculated %0X\n",
                    crc_received, crc_calculated);
        }
        if (ctx->error_recovery) {
            _modbus_rtai_flush(ctx);
        }
        errno = EMBBADCRC;
        return -1;
    }
}

/* Sets up a TTY port for RTAI RTU communications */
static int _modbus_rtai_connect(modbus_t *ctx)
{
  modbus_rtai_t *ctx_rtai = ctx->backend_data;
  
  if (ctx->debug) {
    printf("Opening TTY %d at %d baud (%d, %d, %d)\n",
           ctx_rtai->tty, ctx_rtai->baud, ctx_rtai->parity,
           ctx_rtai->data_bits, ctx_rtai->stop_bits);
  }

  int ret = rt_spopen(ctx_rtai->tty, ctx_rtai->baud, ctx_rtai->data_bits,
                      ctx_rtai->stop_bits, ctx_rtai->parity,
                      ctx_rtai->mode, ctx_rtai->fifotrig);
  if (ret != 0) {
      fprintf(stderr, "ERROR %d: Can't open TTY %d\n", ret, ctx_rtai->tty);
      return ret;
  }

  return 0;
}

void _modbus_rtai_close(modbus_t *ctx)
{
  modbus_rtai_t *ctx_rtai = ctx->backend_data;
  rt_spclose(ctx_rtai->tty);
}

int _modbus_rtai_flush(modbus_t *ctx)
{
  modbus_rtai_t *ctx_rtai = ctx->backend_data;  
  rt_spclear_rx(ctx_rtai->tty);
  rt_spclear_tx(ctx_rtai->tty);
  return 0;
}

int _modbus_rtai_select(modbus_t *ctx, fd_set *rfds, struct timeval *tv, int length_to_read)
{ return 1; }
    

int _modbus_rtai_filter_request(modbus_t *ctx, int slave)
{
    /* Filter on the Modbus unit identifier (slave) in RTU mode */
    if (slave != ctx->slave && slave != MODBUS_BROADCAST_ADDRESS) {
        /* Ignores the request (not for me) */
        if (ctx->debug) {
            printf("Request for slave %d ignored (not %d)\n",
                   slave, ctx->slave);
        }
        return 1;
    } else {
        return 0;
    }
}

const modbus_backend_t _modbus_rtai_backend = {
    _MODBUS_BACKEND_TYPE_RTAI,
    _MODBUS_RTAI_HEADER_LENGTH,
    _MODBUS_RTAI_CHECKSUM_LENGTH,
    MODBUS_RTAI_MAX_ADU_LENGTH,
    _modbus_set_slave,
    _modbus_rtai_build_request_basis,
    _modbus_rtai_build_response_basis,
    _modbus_rtai_prepare_response_tid,
    _modbus_rtai_send_msg_pre,
    _modbus_rtai_send,
    _modbus_rtai_recv,
    _modbus_rtai_check_integrity,
    _modbus_rtai_connect,
    _modbus_rtai_close,
    _modbus_rtai_flush,
    _modbus_rtai_select,
    _modbus_rtai_filter_request
};

/* Allocate and initialize the modbus_t structure for RTAI
   - tty: 0, 1, 2, etc.
   - baud: 9600, 19200, 57600, 115200, etc.
   - parity(rtai_serial.h): RT_SP_PARITY_EVEN, RT_SP_PARITY_NONE, RT_SP_PARITY_ODD,
                            RT_SP_PARITY_HIGH, RT_SP_PARITY_LOW (rtai_serial.h)
   - data_bits: 5, 6, 7, 8
   - stop_bits 1, 2
   - mode: RT_SP_NO_HAND_SHAKE, RT_SP_DSR_ON_TX, RT_SP_HW_FLOW
   - fifotrig (rtai_serial.h): RT_SP_FIFO_DISABLE, RT_SP_FIFO_SIZE_1, RT_SP_FIFO_SIZE_4,
                               RT_SP_FIFO_SIZE_8, RT_SP_FIFO_SIZE_14 
*/
modbus_t* modbus_new_rtai(uint32_t tty, uint32_t baud, uint32_t parity,
                          uint32_t data_bits, uint32_t stop_bits,
                          int mode, int fifotrig, RTIME delay)
{
    modbus_t *ctx;
    modbus_rtai_t *ctx_rtai;

    ctx = (modbus_t *) malloc(sizeof(modbus_t));
    _modbus_init_common(ctx);

    ctx->backend = &_modbus_rtai_backend;
    ctx->backend_data = (modbus_rtai_t *) malloc(sizeof(modbus_rtai_t));
    ctx_rtai = (modbus_rtai_t *)ctx->backend_data;

    ctx_rtai->tty = tty;
    ctx_rtai->baud = baud;
    ctx_rtai->parity = parity;
    /* if (parity == RT_SP_PARITY_EVEN || parity == RT_SP_PARITY_NONE || */
    /*     parity == RT_SP_PARITY_ODD || parity == RT_SP_PARITY_HIGH || */
    /*     parity == RT_SP_PARITY_LOW) */
    /*     ctx_rtai->parity = parity; */
    /* else { */
    /*     modbus_free(ctx); */
    /*     errno = EINVAL; */
    /*     return NULL; */
    /*}*/
    ctx_rtai->data_bits = data_bits;
    ctx_rtai->stop_bits = stop_bits;
    ctx_rtai->mode = mode;
    ctx_rtai->fifotrig = fifotrig;
    ctx_rtai->delay = delay;
    
    return ctx;
}
