#ifndef __CRC16_H
#define __CRC16_H

#include <stdint.h>

uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint16_t len);

#endif