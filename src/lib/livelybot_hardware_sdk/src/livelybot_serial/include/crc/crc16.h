#ifndef __CRC16_H
#define __CRC16_H

#include <stdint.h>

/**
 * @brief Calculate CRC-CCITT (XModem) checksum.
 *
 * @param crc     Initial CRC value (typically 0xFFFF).
 * @param buffer  Pointer to the data buffer.
 * @param len     Number of bytes to process.
 * @return        Resulting 16-bit CRC.
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint16_t len);

#endif /* __CRC16_H */
