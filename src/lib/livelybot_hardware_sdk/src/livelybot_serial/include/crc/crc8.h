#ifndef _CRC8_H_
#define _CRC8_H_

#include <stdint.h>

/**
 * @brief Calculate CRC-8 checksum (Dallas/Maxim algorithm).
 *
 * @param pchMessage  Pointer to the data buffer.
 * @param dwLength    Number of bytes to process.
 * @param ucCRC8      Initial CRC value (typically 0xFF).
 * @return            Resulting 8-bit checksum.
 */
uint8_t Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

#endif /* _CRC8_H_ */
