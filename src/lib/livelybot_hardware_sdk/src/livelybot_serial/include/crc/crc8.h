#ifndef _CRC8_H_
#define _CRC8_H_

#include <stdint.h>


uint8_t Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);


#endif