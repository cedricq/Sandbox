/*
 * crc.h
 *
 *  Created on: Oct 29, 2021
 *      Author: cedqu
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include <stdint.h>

typedef enum{
    CHECKSUM_ERROR = 0X04
}etError;

uint8_t SF04_CheckCrc (uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
uint8_t SF04_CalcCrc (uint8_t data[], uint8_t nbrOfBytes);

#endif /* INC_CRC_H_ */
