#ifndef _CRC_LIB_H
#define _CRC_LIB_H

#include<inttypes.h>

uint8_t make_crc8_tabled(uint8_t header, uint8_t data[], uint8_t numDataBytes);
uint8_t make_crc8_nontabled(uint8_t header, uint8_t data[], uint8_t numDataBytes);
#endif