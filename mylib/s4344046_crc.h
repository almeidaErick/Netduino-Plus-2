
/* -----------------------------------------------------------------------------
# Author name: Erick Almeida - 43440461
# Creation date: 150416
# Revision date (name): -
# Changes implemented (date):
           12-May-2016 Create CRC function
           14-May-2016 - Fix CRC for 32-bit message
           18-May-2016 - Add macros for s4344046_crc.h
#(Comments):
------------------------------------------------------------------------------*/


#ifndef S4344046_CRC_H
#define S4344046_CRC_H

uint16_t s4344046_crc_update(uint16_t crc, uint8_t c);
uint16_t s4344046_crc_update32(uint16_t crcValue, uint32_t input);

#endif
