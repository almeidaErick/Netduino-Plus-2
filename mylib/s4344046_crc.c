/**
  ******************************************************************************
  * @file    s4344046_crc.c
  * @author  Erick Almeida
  * @date    12-May-2016
  * @brief   CRC calculation for a given message.
  ******************************************************************************
  *
	*	Modified: 12-May-2016 - Create CRC function.
  *           14-May-2016 - Fix CRC for 32-bit message
	*	Functions:
	*	uint16_t s4344046_crc_update(uint16_t crc, uint8_t c);
	*	uint16_t s4344046_crc_update32(uint16_t crcValue, uint32_t input);
  */

/**
	* @brief	Get CRC from an 8bit input
	* @param	crcValue - variable where the CRC value is going to be stored
						newByte - Input to be calculated the CRC from.
	* @retval the new value for crcValue.
	*/


/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"

/* Private define ------------------------------------------------------------*/
#define POLY  0x1021 // CRC-CCITT

/* Private function prototypes -----------------------------------------------*/
uint16_t s4344046_crc_update(uint16_t crc, uint8_t c);
uint16_t s4344046_crc_update32(uint16_t crcValue, uint32_t input);


uint16_t s4344046_crc_update(uint16_t crcValue, uint8_t newByte)
{
	unsigned char i;

	for (i = 0; i < 8; i++) {

		if (((crcValue & 0x8000) >> 8) ^ (newByte & 0x80)){

			crcValue = (crcValue << 1)  ^ POLY;

		}else{

			crcValue = (crcValue << 1);
		}

		newByte <<= 1;
	}
	return crcValue;
}


/**
	* @brief	Get CRC from an 32bit input
	* @param	crcValue - variable where the CRC value is going to be stored
						newByte - Input to be calculated the CRC from.
	* @retval the new value for crcValue.
	*/
uint16_t s4344046_crc_update32(uint16_t crcValue, uint32_t input) {
		int i;
		/*Interact with the entire 32bit number to calculate the 8bit number CRC*/
		for (i = 3; i >= 0; i--){

			crcValue = s4344046_crc_update(crcValue, ((input >> i*8) & 0xFF));
		}
		return crcValue;
}
