/**
  ******************************************************************************
  * @file    s4344046_hamming.c
  * @author  Erick Almeida
  * @date    15-April-2016
  * @brief   Hamming encoder and decoder.
  *			 Bytes received from the VCP are Hamming encoded and displayed.
  ******************************************************************************
  *
	*	Modified: 12-May-2016 - Create CRC function.
	*						16-May-2016 - Fix encoder and decoder.
	*	Functions:
	*	uint16_t s4344046_hamming_byte_encoder(uint8_t input, uint8_t new_error);
	*	int s4344046_check_for_parity(uint8_t correct);
	*	uint8_t s4344046_hamming_hbyte_encoder(uint8_t send_bits, uint8_t error_mask);
	*	void s4344046_manchester_encoder(uint16_t encode_message);
	*	uint8_t s4344046_hamming_byte_decoder(uint8_t lower, uint8_t upper);
	*	uint16_t s4344046_get_error_mask();
	*	uint16_t s4344046_get_raw_input();
	*	uint8_t* s4344046_get_manchester_encoding();
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//Hamming encoded message variable
unsigned char s4344046_hamming_encode[22];
//Manchester encoded message variable
uint8_t s4344046_manchester_encode[44];
//Syndrome of the message received
uint8_t s4344046_syndrome[3];

//Hols the colums of H matrix to easy check for row error
uint8_t s4344046_parity_check[7][3] = {{1,0,0}, {0,1,0}, {0,0,1}, {1,1,1},
		{1,1,0}, {1,0,1}, {0,1,1}};

//hold the error of the entire message received
uint16_t s4344046_final_error;

//hold the raw input received from the laser (errors included)
uint16_t s4344046_raw_input;

/* Private function prototypes -----------------------------------------------*/
uint16_t s4344046_hamming_byte_encoder(uint8_t input, uint8_t new_error);
int s4344046_check_for_parity(uint8_t correct);
uint8_t s4344046_hamming_hbyte_encoder(uint8_t send_bits, uint8_t error_mask);
void s4344046_manchester_encoder(uint16_t encode_message);
uint8_t s4344046_hamming_byte_decoder(uint8_t lower, uint8_t upper);
uint16_t s4344046_get_error_mask();
uint16_t s4344046_get_raw_input();
uint8_t* s4344046_get_manchester_encoding();


/**
  * Implement Hamming Code + parity checking
  * Hamming code is based on the following generator and parity check matrices
  * G = [ 1 1 1 | 1 0 0 0 ;
  *       1 1 0 | 0 1 0 0 ;
  *       1 0 1 | 0 0 1 0 ;
  *       0 1 1 | 0 0 0 1 ;
  *
  * hence H =
  * [ 1 0 0 | 1 1 1 0 ;
  *   0 1 0 | 1 1 0 1 ;
  *   0 0 1 | 1 0 1 1 ];
  *
  * y = x * G, syn = H * y'
  *
  *
  * NOTE: !! is used to get 1 out of non zeros
  */

/**
	* @brief	Encode a byte using hamming encoder, and use an error mask to send
	*					a byte with a known error.
	* @param	send_bits - byte to be encoded
	*					error_mask - the error that is going to be applied into the byte.
	* @retval None
	*/
uint8_t s4344046_hamming_hbyte_encoder(uint8_t send_bits, uint8_t error_mask) {

		uint8_t d0, d1, d2, d3;
		uint8_t p0 = 0, h0, h1, h2;
		uint8_t i;
		uint8_t encoded;
		uint8_t e0, e1, e2, e3, e4, e5, e6, e7;
		uint8_t error;

  	s4344046_hamming_encode[0] = '1';
  	s4344046_hamming_encode[1] = '1';

		/* extract bits */
		d0 = !!(send_bits & (1<<0));
		d1 = !!(send_bits & (1<<1));
		d2 = !!(send_bits & (1<<2));
		d3 = !!(send_bits & (1<<3));
		e0 = !!(error_mask & (1<<0));
		e1 = !!(error_mask & (1<<1));
		e2 = !!(error_mask & (1<<2));
		e3 = !!(error_mask & (1<<3));
		e4 = !!(error_mask & (1<<4));
		e5 = !!(error_mask & (1<<5));
		e6 = !!(error_mask & (1<<6));
		e7 = !!(error_mask & (1<<7));

		/* calculate hamming parity bits */
		h0 = d0 ^ d1 ^ d2;
		h1 = d0 ^ d1 ^ d3;
		h2 = d0 ^ d2 ^ d3;

		/* Set the error mask */
		error = (e0 << 7) | (e1 << 6) | (e2 << 5) | (e3 << 4) | (e4 << 3) |
					(e5 << 2) | (e6 << 1) | (e7 << 0);

		/* Generate out byte without parity bit P0 */
		encoded = (h0 << 1) | (h1 << 2) | (h2 << 3) |
				(d0 << 4)  | (d1 << 5) | (d2 << 6) |
				(d3 << 7);

		/* calculate even parity bit */
		for (i = 1; i <= 7; i++)
				p0 = p0 ^ !!(encoded & (1 << i));

		encoded |= (p0<<0);
		encoded ^= error;
		return(encoded);
}


/**
	* @brief	Implement Hamming Code on a full byte of input, this means that
	*					16-bits out output is needed.
	* @param	input - full byte message.
	* 					new_error - error mask predefined to set into entire message.
	* @retval Hamming encoder of the entire message (2 bytes encoded with error
	*					mask)
	*/
uint16_t s4344046_hamming_byte_encoder(uint8_t input, uint8_t new_error) {
		uint16_t out;

	/* first encode D0..D3 (first 4 bits),
	 * then D4..D7 (second 4 bits).
	 */
		out = (s4344046_hamming_hbyte_encoder(input & 0xF, new_error)) |
			(s4344046_hamming_hbyte_encoder(input >> 4, new_error)<<8);

  	s4344046_manchester_encoder(out);
		return(out);
}


/**
	* @brief	Convert the message from hamming into Manchester encode form
	* @param	encode_message - message received in Hamming mode.
	* @retval None
	*/
void s4344046_manchester_encoder(uint16_t encode_message) {
  	int i;
  	int j = 0;
  	uint8_t bit_value;

		/*Start bits for first byte*/
  	s4344046_hamming_encode[0] = 1;
  	s4344046_hamming_encode[1] = 1;

		/*Get the data bits and add them two start bits (11) and a stop bit (0)
		for each byte received*/
  	for(i = 2; i < 10; i++) {

    		bit_value = !!(encode_message & (1 << (15-(i-2))));
    		s4344046_hamming_encode[i] = bit_value;
  	}
		/*Stop bit for first byte*/
  	s4344046_hamming_encode[10] = 0;

		/*Start bits from second byte*/
  	s4344046_hamming_encode[11] = 1;
  	s4344046_hamming_encode[12] = 1;

		/*Get the data bits and add them two start bits (11) and a stop bit (0)
			for each byte received*/
  	for(i = 13; i < 21; i++) {

    		bit_value = !!(encode_message & (1 << (15 -(i-5))));
    		s4344046_hamming_encode[i] = bit_value;
  	}

		/*Stop bit for second byte*/
  	s4344046_hamming_encode[21] = 0;

		/*Print Hamming and Manchester encoding bits*/
  	for(i = 0; i < 22 ; i++) {

    		if(s4344046_hamming_encode[i] == 0) {

      			s4344046_manchester_encode[j] = 1;
      			j++;
      			s4344046_manchester_encode[j] = 0;
      			j++;

    		} else {

      			s4344046_manchester_encode[j] = 0;
      			j++;
      			s4344046_manchester_encode[j] = 1;
      			j++;
    		}
  	}
}

/**
	* @brief	Decode the message received from laser, given the lower significant
	*					byte and the upper significant byte.
	* @param	lower - lower byte received after hamming decoding.
	*					higher - higher byte received after hamming decoding.
	* @retval The final message corrected (if one error detected) or (no error
	*					detected) or the value 0xff if the message has many errors and
	*					could not be recovered its value.
	*/
uint8_t s4344046_hamming_byte_decoder(uint8_t lower, uint8_t upper) {
		uint8_t help;

		uint8_t decoded;
		uint16_t decoded_first = 0;
		uint16_t decoded_last = 0;
		uint8_t d0, d1, d2, d3;
		uint8_t p0 = 0, h0, h1, h2;
		uint8_t encoded;
		uint8_t e0, e1, e2, e3, e4, e5, e6, e7;
		uint8_t error = 0;
		uint8_t tem_p0;
		uint8_t final_word = 0;
		int check_byte;

		/*Set byte used to be lower or upper in a single loop (both bytes are used
			in this loop)*/
		for (check_byte = 0; check_byte < 2; check_byte++) {

				error = 0;

				if (check_byte == 0) {

						encoded = lower;

				} else {

						encoded = upper;
				}
				p0 = !!(encoded & (1<<0));
				h0 = !!(encoded & (1<<1));
				h1 = !!(encoded & (1<<2));
				h2 = !!(encoded & (1<<3));
				d0 = !!(encoded & (1<<4));
				d1 = !!(encoded & (1<<5));
				d2 = !!(encoded & (1<<6));
				d3 = !!(encoded & (1<<7));

				/*Get the value of the both bytes inserted in a single 4byte variable*/
				if (check_byte) {

						decoded_first |= (p0 << 8) | (h0 << 9) | (h1 << 10) | (h2 << 11) |
							(d0 << 12) | (d1 << 13) | (d2 << 14) | (d3 << 15);

				} else {

						decoded_first |= (p0 << 0) | (h0 << 1) | (h1 << 2) | (h2 << 3) |
							(d0 << 4) | (d1 << 5) | (d2 << 6) | (d3 << 7);
				}

				/*Calculate syndrome*/
				s4344046_syndrome[0] = h0 ^ d0 ^ d1 ^ d2;
				s4344046_syndrome[1] = h1 ^ d0 ^ d1 ^ d3;
				s4344046_syndrome[2] = h2 ^ d0 ^ d2 ^ d3;

				/*Get the expected parity bit value*/
				tem_p0 = h0 ^ h1 ^ h2 ^ d0 ^ d1 ^ d2 ^ d3;

				/*If no syndrome has been detected, the check if the value of the
					parity bit is correct*/
				if((s4344046_syndrome[0] == 0) && (s4344046_syndrome[1] == 0) &&
						(s4344046_syndrome[2] == 0)) {

						if(tem_p0 != p0) {

								p0 = tem_p0;
						}

				} else {

						int i, j, z;
						/*Check row of the H matrix (parity matrix) to check for the error
							column*/
						for (i = 0; i < 7; i++) {

						/*Check row value for a specific column*/
								for (j = 0; j < 3; j++) {

										/*Compare syndrome to error colum*/
										if (s4344046_syndrome[j] != s4344046_parity_check[i][j]) {

												z = 0;
												j = 3;  //break

										} else {

												z = 1;
										}
								}

								if (z) {

										error |= (1 << (i + 1));
										i = 7; //break
								}
						}
				}

				/*Get the decoded byte message*/
				decoded = (p0 << 0) | (h0 << 1) | (h1 << 2) | (h2 << 3) | (d0 << 4) |
								(d1 << 5) | (d2 << 6) | (d3 << 7);

				/*Get decoded message with its error mask*/
				decoded ^= error;

				/*Check if message can be decoded*/
				if(!(s4344046_check_for_parity(decoded))) {

						debug_printf("ERROR DETECTED IN HAMMING \n");
						return 0xff;
				}

				/*Get the final encoded message into a single variable*/
				if(check_byte == 0) {

						final_word |= ((!!(decoded & (1<<7))) << 3) |
							((!!(decoded & (1<<6))) << 2) | ((!!(decoded & (1<<5))) << 1) |
							((!!(decoded & (1<<4))) << 0);
							decoded_last |= (decoded << 0);

				} else {

						final_word |= ((!!(decoded & (1<<7))) << 7) |
							((!!(decoded & (1<<6))) << 6) | ((!!(decoded & (1<<5))) << 5) |
							((!!(decoded & (1<<4))) << 4);
							decoded_last |= (decoded << 8);
				}
		}
		/*Get the full error mask in 4 bytes */
		s4344046_final_error = decoded_last ^ decoded_first;
		s4344046_raw_input = decoded_first;
		return final_word;
}

/**
	* @brief	Return the final error mask of 4 bytes after comparing the initial
	*					message decoded and the corrected (if one error detected).
	* @param	None
	* @retval The complete error mask represented in 4 bytes.
	*/
uint16_t s4344046_get_error_mask() {
		return s4344046_final_error;
}

/**
	* @brief	Return the raw input of the message, this is the message represented
	*					in 4 bytes including errors (if presented).
	* @param	None
	* @retval The raw received message represented in 4 bytes.
	*/
uint16_t s4344046_get_raw_input() {
		return s4344046_raw_input;
}

/**
	* @brief	If no errors are detected or one error has been detected, then check
	*					the parity bit, if the parity bit is wrong (1 error including the
	*					parity bit) then fix it, otherwise if the parity bit is wrong and
	*					another error is detected, then return a 0 to show that the message
	*					can not be decoded.
	* @param	correct - the final message decoded (no parity bit checked)
	* @retval 0 is error in parity and error in data, or 1 if the message can be
	*					decoded perfectly.
	*/
int s4344046_check_for_parity(uint8_t correct) {
		uint8_t d0, d1, d2, d3;
		uint8_t p0 = 0, h0, h1, h2;
		uint8_t tem_p0;
		p0 = !!(correct & (1<<0));
		h0 = !!(correct & (1<<1));
		h1 = !!(correct & (1<<2));
		h2 = !!(correct & (1<<3));
		d0 = !!(correct & (1<<4));
		d1 = !!(correct & (1<<5));
		d2 = !!(correct & (1<<6));
		d3 = !!(correct & (1<<7));
		tem_p0 = p0 ^ h0 ^ h1 ^ h2 ^ d0 ^ d1 ^ d2 ^ d3;

		if(tem_p0) {

				return 0;
		}
		return 1;
}

/**
	* @brief	Return the manchester encoding message, after hamming is applied
	*					in the received message
	* @param	None
	* @retval The manchester encoded message.
	*/
uint8_t* s4344046_get_manchester_encoding() {
  	return s4344046_manchester_encode;
}
