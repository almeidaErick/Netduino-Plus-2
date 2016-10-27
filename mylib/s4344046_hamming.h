
 /* -----------------------------------------------------------------------------
 # Author name: Erick Almeida - 43440461
 # Creation date: 150416
 # Revision date (name): -
 # Changes implemented (date):
            12-May-2016 Changed names for functions
            16-May-2016 Hamming encoder and decoder fixed.
 #(Comments):
 ------------------------------------------------------------------------------*/


 #ifndef S4344046_HAMMING_H
 #define S4344046_HAMMING_H


uint8_t s4344046_hamming_hbyte_encoder(uint8_t send_bits,
    uint8_t error_mask);
uint16_t s4344046_hamming_byte_encoder(uint8_t input, uint8_t new_error);
void s4344046_manchester_encoder(uint16_t encode_message);
uint8_t* s4344046_get_manchester_encoding();
uint8_t s4344046_hamming_byte_decoder(uint8_t lower, uint8_t upper);
uint16_t s4344046_get_error_mask();
uint16_t s4344046_get_raw_input();
int s4344046_check_for_parity(uint8_t correct);

#endif
