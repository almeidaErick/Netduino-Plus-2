/**
 ******************************************************************************
 * @file    mylib/s4344046_radio.h
 * @author  Erick Almeida – 43440461
 * @date    01042016
 * @brief   Wireless radio communication peripheral driver
 *
 * Modified:  13-April-2016 - Fixed Radio for according to requirements for
 *                            project 1.
 *            19-April-2016 - Fixed radio communication to send ERROR for
 *                            project 1.
 *            16-May-2016 - Create function t be added as a task to continiously
 *                          read messages arriving to the radio.
 *            17-May-2016 - s4344046_radio.c fixed as freeRTOS lib.
 * REFERENCE: Page 20, 50, 57 nRF24L01+ datasheet.
 ******************************************************************************
 ******************************************************************************
 */


#ifndef S4344046_RADIO_H
#define S4344046_RADIO_H

void s4344046_radio_init();
void s4344046_radio_setchan(unsigned	char	chan);
void s4344046_radio_settxaddress(unsigned	char	*addr);
unsigned char	s4344046_radio_getchan();
void s4344046_radio_gettxaddress(unsigned	char	*addr);
void s4344046_radio_fsmprocessing();
void s4344046_radio_sendpacket(char chan, unsigned char *addr,
  unsigned char	*txpacket);

void s4344046_radio_setfsmrx();
int	s4344046_radio_getrxstatus() ;
void s4344046_radio_getpacket(unsigned	char	*rxpacket);
unsigned char* s4344046_getpacket();
int s4344046_packet_status();
unsigned char* s4344046_get_buffer();
void s4344046_get_tx();
void s4344046_no_tx();
unsigned char* s4344046_get_pointer_rx();
void s4344046_set_tx_state();
void s4344046_TaskRadio(void);

#endif
