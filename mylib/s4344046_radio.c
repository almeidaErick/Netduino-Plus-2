/**
  ******************************************************************************
  * @file    s4344046_radio.c
  * @author  Erick Almeida
  * @date    1-April-2016
  * @brief   Radio	Communications Finite	State	Machine	Controller
  *			 Page 20, 50, 57 nRF24L01+ datasheet.
  *
  *  Modified: 13-April-2016 - Fixed Radio for according to requirements for
  *                            project 1.
  *            19-April-2016 - Fixed radio communication to send ERROR for
  *                            project 1.
  *            16-May-2016 - Create function t be added as a task to continiously
  *                          read messages arriving to the radio.
  *            17-May-2016 - s4344046_radio.c fixed as freeRTOS lib.
  *            28-May-2016 - Create function that set the radio and the control
  *                          Variable in IDLE state.
  *
  *  Functions:
  *            void s4344046_radio_init();
  *            void s4344046_radio_setchan(unsigned	char	chan);
  *            void s4344046_radio_settxaddress(unsigned	char	*addr);
  *            unsigned char	s4344046_radio_getchan();
  *            void s4344046_radio_gettxaddress(unsigned	char	*addr);
  *            void s4344046_radio_fsmprocessing();
  *            void s4344046_radio_sendpacket(char chan, unsigned char *addr,
  *              unsigned char	*txpacket);
  *
  *            void s4344046_radio_setfsmrx();
  *            int	s4344046_radio_getrxstatus() ;
  *            void s4344046_radio_getpacket(unsigned	char	*rxpacket);
  *            unsigned char* s4344046_getpacket();
  *            int s4344046_packet_status();
  *            unsigned char* s4344046_get_buffer();
  *            void s4344046_get_tx();
  *            void s4344046_no_tx();
  *            unsigned char* s4344046_get_pointer_rx();
  *            void s4344046_set_tx_state();
  *            void s4344046_TaskRadio(void);
  *            void s4344046_start_idle();
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"


#include "FreeRTOS_CLI.h"
#include "s4344046_cli.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define S4344046_IDLE_STATE 0 //IDLE	state	(used	for	reading/writing	registers)
#define S4344046_RX_STATE 2 //Put	radio	FSM	into	receiving	mode
#define S4344046_TX_STATE 1 //Put	radio	FSM	into	transmitting	mode
#define S4344046_WAITING_STATE 3 //Wait	for	radio	FSM	to	receive	packet
#define WRITE_VALUE 5
#define READ_VALUE 4
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef SpiHandle;
//Internal	(static)	variable.	Hold	current	state	of FSM. (0, 1, 2, 3)
int	s4344046_radio_fsmcurrentstate;
//Status	of	radio	RX	(0	=	no	packet	received,	1	=	packet	received)
//Must	be	reset	when	FSM	is	in IDLE	state.
int	s4344046_radio_rxstatus;
//Is packet sent
int s4344046_packet_sent = 1;
int s4344046_next = 0;
unsigned char s4344046_buffer[32] = {0x20, 0x7B, 0x56, 0x34, 0x12, 0x61,
              0x04, 0x44, 0x43};
//Message buffer
unsigned char s4344046_get_new_buffer[7];
//buffer Count
int s4344046_count_buffer = 0;
int s4344046_activate_tx = 1;
//Radio	RX	buffer,	used	to	hold	received	packet
//(called	by	radio_fsm_read()).
uint8_t	s4344046_rx_buffer[32];
uint8_t s4344046_get_new_packet[32];
//Check if it is in RX mode
int s4344046_rx_mode = 1;
char RxChar;
//Variable to check if the message received belongs to the student number
//predefined
int s4344046_student_number();

//Variable that stores the address of the radio
uint8_t s4344046_address[READ_VALUE];
//Student number to check
uint8_t s4344046_student[4] = {0x61, 0x04, 0x44, 0x43};

/* Private function prototypes -----------------------------------------------*/
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
void s4344046_start_idle();



/**
  * @brief  clean buffer read
  * @param  None
  * @retval None
  */
void s4344046_clean_buffer() {
  int i;
  int j;
  j = 0;

  for (i = 9; i < 32; i++) {

    s4344046_buffer[i] = 0x00;

    if(j < 7) {

      s4344046_get_new_buffer[j] = 0x00;
      j++;
    }
  }
}

/**
  * @brief  Write message read to buffer predefined
  * @param  buf: buffer containing the message from station
  * @retval None
  */
void s4344046_write_buffer(unsigned char* buf) {
  int i;
  int j;
  j = 0;

  for(i = 9; i < 16; i++) {

    s4344046_buffer[i] = buf[j];
    j++;
  }
}

/**
  * @brief check if packet sent when tx
  * @param  None
  * @retval 1 or 0 if packet in sent
  */
int s4344046_sent_packet() {
  return s4344046_packet_sent;
}



/**
  * @brief  Initialise	radio	(GPIO,	SPI,	etc)
  * @param  None
  * @retval None
  */
void s4344046_radio_init() {
  radio_fsm_init();
  s4344046_radio_fsmcurrentstate = S4344046_IDLE_STATE;

}

void s4344046_get_tx() {
  s4344046_activate_tx = 1;
}

void s4344046_no_tx() {
  s4344046_activate_tx = 0;
}

/**
  * @brief  Radio	FSM	processing	loop (internal	and
            external	function)	Called	from	main()	and
            mylib	files,	whenever	the	radio	FSM	must
            change	state
  * @param  None
  * @retval None
  */

void s4344046_radio_fsmprocessing() {
  int i;
  char RxChar;
  s4344046_radio_rxstatus = 0;

  switch (s4344046_radio_fsmcurrentstate) {
    case S4344046_IDLE_STATE: //IDLE	state	(used	for	reading/writing	registers)
    //debug_printf("IDLE STATE\n");
    if (radio_fsm_getstate() == S4344046_IDLE_STATE) {

      if(s4344046_rx_mode) {

        s4344046_radio_fsmcurrentstate = S4344046_RX_STATE; //Set next state as TX state.

      } else {

        s4344046_radio_fsmcurrentstate = S4344046_TX_STATE;	//Set next state as TX state.

      }

    } else {

        /* if error occurs, set state back to IDLE state */
        debug_printf("ERROR: Radio FSM not in Idle state putooo\n\r");
        radio_fsm_setstate(S4344046_IDLE_STATE);
    }

    break;

    case S4344046_RX_STATE: //Put	radio	FSM	into	receiving	mode
      /* Put radio FSM in RX state, if radio FSM is in IDLE or in waiting state */
      //debug_printf("RX STATE\n");
      if ((radio_fsm_getstate() == S4344046_IDLE_STATE) || (radio_fsm_getstate() == S4344046_WAITING_STATE)) {

        if (radio_fsm_setstate(S4344046_RX_STATE) == RADIO_FSM_ERROR) {

          debug_printf("ERROR: Cannot set Radio FSM RX state\n\r");
          HAL_Delay(100);

        } else {

          s4344046_radio_fsmcurrentstate = S4344046_WAITING_STATE;		//set next state as Waiting state
        }

      } else {

          /* if error occurs, set state back to IDLE state */
          debug_printf("ERROR: Radio FSM not in Idle state\n\r");
          radio_fsm_setstate(S4344046_IDLE_STATE);
      }

    break;

    case S4344046_TX_STATE: //Put	radio	FSM	into	transmitting	mode
    /* Put radio FSM in TX state, if radio FSM is in IDLE state */
      //debug_printf("TX STATE\n\r");
      if (radio_fsm_getstate() == S4344046_IDLE_STATE) {

        if (radio_fsm_setstate(S4344046_TX_STATE) == RADIO_FSM_ERROR) {

          debug_printf("ERROR: Cannot set Radio FSM RX state\n\r");
          HAL_Delay(100);

        } else {

          if(radio_fsm_getstate() == S4344046_TX_STATE){

            BRD_LEDToggle();
          }
          radio_fsm_setstate(S4344046_TX_STATE);
          s4344046_packet_sent = 0;
          s4344046_write_buffer(s4344046_get_new_buffer);
        }

      } else {

          /* if error occurs, set state back to IDLE state */
          debug_printf("ERROR: Radio FSM not in Idle state\n\r");
          radio_fsm_setstate(S4344046_IDLE_STATE);
      }
      break;

    case S4344046_WAITING_STATE: //Wait	for	radio	FSM	to	receive	packet
      /* Check if radio FSM is in WAITING STATE */
      //debug_printf("WAITING STATE\n");

      s4344046_next = 0;
      HAL_Delay(100);

      if (radio_fsm_getstate() == S4344046_WAITING_STATE) {

        /* Check for received packet and display  */
        if(s4344046_activate_tx){

          RxChar = debug_getc();

          if(RxChar != '\0') {

            if((s4344046_count_buffer < 7) && (RxChar != '\r')) {

              s4344046_get_new_buffer[s4344046_count_buffer] = RxChar;
              s4344046_count_buffer++;

            } else {

              s4344046_rx_mode = 0;
              s4344046_count_buffer = 0;
              radio_fsm_setstate(S4344046_IDLE_STATE);
              s4344046_radio_fsmcurrentstate = S4344046_IDLE_STATE;
              s4344046_next = 1;

            }
          }
        }

        if (radio_fsm_read(s4344046_rx_buffer) == RADIO_FSM_DONE) {
            //debug_printf("ESTADOOO: %d\n", radio_fsm_getstate());
            s4344046_radio_rxstatus = 1;
            s4344046_rx_mode = 1;
            s4344046_packet_sent = 1;
            radio_fsm_setstate(S4344046_IDLE_STATE);	//Set Radio FSM back to IDLE state.
            s4344046_radio_fsmcurrentstate = S4344046_IDLE_STATE;
        }
      }
      break;
  }
}

/**
  * @brief  Set the channel of the radio.
  * @param  chan - channel to be set.
  * @retval None
  */

void s4344046_radio_setchan(unsigned	char	chan) {
  NRF_CS_LOW();

  radio_fsm_register_write(NRF24L01P_RF_CH, &chan);

  NRF_CS_HIGH();
}

/**
  * @brief  Set	the	transmit	address	of	the	radio.
  * @param  *addr - addr used to transmt for the radio.
  * @retval None
  */
void s4344046_radio_settxaddress(unsigned	char	*addr) {
  NRF_CS_LOW();

  radio_fsm_buffer_write(NRF24L01P_TX_ADDR, addr, WRITE_VALUE);

  NRF_CS_HIGH();
}


/**
  * @brief  Get	the	channel	of	the	radio.
  * @param  None
  * @retval The channel of the radio.
  */
unsigned	char	s4344046_radio_getchan() {
  unsigned char current_channel;
  NRF_CS_LOW();

  radio_fsm_register_read(NRF24L01P_RF_CH, &current_channel);

  NRF_CS_HIGH();
  return current_channel;

}


/**
  * @brief  Get	the	transmit	address	of	the	radio
  * @param  *addr - radio address
  * @retval None
  */
void s4344046_radio_gettxaddress(unsigned	char	*addr) {
  NRF_CS_LOW();

  radio_fsm_buffer_read(NRF24L01P_TX_ADDR, addr, READ_VALUE);

  NRF_CS_HIGH();
}


/**
  * @brief  Function	to	send	a	packet
  * @param  chan - channel where to send packet
            *addr - address where the channel is set
            *txpacket - packet to be send
  * @retval None.
  */
void s4344046_radio_sendpacket(char chan, unsigned char *addr,
  unsigned char	*txpacket) {

    if (s4344046_radio_fsmcurrentstate == S4344046_TX_STATE) {
      //debug_printf("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO\n");
      s4344046_radio_settxaddress(addr);
      s4344046_radio_setchan(chan);
      radio_fsm_write(txpacket);
      s4344046_clean_buffer();
      radio_fsm_setstate(S4344046_IDLE_STATE);
      s4344046_packet_sent = 1;
      s4344046_radio_fsmcurrentstate = S4344046_IDLE_STATE;
      s4344046_next = 0;
    }
}

/**
  * @brief  Set	Radio	FSM	into IDLE state.
  * @param  None
  * @retval None
  */
void s4344046_start_idle() {
  radio_fsm_setstate(S4344046_IDLE_STATE);
  s4344046_radio_fsmcurrentstate = S4344046_IDLE_STATE;
}



/**
  * @brief  Set	Radio	FSM	into	RX	mode.
  * @param  None
  * @retval None
  */
void s4344046_radio_setfsmrx() {

  if(!(s4344046_next)) {

    s4344046_rx_mode = 1;

  } else {

    s4344046_rx_mode = 0;
  }

}


/**
  * @brief  Function	to	check	when	packet	is	received
  * @param  None
  * @retval s4344046_radio_rxstatus - status of message sent.
  */
int	s4344046_radio_getrxstatus() {
  return s4344046_radio_rxstatus;
}


/**
  * @brief  function	to	receive	a	packet,	when
            s4344046_radio_rxstatus	is	1.	Must	ONLY	be
            called	when	s4344046_radio_getrxstatus()
            ==	1. Create a coppy of the buffer to be sent.
  * @param  *rxpacket - address where the packet is.
  * @retval None
  */
void s4344046_radio_getpacket(uint8_t	*rxpacket) {
  int i;
  memcpy(s4344046_get_new_packet, rxpacket, 32);
}

/**
  * @brief  function to return the buffer read from station
  * @param  None
  * @retval buffer from station.
  */
unsigned char* s4344046_getpacket(){
  return s4344046_get_new_packet;
}

/**
  * @brief  Get the buffer that is going to be sent
  * @param  None
  * @retval s4344046_rx_buffer - the buffer that contains the message
  *          to be sent.
  */
uint8_t* s4344046_get_pointer_rx(){
  return s4344046_rx_buffer;
}

/**
  * @brief  Check if the message sent from the station is for me.
  * @param  None
  * @retval None
  */
int s4344046_student_number() {
  int i;

  for (i = 1; i <= 4; i++){

    if((s4344046_rx_buffer[i] != s4344046_student[i - 1])){

      return 0;
    }
  }
  return 1;
}

/**
  * @brief  Check if packet has been sent.
  * @param  *rxpacket - address where the packet is.
  * @retval None
  */
int s4344046_packet_status() {
  return s4344046_packet_sent;
}

/**
  * @brief  set initial buffer to send packet format
  * @param  None
  * @retval None
  */
unsigned char* s4344046_get_buffer() {
  return s4344046_buffer;
}

/**
  * @brief  Set the radio state to transmitter mode, to send a message
            first go to idle, then change it to tx state.
  * @param  None
  * @retval None
  */
void s4344046_set_tx_state() {
  radio_fsm_setstate(S4344046_IDLE_STATE);
  radio_fsm_setstate(S4344046_TX_STATE);
  s4344046_radio_fsmcurrentstate = S4344046_TX_STATE;
}


/**
  * @brief  Task that constiniously receives messages comming from radio
            NOTE - Writing to the radio is disabled.
  * @param  None
  * @retval None
  */
void s4344046_TaskRadio(void) {

  for (;;) {

		if(s4344046_packet_status()) {

      s4344046_radio_setfsmrx();
      s4344046_radio_fsmprocessing();

      if (s4344046_radio_getrxstatus() == 1) {

        s4344046_radio_getpacket(s4344046_get_pointer_rx());

        if (s4344046_QueueAcc != NULL) {	/* Check if queue exists */

          /*Send message to the front of the queue - wait atmost 10 ticks */
          if( xQueueSendToFront(s4344046_QueueRadio, ( void * ) &s4344046_get_new_packet, ( portTickType ) 10 ) != pdPASS ) {

            debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
        }
      }
    }
		vTaskDelay(0);
	}
}
