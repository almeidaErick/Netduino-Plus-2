/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"
#include "s4344046_radio.h"


//Initial Address for message
uint8_t initAddress[5] = {0x7B, 0x56, 0x34, 0x12, 0x00};

//Initial channel
#define ACTIVE_CHANNEL 50

//Array that stores the address of the station
uint8_t address[4];


void HardwareInit();
void print_communication();
void print_received_packet(unsigned char* rxpacket);

int main(void) {
  HardwareInit();
  s4344046_radio_settxaddress(&initAddress);
  s4344046_radio_setchan(ACTIVE_CHANNEL);
  s4344046_radio_gettxaddress(&address);
  print_communication();
  while(1) {
    if(s4344046_packet_status()) {
      s4344046_radio_setfsmrx();
      s4344046_radio_fsmprocessing();
      if (s4344046_radio_getrxstatus() == 1) {
        s4344046_radio_getpacket(s4344046_get_pointer_rx());
        print_received_packet(s4344046_getpacket());
      }
    } else {

      s4344046_radio_sendpacket(ACTIVE_CHANNEL, initAddress, s4344046_get_buffer());
    }
  }
}

void print_communication() {
  int print_addr;
  debug_printf("Channel: %d  ", s4344046_radio_getchan());
  debug_printf("Address: ");
  for (print_addr = 3; print_addr >= 0; print_addr--) {
    debug_printf("%x", address[print_addr]);
  }
  debug_printf("\r\n");
}

void print_received_packet(unsigned char* rxpacket) {
  int i;
  debug_printf("RECEIVED FROM RADIO: ");
  for(i = 9; i <= 15; i++) {
    debug_printf("%c", rxpacket[i]);
  }
  debug_printf("\r\n");
}



/**
  * @brief Hardware Initialisation Function.
  * @param  None
  * @retval None
  */
void HardwareInit() {

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED
  BRD_init();
  s4344046_radio_init();

  //delete this part when testing at uni.
  radio_fsm_buffer_write(NRF24L01P_RX_ADDR_P0, initAddress, 5); //To send between NP2

  HAL_Delay(5000); //delay for 5 seconds

}
