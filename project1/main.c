/**
	************************************************************************
	* @file		Project1.c
	* @author	Erick Almeida
	* @date		19/04/2016
	* @brief	Multi‐Communications Platform
	************************************************************************
	*
	*/

/* Includes ------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4344046_lightbar.h"
#include "s4344046_joystick.h"
#include "s4344046_pantilt.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"
#include "s4344046_radio.h"
#include "s4344046_hamming.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------*/
TIM_HandleTypeDef TIM4_Init;
TIM_HandleTypeDef TIM_Init;

/* Private define ------------------------------------------------------*/
#define ACTIVE_CHANNEL 50
#define pan_tilt_velocity 1.45
#define terminal_mode 0
#define joystick_mode 1
#define laser_1k_mode 2
#define laser_2k_mode 3

/* Private macro -------------------------------------------------------*/

/* Private variables ---------------------------------------------------*/
unsigned char s4344046_buffer_error[32] = {0x20, 0x7B, 0x56, 0x34, 0x12,
		0x61, 0x04, 0x44, 0x43, 'E', 'R', 'R', 'O', 'R'};

/*Initial Address for message*/
uint8_t initAddress[5] = {0x7B, 0x56, 0x34, 0x12, 0x00};

/*Array that helps to add an error mask for a message*/
uint8_t input_check[16] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8,
		0x9, 0xA,0xB, 0xC, 0xD, 0xE, 0xF};

/* stores the address of the station*/
uint8_t address[4];
/*Stores all the bits to send in Manchester mode*/
uint8_t manchester_send[44];

/*Stores the times that the interrupt for laser receive*/
uint16_t manchester_read_time[44];

/*Stores the value of 1s and 0s when the interrupt from laser occurs.*/
uint8_t manchester_read_values[44];

/*Stores 1s and 0s for the information read from laser (Manchester form)*/
uint8_t manchester_final[44];

/*Stores 1s and 0s for the information read from laser (Hamming mode)*/
uint8_t hamming_rec[22];

/*Character read from terminal*/
uint8_t RxByte;

/*Hamming Byte encoder*/
uint16_t CodedWord;

int write_value = 0; /*Value to store 1 and 0s to decode hamming receive*/
int read_counter = 0; /*Stores the number of bits read.*/
/*Stores the number of bits added in Manchester Reconstructed*/
int man_counter;
/*Stores the character read to resend if an error is detected*/
char char_read;
/*Stores the state of the laser (if the laser read something)*/
int flag_read = 0;
/* Set velocity for laser 1kbit in Timer 4.*/
int laser_first_help = 0;
uint16_t external_interrupt = 1;/*Helps to fix switch debouncing effect*/
uint16_t rise_pin = 2; /*Check if switch has been pressed.*/
char pos_char;

/*Initial Position of joystick in x and y axis.*/
int first_pos_joystick, first_pos_joystick2;

/*Set state (joystick, terminal, laser mode)*/
int controller_mode = terminal_mode;

/*Separate lasers of 1kbit and 2kbit velocity*/
int first_laser = 0, second_laser = 0;

int received_byte = 0;/*Check if laser received something*/
char RxChar; /*Read characters to send using Radio*/
int bit_count = 0; /*Count number of bits read from laser*/
int activate_input = 1; /*Flasg set when an error mask is added*/
uint8_t get_error; /*Stores the error mask*/

/* Private function prototypes -----------------------------------------*/
void Hardware_init();
void exti_pb_interrupt_handler(void);
void timer4_interrupt (void);
void print_received_packet(unsigned char* rxpacket);
void tim3_irqhandler (void);
void reconstruct_manchester(uint8_t* values_read, uint16_t* times_read);
void reconstruct_hamming(uint8_t* end_manch);
void print_communication();
void control_state(int type);
void move_joystick();
void check_input_error(char hex);
int check_if_error(unsigned char* rxpacket);


/**
	* @brief	Main program
	* @param	None
	* @retval None
	*/
void main(void) {
		int print_joystick = 1;
		uint32_t time_joystick;
		BRD_init();	//Initalise NP2
		Hardware_init();	//Initalise hardware modules

		// save initial position of joystick
		first_pos_joystick = s4344046_joystick_y_read();
		first_pos_joystick2 = s4344046_joystick_x_read();

	  s4344046_radio_init();
	  HAL_Delay(5000); //delay for 5 seconds
	  s4344046_radio_settxaddress(&initAddress);
	  s4344046_radio_setchan(ACTIVE_CHANNEL);
	  s4344046_radio_gettxaddress(&address);
	  s4344046_no_tx();

		//To send between NP2
		radio_fsm_buffer_write(NRF24L01P_RX_ADDR_P0, initAddress, 5);
		HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 1);

		time_joystick = HAL_GetTick();
		/*
			Controle state for each case, Joystick, Terminal and Laser with both
			velocity of transmision.
		*/
		while (1) {
		    if(print_joystick){
						if(((HAL_GetTick() - time_joystick) > 1500000)) {
								print_joystick = 0;
									debug_printf("Pan: %d  Tilt: %d\n\r", s4344046_angle_x(),
										s4344046_angle_y());
			    	}

				} else {
							time_joystick = HAL_GetTick();
							print_joystick = 1;
				}


		    control_state(controller_mode);

				/*
					Control switch debouncing (times pressed and times read)
				*/
				if((rise_pin % 2 == 0) && (external_interrupt % 2 == 0)) {
							external_interrupt++;
				}

				/*
					Check if laser receiver has detected something, if so read all
					bits delay 50ms (max time to read all bits) and execute
					manchester and hamming transformation
				*/
				if(flag_read) {
							flag_read = 0;
							HAL_Delay(50);
							reconstruct_manchester(manchester_read_values,
								manchester_read_time);
							reconstruct_hamming(manchester_final);
							write_value = 0;
							read_counter = 0;
				}
				BRD_LEDToggle();	//Toggle 'Alive' LED on/off
		}
}


/**
	* @brief	Print channel and adddress for radio communication
	* @param	None
	* @retval None
	*/
void print_communication() {
	  int print_addr;
	  debug_printf("Channel: %d  ", s4344046_radio_getchan());
	  debug_printf("Address: ");

		/*
			Interact into address hex values and print them one by one (as hex).
		*/
	  for (print_addr = 3; print_addr >= 0; print_addr--) {
	    	debug_printf("%x", address[print_addr]);
	  }
	  debug_printf("\r\n");
	  debug_printf("Channel: %d  ", s4344046_radio_getchan());
}


/**
	* @brief	Configure the hardware,
	* @param	None
	* @retval None
	*/
void Hardware_init(void) {

		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_IC_InitTypeDef  TIM_ICInitStructure;

		uint16_t PrescalerValue = 0;

		BRD_LEDInit();		/*Initialise Blue LED*/
		BRD_LEDOff();		/*Turn off Blue LED*/

		/* Timer 4 clock enable */
		__TIM4_CLK_ENABLE();

		/* Timer 3 clock enable */
		__TIM3_CLK_ENABLE();

		/* Enable A2 GPIO Clock */
		__BRD_A2_GPIO_CLK();

	  /* Enable the D0 & D1 Clock */
	  __BRD_D0_GPIO_CLK();

		__BRD_D1_GPIO_CLK();

		s4344046_joystick_init(); //Initialise joystick
		//Initialise pantilt
		s4344046_pantilt_init(pan_tilt_velocity, pan_tilt_velocity);

		/* Compute the prescaler value.
				SystemCoreClock = 168000000 - set for 50Khz clock */
		PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 50000) - 1;

		/* Configure A2 interrupt for Prac 1, Task 2 or 3 only */

		//Set Main priority ot 8 and sub-priority ot 0
		HAL_NVIC_SetPriority(BRD_PB_EXTI_IRQ, 8, 0);

		/* Configure the D0 pin with TIM3 input capture */
		GPIO_InitStructure.Pin = BRD_D0_PIN;				/*Pin D0*/
		/*Set mode to be alternate function*/
		GPIO_InitStructure.Mode =GPIO_MODE_AF_PP;
		/*Enable Pull up, down or no pull resister*/
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			/*Pin latency*/
		/*Set alternate function to be timer 2*/
		GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
		/*Initialise Pin D0*/
		HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);

		/* Configure the D1 pin as an input */
		GPIO_InitStructure.Pin = BRD_D1_PIN;				/*Pin D1*/
	  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;			/*Output Mode*/
		/*Enable Pull up, down or no pull resister*/
	  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			/*Pin latency*/
		/*Initialise Pin D1*/
	  HAL_GPIO_Init(BRD_D1_GPIO_PORT, &GPIO_InitStructure);

		/*Enable external GPIO interrupt and interrupt vector for pin A2*/
		/* Configure A2 pin as pull down input */
		NVIC_SetVector(BRD_PB_EXTI_IRQ, (uint32_t)&exti_pb_interrupt_handler);
		NVIC_EnableIRQ(BRD_PB_EXTI_IRQ);
		GPIO_InitStructure.Pin = BRD_PB_PIN;				/*Pin A2*/
		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;		/*interrupt Mode*/
		/*Enable Pull up, down or no pull resister*/
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			/*Pin latency*/
		/*Initialise Pin*/
		HAL_GPIO_Init(BRD_PB_GPIO_PORT, &GPIO_InitStructure);


		/* Time base configuration */
		TIM4_Init.Instance = TIM4;				/*Enable Timer 3*/
		/*Set period count to be 0.25s, so timer interrupt occurs
			every 0.25s*/
		TIM4_Init.Init.Period = (5*50000)/20000;
		TIM4_Init.Init.Prescaler = PrescalerValue;	/*Set presale value*/
		TIM4_Init.Init.ClockDivision = 0;			/*Set clock division*/
		TIM4_Init.Init.RepetitionCounter = 0;	/*Set Reload Value*/
		/*Set timer to count up.*/
		TIM4_Init.Init.CounterMode = TIM_COUNTERMODE_UP;


		/* Configure Timer 3 settings */
		TIM_Init.Instance = TIM3;					/*Enable Timer 3*/
	  TIM_Init.Init.Period = 2*50000;			/*Set for 100ms (10Hz) period*/
	  TIM_Init.Init.Prescaler = PrescalerValue;	/*Set presale value*/
	  TIM_Init.Init.ClockDivision = 0;			/*Set clock division*/
		TIM_Init.Init.RepetitionCounter = 0; 		/* Set Reload Value*/
		/*Set timer to count up.*/
	  TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;

		/* Configure TIM3 Input capture */

		/*Set to trigger on rising and falling edge*/
	  TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
	  TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
	  TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
	  TIM_ICInitStructure.ICFilter = 0;

		/* Initialise Timer 4 */
		HAL_TIM_Base_Init(&TIM4_Init);

		/*Set Main priority ot 10 and sub-priority ot 0.*/
		HAL_NVIC_SetPriority(TIM4_IRQn, 10, 0);
		/* Enable timer update interrupt and interrupt vector for Timer  */
		NVIC_SetVector(TIM4_IRQn, (uint16_t)&timer4_interrupt);
		NVIC_EnableIRQ(TIM4_IRQn);

		/* Start Timer */
		HAL_TIM_Base_Start_IT(&TIM4_Init);


		/* Set priority of Timer 3 Interrupt
			[0 (HIGH priority) to 15(LOW priority)] */

		/*Set Main priority ot 10 and sub-priority ot 0.*/
		HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);

		/*Enable Timer 3 interrupt and interrupt vector*/
		NVIC_SetVector(TIM3_IRQn, (uint32_t)&tim3_irqhandler);
		NVIC_EnableIRQ(TIM3_IRQn);

		/* Enable input capture for Timer 3, channel 2 */
		HAL_TIM_IC_Init(&TIM_Init);
		HAL_TIM_IC_ConfigChannel(&TIM_Init, &TIM_ICInitStructure,
			TIM_CHANNEL_2);

		/* Start Input Capture */
		HAL_TIM_IC_Start_IT(&TIM_Init, TIM_CHANNEL_2);
}


/**
	* @brief	Control the sate of the program, switch between cases
	*					according of the properties we want to use
	* @param	type - type of case to switch.
	* @retval None
	*/
void control_state(int type) {
  	switch(type) {
    case terminal_mode:
	      RxChar = debug_getc();

				/*
					If character send by the user is Null (no character typed),
					then avoid it and wait for a character different from null.
				*/
      	if (RxChar != '\0') {
						/*reflect byte using printf - must delay before calling
							printf again.*/
        		debug_printf("%c\n\r", RxChar);
						/*
							Check the character read and move pan/tilt according the
							direction required.
						*/
		  			if((RxChar == 'a') || (RxChar == 'A')) {
		          	s4344046_pantilt_angle_write(0, -1);
		        } else if((RxChar == 's') || (RxChar == 'S')) {
		          	s4344046_pantilt_angle_write(1, -1);
		        } else if((RxChar == 'd') || (RxChar == 'D')) {
		          	s4344046_pantilt_angle_write(0, 1);
		        } else if((RxChar == 'w') || (RxChar == 'W')) {
		          	s4344046_pantilt_angle_write(1, 1);
		        } else if(RxChar == 'l') {

								//Change to laser mode(1kbit velocity)
								HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 0);
			          debug_printf("*****Laser Mode (1K bits/second)*****\n\r");
								read_counter = 0;
			          controller_mode = laser_1k_mode;
			          first_laser = 1;
			          second_laser = 0;
								flag_read = 0;
			          s4344046_no_tx();
		        } else if(RxChar == 'L') {

						//change to laser mode(2kbit velocity)
						HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 0);
	          debug_printf("*******Laser Mode (2K bits/second)*******\n\r");
						read_counter = 0;
	          controller_mode = laser_2k_mode;
	          first_laser = 0;
	          second_laser = 1;
						flag_read = 0;
	          s4344046_no_tx();
	        }
	      }

    break;

    case joystick_mode:
        move_joystick(); //Use joystick to move pan/tilt
    break;

    case laser_1k_mode:
      	RxByte = debug_getc();
      	/* Check if character is not Null */
				if (RxByte != '\0') {

						/* If character is '!', then analize if the values added
							after can be used as an error mask */
						if ((RxByte == '!')) {
								bit_count = 0;
								activate_input = 0;
								get_error = 0x00;
								break;
						}

						/* If a '+' character is pressed in a 1kbit/second velocity,
						then increase the velocity up to 2kbit/second */
						if(RxByte == '+') {
								controller_mode = laser_2k_mode;
								HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 0);
			          debug_printf("*****Laser Mode (2K bits/second)*****\n\r");
								read_counter = 0;
			          first_laser = 0;
			          second_laser = 1;
								flag_read = 0;
			          s4344046_no_tx();
								break;
						}

						/* Check if laser is ready to send a character, if so the
							send it by encoding it first */
						if (activate_input) {
				        /* Hamming encode received character */
								bit_count = 0;
								char_read = RxByte;
				        CodedWord =
										s4344046_hamming_byte_encoder(RxByte, get_error);
				        memcpy(manchester_send,
										s4344046_get_manchester_encoding(), 44);
				        received_byte = 1;
						} else {
								check_input_error(RxByte);
						}
      	}
    break;

    case laser_2k_mode:
      	RxByte = debug_getc();
      	/* Check if character is not Null */
      	if (RxByte != '\0') {

						/* If character is '!', then analize if the values added
							after can be used as an error mask */
						if ((RxByte == '!') && (bit_count < 2)) {
								bit_count = 0;
								activate_input = 0;
								get_error = 0x00;
								break;
						}

						/* If a '-' character is pressed in a 2kbit/second velocity,
						then decrease the velocity down to 1kbit/second */
						if(RxByte == '-') {
								controller_mode = laser_1k_mode;
								HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 0);
			          debug_printf("*****Laser Mode (1K bits/second)*****\n\r");
								read_counter = 0;
			          first_laser = 1;
			          second_laser = 0;
								flag_read = 0;
			          s4344046_no_tx();
						}

						/* Check if laser is ready to send a character, if so the
							send it by encoding it first */
						if (activate_input) {
								bit_count = 0;
				        /* Hamming encode received character */
								char_read = RxByte;
				        CodedWord =
										s4344046_hamming_byte_encoder(RxByte, get_error);
				        memcpy(manchester_send,
										s4344046_get_manchester_encoding(), 44);
				        received_byte = 1;
						} else {
								check_input_error(RxByte);
						}
      	}
    break;
  	}
		/*Check packet status in radio communication*/
  	if(s4344046_packet_status()) {
    		s4344046_radio_setfsmrx();
    		s4344046_radio_fsmprocessing();

				/*If a packet is received then print the message, otherwise send a
				message in radio is in Transmit mode*/
    		if (s4344046_radio_getrxstatus() == 1) {
      			s4344046_radio_getpacket(s4344046_get_pointer_rx());
      			print_received_packet(s4344046_getpacket());
    		}
  	} else {
				s4344046_radio_sendpacket(ACTIVE_CHANNEL, initAddress,
				s4344046_get_buffer());
  	}
}

/**
	* @brief	Actions taken when the push button from NP2 is pressed,
						switch between joystick and terminal mode.
	* @param	None
	* @retval None
	*/
void exti_pb_interrupt_handler(void) {
		//press_counter_val variable used to modify clock speed

		/*if the program is in laser mode (both velocities), then change it to
			Joystick mode*/
	  if((controller_mode == laser_1k_mode) ||
				(controller_mode == laser_2k_mode)) {
	    	controller_mode = terminal_mode;
	  }

		/*Check and control switch debouncing*/
		if(external_interrupt % 2 != 0){
				external_interrupt++;
				/*Switch between modes when switch is pressed, if program is in
					Terminal mode, then change it to Joystick mode, if program is
					in joystick mode then change it to terminal mode*/
		    if(controller_mode == joystick_mode) {
						/*Turn laser on for calibration*/
						HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 1);
			      debug_printf("***************Terminal Mode*************\n\r");
			      s4344046_no_tx();
			      controller_mode = terminal_mode;
    		} else if (controller_mode == terminal_mode){
						int w;
						/*Turn laser on for calibration*/
						HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 1);
			      debug_printf("***************Joystick Mode*************\n\r");
			      s4344046_get_tx();
			      controller_mode = joystick_mode;
    		}
				rise_pin++;
			} else if ((external_interrupt % 2 == 0) && (rise_pin % 2 != 0)){
					rise_pin++;
			}
		/*Clear A2 pin external interrupt flag*/
		HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);
}


/**
	* @brief	Move pan\tilt using the joystick, (x-axis move left and right)
						(y-axis move up and down)
	* @param	None
	* @retval None
	*/
void move_joystick() {
  	if(controller_mode) {
  			int y_pos;
  			int x_pos;
  			y_pos = s4344046_joystick_y_read(); /* read joystick position*/
  			x_pos = s4344046_joystick_x_read(); /* read joystick position*/
				/*Check is joystick has changed its position, if so, move the
					pan/tilt to a specific direction ccording hwere the joystick
					has moved*/
		  	if((y_pos != first_pos_joystick) ||
						(x_pos != first_pos_joystick2)) {
						/*If joystick has move horizaontally or vertically, move 1
							degree according the direction (left, right, up, down) */
			  		if(y_pos < first_pos_joystick) {/*if joystick is on the left*/
			  				s4344046_pantilt_angle_write(1, -1);
			  		}
						if (y_pos > first_pos_joystick) {
							/*if joystick is on the right*/
			  				s4344046_pantilt_angle_write(1, 1);
			  		}
						if (x_pos < first_pos_joystick2) {
			  				s4344046_pantilt_angle_write(0, -1);
			  		}
						if (x_pos > first_pos_joystick2) {
			  				s4344046_pantilt_angle_write(0, 1);
  					}
  			}
  	}
}


/**
	* @brief	Timer 4 interrupt used to send bit by bit through laser.
	* @param	None
	* @retval None
	*/
void timer4_interrupt (void) {
		/*Clear Update Flag*/
		__HAL_TIM_CLEAR_IT(&TIM4_Init, TIM_IT_UPDATE);

		/*Check mode of the program, Laser 1kbit/second or Laser 2kbit/second
			and send bits at different speed according to the mode, originally
			the clock is set to send at 2kbit/second, but if the mode is 1kbit/
			second the the interrupt must be activate twice to send a bit*/
	  if((controller_mode == laser_2k_mode) && (received_byte == 1) &&
				(second_laser == 1)) {

				//Write 0 or 1 according to the manchester_send array content
		    HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN,
					manchester_send[++bit_count]);

				/*If 44 bits have been sent, then stop laser communication*/
		    if(bit_count == 44){
			      received_byte = 0;
			      bit_count = 0;
    		}
  	} else if((controller_mode == laser_1k_mode) && (received_byte == 1)
				&& (second_laser == 0)) {
      	laser_first_help++;
      	if(laser_first_help % 2 == 0) {
						/*Write 0 or 1 according to manchester_send array content*/
		        HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN,
							manchester_send[++bit_count]);
		        laser_first_help = 0;

						/*If 44 bits have been sent, then stop laser communication*/
		        if(bit_count == 44){
		          	received_byte = 0;
		          	bit_count = 0;
        		}
      	}
  	}
}


/**
	* @brief	Timer 3 interrupt handler, used to read bit using the laser
						receiver, when the interrupt triggers save the time
						when the interrupt occurs to use it later in Manchester
						decoding.
	* @param	None
	* @retval None
	*/
void tim3_irqhandler (void) {
		unsigned int input_capture_value;

		//Clear Input Capture Flag
		__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_TRIGGER);

		//Save the times when the interrupt occurs in an array
		manchester_read_time[read_counter] =
							__HAL_TIM_GetCompare(&TIM_Init, TIM_CHANNEL_2);

		/*Only add the counter for the timers saved when the mode of the
			program is Laser at 1kbit/second or 2kbit/second*/
		if((controller_mode == laser_2k_mode) ||
				(controller_mode == laser_1k_mode)) {
				flag_read = 1;
				write_value = ~write_value;
				manchester_read_values[read_counter] = write_value & 0x01;
				read_counter += 1;
		}
}


/**
	* @brief	Reconstruct the message received in Manchester mode, using the
						times from Timer 4 and the value triggered in Tmer 4.

						If laser in 1kbit/second velocity, then check the difference
						in times to be 25ms when just 1 bit has been sent (1 or 0),
						or 50ms when the bit is repeated (11 or 00).

						if laser in 2kbit/second velocity, then check the difference
						in times to be 12.5ms when just 1 bit has been sent (1 or 0),
						or 25ms when the bit is repeated (11 or 00).
	* @param	values_read - 1's or 0's  saved when the variable trigger each
						time the interrupt occurs.

						times_read - the times when the interrupt occurs.
	* @retval None
	*/
void reconstruct_manchester(uint8_t* values_read, uint16_t* times_read) {
		int i;
		man_counter = 0;
		int diff;
		manchester_final[man_counter] = 0x00;
		man_counter++;

		/*Check the program laser velocity to set limits for the difference
			in interrupts times*/
		if(controller_mode == laser_1k_mode){

				/*Get the difference of times when the interrupt occurs*/
				for(i = 0; i < (read_counter -1); i++) {
						diff = times_read[i + 1] - times_read[i];

						/*Desired value is 25 (return by LA), but in case the times
							are disturbed add a small difference to the limits
							(from 19 to 28), same to 50 from (44 to 56)*/
						if((diff <= 28) && (diff >= 19)) {
								manchester_final[man_counter] = values_read[i];
								man_counter++;
						} else if ((diff <= 56) && (diff >= 44)) {
								manchester_final[man_counter] = values_read[i];
								man_counter++;
								manchester_final[man_counter] = values_read[i];
								man_counter++;
						}
				}
		} else if (controller_mode == laser_2k_mode) {

				/*Get the difference of times when the interrupt occurs*/
				for(i = 0; i < (read_counter -1); i++) {
						diff = times_read[i + 1] - times_read[i];

						/*Desired value is 12 (return by LA), but in case the times
							are disturbed add a small difference to the limits
							(from 17 to 8), same to 25 from (12 to 31)*/
						if((diff <= 17) && (diff >= 8)) {
								manchester_final[man_counter] = values_read[i];
								man_counter++;
						} else if ((diff <= 30) && (diff >= 21)) {
								manchester_final[man_counter] = values_read[i];
								man_counter++;
								manchester_final[man_counter] = values_read[i];
								man_counter++;
						}
				}
		}
		man_counter = 43;

		//Add a 0 at the end when of the values read to show the stop bit.
		manchester_final[man_counter] = 0;
		read_counter = 0;
}


/**
	* @brief	Reaconstruct Hamming message using The Manchester
						Reconstructed message, if in manchester message a 0 and after
						a 1 is shown (rising edge), then the hamming value is 1,
						furthermore is a 1 and after a 0 is shown (falling edge),
						then the hamming value is 0.
	* @param	end_manch - The message received in Manchester format.
	* @retval None
	*/
void reconstruct_hamming(uint8_t* end_manch){
		int i, j, z;
		j = 0;
		uint8_t lower = 0;
		uint8_t high = 0;
		uint8_t d0, d1, d2, d3;
		uint8_t p0 = 0, h0, h1, h2;
		uint8_t start1, start2, stop;
		uint8_t final;

		/*Check each bit from the message in Manchester format, check if it
			has a rising edge or falling edge*/
		for(i = 0; i < 44; i++) {

				/*If has a start bit at 0, then check if the next bit is 1 or 0*/
				if (end_manch[i] == 0) {
						i++;

						/*If bit is 1, then it is a rising edge, for instance the
							hamming value is 1*/
						if(end_manch[i] == 1) {
								hamming_rec[j] = 1;
								j++;
						}
				} else if(end_manch[i] == 1) {
						i++;

						/*If the start bit is 1 and the next bit is 0, the it is a
							falling edge, for instance the hamming value is 0*/
						if(end_manch[i] == 0) {
								hamming_rec[j] = 0;
								j++;
						}
				}
		}

		/*Extract each bit from the message in hamming format*/
		for(z = 0; z < 2; z++) {
				/*Check for a lower significant byte and a high significant byte*/
				if (z == 0) {
						start1 = hamming_rec[0];
						start2 = hamming_rec[1];
						p0 = hamming_rec[2];
						h0 = hamming_rec[3];
						h1 = hamming_rec[4];
						h2 = hamming_rec[5];
						d0 = hamming_rec[6];
						d1 = hamming_rec[7];
						d2 = hamming_rec[8];
						d3 = hamming_rec[9];
						stop = hamming_rec[10];
				} else {
						start1 = hamming_rec[11];
						start2 = hamming_rec[12];
						p0 = hamming_rec[13];
						h0 = hamming_rec[14];
						h1 = hamming_rec[15];
						h2 = hamming_rec[16];
						d0 = hamming_rec[17];
						d1 = hamming_rec[18];
						d2 = hamming_rec[19];
						d3 = hamming_rec[20];
						stop = hamming_rec[22];
				}

				/*Check if the hamming message lower/higher significant byte
				 	 has a start bits (11) and a stop bit (0), if no then send an
					 error message through radio
					*/
				if ((start1 != 1) || (start2 != 1) || (stop != 0)) {
						s4344046_set_tx_state();
						s4344046_radio_sendpacket(ACTIVE_CHANNEL, initAddress,
								s4344046_buffer_error);
						return;
				} else {
						if (z == 0) {
								lower |= (p0 << 7) | (h0 << 6) | (h1 << 5) | (h2 << 4) |
										(d0 << 3) | (d1 << 2) | (d2 << 1) | (d3 << 0);
						} else {
								high |= (p0 << 7) | (h0 << 6) | (h1 << 5) | (h2 << 4) |
										(d0 << 3) | (d1 << 2) | (d2 << 1) | (d3 << 0);
						}
				}
		}
		//Get state of hamming decoding
		final = s4344046_hamming_byte_decoder(lower, high);

		/* If the lower and high significant bytes can not be decoded
			(too many errors), the return a 0xff value, and send an error
			message through radio
			*/
		if(final == 0xff) {
			s4344046_set_tx_state();
			s4344046_radio_sendpacket(ACTIVE_CHANNEL, initAddress,
					s4344046_buffer_error);
			return;
		}

		//Print format of received message.
		debug_printf("RECEIVED FROM LASER: %c - Raw: %04x (ErrMask %04x)\n\r",
				final, s4344046_get_raw_input(), s4344046_get_error_mask());
}


/**
	* @brief	Check for values to be added as an error mask, correct values
	*					allowed are number from 0 to 9 and letters from a to f
	* @param	hex - the hex value of the character given by the user
	* @retval None
	*/
void check_input_error(char hex) {
		int read;

		/* If character is a number/letter, then add the number/letter
			 into the hex mask
			*/
		if ((hex <= '9') && (hex >= '0')) {
				read = hex - '0';
		} else {
				read = hex - 87;
		}

		/*Check if the error mask have enough values, if so then go back to
			the normal process of sending and receving messages from radio or
			laser
			*/
		if (bit_count == 0) {
				get_error |= (input_check[read] << 4);
				debug_printf("!");
				debug_printf("%x", input_check[read]);
		} else {
				get_error |= input_check[read];
				debug_printf("%x\n\r", input_check[read]);
				activate_input = 1;
		}
		bit_count++;
}


/**
	* @brief	Print the received message according to the format required,
						additionally check if the message received is "ERROR", if so
						send the message again clearing the error mask (set error mask
						to 0x00)
	* @param	rxpacket - the entire buffer received from radio
	* @retval None
	*/
void print_received_packet(unsigned char* rxpacket) {
  	int i;

		/*Print the message received, either the letter or the error message*/
		if(check_if_error(rxpacket) == 0) {
			  debug_printf("RECEIVED FROM RADIO: ");
			  for(i = 9; i <= 15; i++) {
	    			debug_printf("%c", rxpacket[i]);
	  		}
	  		debug_printf("\r\n");
		} else {
				debug_printf("RECEIVED FROM RADIO: ");

				/*Print the message (ERROR) and send the message again*/
				for(i = 9; i <= 15; i++) {
						debug_printf("%c", rxpacket[i]);
				}
				debug_printf("\r\n");
				CodedWord = s4344046_hamming_byte_encoder(char_read, 0x00);
				memcpy(manchester_send, s4344046_get_manchester_encoding(), 44);
				received_byte = 1;
		}
}


/**
	* @brief	Check is the message receive is the error message, if so
						return 1, 0 otherwise.
	* @param	rxpacket - enture buffer to check for the message.
	* @retval return 1 if the message is "ERROR", return o otherwise.
	*/
int check_if_error(unsigned char* rxpacket) {

		/*Check letter by letter is the message is "ERROR"*/
		if((rxpacket[9] == 'E') && (rxpacket[10] == 'R') &&
				(rxpacket[11] == 'R') && (rxpacket[12] == 'O') &&
				(rxpacket[13] == 'R')) {
				return 1;
			}
		return 0;
}
