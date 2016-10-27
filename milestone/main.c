/**
  ******************************************************************************
  * @file    repo/milestone/main.c
  * @author  Erick Almeida
  * @date    18052016
  * @brief   Program accelerometer and read is position according to the
	*					 datasheet (portrait or layout) and position (x, y, z). Additionally
	*					 use radio to receive messages to get position of a marker.
  *
	*	Modified: 12-May-2016 - Timer added for to calculate the time that a task
	*													is running.
	*						13-May-2016 - Use of CLI to get hamming encode and decode working,
	*						              additionally of the calculation for CRC.
	*						15-May-2016 - Accelerometer implemented.
	*						16-May-2016 - Add radio configuration.
	*
  *
  *
  *	Functions:
	*	static void Hardware_init();
	*	void ApplicationIdleHook( void );
	*	void MANAGE_Task(void);
	*	void tim2_irqhandler (void);
	*	void READ_Task(void);
	*	uint16_t get_coordinate_x(uint8_t* radio_message, uint16_t x_coord);
	*	uint16_t get_coordinate_y(uint8_t* radio_message, uint16_t y_coord);
	*	uint16_t get_coordinate_height(uint8_t* radio_message, uint16_t height);
	*	uint16_t get_coordinate_width(uint8_t* radio_message, uint16_t width);
	*	uint16_t get_ID(uint8_t* radio_message, uint16_t id);
	*	uint8_t check_crc(uint8_t* message_radio);
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "FreeRTOS_CLI.h"
#include "s4344046_sysmon.h"
#include "s4344046_cli.h"
#include "s4344046_hamming.h"
#include "queue.h"
#include "s4344046_accelerometer.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"
#include "s4344046_radio.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;
/* Private define ------------------------------------------------------------*/
#define QUEUE_LENGTH    10
/* Binary semaphores have an effective length of 1. */
#define BINARY_SEMAPHORE_LENGTH	1

#define TOTAL_TASKS 6

/* The combined length of the two queues and binary semaphore that will be
added to the queue set. */
#define COMBINED_LENGTH ( QUEUE_LENGTH*3 + BINARY_SEMAPHORE_LENGTH*2 )


/* Private macro -------------------------------------------------------------*/
#define  BACK_SPACE   0x08
#define ACTIVE_CHANNEL 45
#define PI 3.14159265

/* Private variables ---------------------------------------------------------*/
QueueSetMemberHandle_t xActivatedMember;
QueueSetMemberHandle_t xActivatedAcc;
struct Message RecvMessage;
/*Initial Address for message*/
uint8_t initAddress[5] = {0x56, 0x34, 0x22, 0x11, 0x00};
static uint8_t message_radio[33] = {};
/* stores the address of the station*/
uint8_t address[4];
static uint16_t x_final;
static uint16_t y_final;
static uint16_t height_final;
static uint16_t width_final;
static uint16_t id_final;
volatile long count_interrupt;


/* Private function prototypes -----------------------------------------------*/
static void Hardware_init();
void ApplicationIdleHook( void ); // The idle hook is just used to stream data to the USB port.
void CLI_Task(void);
void MANAGE_Task(void);
void tim2_irqhandler (void);
void READ_Task(void);
uint16_t get_coordinate_x(uint8_t* radio_message, uint16_t x_coord);
uint16_t get_coordinate_y(uint8_t* radio_message, uint16_t y_coord);
uint16_t get_coordinate_height(uint8_t* radio_message, uint16_t height);
uint16_t get_coordinate_width(uint8_t* radio_message, uint16_t width);
uint16_t get_ID(uint8_t* radio_message, uint16_t id);
uint8_t check_crc(uint8_t* message_radio);


/* Task Priorities ------------------------------------------------------------*/
#define mainTAKETASK_PRIORITY					( tskIDLE_PRIORITY + 3 )
#define mainCLI_PRIORITY					( tskIDLE_PRIORITY + 5 )
#define mainMANAGE_PRIORITY       (tskIDLE_PRIORITY + 6)


/* Task Stack Allocations -----------------------------------------------------*/
#define mainGIVETASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainTAKETASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainCLI_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 3 )


/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

		BRD_init();
		Hardware_init();
		/*Start timer counter as 0*/
		count_interrupt = 0;

		/* Start tasks */
		xTaskCreate( (void *) &s4344046_TaskRadio, (const signed char *) "RADIO", mainTAKETASK_STACK_SIZE, NULL, mainTAKETASK_PRIORITY, NULL );
	  xTaskCreate( (void *) &CLI_Task, (const signed char *) "CLI", mainCLI_TASK_STACK_SIZE, NULL, mainCLI_PRIORITY, NULL );
	  xTaskCreate( (void *) &MANAGE_Task, (const signed char *) "MANAGE", mainCLI_TASK_STACK_SIZE, NULL, mainMANAGE_PRIORITY, NULL );
		xTaskCreate( (void *) &s4344046_TaskAcc, (const signed char *) "ACCEL.", mainCLI_TASK_STACK_SIZE, NULL, mainMANAGE_PRIORITY, NULL );
		xTaskCreate( (void *) &READ_Task, (const signed char *) "READ", mainCLI_TASK_STACK_SIZE, NULL, mainMANAGE_PRIORITY, NULL );

	  /* Register CLI commands */
		FreeRTOS_CLIRegisterCommand(&xTop);
	  FreeRTOS_CLIRegisterCommand(&xSuspend);
	  FreeRTOS_CLIRegisterCommand(&xResume);
	  FreeRTOS_CLIRegisterCommand(&xHamenc);
	  FreeRTOS_CLIRegisterCommand(&xHamdec);
		FreeRTOS_CLIRegisterCommand(&xCrc);
		FreeRTOS_CLIRegisterCommand(&xAcc);
		FreeRTOS_CLIRegisterCommand(&xTrack);

	  /*Start semaphores*/
	  s4344046_SemaphoreTop = xSemaphoreCreateBinary();
		s4344046_SemaphoreAcc = xSemaphoreCreateBinary();
		s4344046_SemaphoreAccPl = xSemaphoreCreateBinary();

		/*Start Queues*/
	  s4344046_QueueResumeTask = xQueueCreate(10, sizeof(int));		/* Create queue of length 10 Message items */
	  s4344046_QueueSuspendTask = xQueueCreate(10, sizeof(int));		/* Create queue of length 10 Message items */
		s4344046_QueueRadio = xQueueCreate(10, (sizeof(uint8_t))*32);
		s4344046_QueueAcc = xQueueCreate(10, sizeof(RecvMessage));
		s4344046_QueueAccPl = xQueueCreate(10, sizeof(RecvMessage));


	  /* Create the queue set large enough to hold an event for every space in
	    every queue and semaphore that is to be added to the set. */
	  xQueueSet = xQueueCreateSet( COMBINED_LENGTH );
		xQueueAcc = xQueueCreateSet( COMBINED_LENGTH );
		xQueueSem = xQueueCreateSet( COMBINED_LENGTH );

	  /* Check everything was created. */
	  configASSERT( xQueueSet );
	  configASSERT( s4344046_QueueResumeTask );
	  configASSERT( s4344046_QueueSuspendTask );
	  configASSERT( s4344046_SemaphoreTop );

		configASSERT( xQueueAcc );
		configASSERT( s4344046_SemaphoreAcc );
		configASSERT( s4344046_SemaphoreAccPl);

		configASSERT( xQueueSem );
		configASSERT( s4344046_QueueAccPl );
		configASSERT( s4344046_QueueAcc );
		configASSERT( s4344046_QueueRadio );

	  /* Add the queues and semaphores to the set.  Reading from these queues and
	    semaphore can only be performed after a call to xQueueSelectFromSet() has
	    returned the queue or semaphore handle from this point on. */
	  xQueueAddToSet( s4344046_QueueResumeTask, xQueueSet );
	  xQueueAddToSet( s4344046_QueueSuspendTask, xQueueSet );
	  xQueueAddToSet( s4344046_SemaphoreTop, xQueueSet );

		xQueueAddToSet( s4344046_SemaphoreAcc, xQueueAcc );
		xQueueAddToSet( s4344046_SemaphoreAccPl, xQueueAcc );

		xQueueAddToSet( s4344046_QueueAcc, xQueueSem );
		xQueueAddToSet( s4344046_QueueAccPl, xQueueSem );
		xQueueAddToSet( s4344046_QueueRadio, xQueueSem );

		/* Start the scheduler.

		NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
		The processor MUST be in supervisor mode when vTaskStartScheduler is
		called.  The demo applications included in the FreeRTOS.org download switch
		to supervisor mode prior to main being called.  If you are not using one of
		these demo application projects then ensure Supervisor mode is used here. */

		vTaskStartScheduler();

		/* We should never get here as control is now taken by the scheduler. */
	  	return 0;
}


/**
  * @brief  Read from buffer and print in the specified format.
  * @param  None
  * @retval None
  */
void READ_Task(void) {
		uint8_t orientation;

		for(;;){
				xActivatedAcc = xQueueSelectFromSet( xQueueSem, portMAX_DELAY );
				/*If buffer contains coordinates for x y and z*/
				if(xActivatedAcc == s4344046_QueueAcc) {

						if (xQueueReceive( s4344046_QueueAcc, &RecvMessage, 10)) {
								/* display received item */
								debug_printf("x: %d      y: %d      z: %d\n\r", RecvMessage.x_location,
											RecvMessage.y_location, RecvMessage.z_location);
		            	/* Toggle LED */
								BRD_LEDToggle();
		    		}
				/*If buffer contains Portrait or landscape location*/
				} else if(xActivatedAcc == s4344046_QueueAccPl) {

						if (xQueueReceive( s4344046_QueueAccPl, &orientation, 10)) {

								debug_printf("STATE: %02X\n\r", orientation);
				            	/* Toggle LED */
								if((orientation == 0x80) || (orientation == 0x00) ||
										(orientation == 0x81) || (orientation == 0x01)){

										debug_printf("PORTRAIT UP\n");

								} else if((orientation == 0x82) || (orientation == 0x02) ||
										(orientation == 0x83) || (orientation == 0x03)) {

										debug_printf("PORTRAIT DOWN\n");

								} else if((orientation == 0x84) || (orientation == 0x04) ||
										(orientation == 0x85) || (orientation == 0x05)) {

										debug_printf("LANDSCAPE RIGHT\n");

								} else if((orientation == 0x86) || (orientation == 0x06) ||
										(orientation == 0x87) || (orientation == 0x07)) {

										debug_printf("LANDSCAPE LEFT\n");

								} else {

										debug_printf("ERROR READING TRY AGAIN\n");
								}
								BRD_LEDToggle();
		    		}
				/*If buffer contains a radio message*/
				} else if (xActivatedAcc == s4344046_QueueRadio) {

					int i;

					if (xQueueReceive( s4344046_QueueRadio, &message_radio, 10)) {

							/*Print entire message received from radio*/
							/*for(i = 0; i < 32; i++) {

									debug_printf("%02x", message_radio[i]);
							}*/

							debug_printf("\n");
							/*ACTIVATE LINE TO PRINT MESSAGE*/

							/*Check if CRC is correct*/
							if(check_crc(message_radio)){

								/*Read x coordinate*/
								x_final = get_coordinate_x(message_radio, x_final);
								/*Read y coordinate*/
								y_final = get_coordinate_y(message_radio, y_final);
								/*Read height*/
								height_final = get_coordinate_height(message_radio, height_final);
								/*Read width*/
								width_final = get_coordinate_width(message_radio, width_final);
								/*Read ID*/
								id_final = get_ID(message_radio, id_final);

								/*Check for errors in coordinates*/
								if((x_final <= 319) && (y_final <= 199) && (width_final <= 320) && (height_final <= 200)) {

										double x, ret, val;
									  val = 180.0 / PI;
									  ret = atan(((float)width_final)/((float)height_final)) * val;
									  debug_printf("Angle: %.02f\n", ret);

										debug_printf("X: %d  Y: %d\n", x_final, y_final);
										debug_printf("id: %d   height: %d   width: %d\n", id_final, height_final, width_final);

								} else {

										debug_printf("BAD COORDINATES OR DIMENSIONS\n");
								}

							} else {

									debug_printf("BAD MESSAGE - CRC INCORRECT\n");
							}
						}
				}
				vTaskDelay(0);
		}
}


/**
  * @brief  Suspend or resume a task and additional print output of command TOP.
  * @param  None
  * @retval None
  */
void MANAGE_Task(void){
	  for(;;){
		    /* Block to wait for something to be available from the queues or
		        semaphore that have been added to the set.*/
		    xActivatedMember = xQueueSelectFromSet( xQueueSet,
		                                                portMAX_DELAY );

		    if(xActivatedMember == s4344046_QueueResumeTask){

			      BRD_LEDToggle();
			      volatile UBaseType_t uxArraySize;
			      TaskStatus_t pxTaskStatusArray[TOTAL_TASKS];
			      unsigned long ulTotalRunTime;
			      int i;
			      char* message;

			      if (xQueueReceive( s4344046_QueueResumeTask, &message, 10 )) {

			        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
			                                       uxArraySize,
			                                       &ulTotalRunTime );

						 /*Check all tasks name to resume it*/
			      	for(i = 0; i < (int)uxArraySize; i++){

			      		if(!(strcmp(pxTaskStatusArray[i].pcTaskName, message))){

			      			vTaskResume(pxTaskStatusArray[i].xHandle);
			            break;
			      		}
			      	}
			      }

		    } else if(xActivatedMember == s4344046_QueueSuspendTask){

			      BRD_LEDToggle();
			      volatile UBaseType_t uxArraySize;
			      TaskStatus_t pxTaskStatusArray[TOTAL_TASKS];
			      unsigned long ulTotalRunTime;
			      int i;
			      char* message;

			      if (xQueueReceive( s4344046_QueueSuspendTask, &message, 10 )) {

			        	uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
			                                       uxArraySize,
			                                       &ulTotalRunTime );

								/*Check for all tasks name to suspend it*/
				      	for(i = 0; i < (int)uxArraySize; i++){

				      			if(!(strcmp(pxTaskStatusArray[i].pcTaskName, message))){

				      					vTaskSuspend(pxTaskStatusArray[i].xHandle);
				            		break;
				      			}
				      	}
			      }

		    } else if(xActivatedMember == s4344046_SemaphoreTop) {

		      if(xSemaphoreTake( s4344046_SemaphoreTop, 0 ) == pdTRUE){

		        volatile UBaseType_t uxArraySize, x;
		        TaskStatus_t pxTaskStatusArray[6];
		        unsigned long ulTotalRunTime;
		        int i;
		        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
		                                       uxArraySize,
		                                       &ulTotalRunTime );

		        debug_printf("Name\tNumber\tPriority\tState\tRunning Time\n");
		        debug_printf("*************************************************************\n");

						/*Print all task name, number, priority and time*/
		        for(i = 0; i < (int)uxArraySize; i++) {

		          char task_buffer[40];
		          sprintf(task_buffer, "%s\t%d\t%d\t\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
		                  pxTaskStatusArray[i].uxCurrentPriority);

		          if(pxTaskStatusArray[i].eCurrentState == eBlocked){

								sprintf(task_buffer, "%s\t%d\t%d\t\t%s\t%ld\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
			                  pxTaskStatusArray[i].uxCurrentPriority, "Block", pxTaskStatusArray[i].ulRunTimeCounter);
		            PRINT_RED(task_buffer);

		          } else if(pxTaskStatusArray[i].eCurrentState == eRunning){

								sprintf(task_buffer, "%s\t%d\t%d\t\t%s\t%ld\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
			                  pxTaskStatusArray[i].uxCurrentPriority, "Running", pxTaskStatusArray[i].ulRunTimeCounter);
		            PRINT_GREEN(task_buffer);

		          } else if(pxTaskStatusArray[i].eCurrentState == eReady){

								sprintf(task_buffer, "%s\t%d\t%d\t\t%s\t%ld\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
			                  pxTaskStatusArray[i].uxCurrentPriority, "Ready", pxTaskStatusArray[i].ulRunTimeCounter);
		            PRINT_YELLOW(task_buffer);

		          } else if(pxTaskStatusArray[i].eCurrentState == eSuspended){

								sprintf(task_buffer, "%s\t%d\t%d\t\t%s\t%ld\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
			                  pxTaskStatusArray[i].uxCurrentPriority, "Suspend", pxTaskStatusArray[i].ulRunTimeCounter);
		            PRINT_BLUE(task_buffer);
		          }
		        }
		      }
		    }
		    vTaskDelay(10);
	  }
}



/**
  * @brief  CLI Receive Task.
  * @param  None
  * @retval None
  */
void CLI_Task(void) {

		char cRxedChar;
		char cInputString[100];
		int InputIndex = 0;
		char *pcOutputString;
		BaseType_t xReturned;

		/* Initialise pointer to CLI output buffer. */
		memset(cInputString, 0, sizeof(cInputString));
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		for (;;) {

			/* Receive character */
			cRxedChar = debug_getc();

	    if((int)cRxedChar != 127){
	  		/* Process if chacater if not Null */
	  		if (cRxedChar != '\0') {

	  			/* Put byte into USB buffer */
	  			debug_putc(cRxedChar);

	  			/* Process only if return is received. */
	  			if (cRxedChar == '\r') {

	  				//Put new line and transmit buffer
	  				debug_putc('\n');
	  				debug_flush();

	  				/* Put null character in command input string. */
	  				cInputString[InputIndex] = '\0';

	  				xReturned = pdTRUE;
	  				/* Process command input string. */
	  				while (xReturned != pdFALSE) {

	  					/* Returns pdFALSE, when all strings have been returned */
	  					xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

	  					/* Display CLI output string */
	  					debug_printf("%s\n\r",pcOutputString);
	  				    vTaskDelay(5);	//Must delay between debug_printfs.
	  				}
	  				memset(cInputString, 0, sizeof(cInputString));
	  				InputIndex = 0;
	  			} else {
	  				debug_flush();		//Transmit USB buffer
	  				if( cRxedChar == '\r' ) {

	  					/* Ignore the character. */
	  				} else if( cRxedChar == '\b' ) {

	  					/* Backspace was pressed.  Erase the last character in the
	  					 string - if any.*/
	  					if( InputIndex > 0 ) {
	  						InputIndex--;
	  						cInputString[ InputIndex ] = '\0';
	  					}
	  				} else {

	  					/* A character was entered.  Add it to the string
	  					   entered so far.  When a \n is entered the complete
	  					   string will be passed to the command interpreter. */
	  					if( InputIndex < 20 ) {

	  						cInputString[ InputIndex ] = cRxedChar;
	  						InputIndex++;
	  					}
	  				}
	  			}
	  		}
			/*Fix delete function*/
	    } else{

	      if( InputIndex > 0 ) {

	        InputIndex--;
	        cInputString[ InputIndex ] = '\0';
	      }
	      debug_putc('\b');
	      debug_putc(' ');
	      debug_putc('\b');
	      debug_flush();
	    }
	    vTaskDelay(50);
		}
}




/* Defined in main.c. */
void vConfigureTimerForRunTimeStats( void )
{
		unsigned short PrescalerValue;
		/* Timer 2 clock enable */
		__TIM2_CLK_ENABLE();

		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1;		//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

		/* Time base configuration */
		TIM_Init.Instance = TIM2;				//Enable Timer 2
		TIM_Init.Init.Period = 50000/1000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
		TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
		TIM_Init.Init.ClockDivision = 0;			//Set clock division
		TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
		TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

		/* Initialise Timer */
		HAL_TIM_Base_Init(&TIM_Init);

		/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
		/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
		HAL_NVIC_SetPriority(TIM2_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

		/* Enable timer update interrupt and interrupt vector for Timer  */
		NVIC_SetVector(TIM2_IRQn, (uint32_t)&tim2_irqhandler);
		NVIC_EnableIRQ(TIM2_IRQn);

		/* Start Timer */
		HAL_TIM_Base_Start_IT(&TIM_Init);
}


/**
  * @brief  Start counter for calculate the time that a task is running
  * @param  None.
  * @retval None.
  */
void tim2_irqhandler( void )
{
    /* Interrupt handler for TIM 6

    The time base for the run time stats is generated by the 16 bit timer 6.
    Each time the timer overflows ulTIM6_OverflowCount is incremented.
    Therefore, when converting the total run time to a 32 bit number, the most
    significant two bytes are given by ulTIM6_OverflowCount and the least
    significant two bytes are given by the current TIM6 counter value.  Care
    must be taken with data    consistency when combining the two in case a timer
    overflow occurs as the value is being read. */

		//Clear Update Flag
		__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);
		count_interrupt++;		//increment counter, when the interrupt occurs


}


/**
  * @brief  Calculate the y-coordinate given the buffer message and the variable
	*					where the y-coordinate is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					y_coord - variable that will store the value for the y-coordinate
  * @retval the value for the coordinate y.
  */
uint16_t get_coordinate_y(uint8_t* radio_message, uint16_t y_coord) {
		int i;
		int j = 0;
		uint8_t temp_y[4];
		uint16_t get_y_high;
		uint16_t get_y_low;
		j = 0;

		for(i = 18; i < 22; i++) {

				temp_y[j] = radio_message[i];
				j++;
		}

		get_y_high = temp_y[0];
		get_y_high = (get_y_high << 8) | temp_y[1];

		get_y_low = temp_y[2];
		get_y_low = (get_y_low << 8) | temp_y[3];

		y_coord = (s4344046_hamming_byte_decoder((get_y_high & 0xFF00) >>8, (get_y_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_y_low & 0xFF00)>>8, (get_y_low & 0x00FF))<<8);

		return y_coord;

}



/**
  * @brief  Calculate the x-coordinate given the buffer message and the variable
	*					where the x-coordinate is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					x_coord - variable that will store the value for the x-coordinate
  * @retval the value for the coordinate x.
  */
uint16_t get_coordinate_x(uint8_t* radio_message, uint16_t x_coord) {
		int i;
		int j = 0;
		uint8_t temp_x[4];
		uint16_t get_x_high;
		uint16_t get_x_low;

		for (i = 14; i < 18; i++) {

				temp_x[j] = radio_message[i];
				j++;
		}

		get_x_high = temp_x[0];
		get_x_high = (get_x_high << 8) | temp_x[1];

		get_x_low = temp_x[2];
		get_x_low = (get_x_low << 8) | temp_x[3];

		x_coord = (s4344046_hamming_byte_decoder((get_x_high & 0xFF00) >>8, (get_x_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_x_low & 0xFF00)>>8, (get_x_low & 0x00FF))<<8);
		return x_coord;
}


/**
  * @brief  Calculate the height given the buffer message and the variable
	*					where the height is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					height - variable that will store the value for the height
  * @retval the value for the height.
  */
uint16_t get_coordinate_height(uint8_t* radio_message, uint16_t height) {
		int i;
		int j = 0;
		uint8_t temp_height[4];
		uint16_t get_height_high;
		uint16_t get_height_low;

		for (i = 26; i < 30; i++) {

				temp_height[j] = radio_message[i];
				j++;
		}

		get_height_high = temp_height[0];
		get_height_high = (get_height_high << 8) | temp_height[1];

		get_height_low = temp_height[2];
		get_height_low = (get_height_low << 8) | temp_height[3];

		height = (s4344046_hamming_byte_decoder((get_height_high & 0xFF00) >>8, (get_height_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_height_low & 0xFF00)>>8, (get_height_low & 0x00FF))<<8);

	return height;
}


/**
  * @brief  Calculate the width given the buffer message and the variable
	*					where the width is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					width - variable that will store the value for the width
  * @retval the value for the coordinate width.
  */
uint16_t get_coordinate_width(uint8_t* radio_message, uint16_t width) {
		int i;
		int j = 0;
		uint8_t temp_width[4];
		uint16_t get_width_high;
		uint16_t get_width_low;

		for (i = 22; i < 26; i++) {

				temp_width[j] = radio_message[i];
				j++;
		}

		get_width_high = temp_width[0];
		get_width_high = (get_width_high << 8) | temp_width[1];

		get_width_low = temp_width[2];
		get_width_low = (get_width_low << 8) | temp_width[3];

		width = (s4344046_hamming_byte_decoder((get_width_high & 0xFF00) >>8, (get_width_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_width_low & 0xFF00)>>8, (get_width_low & 0x00FF))<<8);

		return width;
}


/**
  * @brief  Calculate the ID given the buffer message and the variable
	*					where the ID is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					id - variable that will store the value for the ID
  * @retval the value for the ID.
  */
uint16_t get_ID(uint8_t* radio_message, uint16_t id) {
		int i;
		int j = 0;
		uint8_t temp_id[4];
		uint16_t get_id_high;
		uint16_t get_id_low;

		for (i = 10; i < 14; i++) {

				temp_id[j] = radio_message[i];
				j++;
		}

		get_id_high = temp_id[0];
		get_id_high = (get_id_high << 8) | temp_id[1];

		get_id_low = temp_id[2];
		get_id_low = (get_id_low << 8) | temp_id[3];

		id = (s4344046_hamming_byte_decoder((get_id_high & 0xFF00) >>8, (get_id_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_id_low & 0xFF00)>>8, (get_id_low & 0x00FF))<<8);

		return id;
}


/**
  * @brief  Calculate the crc for the entire message (30 bytes, last 2 bytes
	*					are to check if crc is correct or not)
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
  * @retval the value for the crc calculated from message given.
  */
uint8_t check_crc(uint8_t* message_radio) {
		uint16_t output_crc = 0x0000;
		uint16_t crc_check = ((message_radio[31]<<8) | message_radio[30]);
		int i;

		for(i = 0; i < 30; i++) {

				output_crc = s4344046_crc_update(output_crc, message_radio[i]);
		}

		if(output_crc == crc_check) {

				return 1;
		} else {
				return 0;
		}
}



/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
static void Hardware_init( void ) {

		GPIO_InitTypeDef GPIO_InitStructure;

		portDISABLE_INTERRUPTS();	//Disable interrupts
		Hardware_init_Acc();

		BRD_LEDInit();				//Initialise Blue LED
		BRD_LEDOff();				//Turn off Blue LED
		vConfigureTimerForRunTimeStats();

		portENABLE_INTERRUPTS();	//Enable interrupts

		s4344046_radio_init();
		s4344046_radio_settxaddress(&initAddress);
		s4344046_radio_setchan(ACTIVE_CHANNEL);
		HAL_Delay(5000); //delay for 5 seconds
		s4344046_radio_gettxaddress(&address);
		radio_fsm_buffer_write(NRF24L01P_RX_ADDR_P0, initAddress, 5); //To send between NP2

}

/**
  * @brief  Application Tick Task.
  * @param  None
  * @retval None
  */
void vApplicationTickHook( void ) {

		BRD_LEDOff();
}

/**
  * @brief  Idle Application Task (Disabled)
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
		static portTickType xLastTx = 0;

		BRD_LEDOff();

		for (;;) {
				/* The idle hook simply prints the idle tick count */
				if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {

						xLastTx = xTaskGetTickCount();
				}
		}
}

/**
  * @brief  vApplicationStackOverflowHook
  * @param  Task Handler and Task Name
  * @retval None
  */
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {

		/* This function will get called if a task overflows its stack.   If the
		parameters are corrupt then inspect pxCurrentTCB to find which was the
		offending task. */

		BRD_LEDOff();
		( void ) pxTask;
		( void ) pcTaskName;

		for( ;; );
}
