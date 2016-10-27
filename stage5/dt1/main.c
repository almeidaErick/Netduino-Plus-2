/**
  ******************************************************************************
  * @file    ex14_fr_ledflashing/main.c
  * @author  MDS
  * @date    04022015
  * @brief   FreeRTOS LED Flashing program.Creates a task to flash the onboard
  *			 Blue LED. Note the Idle task will also flash the Blue LED.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "s4344046_sysmon.h"

/* Task Stack Allocations -----------------------------------------------------*/
#define TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
void Hardware_init();
void CHANNEL0_Task( void );
void CHANNEL1_Task( void );
void CHANNEL2_Task( void );

/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();

	/* Start the task to flash the LED. */
    xTaskCreate( (void *) &CHANNEL0_Task, (const signed char *) "Task1", TASK_STACK_SIZE, NULL, 2, NULL );
		xTaskCreate( (void *) &CHANNEL1_Task, (const signed char *) "Task2", TASK_STACK_SIZE, NULL, 2, NULL );
    xTaskCreate( (void *) &CHANNEL2_Task, (const signed char *) "Task3", TASK_STACK_SIZE, NULL, 1, NULL );

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
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
void Hardware_init( void ) {

	portDISABLE_INTERRUPTS();	//Disable interrupts

	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED
  s4344046_sysmon_init();

	portENABLE_INTERRUPTS();	//Enable interrupts

}

void CHANNEL0_Task( void ) {
//	BRD_LEDOff();
  S4344046_LA_CHAN0_CLR(); //Clear LA Channel 0
	for (;;) {
    S4344046_LA_CHAN0_SET(); //Set LA Channel 0
    BRD_LEDToggle(); //Random instruction
    vTaskDelay(3); //Extra Delay for 3ms
    S4344046_LA_CHAN0_CLR(); //Clear LA Channel 0
    vTaskDelay(1); //Mandatory delay ONLY for
	}
}

void CHANNEL1_Task( void ) {
//	BRD_LEDOff();
  S4344046_LA_CHAN1_CLR(); //Clear LA Channel 0
	for (;;) {
    S4344046_LA_CHAN1_SET(); //Set LA Channel 0
    BRD_LEDToggle(); //Random instruction
    vTaskDelay(3); //Extra Delay for 3ms
    S4344046_LA_CHAN1_CLR(); //Clear LA Channel 0
    vTaskDelay(1); //Mandatory delay ONLY for
	}
}

void CHANNEL2_Task( void ) {
//	BRD_LEDOff();
  S4344046_LA_CHAN2_CLR(); //Clear LA Channel 0
	for (;;) {
    S4344046_LA_CHAN2_SET(); //Set LA Channel 0
    BRD_LEDToggle(); //Random instruction
    //vTaskDelay(1); //Extra Delay for 3ms
    S4344046_LA_CHAN2_CLR(); //Clear LA Channel 0
    //vTaskDelay(1); //Mandatory delay ONLY for
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
