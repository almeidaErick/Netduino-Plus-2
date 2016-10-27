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
#include "semphr.h"
#include "s4344046_lightbar.h"
#include "s4344046_sysmon.h"

/* Task Stack Allocations -----------------------------------------------------*/
#define TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
void Hardware_init();
void TaskTimerLeft( void );
void TaskTimerRight( void );
void TaskTimerControl( void );
void exti_pb_irqhandler(void);
void semaphore_check(void);


/*struct dualtimer_msg {
  char type; //type is either ‘l’ or ‘r’
  uint8_t timer_value;
};*/

QueueHandle_t MessageQueue;	/* Queue used */
SemaphoreHandle_t PBSemaphore;	/* Semaphore for pushbutton interrupt */
uint8_t mode = 1;



/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

	BRD_init();
	Hardware_init();
  PBSemaphore = xSemaphoreCreateBinary();

  /* Start the task to flash the LED. */
  xTaskCreate( (void *) &TaskTimerLeft, (const signed char *) "Left", TASK_STACK_SIZE, NULL, 2, NULL );
  xTaskCreate( (void *) &TaskTimerRight, (const signed char *) "Right", TASK_STACK_SIZE, NULL, 2, NULL );
  xTaskCreate( (void *) &s4344046_TaskLightBar, (const signed char *) "Control", TASK_STACK_SIZE, NULL, 2, NULL );
  xTaskCreate( (void *) &semaphore_check, (const signed char *) "SEMAPHORE", TASK_STACK_SIZE, NULL, 3, NULL );

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
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable PB clock */
  __BRD_PB_GPIO_CLK();
	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED
  s4344046_lightbar_init();
  s4344046_sysmon_init();
  /* Set priority of PB Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	HAL_NVIC_SetPriority(BRD_PB_EXTI_IRQ, 4, 0);	//Set Main priority ot 10 and sub-priority ot 0.

	//Enable PB interrupt and interrupt vector for pin DO
	NVIC_SetVector(BRD_PB_EXTI_IRQ, (uint32_t)&exti_pb_irqhandler);
	NVIC_EnableIRQ(BRD_PB_EXTI_IRQ);

  	/* Configure PB pin as pull down input */
	GPIO_InitStructure.Pin = BRD_PB_PIN;				//Pin
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;		//interrupt Mode
  GPIO_InitStructure.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  HAL_GPIO_Init(BRD_PB_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
	portENABLE_INTERRUPTS();	//Enable interrupts

}

void TaskTimerLeft( void ) {
//	BRD_LEDOff();
    struct dualtimer_msg SendLeftTimer;
    MessageQueue = xQueueCreate(11, sizeof(SendLeftTimer));		/* Create queue of length 10 Message items */
    /*Initialise Message Item payload */
    SendLeftTimer.type = 'l';
    SendLeftTimer.timer_value = 0;
    for(;;) {
      if(mode == 1) {
        if (MessageQueue != NULL) {	/* Check if queue exists */

          /*Send message to the front of the queue - wait atmost 10 ticks */
          if( xQueueSendToFront(MessageQueue, ( void * ) &SendLeftTimer, ( portTickType ) 11 ) != pdPASS ) {
            debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
        }
        if(SendLeftTimer.timer_value == 31) {
          SendLeftTimer.timer_value = 0;
        } else {
          SendLeftTimer.timer_value++;
        }
        if(SendLeftTimer.timer_value % 2) {
          S4344046_LA_CHAN1_CLR();
        } else {
          S4344046_LA_CHAN1_SET()
        }
      }
      vTaskDelay(1000);
    }
}

void TaskTimerRight( void ) {
    struct dualtimer_msg SendRightTimer;
    MessageQueue = xQueueCreate(11, sizeof(SendRightTimer));		/* Create queue of length 10 Message items */
    /*Initialise Message Item payload */
    SendRightTimer.type = 'r';
    SendRightTimer.timer_value = 0;
    for(;;) {
      if(mode == 1) {
        if (MessageQueue != NULL) {	/* Check if queue exists */

          /*Send message to the front of the queue - wait atmost 10 ticks */
          if( xQueueSendToFront(MessageQueue, ( void * ) &SendRightTimer, ( portTickType ) 11 ) != pdPASS ) {
            debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
        }
        if(SendRightTimer.timer_value == 31) {
          SendRightTimer.timer_value = 0;
        } else {
          SendRightTimer.timer_value++;
        }
        if(SendRightTimer.timer_value % 2) {
          S4344046_LA_CHAN0_CLR();
        } else {
          S4344046_LA_CHAN0_SET()
        }
      }
      vTaskDelay(100);
    }
}

void exti_pb_irqhandler(void) {

	BaseType_t xHigherPriorityTaskWoken;

    /* Is it time for another Task() to run? */
    xHigherPriorityTaskWoken = pdFALSE;

	/* Check if Pushbutton external interrupt has occured */
  	HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);				//Clear D0 pin external interrupt flag

	if (PBSemaphore != NULL) {	/* Check if semaphore exists */
		xSemaphoreGiveFromISR( PBSemaphore, &xHigherPriorityTaskWoken );		/* Give PB Semaphore from ISR*/
		debug_printf("Triggered \n\r");    //Print press count value
	}

	/* Perform context switching, if required. */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void semaphore_check(void) {
  for (;;) {
    if (PBSemaphore != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
            wait 10 ticks to see if it becomes free. */
      if( xSemaphoreTake( PBSemaphore, 10 ) == pdTRUE ) {

        /* Invert mode to stop or start timers */
        mode = ~mode & 0x01;
        S4344046_LA_CHAN2_CLR();
        S4344046_LA_CHAN2_SET();
        S4344046_LA_CHAN2_CLR();

      }
    }
    vTaskDelay(1);
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
