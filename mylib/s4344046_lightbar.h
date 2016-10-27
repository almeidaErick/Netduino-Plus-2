/**
 ******************************************************************************
 * @file    mylib/sxxxxxx_ledbar.h
 * @author  MyName – MyStudent ID
 * @date    03032015
 * @brief   LED Light Bar peripheral driver
 *	     REFERENCE: LEDLightBar_datasheet.pdf
 *
 *			NOTE: REPLACE sxxxxxx with your student login.
 ******************************************************************************
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * sxxxxxx_ledbar_init() – intialise LED Light BAR
 * sxxxxxx_ledbar_set() – set LED Light BAR value
 ******************************************************************************
 */
 /* Scheduler includes. */
 #include "FreeRTOS.h"
 #include "task.h"
 #include "queue.h"



#ifndef S4344046_LIGHTBAR_H
#define S4344046_LIGHTBAR_H

QueueHandle_t MessageQueue;	/* Queue used */

struct dualtimer_msg {
  char type; //type is either ‘l’ or ‘r’
  uint8_t timer_value;
};
//#define s4344046_QueueLightBar

void s4344046_TaskLightBar(void);
void s4344046_clk_light(char type, uint8_t time_rec);



/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* External function prototypes -----------------------------------------------*/

extern void s4344046_lightbar_init(void);
extern void s4344046_lightbar_write(unsigned short value);
#endif
