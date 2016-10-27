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
 #include "semphr.h"

#ifndef S4344046_PANTILT_H
#define S4344046_PANTILT_H

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
SemaphoreHandle_t s4344046_SemaphoreLaser;
SemaphoreHandle_t s4344046_SemaphorePanLeft;
SemaphoreHandle_t s4344046_SemaphorePanRight;
SemaphoreHandle_t s4344046_SemaphoreTiltUp;
SemaphoreHandle_t s4344046_SemaphoreTiltDown;
SemaphoreHandle_t s4344046_SemaphoreBox;

QueueHandle_t s4344046_QueuePan;
QueueHandle_t s4344046_QueueTilt;

/* Private variables ---------------------------------------------------------*/

/* External function prototypes -----------------------------------------------*/

void s4344046_pantilt_init(float duty_cyclex, float duty_cycley);
float write_angle_direction(int type, int angle);
int s4344046_angle_x();
int s4344046_angle_y();
void s4344046_pantilt_angle_write(int type, int angle);
void s4344046_TaskPanTilt();
void show_angle(int type, int angle);
#endif
