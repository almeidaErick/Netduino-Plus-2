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

#ifndef S4344046_JOYSTICK_H
#define S4344046_JOYSTICK_H

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* External function prototypes ----------------------------------------------*/

void s4344046_joystick_init();
unsigned int  s4344046_joystick_x_read();
unsigned int  s4344046_joystick_y_read();
#endif
