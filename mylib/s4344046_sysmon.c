/**
  ******************************************************************************
  * @file    s4344046_sysmon.c
  * @author  Erick Almeida
  * @date    15-April-2016
  * @brief   Hamming encoder and decoder.
  *			 Bytes received from the VCP are Hamming encoded and displayed.
  *
  *  Functions:
  *        void s4344046_sysmon_init();
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
void s4344046_sysmon_init();

void s4344046_sysmon_init(){

  	GPIO_InitTypeDef  GPIO_InitStructure;

  	BRD_LEDInit();		//Initialise Blue LED
  	BRD_LEDOff();		//Turn off Blue LED

  	/* Enable the A3, A4 & A5 Clock */
    __BRD_A3_GPIO_CLK();
  	__BRD_A4_GPIO_CLK();
    __BRD_A5_GPIO_CLK();

    	/* Configure the D0 pin as an output */
  	GPIO_InitStructure.Pin = BRD_A3_PIN;				//Pin
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
    HAL_GPIO_Init(BRD_A3_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

    /* Configure the D0 pin as an output */
    GPIO_InitStructure.Pin = BRD_A4_PIN;				//Pin
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
    HAL_GPIO_Init(BRD_A4_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

  	/* Configure the D1 pin as an input */
  	GPIO_InitStructure.Pin = BRD_A5_PIN;				//Pin
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
    HAL_GPIO_Init(BRD_A5_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
}
