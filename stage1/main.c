/**
  ******************************************************************************
  * @file    stage1/main.c
  * @author  MY_FIRST_NAME + SURNAME
  * @date    10-January-2015
  * @brief   Prac 1 Template C main file - BCD timer and press counter.
  *			 NOTE: THIS CODE IS PSEUDOCODE AND DOES NOT COMPILE.
  *				   GUIDELINES ARE GIVEN AS COMMENTS.
  *			 REFERENCES: ex1_led, ex2_gpio, ex3_gpio, ex11_character
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4344046_lightbar.h"		////////CHANGE THIS//////////

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t counter_value = 0;
uint16_t press_counter_val = 1;
uint16_t external_interrupt = 1;
uint16_t rise_pin = 2;

/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void exti_a2_interrupt_handler(void);

/**
  * @brief  Main program - timer and press counter.
  * @param  None
  * @retval None
  */
void main(void) {
	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules

	int count;
	count = 64;
	//The timer and display timer should display the same number at the same time
	HAL_Delay(4000); //delay for 2 seconds
	/* Main processing loop */
  	while (1) {
		debug_printf("Countdown %d\n\r", count);	//Print debug message
		counter_value++;	//Increment counter

		/****************** Display counter. ***************/
		/* First, turn off each LED light bar segment
			write 0 to D0
			Write 0 to D1
			....
			Write 0 to D9

			Call sxxxxxx_ledbar_set(0)

			then call
	*/
			//sxxxxxx_lightbar_write(counter_value);
		//*/
		s4344046_lightbar_write(0);
		s4344046_lightbar_write(count);
		/* Toggle 'Keep Alive Indicator' BLUE LED */

		count--;
		if (count < 0) {
			count = 64;
		}
		BRD_LEDToggle();	//Toggle LED on/off
    HAL_Delay(1000 / press_counter_val);		//Delay for 1s (1000ms)
		if((rise_pin % 2 == 0) && (external_interrupt % 2 == 0)) {
			external_interrupt++;
		}

	}
}

/**
  * @brief  Initialise Hardware
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	__BRD_A2_GPIO_CLK();
	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Initialise LEDBar
       Call
	   sxxxxxx_ledbar_init();

	*/

	s4344046_lightbar_init();
	/* Configure A2 interrupt for Prac 1, Task 2 or 3 only */
	HAL_NVIC_SetPriority(BRD_A2_EXTI_IRQ, 10, 0);	//Set Main priority ot 10 and sub-priority ot 0
	//Enable external GPIO interrupt and interrupt vector for pin A2
	NVIC_SetVector(BRD_A2_EXTI_IRQ, (uint32_t)&exti_a2_interrupt_handler);/* Configure A2 pin as pull down input */
	NVIC_EnableIRQ(BRD_A2_EXTI_IRQ);
	GPIO_InitStructure.Pin = BRD_A2_PIN;				//Pin A2
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;		//interrupt Mode
	GPIO_InitStructure.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
	HAL_GPIO_Init(BRD_A2_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
}

/**
  * @brief  exti_a2 GPIO Interrupt handler
  * @param  None.
  * @retval None
  */
void exti_a2_interrupt_handler(void) {
	//press_counter_val variable used to modify clock speed

	if(external_interrupt % 2 != 0){ //check if interrupt is pressed once
		external_interrupt++;
		debug_printf("Velocity: %d\n\r", external_interrupt);	//Print debug message
		press_counter_val = press_counter_val * 2;  //Double the speed of the clock
		rise_pin++;
		HAL_Delay(1000);		//Delay for 1s (1000ms)
		/* Speed up the counter by reducing the delay value */
	} else if ((external_interrupt % 2 == 0) && (rise_pin % 2 != 0)){
		rise_pin++;
	}
	HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);				//Clear A2 pin external interrupt flag

}
