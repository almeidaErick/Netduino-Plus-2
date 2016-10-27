/**
  ******************************************************************************
  * @file    stage2/main.c
  * @author  Erick Almeida
  * @date    18-03-2015
  * @brief   Prac 2 Template C main file – Waveform Duty Cycle Controller
  *			 NOTE: THIS CODE IS PSEUDOCODE AND DOES NOT COMPILE.
  *				   GUIDELINES ARE GIVEN AS COMMENTS.
  *			 REFERENCES: ex4_adc, ex5_timer_interrupt.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4344046_lightbar.h"
#include "s4344046_joystick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef TIM_Init;
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConfig;

int count_interrupt;	//increment each time a timer interrupt occurs
int read_x = 0; // Initialize read value from x axis

/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void tim2_irqhandler (void);
void set_light_bar(int cycle);

/**
  * @brief  Main program - timer and press counter.
  * @param  None
  * @retval None
  */
void main(void) {
	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
  while (1) {
			read_x = s4344046_joystick_x_read();
			
	}
}



/**
  * @brief  Initialise Hardware
  * @param  None
  * @retval None
  */

void Hardware_init(void) {
		GPIO_InitTypeDef  GPIO_InitStructure;


		unsigned short PrescalerValue;

		s4344046_joystick_init(); //Initialise joystick
		s4344046_lightbar_init(); //Initialize lightbar
		BRD_LEDInit();		//Initialise onboard LED
		BRD_LEDOff();		//Turn off onboard LED

	  	/* Timer 2 clock enable */
		__TIM2_CLK_ENABLE();

		/* Compute the prescaler value */
	  	PrescalerValue = (uint16_t) ((SystemCoreClock /2)/500000) - 1;		//Set clock prescaler to 500kHz - SystemCoreClock is the system clock frequency.

	  	/* Time base configuration */
		TIM_Init.Instance = TIM2;				//Enable Timer 2
	  	TIM_Init.Init.Period = 100;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
	  	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
	  	TIM_Init.Init.ClockDivision = 0;			//Set clock division
		TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
	  	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.


			/* Configure the D1 pin as an output */
		GPIO_InitStructure.Pin = BRD_D1_PIN;				//Pin
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
			GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
			GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
			HAL_GPIO_Init(BRD_D1_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin


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
  * @brief  Timer vector to control duty value read from joystick
  * @param  None
  * @retval None
  */

void tim2_irqhandler (void) {

	//Clear Update Flag
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);

	//turn on lights depending on duty cycle
	set_light_bar(read_x);

	// Count up to 100 to control duty cycle from 0% up to 100%
	if (count_interrupt >= 100) {

		BRD_LEDToggle();
		count_interrupt = 0; //if value is greater than 100% then reset it to 0

	} else {
		/* Toggle LED */
		if(count_interrupt <= read_x){ // control duty cycle from value read from joystick
			HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 1);	//Write Digital 0 bit value
		} else {
			HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 0);	//Write Digital 0 bit value
		}

	}
	count_interrupt++;		//increment counter, when the interrupt occurs
}


/**
  * @brief  Turn on lightbar according to the percentge given by the
						duty cycle.
  * @param  None
  * @retval None
  */
void set_light_bar (int cycle){
	if ((cycle / 10) == 1) {
		s4344046_lightbar_write(1);
	} else if ((cycle / 10) == 2) {
		s4344046_lightbar_write(3);
	} else if ((cycle / 10) == 3) {
		s4344046_lightbar_write(7);
	} else if ((cycle / 10) == 4) {
		s4344046_lightbar_write(15);
	} else if ((cycle / 10) == 5) {
		s4344046_lightbar_write(31);
	} else if ((cycle / 10) == 6) {
		s4344046_lightbar_write(63);
	} else if ((cycle / 10) == 7) {
		s4344046_lightbar_write(127);
	} else if ((cycle / 10) == 8) {
		s4344046_lightbar_write(255);
	} else if ((cycle / 10) == 9) {
		s4344046_lightbar_write(511);
	} else if ((cycle / 10) == 10) {
		s4344046_lightbar_write(1023);
	}
}
