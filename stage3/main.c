/**
	******************************************************************************
	* @file		ex6_pwm/main.c
	* @author	MDS
	* @date		02022015
	* @brief	 Enable the PWM output on pin DO.
	*			 See Section 18 (TIM3), P576 of the STM32F4xx Reference Manual.
	******************************************************************************
	*
	*/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4344046_lightbar.h"
#include "s4344046_joystick.h"
#include "s4344046_pantilt.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void exti_a2_interrupt_handler(void);
void tim3_irqhandler (void);

int read_z = 0; // Initialize read value from z axis
uint16_t external_interrupt = 1;
uint16_t rise_pin = 2;
char pos_char;
float complete_duty_cycle = 1.45;
int first_pos_joystick;
int counter = 0;

TIM_HandleTypeDef TIM3_Init;


/**
	* @brief	Main program
	* @param	None
	* @retval None
	*/
void main(void) {
	char RxChar;
	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	first_pos_joystick = s4344046_joystick_y_read(); // save initial position of joystick

	while (1) {
		/* Receive characters using getc */
		RxChar = debug_getc();

		/* Check if character is not Null */
		if (RxChar != '\0') {
			pos_char = RxChar;

#ifdef PRINTF_REFLECT
			debug_printf("%c\n\r", RxChar);		//reflect byte using printf - must delay before calling printf again.
#else
			debug_putc(RxChar);				//reflect byte using putc - puts character into buffer
			debug_flush();					//Must call flush, to send character
#endif
		}
		if((rise_pin % 2 == 0) && (external_interrupt % 2 == 0)) {
			external_interrupt++;
		}
		BRD_LEDToggle();	//Toggle 'Alive' LED on/off
		HAL_Delay(1000);	//Delay for 1s
	}
}

/**
	* @brief	Configure the hardware,
	* @param	None
	* @retval None
	*/
void Hardware_init(void) {

	GPIO_InitTypeDef GPIO_InitStructure;

	uint16_t PrescalerValue = 0;

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED

	/* Timer 3 clock enable */
	__TIM3_CLK_ENABLE();

	/* Enable A2 GPIO Clock */
	__BRD_A2_GPIO_CLK();

	s4344046_joystick_init(); //Initialise joystick
	s4344046_pantilt_init(complete_duty_cycle); //Initialise pantilt

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 50000) - 1;

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


	/* Time base configuration */
	TIM3_Init.Instance = TIM3;				//Enable Timer 3
	TIM3_Init.Init.Period = 404000;			//Set period count to be 0.25s, so timer interrupt occurs every 0.25s.
	TIM3_Init.Init.Prescaler = PrescalerValue;	//Set presale value
	TIM3_Init.Init.ClockDivision = 0;			//Set clock division
	TIM3_Init.Init.RepetitionCounter = 0;	// Set Reload Value
	TIM3_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.


	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM3_Init);

	HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.
	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(TIM3_IRQn, (uint16_t)&tim3_irqhandler);
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Start Timer */
	HAL_TIM_Base_Start_IT(&TIM3_Init);

}

/**
	* @brief	Actions taken when the joystick is pressed
	* @param	None
	* @retval None
	*/
void exti_a2_interrupt_handler(void) {
	//press_counter_val variable used to modify clock speed

	if(external_interrupt % 2 != 0){ //check if interrupt is pressed once
		external_interrupt++;
		/*change duty cycle by 10 degrees according the direction given in console*/
		complete_duty_cycle = write_angle_direction(pos_char, 10);
		/*Start PWM again*/
		s4344046_pantilt_init(complete_duty_cycle);
		rise_pin++;

	} else if ((external_interrupt % 2 == 0) && (rise_pin % 2 != 0)){
		rise_pin++;
	}


	HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);				//Clear A2 pin external interrupt flag

}


/**
	* @brief	Check if the joystick has moved, if so wait for 1 second and move
						to the left or right according the location of the joystick.
	* @param	None
	* @retval None
	*/
void tim3_irqhandler (void) {
	//Clear Update Flag
	__HAL_TIM_CLEAR_IT(&TIM3_Init, TIM_IT_UPDATE);

	int y_pos;
	y_pos = s4344046_joystick_y_read(); // read joystick position

	if(y_pos != first_pos_joystick) { //check is joystick has moved

		if(y_pos < first_pos_joystick) { // if joystick is on the left
			if(counter < 4) { // count 4 times to make a second
				counter ++;
			} else {
				counter = 0;
				/*move pantilt to the left by 10 degrees*/
				complete_duty_cycle = write_angle_direction('l', 10);
				s4344046_pantilt_init(complete_duty_cycle);
				debug_printf("LEFT\n\r");	//Print debug message
			}

		} else if (y_pos > first_pos_joystick) { // if joystick is on the right
			if(counter < 4) { // count 4 times to make a second
				counter ++;
			} else {
				counter = 0;
				/*move pantilt to he right by 5 degrees*/
				complete_duty_cycle = write_angle_direction('r', 5);
				s4344046_pantilt_init(complete_duty_cycle);
				debug_printf("RIGHT\n\r");	//Print debug message
			}
		} else {
			counter = 0;
		}
	} else {
		counter = 0;
	}
}
