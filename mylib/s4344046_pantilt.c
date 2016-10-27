/**
  ******************************************************************************
  * @file    s4344046_pantilt.c
  * @author  Erick Almeida
  * @date    22-March-2014
  * @brief   Configure a servo motor
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4344046_pantilt.h"


/* Variables .................................................................*/
int angle_init_y = 90; //initial angle (from 0 to 180) instead of (-90 to 90)
int angle_init_x = 90;
int return_angle_x = 90;
int return_angle_y = 90;
int angle_init = 90;
int pan_tilt_angle;
int write_value = 0;

float duty_cycle_y = 1.45; //initial duty cycle (position for servo)
float duty_cycle_x = 1.45;

int RecvDir;


void show_angle(int type, int angle);


/**
  * @brief  Initialize pantilt
  * @param  dury_cycle - value to star PWM at the specified duty cycle.
  * @retval None
  */
void s4344046_pantilt_init(float duty_cyclex, float duty_cycley) {

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure2;
	TIM_OC_InitTypeDef PWMConfig;
  TIM_OC_InitTypeDef PWMConfig2;
	TIM_HandleTypeDef TIM_Init;

  uint16_t PrescalerValue = 0;

  /* Enable the D2 Clock */
  __BRD_D2_GPIO_CLK();

  /* Enable the D2 Clock */
  __BRD_D3_GPIO_CLK();


  /* Timer 2 clock enable */
	__TIM2_CLK_ENABLE();

  /* Configure the D2 pin with TIM2 output*/
  GPIO_InitStructure.Pin = BRD_D2_PIN;				//Pin
  GPIO_InitStructure.Mode =GPIO_MODE_AF_PP; 		//Set mode to be output alternate
  GPIO_InitStructure.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;	//Set alternate function to be timer 2
  HAL_GPIO_Init(BRD_D2_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

  /* Configure the D3 pin with TIM2 output*/
  GPIO_InitStructure2.Pin = BRD_D3_PIN;				//Pin
  GPIO_InitStructure2.Mode =GPIO_MODE_AF_PP; 		//Set mode to be output alternate
  GPIO_InitStructure2.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
  GPIO_InitStructure2.Speed = GPIO_SPEED_FAST;			//Pin latency
  GPIO_InitStructure2.Alternate = GPIO_AF1_TIM2;	//Set alternate function to be timer 2
  HAL_GPIO_Init(BRD_D3_GPIO_PORT, &GPIO_InitStructure2);	//Initialise Pin


  /* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 50000) - 1;

  /* Configure Timer settings */
  TIM_Init.Instance = TIM2;					//Enable Timer 2
  TIM_Init.Init.Period = (1.999/100)*50000;			//Set for 200ms (5Hz) period
  TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  TIM_Init.Init.ClockDivision = 0;			//Set clock division
  TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
  TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

  /* PWM Mode configuration for Channel 4 - set pulse width*/
	PWMConfig.OCMode			 = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
	PWMConfig.Pulse				= (duty_cyclex/1000)*50000;		//1ms pulse width to 10ms
	PWMConfig.OCPolarity	 = TIM_OCPOLARITY_HIGH;
	PWMConfig.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
	PWMConfig.OCFastMode	 = TIM_OCFAST_DISABLE;
	PWMConfig.OCIdleState	= TIM_OCIDLESTATE_RESET;
	PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  /* PWM Mode configuration for Channel 4 - set pulse width*/
	PWMConfig2.OCMode			 = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
	PWMConfig2.Pulse				= (duty_cycley/1000)*50000;		//1ms pulse width to 10ms
	PWMConfig2.OCPolarity	 = TIM_OCPOLARITY_HIGH;
	PWMConfig2.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
	PWMConfig2.OCFastMode	 = TIM_OCFAST_DISABLE;
	PWMConfig2.OCIdleState	= TIM_OCIDLESTATE_RESET;
	PWMConfig2.OCNIdleState = TIM_OCNIDLESTATE_RESET;


	/* Enable PWM for Timer 2, channel 4 */
	HAL_TIM_PWM_Init(&TIM_Init);
	HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, TIM_CHANNEL_4);

  /*Enable PWM for Timer 2, channel 3*/
//  HAL_TIM_PWM_Init(&TIM_Init);
	HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig2, TIM_CHANNEL_3);

  /* Start PWM channel 4 */
  HAL_TIM_PWM_Start(&TIM_Init, TIM_CHANNEL_4);

  /* Start PWM channel 3*/
  HAL_TIM_PWM_Start(&TIM_Init, TIM_CHANNEL_3);

  //HAL_Delay(50);
}


void s4344046_TaskPanTilt() {
  for(;;) {
    if( xSemaphoreTake( s4344046_SemaphoreLaser, 10 ) == pdTRUE ) {
      write_value = ~write_value;
      HAL_GPIO_WritePin(BRD_D0_GPIO_PORT, BRD_D0_PIN, write_value & 0x01);
    } else if(xSemaphoreTake( s4344046_SemaphorePanLeft, 10 ) == pdTRUE ) {
      s4344046_pantilt_angle_write(0, -5);
    } else if(xSemaphoreTake( s4344046_SemaphorePanRight, 10 ) == pdTRUE ) {
      s4344046_pantilt_angle_write(0, 5);
    } else if(xSemaphoreTake( s4344046_SemaphoreTiltUp, 10 ) == pdTRUE ) {
      s4344046_pantilt_angle_write(1, 5);
    } else if(xSemaphoreTake( s4344046_SemaphoreTiltDown, 10 ) == pdTRUE ) {
      s4344046_pantilt_angle_write(1, -5);
    }

    if (s4344046_QueuePan != NULL) {	/* Check if queue exists */
      /* Check for item received - block atmost for 10 ticks */
      if (xQueueReceive( s4344046_QueuePan, &RecvDir, 10 )) {

        /* display received item */

        if((RecvDir <= 75) && (RecvDir >= (-75))) {
          angle_init_x = 90;
          s4344046_pantilt_angle_write(0, RecvDir);
        }

              /* Toggle LED */
        BRD_LEDToggle();
      }
    }

    if (s4344046_QueueTilt != NULL) {	/* Check if queue exists */

      if (xQueueReceive( s4344046_QueueTilt, &RecvDir, 10 )) {

        /* display received item */

        if((RecvDir <= 75) && (RecvDir >= (-75))) {
          angle_init_y = 90;
          s4344046_pantilt_angle_write(1, RecvDir);
        }

              /* Toggle LED */
        BRD_LEDToggle();
      }
    }
  }
}



/**
  * @brief  Write the pan servo to an angle (0° to ±90°).
  * @param  type -pan or tilt
            angle- positive or negative angle to move servo
  * @retval None
  */

void s4344046_pantilt_angle_write(int type, int angle) {
  int fix_angle_x;
  int fix_angle_y;
  switch(type) {
  case 0:
    if(angle > 0) {
      fix_angle_x = angle_init_x - angle;
      if (fix_angle_x >= 15) { // make sure the maximum positive angle is 75
        angle_init_x -= angle;
        return_angle_x += angle;
        duty_cycle_x = (angle_init_x/90.0) + 0.45; // calculate duty cycle at the specified angle
      } else {
        duty_cycle_x = (angle_init_x / 90.0) + 0.45; // if angle is greater than 85, keep the last duty cycle read.
        debug_printf("duty cycle in pan: %f\n\r", duty_cycle_x);
        debug_printf("Pan Motor dead\n\r");
      }
    } else {
      angle = angle * (-1);
      fix_angle_x = angle_init_x + angle;
      if (fix_angle_x <= 165) { // make sure the minimum angle is -75
        angle_init_x += angle;
        return_angle_x -= angle;
        duty_cycle_x = (angle_init_x / 90.0) + 0.45; // calculate duty cycle at specified angle
      } else {
        duty_cycle_x = (angle_init_x / 90.0) + 0.45; // if angle is less than -85, then keep the last duty cycle read.
        debug_printf("duty cycle in pan: %f\n\r", duty_cycle_x);
        debug_printf("Pan Motor dead\n\r");
      }
    }
    s4344046_pantilt_init(duty_cycle_x, duty_cycle_y);
    break;

  case 1:

    if(angle > 0) {
      fix_angle_y = angle_init_y - angle;
      if (fix_angle_y >= 15) { // make sure the maximum positive angle is 75
        angle_init_y -= angle;
        return_angle_y += angle;
        duty_cycle_y = (angle_init_y/90.0) + 0.45; // calculate duty cycle at the specified angle
      } else {
        duty_cycle_y = (angle_init_y / 90.0) + 0.45; // if angle is greater than 85, keep the last duty cycle read.
        debug_printf("duty cycle in Tilt: %f\n\r", duty_cycle_y);
        debug_printf("Tilt Motor dead\n\r");
      }
    } else {
      angle = angle * (-1);
      fix_angle_y = angle_init_y + angle;
      if (fix_angle_y <= 165) { // make sure the minimum angle is -75
        angle_init_y += angle;
        return_angle_y -= angle;
        duty_cycle_y = (angle_init_y / 90.0) + 0.45; // calculate duty cycle at specified angle
      } else {
        duty_cycle_y = (angle_init_y / 90.0) + 0.45; // if angle is less than -85, then keep the last duty cycle read.
        debug_printf("duty cycle in Tilt: %f\n\r", duty_cycle_y);
        debug_printf("Tilt Motor dead\n\r");
      }
    }
  s4344046_pantilt_init(duty_cycle_x, duty_cycle_y);
  break;
  }
}


void show_angle(int type, int angle) {
  int fix_angle_x;
  int fix_angle_y;
  switch(type) {
  case 0:
    if (angle > 0) {
      //angle+=12;
      fix_angle_x = 90 - angle;
      if (fix_angle_x >= 5) { // make sure the maximum positive angle is 75
        duty_cycle_x = (fix_angle_x/90.0) + 0.45; // calculate duty cycle at the specified angle
      } else {
        debug_printf("Pan Motor dead\n\r");
      }
    } else {
      angle = angle * (-1);
      fix_angle_x = 90 + angle;
      if (fix_angle_x <= 192) { // make sure the minimum angle is -75
        duty_cycle_x = (fix_angle_x / 90.0) + 0.45; // calculate duty cycle at specified angle
      } else {
        debug_printf("Pan Motor dead\n\r");
      }
    }
    s4344046_pantilt_init(duty_cycle_x, duty_cycle_y);
    break;

  case 1:

  if (angle > 0) {
    fix_angle_y = 90 - angle + 12;
    if (fix_angle_y >= 5) { // make sure the maximum positive angle is 75
      duty_cycle_y = (fix_angle_y/90.0) + 0.45; // calculate duty cycle at the specified angle
    } else {
      duty_cycle_y = (angle_init_y / 90.0) + 0.45; // if angle is greater than 85, keep the last duty cycle read.
      debug_printf("Tilt Motor dead\n\r");
    }
  } else {
    angle = angle * (-1);
    fix_angle_y = 90 + angle - 12;
    if (fix_angle_y <= 192) { // make sure the minimum angle is -75
      duty_cycle_y = (fix_angle_y / 90.0) + 0.45; // calculate duty cycle at specified angle
    } else {
      debug_printf("Tilt Motor dead\n\r");
    }
  }

  debug_printf("x: %.02f     y: %.02f\n", duty_cycle_x, duty_cycle_y);
  s4344046_pantilt_init(duty_cycle_x, duty_cycle_y);
  break;
  }
}



int s4344046_angle_x(){
  return return_angle_x;
}

int s4344046_angle_y(){
  return return_angle_y;
}
