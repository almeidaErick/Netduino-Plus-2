/**
 ******************************************************************************
 * @file    mylib/s4344046_ledbar.c
 * @author  Erick Almeida – 43440461
 * @date    03032016
 * @brief   LED Light Bar peripheral driver
 *	     REFERENCE: LEDLightBar_datasheet.pdf
 *
 * Modified: 26-May-2016 - Change pin settings to receive sensor from radio.
 						 03-jun-2016 - Assign other pins configuration to avoid problems
						 							 when reading from radio.
 ******************************************************************************
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4344046_lightbar_init() – intialise LED Light BAR
 * s4344046_lightbar_write() – set LED Light BAR value
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4344046_lightbar.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define LDE_CLK_0 __BRD_D0_GPIO_CLK()
#define LDE_CLK_1 __BRD_D4_GPIO_CLK()
#define LDE_CLK_2 __BRD_D5_GPIO_CLK()
#define LDE_CLK_3 __BRD_D6_GPIO_CLK()
#define LDE_CLK_4 __BRD_D7_GPIO_CLK()
#define LDE_CLK_5 __BRD_D8_GPIO_CLK()
#define LDE_CLK_6 __BRD_D8_GPIO_CLK()
#define LDE_CLK_7 __BRD_D8_GPIO_CLK()
#define LDE_CLK_8 __BRD_D8_GPIO_CLK()
#define LDE_CLK_9 __BRD_D8_GPIO_CLK()

#define LDE_PIN_0 BRD_D0_PIN
#define LDE_PIN_1 BRD_D4_PIN
#define LDE_PIN_2 BRD_D5_PIN
#define LDE_PIN_3 BRD_D6_PIN
#define LDE_PIN_4 BRD_D7_PIN
#define LDE_PIN_5 BRD_D8_PIN
#define LDE_PIN_6 BRD_D8_PIN
#define LDE_PIN_7 BRD_D8_PIN
#define LDE_PIN_8 BRD_D8_PIN
#define LDE_PIN_9 BRD_D8_PIN

#define LDE_BRD_PIN_0 BRD_D0_GPIO_PORT
#define LDE_BRD_PIN_1 BRD_D4_GPIO_PORT
#define LDE_BRD_PIN_2 BRD_D5_GPIO_PORT
#define LDE_BRD_PIN_3 BRD_D6_GPIO_PORT
#define LDE_BRD_PIN_4 BRD_D7_GPIO_PORT
#define LDE_BRD_PIN_5 BRD_D8_GPIO_PORT
#define LDE_BRD_PIN_6 BRD_D8_GPIO_PORT
#define LDE_BRD_PIN_7 BRD_D8_GPIO_PORT
#define LDE_BRD_PIN_8 BRD_D8_GPIO_PORT
#define LDE_BRD_PIN_9 BRD_D8_GPIO_PORT




void lightbar_seg_set(int segment, unsigned char segment_value) {

	/*
		Turn segment on (segment_value = 1) or off (segment_value = 0)

     */

	if (segment == 0) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_0, LDE_PIN_0, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_0, LDE_PIN_0, 0);
		}
	} else if (segment == 1) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_1, LDE_PIN_1, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_1, LDE_PIN_1, 0);
		}

	} else if (segment == 2) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_2, LDE_PIN_2, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_2, LDE_PIN_2, 0);
		}
	} else if (segment == 3) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_3, LDE_PIN_3, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_3, LDE_PIN_3, 0);
		}
	} else if (segment == 4) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_4, LDE_PIN_4, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_4, LDE_PIN_4, 0);
		}
	} else if (segment == 5) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_5, LDE_PIN_5, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_5, LDE_PIN_5, 0);
		}
	} else if (segment == 6) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_6, LDE_PIN_6, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_6, LDE_PIN_6, 0);
		}
	} else if (segment == 7) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_7, LDE_PIN_7, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_7, LDE_PIN_7, 0);
		}
	} else if (segment == 8) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_8, LDE_PIN_8, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_8, LDE_PIN_8, 0);
		}
	} else if (segment == 9) {
		if (segment_value == 1) {
			HAL_GPIO_WritePin(LDE_BRD_PIN_9, LDE_PIN_9, 1);
		} else {
			HAL_GPIO_WritePin(LDE_BRD_PIN_9, LDE_PIN_9, 0);
		}
	}


}

/**
  * @brief  Initialise LEDBar.
  * @param  None
  * @retval None
  */
extern void s4344046_lightbar_init(void) {

	/* Configure the GPIO_D0 pin

	 	....

		Configure the GPIO_D9 pin
    */

	GPIO_InitTypeDef  GPIO_InitStructure;

	/*Initialize clock for each pin needed to use*/
	LDE_CLK_0;
	LDE_CLK_1;
	LDE_CLK_2;
	LDE_CLK_3;
	LDE_CLK_4;
	LDE_CLK_5;
	LDE_CLK_6;
	LDE_CLK_7;
	LDE_CLK_8;
	LDE_CLK_9;

	/* Configure the D2 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_0;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_0, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D3 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_1;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_1, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D4 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_2;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_2, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D5 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_3;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_3, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D6 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_4;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_4, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D7 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_5;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_5, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D8 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_6;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_6, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D9 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_7;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_7, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D10 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_8;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_8, &GPIO_InitStructure);	//Initialise Pin

	/* Configure the D11 pin as an output */
	GPIO_InitStructure.Pin = LDE_PIN_9;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		//Output Mode
  	GPIO_InitStructure.Pull = GPIO_PULLDOWN;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(LDE_BRD_PIN_9, &GPIO_InitStructure);	//Initialise Pin
}

/**
  * @brief  Set the LED Bar GPIO pins high or low, depending on the bit of ‘value’
  *         i.e. value bit 0 is 1 – LED Bar 0 on
  *          value bit 1 is 1 – LED BAR 1 on
  *
  * @param  value
  * @retval None
  */
extern void s4344046_lightbar_write(unsigned short value) {

	/* Use bit shifts (<< or >>) and bit masks (1 << bit_index) to determine if a bit is set

	   e.g. The following pseudo code checks if bit 0 of value is 1.
			if ((value & (1 << 0)) == (1 << 0))	{
				Turn on LED BAR Segment 0.
			}
	*/
	debug_printf("llego: %02x\n", value);
	int i;  //Variable that will change according to the bit that needs to be checked
	i = 0;
	while (i < 10) {  //Loop to check from bit 2 to bit 11 (D2 ... D11)
		if ((value & (1 << i)) == (1 << i)) {
			lightbar_seg_set(i, 1);
		} else {
			lightbar_seg_set(i, 0);
		}
		i++;
	}
}

void s4344046_clk_light(char type, uint8_t time_rec) {
	int i, j;
	j = 0;
	if(type == 'r') {
		i = 0;
		while (i < 5) {  //Loop to check from bit 2 to bit 11 (D2 ... D11)
			if ((time_rec & (1 << j)) == (1 << j)) {
				lightbar_seg_set(i, 1);
			} else {
				lightbar_seg_set(i, 0);
			}
			i++;
			j++;
		}
	} else if (type == 'l') {
		i = 5;
		while (i < 10) {  //Loop to check from bit 2 to bit 11 (D2 ... D11)
			if ((time_rec & (1 << j)) == (1 << j)) {
				lightbar_seg_set(i, 1);
			} else {
				lightbar_seg_set(i, 0);
			}
			i++;
			j++;
		}
	}
}

void s4344046_TaskLightBar(void){
	struct dualtimer_msg RecvMessage;
  BRD_LEDOff();
  for (;;) {
    if (MessageQueue != NULL) {	/* Check if queue exists */
      /* Check for item received - block atmost for 10 ticks */
      if (xQueueReceive( MessageQueue, &RecvMessage, 10)) {
        /* display received item */
        //debug_printf("Received: %c - %d\n\r", RecvMessage.type, RecvMessage.timer_value);
        s4344046_clk_light(RecvMessage.type, RecvMessage.timer_value);
        /* Toggle LED */
        BRD_LEDToggle();
      }
    }
    /* Delay for 10ms */
    vTaskDelay(10);
  }
}
