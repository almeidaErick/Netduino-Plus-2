/**
  ******************************************************************************
  * @file    s4344046_joystick.c
  * @author  Erick Almeida
  * @date    17-March-2014
  * @brief   Enable the ADC1 on pin A0.
  *			 See Section 13 (ADC), P385 of the STM32F4xx Reference Manual.
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
ADC_HandleTypeDef AdcHandle;
ADC_HandleTypeDef AdcHandle2;

ADC_ChannelConfTypeDef AdcChanConfig;
ADC_ChannelConfTypeDef AdcChanConfig2;

/* Private function prototypes -----------------------------------------------*/
void s4344046_joystick_init();



/**
  * @brief  Initialise Hardware Peripherals used.
  * @param  None
  * @retval None
  */

void s4344046_joystick_init(){

		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure2;

		BRD_LEDInit();		//Initialise Blue LED
		BRD_LEDOff();		//Turn off Blue LED

		/* Enable A0 GPIO Clock */
		__BRD_A0_GPIO_CLK();

		/* Enable A1 GPIO Clock */
		__BRD_A1_GPIO_CLK();

		/* Configure A0 as analog input */
  	GPIO_InitStructure.Pin = BRD_A0_PIN;			/*Set A0 pin*/
  	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;		/*Set to Analog input*/
  	GPIO_InitStructure.Pull = GPIO_NOPULL ;			/*No Pull up resister*/

  	HAL_GPIO_Init(BRD_A0_GPIO_PORT, &GPIO_InitStructure);	/*Initialise AO*/


		/* Configure A1 as analog input */
		GPIO_InitStructure2.Pin = BRD_A1_PIN;			/*Set A1 pin*/
		GPIO_InitStructure2.Mode = GPIO_MODE_ANALOG;		/*Set to Analog input*/
		GPIO_InitStructure2.Pull = GPIO_NOPULL ;			/*No Pull up resister*/

		HAL_GPIO_Init(BRD_A1_GPIO_PORT, &GPIO_InitStructure2);	/*Initialise A1*/


		/* Enable ADC1 clock */
		__ADC1_CLK_ENABLE();

    /* Configure ADC1 */
    AdcHandle.Instance = (ADC_TypeDef *)(ADC1_BASE);						/*Use ADC1*/
		/*Set clock prescaler*/
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
		/*Set 12-bit data resolution*/
    AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle.Init.ScanConvMode          = DISABLE;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.NbrOfDiscConversion   = 0;
		/*No Trigger*/
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
		/*No Trigger*/
    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
		/*Right align data	*/
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.NbrOfConversion       = 1;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.EOCSelection          = DISABLE;

    HAL_ADC_Init(&AdcHandle);		/*Initialise ADC*/

		/* Enable ADC1 clock */
		__ADC2_CLK_ENABLE();

		/* Configure ADC1 */
		AdcHandle2.Instance = (ADC_TypeDef *)(ADC2_BASE);						/*Use ADC1*/
		/*Set clock prescaler*/
		AdcHandle2.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
		/*Set 12-bit data resolution*/
		AdcHandle2.Init.Resolution            = ADC_RESOLUTION12b;
		AdcHandle2.Init.ScanConvMode          = DISABLE;
		AdcHandle2.Init.ContinuousConvMode    = DISABLE;
		AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
		AdcHandle2.Init.NbrOfDiscConversion   = 0;
		/*No Trigger*/
		AdcHandle2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
		/*No Trigger*/
		AdcHandle2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
		/*Right align data*/
		AdcHandle2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
		AdcHandle2.Init.NbrOfConversion       = 1;
		AdcHandle2.Init.DMAContinuousRequests = DISABLE;
		AdcHandle2.Init.EOCSelection          = DISABLE;

		HAL_ADC_Init(&AdcHandle2);		/*Initialise ADC*/

		/* Configure ADC Channel */
		AdcChanConfig2.Channel = BRD_A1_ADC_CHAN;							/*Use A1 pin*/
		AdcChanConfig2.Rank         = 1;
		AdcChanConfig2.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		AdcChanConfig2.Offset       = 0;

		/* Configure ADC Channel */
		AdcChanConfig.Channel = BRD_A0_ADC_CHAN;							/*Use AO pin*/
		AdcChanConfig.Rank         = 1;
	  AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  AdcChanConfig.Offset       = 0;
		/*Initialise ADC channel*/
		HAL_ADC_ConfigChannel(&AdcHandle2, &AdcChanConfig2);
		/*Initialise ADC channel*/
		HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConfig);
}


/**
  * @brief  read Value from the joystick as analog and use it as digital
						and return it as a number from 0 to 100 and use it in the cycle
						modification.
  * @param  None
  * @retval cycle -> value read from joystick ready to use as a cycle
											range from 0 to 100.
  */
unsigned int s4344046_joystick_x_read() {
		unsigned int adc_value;
		int cycle;
		HAL_ADC_Start(&AdcHandle2); // Start ADC conversion

		//Wait for ADC Conversion to complete
		while (HAL_ADC_PollForConversion(&AdcHandle2, 10) != HAL_OK);
				adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle2));
		// Covert value read from joystick as percentage
		cycle = (100/4095.0) * adc_value;
		return cycle;
}

unsigned int s4344046_joystick_y_read() {
		unsigned int adc_value;
		int cycle;
		HAL_ADC_Start(&AdcHandle); // Start ADC conversion

		//Wait for ADC Conversion to complete
		while (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK);
				adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle));

		cycle = (100/4095.0) * adc_value; // Covert value read from joystick as percentage
		return cycle;

}
