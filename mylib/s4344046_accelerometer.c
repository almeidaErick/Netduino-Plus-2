/**
  ******************************************************************************
  * @file    s4344046_accelerometer.c
  * @author  Erick Almeida
  * @date    140516
  * @brief   I2C example with the MMA8462Q.
  * Modified: 14-May-2016 - Create functions to read and write into registers
  *           15-May-2016 - Simplify all write and read functions into a
  *                          general function.
  *           01-Jun-2016 - Fix values read from register in accelerometer.
  *           03-jun-2016 - Change output names when readed from a register
  * Functions:
  *          void Hardware_init();
  *          int16_t get_twos_complement (uint16_t get_number);
  *          void write_register_value(uint8_t reg_number, uint8_t reg_value);
  *          uint8_t read_register(uint8_t register_num);
  *          void s4344046_TaskAcc(void);
  *
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
/* Scheduler includes. */
#include "FreeRTOS.h"


#include "FreeRTOS_CLI.h"
#include "s4344046_cli.h"
#include "s4344046_accelerometer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef  I2CHandle;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init_Acc();
int16_t get_twos_complement (uint16_t get_number);
void write_register_value(uint8_t reg_number, uint8_t reg_value);
uint8_t read_register(uint8_t register_num);
void s4344046_TaskAcc(void);


/**
  * @brief  Get twos complement for a number.
  * @param  get_number: Number we want to get the twos complement
  * @retval The 2s complement of a number.
  */
int16_t get_twos_complement (uint16_t get_number) {
  uint8_t negative;
  int16_t final_number;
  negative = !!(get_number & (1<<11));
  if (negative) {
    get_number = (get_number ^ 0x800);
    final_number = get_number - (2048);
  } else {
    final_number = get_number;
  }
  return final_number;
}


/**
  * @brief  Read from a register.
  * @param  register_num: register direction to read from.
  * @retval Infomation read from the register given.
  */
uint8_t read_register(uint8_t register_num) {
	uint8_t read_reg_val;
	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

		I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

		/*  Wait the START condition has been correctly sent */
		while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

		/* Wait for address to be acknowledged */
		while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
		__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/* Send Read Register Address - WHO_AM_I Register Address */
	I2CHandle.Instance->DR = register_num;

		/* Wait until register Address byte is transmitted */
		while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));
	/* Generate the START condition, again */
		I2CHandle.Instance->CR1 |= I2C_CR1_START;

		/* Wait the START condition has been correctly sent */
		while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Read Address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_READ(MMA8452Q_ADDRESS);

		/* Wait address is acknowledged */
		while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
		__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/* Wait to read */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_RXNE) == RESET);

	/* Read received value */
	read_reg_val = I2CHandle.Instance->DR;
	/* Generate NACK */
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;

	/* Generate the STOP condition */
		I2CHandle.Instance->CR1 |= I2C_CR1_STOP;
return read_reg_val;
}


/**
  * @brief  Write onto a register.
  * @param  reg_number: register direction where we want to write.
            reg_value: bit we want to set into the specified register.
  * @retval None
  */
void write_register_value(uint8_t reg_number, uint8_t reg_value) {
  __HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

		I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

		/*  Wait the START condition has been correctly sent */
		while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

		/* Wait for address to be acknowledged */
		while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
		__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/* Send Read Register Address - WHO_AM_I Register Address */
	I2CHandle.Instance->DR = reg_number;

		/* Wait until register Address byte is transmitted */
		while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));
	/* Generate the START condition, again */

	/* Send Read Register Address - WHO_AM_I Register Address */
	I2CHandle.Instance->DR = reg_value;

		/* Wait until register Address byte is transmitted */
		while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));
	/* Generate the START condition, again */

	/* Generate the STOP condition */
		I2CHandle.Instance->CR1 |= I2C_CR1_STOP;

}


/**
  * @brief  Task that returns the raw value of coordinates or PL location of
  *         accelerometer.
  * @param  None
  * @retval None
  */
void s4344046_TaskAcc(void) {
  uint16_t x_coord;
	uint16_t y_coord;
	uint16_t z_coord;
  uint8_t pl_location;
  struct Message SendMessage;

  write_register_value(CTRL_REG1, STAND_MODE);
  write_register_value(PL_CONFIG, PL_MODE);
  write_register_value(CTRL_REG1, ACTIVE_MODE);

  for(;;) {

    xActivatedSem = xQueueSelectFromSet( xQueueAcc,
                                                portMAX_DELAY );

    if(xActivatedSem == s4344046_SemaphoreAcc){

      if(xSemaphoreTake( s4344046_SemaphoreAcc, 0 ) == pdTRUE){

        x_coord = read_register(HIGH_X);
      	x_coord = (x_coord << 8) | read_register(LOW_X);
      	x_coord = (x_coord >> 4);

      	y_coord = read_register(HIGH_Y);
      	y_coord = (y_coord << 8) | read_register(LOW_Y);
      	y_coord = (y_coord >> 4);

      	z_coord = read_register(HIGH_Z);
      	z_coord = (z_coord << 8) | read_register(LOW_Z);
      	z_coord = (z_coord >> 4);

        SendMessage.x_location = get_twos_complement (x_coord);
        SendMessage.x_hex = x_coord;
        SendMessage.y_location = get_twos_complement (y_coord);
        SendMessage.y_hex = y_coord;
        SendMessage.z_location = get_twos_complement (z_coord);
        SendMessage.z_hex = z_coord;

        if (s4344046_QueueAcc != NULL) {	/* Check if queue exists */

          /*Send message to the front of the queue - wait atmost 10 ticks */
          if( xQueueSendToFront(s4344046_QueueAcc, ( void * ) &SendMessage, ( portTickType ) 10 ) != pdPASS ) {

            debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
        }
			}

    } else if(xActivatedSem == s4344046_SemaphoreAccPl){

      if(xSemaphoreTake( s4344046_SemaphoreAccPl, 0 ) == pdTRUE){

        pl_location = read_register(PL_REG);

        if (s4344046_QueueAcc != NULL) {	/* Check if queue exists */

          /*Send message to the front of the queue - wait atmost 10 ticks */
          if( xQueueSendToFront(s4344046_QueueAccPl, ( void * ) &pl_location, ( portTickType ) 10 ) != pdPASS ) {

            debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
        }
			}
    }
  }
}


/**
  * @brief  Initialise Hardware
  * @param  None
  * @retval None
  */
void Hardware_init_Acc() {

	GPIO_InitTypeDef  GPIO_InitStructure;

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOn();		//Turn off Blue LED

	/* Enable GPIO clocks */
	__BRD_SCL_GPIO_CLK();
	__BRD_SDA_GPIO_CLK();

	/* Enable I2C CLK */
	__BRD_I2C_CLK();

	/******************************************************/
	/* IMPORTANT NOTE: SCL Must be Initialised BEFORE SDA */
	/******************************************************/
	/* enable GPIO pins for I2C */
	GPIO_InitStructure.Pin = BRD_SCL_PIN;			//SCL
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP ;
	GPIO_InitStructure.Alternate = BRD_SCL_AF;
	HAL_GPIO_Init(BRD_SCL_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = BRD_SDA_PIN;			//SDA
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP ;
	GPIO_InitStructure.Alternate = BRD_SDA_AF;
	HAL_GPIO_Init(BRD_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* Configure the I2C peripheral */
	I2CHandle.Instance = BRD_I2C;
	I2CHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;                               // 7bit addressing mode
	I2CHandle.Init.ClockSpeed      = 1000000;						// Transmission Frequency
	I2CHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2CHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
	I2CHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2CHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	I2CHandle.Init.OwnAddress1     = 0;
	I2CHandle.Init.OwnAddress2     = 0;

	/* Initialise and Start the I2C peripheral */
	HAL_I2C_Init(&I2CHandle);

	/* -> Wait for the end of the transfer */
	/* Before starting a new communication transfer, you need to check the current
	* state of the peripheral; if it’s busy you need to wait for the end of current
	* transfer before starting a new one.
	* For simplicity reasons, this example is just waiting till the end of the
	* transfer, but application may perform other tasks while transfer operation
	* is ongoing.
	*/
	while (HAL_I2C_GetState(&I2CHandle) != HAL_I2C_STATE_READY);

}
