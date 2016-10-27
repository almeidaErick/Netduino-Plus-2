/**
  ******************************************************************************
  * @file    repo/mylib/cli.c
  * @author  Erick Almeida
  * @date    050516
  * @brief   FreeRTOS CLI program.Creates a task to implement the CLI.
  *
	*	Modified: 6-May-2016 - Fix semafores and queues for Stage 6.
	*						13-May-2016 - Implement the command Top, Suspend, Resume,
	*													Hamdec, hamenc, crc.
	*						15-May-2016 - Implement the command for Acc
	*						16-May-2016 - Implement the command for Tracking.
	*						18-May-2016 - Change the name for Tracking command from "TAKE" to
	*													"RADIO".
	*						25-May-2016 - Implement set rfchanSet command and getpasskey command.
	*						27-May-2016 - Start implementing commands for task 2 (project 2).
	*						28-May-2016 - Implement orb command that takes channel and orb
	*													number to set addres of the ORB.
	*						29-May-2016 - Set commands to move rover
	*						20-May-2016 - Implement calibration for rover.
	*
  *
  *
  *	Functions:
	*	BaseType_t prvEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
	*	BaseType_t prvLaserCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
	*	BaseType_t prvTiltCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvPanCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvBoxCreate(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvTopCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvSuspendTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvResumeTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvHamencTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvHamdecTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvCrcTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvAccCreate(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*	BaseType_t prvTrack(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvSetChan(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvSetPass(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvSetSensor(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvSetTime(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*
	* BaseType_t prvMoveFor(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvMoveRev(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvAngle(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvCali(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvOrb(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	* BaseType_t prvDistance(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
	*BaseType_t prvAdjust(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
  ******************************************************************************
  *
  */



/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"


#include "FreeRTOS_CLI.h"
#include "s4344046_cli.h"
#include "s4344046_pantilt.h"
#include "s4344046_hamming.h"
#include "s4344046_accelerometer.h"
#include "s4344046_crc.h"
#include "s4344046_radio.h"
#include "nrf24l01plus.h"
#include "radio_fsm.h"

/* Private variables ---------------------------------------------------------*/

#define ACTIVE_CHANNEL 50
uint8_t initAddress[5] = {0x07, 0x35, 0x22, 0x11, 0x00};

uint16_t CodedWord;
uint8_t address_support[4];
uint8_t error = 0x00;
/*Initial Address for message*/
uint8_t address[4];


CLI_Command_Definition_t xEcho = {	/* Structure that defines the "echo" command line command. */
	"echo",
	"echo: Echo the input.\r\n",
	prvEchoCommand,
	1
};

CLI_Command_Definition_t xLaser = {	/* Structure that defines the "echo" command line command. */
	"laser",
	"laser: Set state of laser.\r\n",
	prvLaserCommand,
	1
};

CLI_Command_Definition_t xPan = {	/* Structure that defines the "echo" command line command. */
	"pan",
	"pan: move pan at a certain angle.\r\n",
	prvPanCommand,
	1
};

CLI_Command_Definition_t xTilt = {	/* Structure that defines the "echo" command line command. */
	"tilt",
	"tilt: move tilt at a certain angle.\r\n",
	prvTiltCommand,
	1
};

CLI_Command_Definition_t xBox = {	/* Structure that defines the "echo" command line command. */
	"box",
	"box: create a box using laser.\r\n",
	prvBoxCreate,
	0
};

CLI_Command_Definition_t xTop = {	/* Structure that defines the "echo" command line command. */
	"top",
	"top: List the current number of tasks running.\r\n",
	prvTopCommand,
	0
};

CLI_Command_Definition_t xSuspend = {
	"suspend",
	"suspend: Suspend a task given by the name after suspend.\r\n",
	prvSuspendTask,
	1
};

CLI_Command_Definition_t xResume = {
	"resume",
	"resume: Resume a task given by the name after resume.\r\n",
	prvResumeTask,
	1
};

CLI_Command_Definition_t xHamenc = {
	"hamenc",
	"hamenc: Show hex value of hamming encoded value. \r\n",
	prvHamencTask,
	1
};

CLI_Command_Definition_t xHamdec = {
	"hamdec",
	"hamdec: Hamming Decode value. \r\n",
	prvHamdecTask,
	1
};

CLI_Command_Definition_t xCrc = {
	"crc",
	"crc: Calculate CRC 16 of a hex value. \r\n",
	prvCrcTask,
	1
};

CLI_Command_Definition_t xAcc = {
	"acc",
	"acc: Show the raw accelerometer 12-bit X, Y and Z values. \r\n",
	prvAccCreate,
	1
};

CLI_Command_Definition_t xTrack = {
	"tracking",
	"tracking: Enable or disable the tracking output. \r\n",
	prvTrack,
	1
};


CLI_Command_Definition_t xRfchanset = {
	"rfchanset",
	"rfchanset: Set the RF channel of the nrf24l01p. \r\n",
	prvSetChan,
	1
};


CLI_Command_Definition_t xPassKey = {
	"getpasskey",
	"getpasskey: Get a current passkey – must display in terminal. \r\n",
	prvSetPass,
	0
};


CLI_Command_Definition_t xSensor = {
	"getsensor",
	"getsensor: Get the current sensor value – must display in terminal. \r\n",
	prvSetSensor,
	0
};


CLI_Command_Definition_t xGetTime = {
	"gettime",
	"gettime: Get the current time in seconds (2 decimal point precision).\r\n",
	prvSetTime,
	0
};


CLI_Command_Definition_t xForward = {
	"forward",
	"forward: Move the rover forward a specified distance in mm.\r\n",
	prvMoveFor,
	1
};


CLI_Command_Definition_t xReverse = {
	"reverse",
	"reverse: Move the rover in reverse a specified distance in mm.\r\n",
	prvMoveRev,
	1
};


CLI_Command_Definition_t xAngle = {
	"angle",
	"angle: Move the rover to a certain angle (1 degree resolution).\r\n",
	prvAngle,
	1
};


CLI_Command_Definition_t xCalibration = {
	"calibration",
	"calibration: Calibrate linear motion of rover (parameters specified by user). \r\n",
	prvCali,
	3
};


CLI_Command_Definition_t xOrb = {
	"orb",
	"orb: Activate orb radio and start receiving messages. \r\n",
	prvOrb,
	2
};


CLI_Command_Definition_t xDistance = {
	"distance",
	"distance: Display the distance of the rover from the starting edge, in mm. \r\n",
	prvDistance,
	0
};

CLI_Command_Definition_t xAdjust = {
	"adjust",
	"adjust: set origin for laser. \r\n",
	prvAdjust,
	2
};



/* Private function prototypes -----------------------------------------------*/
BaseType_t prvEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
BaseType_t prvLaserCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
BaseType_t prvTiltCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvPanCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvBoxCreate(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvTopCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvSuspendTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvResumeTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvHamencTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvHamdecTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvCrcTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvAccCreate(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvTrack(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvSetChan(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvSetPass(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvSetSensor(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvSetTime(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

BaseType_t prvMoveFor(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvMoveRev(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvAngle(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvCali(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvOrb(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvDistance(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
BaseType_t prvAdjust(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);


/**
  * @brief  Adjust angle for the laser, so it could track where the rover is
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvAdjust(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	struct adjust_laser set_laser;

	long lParam_len_left, lParam_len_right;
	const char *angle_pan;
	const char *angle_tilt;
	int tilt, pan;

	*pcWriteBuffer = 0x00;
	angle_tilt = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len_right);
	angle_pan = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len_left);

	tilt = strtol(angle_tilt, NULL, 10);
	pan = strtol(angle_pan, NULL, 10);

	set_laser.angle_x_axis = pan;
	set_laser.angle_y_axis = tilt;

	if( xQueueSendToFront(s4344046_QueueAdjust, ( void * ) &set_laser, ( portTickType ) 10 ) != pdPASS ) {

		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}

	return pdFALSE;
}


BaseType_t prvDistance(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	*pcWriteBuffer = 0x00;
	xSemaphoreGive( s4344046_SemaphoreDistance );
	return pdFALSE;
}



/**
  * @brief  Set address for an specific orb.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvOrb(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	int i;
	long lParam_len_ch, lParam_len_orb;
	const char *channel_rec;
	const char *orb_numb;
	uint8_t orb, chann;
	int chan_radio;
	*pcWriteBuffer = 0x00;
	channel_rec = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len_ch);
	orb_numb = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len_orb);

	orb = strtol(orb_numb, NULL, 16);
	chann = strtol(channel_rec, NULL, 16);


	initAddress[0] = 0x00;
	initAddress[1] &= 0xF0;

	initAddress[0] = (((chann & 0x0F) << 4) | (orb & 0x0F));
	initAddress[1] |= ((chann & 0xF0) >>4);

	chan_radio = (chann/16 * 10) + chann%16;

	s4344046_start_idle();
	s4344046_radio_settxaddress(&initAddress);
	s4344046_radio_setchan(chan_radio);
	radio_fsm_buffer_write(NRF24L01P_RX_ADDR_P0, initAddress, 5); //To send between NP2
	s4344046_radio_gettxaddress(&address_support);

	return pdFALSE;
}



/**
  * @brief  Get running time since NP@ has started.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvMoveFor(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;
	uint8_t distance;
	*pcWriteBuffer = 0x00;
	/* Get parameters from command string */
	//s4344046_lightbar_write(0x13);
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	distance = strtol(cCmd_string, NULL, 10);
	if( xQueueSendToFront(s4344046_QueueForward, ( void * ) &distance, ( portTickType ) 10 ) != pdPASS ) {

		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}
	return pdFALSE;
}


/**
  * @brief  Move Rover forward.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvMoveRev(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;
	uint8_t distance;
	*pcWriteBuffer = 0x00;
	//s4344046_lightbar_write(0x1b);
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	distance = strtol(cCmd_string, NULL, 10);
	if( xQueueSendToFront(s4344046_QueueReverse, ( void * ) &distance, ( portTickType ) 10 ) != pdPASS ) {

		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}
	return pdFALSE;
}


/**
  * @brief  Move Rover in a defined angle.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvAngle(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;
	int total_angle;
	*pcWriteBuffer = 0x00;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	total_angle = strtol(cCmd_string, NULL, 10);
	if( xQueueSendToFront(s4344046_QueueAngle, ( void * ) &total_angle, ( portTickType ) 10 ) != pdPASS ) {

		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}
	return pdFALSE;
}


/**
  * @brief  Calibrate Rover, so it can move linearly.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvCali(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	int i;
	long lParam_len_left, lParam_len_right, lParam_len_ang;
	const char *motor_left;
	const char *motor_right;
	const char *motor_type;
	struct calibration_rov fix_motor;
	uint8_t left, right, cal_type;
	*pcWriteBuffer = 0x00;
	motor_right = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len_right);
	motor_left = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len_left);
	motor_type = FreeRTOS_CLIGetParameter(pcCommandString, 3, &lParam_len_ang);




	left = strtol(motor_left, NULL, 10);
	right = strtol(motor_right, NULL, 10);
	cal_type = strtol(motor_type, NULL, 10);

	debug_printf("x: %d, y: %d\n", s4344046_angle_x(), s4344046_angle_y());
	fix_motor.left_wheel = left;
	fix_motor.right_wheel = right;
	if(cal_type == 1) {
		fix_motor.type = 1;
	} else if (cal_type == 0){
		fix_motor.type = 0;
	} else if (cal_type == 2){
		fix_motor.type = 2;
	} else if (cal_type == 3) {
		fix_motor.type = 3;
	} else {
		return pdFALSE;
	}

	if( xQueueSendToFront(s4344046_QueueCalibration, ( void * ) &fix_motor, ( portTickType ) 10 ) != pdPASS ) {
		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}
	return pdFALSE;
}



/**
  * @brief  Get running time since NP@ has started.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvSetTime(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
		*pcWriteBuffer = 0x00;
		xSemaphoreGive( s4344046_SemaphoreTime );
		return pdFALSE;
}


/**
  * @brief  Get sensor id for communication.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvSetSensor(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
		*pcWriteBuffer = 0x00;
		xSemaphoreGive( s4344046_SemaphoreSensor );
		return pdFALSE;
}


/**
  * @brief  Get pass key to communication.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvSetPass(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
		*pcWriteBuffer = 0x00;
		xSemaphoreGive( s4344046_SemaphoreKey );
		return pdFALSE;
}


/**
  * @brief  Set the RF channel of the nrf24l01p.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvSetChan(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
		long lParam_len;
		const char *cCmd_string;
		uint8_t new_channel;
		int read;
		*pcWriteBuffer = 0x00;
		/* Get parameters from command string */
		cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
		new_channel = strtol(cCmd_string, NULL, 16);
		if( xQueueSendToFront(s4344046_QueueChan, ( void * ) &new_channel, ( portTickType ) 10 ) != pdPASS ) {

			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}

		return pdFALSE;
}



/**
  * @brief  Box Command, draw a box move the pan and tilt with the laser on.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvBoxCreate(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	int hor_line = 0;
	int ver_line = -75;
	s4344046_QueueTilt = xQueueCreate(10, sizeof(int));		/* Create queue of length 10 Message items */

	if( xQueueSendToFront(s4344046_QueueTilt, ( void * ) &ver_line, ( portTickType ) 10 ) != pdPASS ) {

		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}
	s4344046_QueuePan = xQueueCreate(10, sizeof(int));		/* Create queue of length 10 Message items */

	if( xQueueSendToFront(s4344046_QueuePan, ( void * ) &hor_line, ( portTickType ) 10 ) != pdPASS ) {

		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}

	xSemaphoreGive(s4344046_SemaphoreBox);
	return pdFALSE;
}


/**
  * @brief  Tilt command, move tilt to an angle given.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvTiltCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString){
	long tiltParam_len;
	const char *angle_moved;
	int move_angle;
	angle_moved = FreeRTOS_CLIGetParameter(pcCommandString, 1, &tiltParam_len);
	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "\n\r%s\n\r", angle_moved);

	if(!(strcmp(angle_moved, "0"))) {

		move_angle = atoi(angle_moved);
		s4344046_QueueTilt = xQueueCreate(10, sizeof(int));		/* Create queue of length 10 Message items */

		if( xQueueSendToFront(s4344046_QueueTilt, ( void * ) &move_angle, ( portTickType ) 10 ) != pdPASS ) {

			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}

	} else {

		if(atoi(angle_moved) == 0) {

			if(!(strcmp(angle_moved, "up"))) {

				xSemaphoreGive(s4344046_SemaphoreTiltUp);
			} else if(!(strcmp(angle_moved, "down"))) {

				xSemaphoreGive(s4344046_SemaphoreTiltDown);
			}

		} else {

			move_angle = atoi(angle_moved);
			s4344046_QueueTilt = xQueueCreate(10, sizeof(int));		/* Create queue of length 10 Message items */

			if( xQueueSendToFront(s4344046_QueueTilt, ( void * ) &move_angle, ( portTickType ) 10 ) != pdPASS ) {

				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}
	}
	return pdFALSE;
}


/**
  * @brief  Pan Command, move pan to a angle given.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvPanCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString){
	long panParam_len;
	const char *angle_moved;
	int move_angle;
	angle_moved = FreeRTOS_CLIGetParameter(pcCommandString, 1, &panParam_len);
	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "\n\r%s\n\r", angle_moved);

	if(!(strcmp(angle_moved, "0"))) {

		move_angle = atoi(angle_moved);
		s4344046_QueuePan = xQueueCreate(10, sizeof(int));		/* Create queue of length 10 Message items */

		if( xQueueSendToFront(s4344046_QueuePan, ( void * ) &move_angle, ( portTickType ) 10 ) != pdPASS ) {
			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}

	} else {

		if(atoi(angle_moved) == 0) {

			if(!(strcmp(angle_moved, "right"))) {

				xSemaphoreGive(s4344046_SemaphorePanRight);

			} else if(!(strcmp(angle_moved, "left"))) {

				xSemaphoreGive(s4344046_SemaphorePanLeft);
			}

		} else {

			move_angle = atoi(angle_moved);
			s4344046_QueuePan = xQueueCreate(10, sizeof(int));		/* Create queue of length 10 Message items */
			if( xQueueSendToFront(s4344046_QueuePan, ( void * ) &move_angle, ( portTickType ) 10 ) != pdPASS ) {

				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}
	}
	return pdFALSE;
}

/**
  * @brief  Laser command, turn on or off the laser using semaphores.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvLaserCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	long laserParam_len;
	static const char *state_laser;
	char *actual_laser;

	state_laser = FreeRTOS_CLIGetParameter(pcCommandString, 1, &laserParam_len);
	/* Write command echo output string to write buffer. */
	/* Terminate state. */
	if(!(strcmp(state_laser, "on"))) {

	//	if((laser_on == 0) && (laser_off == 1)) {

		//	laser_on = 1;
		//	laser_off = 0;
			/*TURN ON LASER*/
			sprintf((char *) pcWriteBuffer, "\n\rLASER ON\n\r");
	    HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 1);
			//xSemaphoreGive( s4344046_SemaphoreLaser );
		//}

	} else if(!(strcmp(state_laser, "off"))) {

		//if((laser_on == 1) && (laser_off == 0)) {

			//laser_off = 1;
			//laser_on = 0;
			//xSemaphoreGive( s4344046_SemaphoreLaser );
			/*TURN ON LASER*/
			sprintf((char *) pcWriteBuffer, "\n\rLASER OFF\n\r");
	    HAL_GPIO_WritePin(BRD_D1_GPIO_PORT, BRD_D1_PIN, 0);
		//}

	}

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  Echo Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;
	*pcWriteBuffer = 0x00;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "\n\r%s\n\r", cCmd_string);

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/**
  * @brief  Top Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvTopCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	*pcWriteBuffer = 0x00;
	xSemaphoreGive( s4344046_SemaphoreTop );

	return pdFALSE;
}

/**
  * @brief  Suspend Task Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvSuspendTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString){
	long lParam_len;
	const char *cCmd_string;
	*pcWriteBuffer = 0x00;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if( xQueueSendToFront(s4344046_QueueSuspendTask, ( void * ) &cCmd_string, ( portTickType ) 10 ) != pdPASS ) {

		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}
	return pdFALSE;
}

/**
  * @brief  Resume task Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvResumeTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString){
	long lParam_len;
	const char *cCmd_string;
	*pcWriteBuffer = 0x00;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if( xQueueSendToFront(s4344046_QueueResumeTask, ( void * ) &cCmd_string, ( portTickType ) 10 ) != pdPASS ) {

		debug_printf("Failed to post the message, after 10 ticks.\n\r");
	}
	return pdFALSE;
}


/**
  * @brief  Hamming enconder Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvHamencTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	const char *cCmd_string;
	int i;
	long lParam_len;
	uint8_t hex_val;
	*pcWriteBuffer = 0x00;
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if(lParam_len == 1){

		CodedWord = s4344046_hamming_byte_encoder(cCmd_string[0], error);
		/* Write command echo output string to write buffer. */
	} else if(lParam_len == 2) {

		hex_val = strtol(cCmd_string, NULL, 16);
		CodedWord = s4344046_hamming_byte_encoder(hex_val, error);
	}
	sprintf((char *) pcWriteBuffer, "\n\r%04x\n\r", CodedWord);
	return pdFALSE;
}

/**
  * @brief  Hamming decoder Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvHamdecTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	const char *cCmd_string;
	int i;
	long lParam_len;
	uint8_t lower;
	uint8_t upper;
	uint8_t final;
	uint16_t numb;
	*pcWriteBuffer = 0x00;
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	numb = strtol(cCmd_string, NULL, 16);
	lower = (numb & 0x00FF);
	upper = (numb & 0xFF00)>>8;
	final = s4344046_hamming_byte_decoder(lower, upper);
	sprintf((char *) pcWriteBuffer, "\n\r%02x\n\r", final);
	return pdFALSE;
}


/**
  * @brief  CRC Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvCrcTask(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	const char *cCmd_string;
	int i;
	long lParam_len;
	uint32_t input_crc;
	uint16_t output_crc = 0x0000;
	*pcWriteBuffer = 0x00;
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if((cCmd_string[0] == '0') && (cCmd_string[1] == 'x')) {

		input_crc = strtol(cCmd_string, NULL, 16);
		output_crc = s4344046_crc_update32(output_crc, input_crc);

	} else {

		for(i = 0; i < lParam_len; i++) {

			output_crc = s4344046_crc_update(output_crc, cCmd_string[i]);
		}
	}
	sprintf((char *) pcWriteBuffer, "\n\r%04x\n\r", output_crc);
	return pdFALSE;
}


/**
  * @brief  Accelerometer Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvAccCreate(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	const char *cCmd_string;
	int i;
	long lParam_len;
	*pcWriteBuffer = 0x00;
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if(!(strcmp(cCmd_string, "raw"))) {

		xSemaphoreGive( s4344046_SemaphoreAcc );

	} else if(!(strcmp(cCmd_string, "pl"))) {

		xSemaphoreGive( s4344046_SemaphoreAccPl );

	}

	return pdFALSE;
}


/**
  * @brief  Tracking Command.
  * @param  writebuffer, writebuffer length and command strength
  * @retval None
  */
BaseType_t prvTrack(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString){
	long lParam_len;
	const char *cCmd_string;
	const char *name = "RADIO";
	*pcWriteBuffer = 0x00;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	if(!(strcmp(cCmd_string, "on"))){

		if( xQueueSendToFront(s4344046_QueueResumeTask, ( void * ) &name, ( portTickType ) 10 ) != pdPASS ) {
			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}

	} else if(!(strcmp(cCmd_string, "off"))) {

		if( xQueueSendToFront(s4344046_QueueSuspendTask, ( void * ) &name, ( portTickType ) 10 ) != pdPASS ) {
			debug_printf("Failed to post the message, after 10 ticks.\n\r");
		}
	}
	return pdFALSE;
}
