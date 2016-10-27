/**
  ******************************************************************************
  * @file    repo/project2/main.c
  * @author  Erick Almeida
  * @date    25052016
  * @brief   Program accelerometer and read is position according to the
	*					 datasheet (portrait or layout) and position (x, y, z). Additionally
	*					 use radio to receive messages to get position of a marker.
  *
	*	Modified: 25-May-2016: Copy entire file from milestone.
  *           26-May-2016: Add semaphore to get passkey and sensorkey.
  *                        Add function to decode pass key from message.
  *           28-May-2016: Make radio (ORB and rover) working
  *           29-May-2016: Fix radio idle state.
  *           30-May-2016: Set functions to move rover.
  *           31-May-2016: Send succesfully messages to rover.
  *           01-Jun-2016: Implement laser to track rover.
  *           02-Jun-2016: Calibrate movement for rover and tracking laser.
  *           03-jun-2016: Fix accelerometer to move rover.
  *
  *
  *	Functions:
	*	static void Hardware_init();
	*	void ApplicationIdleHook( void );
	*	void MANAGE_Task(void);
	*	void tim2_irqhandler (void);
	*	void READ_Task(void);
	*	uint16_t get_coordinate_x(uint8_t* radio_message, uint16_t x_coord);
	*	uint16_t get_coordinate_y(uint8_t* radio_message, uint16_t y_coord);
	*	uint16_t get_coordinate_height(uint8_t* radio_message, uint16_t height);
	*	uint16_t get_coordinate_width(uint8_t* radio_message, uint16_t width);
	*	uint16_t get_ID(uint8_t* radio_message, uint16_t id);
	*	uint8_t check_crc(uint8_t* message_radio);
  * uint8_t get_pass_key(uint8_t* radio_message);
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "FreeRTOS_CLI.h"
#include "s4344046_sysmon.h"
#include "s4344046_cli.h"
#include "s4344046_hamming.h"
#include "queue.h"
#include "s4344046_accelerometer.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"
#include "s4344046_radio.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;
/* Private define ------------------------------------------------------------*/
#define QUEUE_LENGTH    10
/* Binary semaphores have an effective length of 1. */
#define BINARY_SEMAPHORE_LENGTH	1

#define TOTAL_TASKS 6

/* The combined length of the two queues and binary semaphore that will be
added to the queue set. */
#define COMBINED_LENGTH_ACC ( QUEUE_LENGTH*0 + BINARY_SEMAPHORE_LENGTH*2 )
#define COMBINED_LENGTH_SET ( QUEUE_LENGTH*5 + BINARY_SEMAPHORE_LENGTH*4 )
#define COMBINED_LENGTH_SEM ( QUEUE_LENGTH*3 + BINARY_SEMAPHORE_LENGTH*1 )
#define COMBINED_LENGTH_ROVER ( QUEUE_LENGTH*4 )
#define MOVE_FORWARD 0x50
#define MOVE_REVERSE 0xA0
#define MOVE_NEGATIVE 0x90
#define MOVE_POSITIVE 0x60
#define NO_ERROR 0x00
#define pan_tilt_velocity 1.45

/* Private macro -------------------------------------------------------------*/
#define  BACK_SPACE   0x08

#define PI 3.14159265

/* Private variables ---------------------------------------------------------*/
QueueSetMemberHandle_t xActivatedMember;
QueueSetMemberHandle_t xActivatedAcc;
QueueSetMemberHandle_t xActivatedRov;

struct Message RecvMessage;
struct pass_msg RecvSensor;
struct pass_msg RecvKey;

struct calibration_rov final_forward;
struct calibration_rov final_reverse;

struct angle_calibration final_angle_pos;
struct angle_calibration final_angle_neg;
/*Initial Address for message*/

uint8_t roverAddress[5] = {0x00, 0x33, 0x22, 0x11, 0x00};
uint8_t get_type = 0x40;

/*index 9 = sequence change for each Message
  index 30 and 31 = CRC calculate after sequence has been modified*/
uint8_t getPassKey[32] = {0x30, 0x00, 0x33, 0x22, 0x11, 0x25, 0x11, 0x25,
  0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*index 9 = sequence change for each Message
  index 30 and 31 = CRC calculate after sequence has been modified
  index 10 = passkey for all messages
  index 11 = start payload encoded*/
uint8_t getPassSensor[32] = {0x31, 0x56, 0x34, 0x22, 0x11, 0x25, 0x11, 0x25,
  0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/*index 9 = sequence change for each MessageQueue
  index 30 and 31 = CRC calculate after sequence has been modified
  index 10 = passkey for all messages
  index 11 = start payload encoded*/
uint8_t getRoverMoved[32] = {0x32, 0x00, 0x33, 0x22, 0x11, 0x25, 0x11, 0x25,
  0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


//Struct that controls the rover's velocities, distance and time.
struct rover_control {
  uint8_t left_motor;
  uint8_t right_motor;
  uint8_t direction;
  uint8_t time_rov;
};

//Struct that keeps laser position
struct laser_control {
  uint16_t move_x;
  uint16_t move_y;
  uint16_t sand_x;
  uint16_t sand_y;
  uint8_t anglex;
  uint8_t angley;
};

struct rover_position {
  uint16_t read_x_pos;
  uint16_t read_y_pos;
};


//Create variables that controls rover position and target position
struct rover_position first_step_rover;
struct rover_position last_step_rover;
struct rover_position target_position;


//Create structs to hold rover position (first read and read after rover moved 5cm)
struct rover_position initial_rover_pos;
struct rover_position final_rover_pos;

//Create variable that keeps info of laser
struct laser_control move_laser;

//Create a variable that holds rover information
struct rover_control move_rover;

struct adjust_laser track_point;


static uint8_t message_radio[33] = {};
static uint8_t* rover_radio;
/* stores the address of the station*/
uint8_t address[4];
static uint16_t x_final;
static uint16_t y_final;
static uint16_t height_final;
static uint16_t width_final;
static uint16_t id_final;
volatile long count_interrupt;
uint8_t get_sequence = 0x00;


/* Private function prototypes -----------------------------------------------*/
static void Hardware_init();
void ApplicationIdleHook( void ); // The idle hook is just used to stream data to the USB port.
void CLI_Task(void);
void MANAGE_Task(void);
void tim2_irqhandler (void);
void READ_Task(void);
void ROVER_move(void);
void AUTO_task(void);

uint16_t get_coordinate_x(uint8_t* radio_message, uint16_t x_coord);
uint16_t get_coordinate_y(uint8_t* radio_message, uint16_t y_coord);
uint16_t get_coordinate_height(uint8_t* radio_message, uint16_t height);
uint16_t get_coordinate_width(uint8_t* radio_message, uint16_t width);
uint16_t get_ID(uint8_t* radio_message, uint16_t id);
uint8_t check_crc(uint8_t* message_radio);
uint16_t calculate_crc(uint8_t* message_radio);
uint8_t get_pass_key(uint8_t* radio_message);


/* Task Priorities ------------------------------------------------------------*/
#define mainTAKETASK_PRIORITY					( tskIDLE_PRIORITY + 3 )
#define mainCLI_PRIORITY					( tskIDLE_PRIORITY + 5 )
#define mainMANAGE_PRIORITY       (tskIDLE_PRIORITY + 6)


/* Task Stack Allocations -----------------------------------------------------*/
#define mainGIVETASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainTAKETASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainCLI_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 3 )
#define mainMANAGE_TASK_STACK_SIZE ( configMINIMAL_STACK_SIZE * 5 )


/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main( void ) {

		BRD_init();
		Hardware_init();
		/*Start timer counter as 0*/
		count_interrupt = 0;



		/* Start tasks */
		xTaskCreate( (void *) &s4344046_TaskRadio, (const signed char *) "RADIO", mainTAKETASK_STACK_SIZE, NULL, mainTAKETASK_PRIORITY, NULL );
	  xTaskCreate( (void *) &CLI_Task, (const signed char *) "CLI", mainCLI_TASK_STACK_SIZE, NULL, mainCLI_PRIORITY, NULL );
	  xTaskCreate( (void *) &MANAGE_Task, (const signed char *) "MANAGE", mainMANAGE_TASK_STACK_SIZE, NULL, mainMANAGE_PRIORITY, NULL );
		xTaskCreate( (void *) &s4344046_TaskAcc, (const signed char *) "ACCEL.", mainCLI_TASK_STACK_SIZE, NULL, mainMANAGE_PRIORITY, NULL );
		xTaskCreate( (void *) &READ_Task, (const signed char *) "READ", mainCLI_TASK_STACK_SIZE, NULL, mainMANAGE_PRIORITY, NULL );
    xTaskCreate( (void *) &ROVER_move, (const signed char *) "ROVER", mainCLI_TASK_STACK_SIZE, NULL, mainMANAGE_PRIORITY, NULL );
    xTaskCreate( (void *) &AUTO_task, (const signed char *) "FOLLOW", mainCLI_TASK_STACK_SIZE, NULL, mainCLI_PRIORITY, NULL );


	  /* Register CLI commands */
		FreeRTOS_CLIRegisterCommand(&xTop);
	  FreeRTOS_CLIRegisterCommand(&xSuspend);
	  FreeRTOS_CLIRegisterCommand(&xResume);
	  FreeRTOS_CLIRegisterCommand(&xHamenc);
	  FreeRTOS_CLIRegisterCommand(&xHamdec);
		FreeRTOS_CLIRegisterCommand(&xCrc);
		FreeRTOS_CLIRegisterCommand(&xAcc);
		FreeRTOS_CLIRegisterCommand(&xTrack);
    FreeRTOS_CLIRegisterCommand(&xRfchanset);
    FreeRTOS_CLIRegisterCommand(&xPassKey);
    FreeRTOS_CLIRegisterCommand(&xSensor);
    FreeRTOS_CLIRegisterCommand(&xGetTime);
    FreeRTOS_CLIRegisterCommand(&xForward);
    FreeRTOS_CLIRegisterCommand(&xReverse);
    FreeRTOS_CLIRegisterCommand(&xAngle);
    FreeRTOS_CLIRegisterCommand(&xCalibration);
    FreeRTOS_CLIRegisterCommand(&xOrb);
    FreeRTOS_CLIRegisterCommand(&xDistance);
    FreeRTOS_CLIRegisterCommand(&xAdjust);
    FreeRTOS_CLIRegisterCommand(&xLaser);



	  /*Start semaphores*/
	  s4344046_SemaphoreTop = xSemaphoreCreateBinary();
		s4344046_SemaphoreAcc = xSemaphoreCreateBinary();
		s4344046_SemaphoreAccPl = xSemaphoreCreateBinary();
    s4344046_SemaphoreKey = xSemaphoreCreateBinary();
    s4344046_SemaphoreSensor = xSemaphoreCreateBinary();
    s4344046_SemaphoreTime = xSemaphoreCreateBinary();
    s4344046_SemaphoreDistance = xSemaphoreCreateBinary();

		/*Start Queues*/
	  s4344046_QueueResumeTask = xQueueCreate(3, sizeof(int));		/* Create queue of length 10 Message items */
	  s4344046_QueueSuspendTask = xQueueCreate(3, sizeof(int));		/* Create queue of length 10 Message items */
		s4344046_QueueRadio = xQueueCreate(3, (sizeof(uint8_t))*32);
		s4344046_QueueAcc = xQueueCreate(3, sizeof(RecvMessage));
		s4344046_QueueAccPl = xQueueCreate(3, sizeof(RecvMessage));
    s4344046_QueueChan = xQueueCreate(3, sizeof(uint8_t));
    s4344046_QueueForward = xQueueCreate(3, sizeof(uint8_t));
    s4344046_QueueReverse = xQueueCreate(3, sizeof(uint8_t));
    s4344046_QueueCalibration = xQueueCreate(3, sizeof(final_forward));
    s4344046_QueueAngle = xQueueCreate(3, sizeof(int));
    s4344046_QueueAdjust = xQueueCreate(3, sizeof(track_point));


	  /* Create the queue set large enough to hold an event for every space in
	    every queue and semaphore that is to be added to the set. */
	  xQueueSet = xQueueCreateSet( COMBINED_LENGTH_SET );
		xQueueAcc = xQueueCreateSet( COMBINED_LENGTH_ACC );
		xQueueSem = xQueueCreateSet( COMBINED_LENGTH_SEM );
    xQueueRov = xQueueCreateSet( COMBINED_LENGTH_ROVER );

	  /* Check everything was created. */
	  configASSERT( xQueueSet );
	  configASSERT( s4344046_QueueResumeTask );
	  configASSERT( s4344046_QueueSuspendTask );
	  configASSERT( s4344046_SemaphoreTop );
    configASSERT( s4344046_SemaphoreKey );
    configASSERT( s4344046_SemaphoreSensor );
    configASSERT( s4344046_SemaphoreTime );
    configASSERT( s4344046_QueueChan );
    configASSERT( s4344046_QueueAdjust );

		configASSERT( xQueueAcc );
		configASSERT( s4344046_SemaphoreAcc );
		configASSERT( s4344046_SemaphoreAccPl);

		configASSERT( xQueueSem );
		configASSERT( s4344046_QueueAccPl );
		configASSERT( s4344046_QueueAcc );
		configASSERT( s4344046_QueueRadio );
    configASSERT( s4344046_SemaphoreDistance );

    configASSERT( xQueueRov );
    configASSERT( s4344046_QueueAngle );
    configASSERT( s4344046_QueueReverse );
    configASSERT( s4344046_QueueForward );
    configASSERT( s4344046_QueueCalibration );

	  /* Add the queues and semaphores to the set.  Reading from these queues and
	    semaphore can only be performed after a call to xQueueSelectFromSet() has
	    returned the queue or semaphore handle from this point on. */
	  xQueueAddToSet( s4344046_QueueResumeTask, xQueueSet );
	  xQueueAddToSet( s4344046_QueueSuspendTask, xQueueSet );
	  xQueueAddToSet( s4344046_SemaphoreTop, xQueueSet );
    xQueueAddToSet( s4344046_SemaphoreKey, xQueueSet );
    xQueueAddToSet( s4344046_SemaphoreSensor, xQueueSet );
    xQueueAddToSet( s4344046_SemaphoreTime, xQueueSet );
    xQueueAddToSet( s4344046_QueueChan, xQueueSet );
    xQueueAddToSet( s4344046_QueueAdjust, xQueueSet );


		xQueueAddToSet( s4344046_SemaphoreAcc, xQueueAcc );
		xQueueAddToSet( s4344046_SemaphoreAccPl, xQueueAcc );

		xQueueAddToSet( s4344046_QueueAcc, xQueueSem );
		xQueueAddToSet( s4344046_QueueAccPl, xQueueSem );
		xQueueAddToSet( s4344046_QueueRadio, xQueueSem );
    xQueueAddToSet( s4344046_SemaphoreDistance, xQueueSem );

    xQueueAddToSet( s4344046_QueueAngle, xQueueRov );
    xQueueAddToSet( s4344046_QueueReverse, xQueueRov );
    xQueueAddToSet( s4344046_QueueForward,xQueueRov );
    xQueueAddToSet( s4344046_QueueCalibration, xQueueRov );

		/* Start the scheduler.

		NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
		The processor MUST be in supervisor mode when vTaskStartScheduler is
		called.  The demo applications included in the FreeRTOS.org download switch
		to supervisor mode prior to main being called.  If you are not using one of
		these demo application projects then ensure Supervisor mode is used here. */

		vTaskStartScheduler();

		/* We should never get here as control is now taken by the scheduler. */
	  	return 0;
}



/**
  * @brief  Check constantly the status of the accelerometer and move the rover.
  * @param  None
  * @retval None
  */
void AUTO_task(void) {
  uint8_t orientation;
  uint8_t linear_distance = 60;
  int angular_distance_pos = 20;
  int angular_distance_neg = -20;

  write_register_value(CTRL_REG1, STAND_MODE);
  write_register_value(PL_CONFIG, PL_MODE);
  write_register_value(CTRL_REG1, ACTIVE_MODE);
  for(;;) {
      orientation = read_register(PL_REG);
      if((orientation == 0x80) || (orientation == 0x00) ||
          (orientation == 0x81) || (orientation == 0x01)){

          if( xQueueSendToFront(s4344046_QueueForward, ( void * ) &linear_distance, ( portTickType ) 10 ) != pdPASS ) {
          		debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
      } else if((orientation == 0x82) || (orientation == 0x02) ||
          (orientation == 0x83) || (orientation == 0x03)) {

          if( xQueueSendToFront(s4344046_QueueReverse, ( void * ) &linear_distance, ( portTickType ) 10 ) != pdPASS ) {

          		debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
      } else if((orientation == 0x84) || (orientation == 0x04) ||
          (orientation == 0x85) || (orientation == 0x05)) {

          if( xQueueSendToFront(s4344046_QueueAngle, ( void * ) &angular_distance_pos, ( portTickType ) 10 ) != pdPASS ) {

          		debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
      } else if((orientation == 0x86) || (orientation == 0x06) ||
          (orientation == 0x87) || (orientation == 0x07)) {

          if( xQueueSendToFront(s4344046_QueueAngle, ( void * ) &angular_distance_neg, ( portTickType ) 10 ) != pdPASS ) {

            		debug_printf("Failed to post the message, after 10 ticks.\n\r");
          }
      } else {

          debug_printf("ERROR READING TRY AGAIN\n");
      }
      vTaskDelay(4000);
  }
}

/**
  * @brief  Controls all the movements for the rover, included calibration.
  * @param  None
  * @retval None
  */
void ROVER_move(void) {
    uint8_t rover_dir;
    uint16_t hamming_left;
    uint16_t hamming_right;
    uint16_t hamming_control;
    double new_time;
    int angle;
    move_rover.left_motor = 20;
    move_rover.right_motor = 20;
    move_rover.time_rov = 0x05;
    for(;;) {
        xActivatedRov = xQueueSelectFromSet( xQueueRov, portMAX_DELAY );

        /*ENCODE 8-bit VARIABLE*/
        uint8_t rov_dir;
        struct calibration_rov new_motor;
        if(xActivatedRov == s4344046_QueueAngle) {
          /*CALCULATE NEW TIME HERE-ADDITIONALLY CHECK WHEN ABGLE IS NEGATIVE OF POSITIVE*/
          if (xQueueReceive( s4344046_QueueAngle, &angle, 10)) {
              if(angle < 0) {
                  /*CALCULATE TIME TO MOVE ROVER AT ANGLE HERE*/
                  move_rover.direction = MOVE_NEGATIVE;
                  /*dont forget to add time*/
                  move_rover.left_motor = final_angle_neg.left_wheel;
                  move_rover.right_motor = final_angle_neg.right_wheel;

              } else {
                  /*CALCULATE TIME TO MOVE ROVER AT ANGLE HERE*/
                  move_rover.direction = MOVE_POSITIVE;
                  move_rover.left_motor = final_angle_pos.left_wheel;
                  move_rover.right_motor = final_angle_pos.right_wheel;
              }

              /*HERE IS TIME CALCULATED FOR ANGLE AND VELOCITY OF MOTORS*/

              new_time = (abs(angle) * 9)/90.0;

              move_rover.time_rov = (int)new_time;
              new_time -= (int)new_time;
              if(new_time > 0.5) {
                  move_rover.time_rov++;
              }
              rov_dir = move_rover.direction | move_rover.time_rov;
              debug_printf("TIME: %d\n", move_rover.time_rov);
          }

        } else if (xActivatedRov == s4344046_QueueForward) {
          /*CALCULATE NEW TIME HERE*/
          if (xQueueReceive( s4344046_QueueForward, &rover_dir, 10)) {
              move_rover.direction = MOVE_FORWARD;

              new_time = (rover_dir)/50.0;
              move_rover.time_rov = (int)new_time;
              new_time -= (int)new_time;
              if(new_time > 0.5){
                  move_rover.time_rov++;
              }

              rov_dir = move_rover.direction | move_rover.time_rov;
              if((final_forward.left_wheel != 0) && (final_forward.right_wheel != 0)) {
                  move_rover.left_motor = final_forward.left_wheel;
                  move_rover.right_motor = final_forward.right_wheel;
              }
          }

        } else if (xActivatedRov == s4344046_QueueReverse) {
          /*CALCULATE NEW TIME HERE*/
          if (xQueueReceive( s4344046_QueueReverse, &rover_dir, 10)) {

              move_rover.direction = MOVE_REVERSE;
              new_time = (rover_dir)/50.0;
              move_rover.time_rov = (int)new_time;
              new_time -= (int)new_time;
              if(new_time > 0.5){
                  move_rover.time_rov++;
              }

              rov_dir = move_rover.direction | move_rover.time_rov;
              if((final_reverse.left_wheel != 0) && (final_reverse.right_wheel != 0)) {
                  move_rover.left_motor = final_reverse.left_wheel;
                  move_rover.right_motor = final_reverse.right_wheel;
              }
          }
        } else if (xActivatedRov == s4344046_QueueCalibration) {

          if (xQueueReceive( s4344046_QueueCalibration, &new_motor, 10)) {
              move_rover.time_rov = 0x01;

              if(new_motor.type == 1) {
                move_rover.direction = MOVE_FORWARD;
                rov_dir = move_rover.direction | move_rover.time_rov;
                /*Linear calibration*/
                final_forward.left_wheel = new_motor.left_wheel;
                final_forward.right_wheel = new_motor.right_wheel;
                move_rover.left_motor = final_forward.left_wheel;
                move_rover.right_motor = final_forward.right_wheel;
                debug_printf("rigth: %d    left: %d\n", new_motor.right_wheel, new_motor.left_wheel);

              } else if (new_motor.type == 0) {
                move_rover.time_rov = 0x09;
                move_rover.direction = MOVE_POSITIVE;
                rov_dir = move_rover.direction | move_rover.time_rov;
                /*Angular calibration*/
                final_angle_pos.left_wheel = new_motor.left_wheel;
                final_angle_pos.right_wheel = new_motor.right_wheel;
                move_rover.left_motor = final_angle_pos.left_wheel;
                move_rover.right_motor = final_angle_pos.right_wheel;
                debug_printf("rigth: %d    left: %d\n", new_motor.right_wheel, new_motor.left_wheel);

              } else if (new_motor.type == 2) {

                move_rover.direction = MOVE_REVERSE;
                rov_dir = move_rover.direction | move_rover.time_rov;
                /*Linear calibration*/
                final_reverse.left_wheel = new_motor.left_wheel;
                final_reverse.right_wheel = new_motor.right_wheel;
                move_rover.left_motor = final_reverse.left_wheel;
                move_rover.right_motor = final_reverse.right_wheel;
                debug_printf("rigth: %d    left: %d\n", new_motor.right_wheel, new_motor.left_wheel);

              } else if (new_motor.type == 3) {
                move_rover.time_rov = 0x09;
                move_rover.direction = MOVE_NEGATIVE;
                rov_dir = move_rover.direction | move_rover.time_rov;
                /*Angular calibration*/
                final_angle_neg.left_wheel = new_motor.left_wheel;
                final_angle_neg.right_wheel = new_motor.right_wheel;
                move_rover.left_motor = final_angle_neg.left_wheel;
                move_rover.right_motor = final_angle_neg.right_wheel;
                debug_printf("rigth: %d    left: %d\n", new_motor.right_wheel, new_motor.left_wheel);
              }
              /*MOVE ROVER HERE TO TEST*/

          }
        }
        hamming_left = s4344046_hamming_byte_encoder(move_rover.left_motor, NO_ERROR);
        hamming_right = s4344046_hamming_byte_encoder(move_rover.right_motor, NO_ERROR);
        hamming_control = s4344046_hamming_byte_encoder(rov_dir, NO_ERROR);

        getRoverMoved[9] = get_sequence;
        getRoverMoved[11] = (hamming_left & 0x00FF);
        getRoverMoved[12] = (hamming_left & 0xFF00) >> 8;
        getRoverMoved[13] = (hamming_right & 0x00FF);
        getRoverMoved[14] = (hamming_right & 0xFF00) >> 8;
        getRoverMoved[16] = (hamming_control & 0x00FF);
        getRoverMoved[15] = (hamming_control & 0xFF00) >> 8;

        get_sequence++;

        uint16_t crc_new;
        crc_new = calculate_crc(getRoverMoved);
        getRoverMoved[30] = crc_new & 0xFF;
        getRoverMoved[31] = (crc_new >> 8) & 0xFF;


        s4344046_set_tx_state();
        s4344046_radio_sendpacket(s4344046_radio_getchan(), address,
            getRoverMoved);


        int i;

        for(i = 0; i < 32; i++) {
          debug_printf("%02x",getRoverMoved[i]);
        }
        debug_printf("\n");

        vTaskDelay(0);
    }
}


/**
  * @brief  Read from buffer and print in the specified format.
  * @param  None
  * @retval None
  */
void READ_Task(void) {
		uint8_t orientation;

		for(;;){
				xActivatedAcc = xQueueSelectFromSet( xQueueSem, portMAX_DELAY );
				/*If buffer contains coordinates for x y and z*/
        if(xActivatedAcc == s4344046_SemaphoreDistance){
          //add distance here
          debug_printf("DISTANCE ROVER: %d\n", move_laser.sand_x);

        } else if(xActivatedAcc == s4344046_QueueAcc) {

						if (xQueueReceive( s4344046_QueueAcc, &RecvMessage, 10)) {
								/* display received item */
								debug_printf("x: %d      y: %d      z: %d\n\r", RecvMessage.x_location,
											RecvMessage.y_location, RecvMessage.z_location);
		            	/* Toggle LED */
								BRD_LEDToggle();
		    		}
				/*If buffer contains Portrait or landscape location*/
				} else if(xActivatedAcc == s4344046_QueueAccPl) {

						if (xQueueReceive( s4344046_QueueAccPl, &orientation, 10)) {
				            	/* Toggle LED */
								if((orientation == 0x80) || (orientation == 0x00) ||
										(orientation == 0x81) || (orientation == 0x01)){

										debug_printf("|\n");

								} else if((orientation == 0x82) || (orientation == 0x02) ||
										(orientation == 0x83) || (orientation == 0x03)) {

										debug_printf("_\n");

								} else if((orientation == 0x84) || (orientation == 0x04) ||
										(orientation == 0x85) || (orientation == 0x05)) {

										debug_printf(">\n");

								} else if((orientation == 0x86) || (orientation == 0x06) ||
										(orientation == 0x87) || (orientation == 0x07)) {

										debug_printf("<\n");

								} else {

										debug_printf("ERROR READING TRY AGAIN\n");
								}
								BRD_LEDToggle();
		    		}
				/*If buffer contains a radio message*/
				} else if (xActivatedAcc == s4344046_QueueRadio) {

					int i;

					if (xQueueReceive( s4344046_QueueRadio, &message_radio, 10)) {
              if(message_radio[0] == 0x40){
    							/*Print entire message received from radio*/
    							debug_printf("\n");
    							/*ACTIVATE LINE TO PRINT MESSAGE*/

    							/*Check if CRC is correct*/
    							if(check_crc(message_radio)){

    								/*Read x coordinate*/
    								x_final = get_coordinate_x(message_radio, x_final);
    								/*Read y coordinate*/
    								y_final = get_coordinate_y(message_radio, y_final);
    								/*Read height*/
    								height_final = get_coordinate_height(message_radio, height_final);
    								/*Read width*/
    								width_final = get_coordinate_width(message_radio, width_final);
    								/*Read ID*/
    								id_final = get_ID(message_radio, id_final);

    								/*Check for errors in coordinates*/
    								if((x_final <= 319) && (y_final <= 199) && (width_final <= 320) && (height_final <= 200)) {

    										double x, ret, val;
    									  val = 180.0 / PI;
    									  ret = atan(((float)width_final)/((float)height_final)) * val;
                        move_laser.sand_x = (uint16_t)(x_final * 900 / 319.0);
                        move_laser.sand_y = (uint16_t)(y_final * 600 / 199.0);
                        PRINT_RED("Received from radio\n");
                        debug_printf("id: %d   height: %d   width: %d   ", id_final, height_final, width_final);
    									  debug_printf("Angle: %.02f   ", ret);
    										debug_printf("X: %d  Y: %d\n", x_final, y_final);

                        move_laser.move_x = (uint16_t)((x_final / 319.0)*43);
                        move_laser.move_y = (uint16_t)((y_final / 199.0)*31);

                        show_angle(0, move_laser.move_x + track_point.angle_x_axis);
                        show_angle(1, track_point.angle_y_axis - move_laser.move_y);

    								} else {
    										debug_printf("BAD COORDINATES OR DIMENSIONS\n");
    								}

    							} else {

    									debug_printf("BAD MESSAGE - CRC INCORRECT\n");
    							}
    					}

            }
				}
				vTaskDelay(0);
		}
}


/**
  * @brief  Suspend or resume a task and additional print output of command TOP.
  * @param  None
  * @retval None
  */
void MANAGE_Task(void){
	  for(;;){
		    /* Block to wait for something to be available from the queues or
		        semaphore that have been added to the set.*/
		    xActivatedMember = xQueueSelectFromSet( xQueueSet,
		                                                portMAX_DELAY );

		    if(xActivatedMember == s4344046_QueueResumeTask){

			      BRD_LEDToggle();
			      volatile UBaseType_t uxArraySize;
			      TaskStatus_t pxTaskStatusArray[TOTAL_TASKS];
			      unsigned long ulTotalRunTime;
			      int i;
			      char* message;

			      if (xQueueReceive( s4344046_QueueResumeTask, &message, 10 )) {

			        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
			                                       uxArraySize,
			                                       &ulTotalRunTime );

  						 /*Check all tasks name to resume it*/
			      	for(i = 0; i < (int)uxArraySize; i++){

			      		if(!(strcmp(pxTaskStatusArray[i].pcTaskName, message))){

			      			vTaskResume(pxTaskStatusArray[i].xHandle);
			            break;
			      		}
			      	}
			      }

		    } else if(xActivatedMember == s4344046_QueueSuspendTask){

			      BRD_LEDToggle();
			      volatile UBaseType_t uxArraySize;
			      TaskStatus_t pxTaskStatusArray[TOTAL_TASKS];
			      unsigned long ulTotalRunTime;
			      int i;
			      char* message;

			      if (xQueueReceive( s4344046_QueueSuspendTask, &message, 10 )) {

			        	uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
			                                       uxArraySize,
			                                       &ulTotalRunTime );

								/*Check for all tasks name to suspend it*/
				      	for(i = 0; i < (int)uxArraySize; i++){

				      			if(!(strcmp(pxTaskStatusArray[i].pcTaskName, message))){

				      					vTaskSuspend(pxTaskStatusArray[i].xHandle);
				            		break;
				      			}
				      	}
			      }

		    } else if(xActivatedMember == s4344046_SemaphoreTop) {

		      if(xSemaphoreTake( s4344046_SemaphoreTop, 0 ) == pdTRUE){

		        volatile UBaseType_t uxArraySize, x;
		        TaskStatus_t pxTaskStatusArray[6];
		        unsigned long ulTotalRunTime;
		        int i;
		        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
		                                       uxArraySize,
		                                       &ulTotalRunTime );

		        debug_printf("Name\tNumber\tPriority\tState\tRunning Time\n");
		        debug_printf("*************************************************************\n");

						/*Print all task name, number, priority and time*/
		        for(i = 0; i < (int)uxArraySize; i++) {

		          char task_buffer[40];
		          sprintf(task_buffer, "%s\t%d\t%d\t\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
		                  pxTaskStatusArray[i].uxCurrentPriority);

		          if(pxTaskStatusArray[i].eCurrentState == eBlocked){

								sprintf(task_buffer, "%s\t%d\t%d\t\t%s\t%ld\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
			                  pxTaskStatusArray[i].uxCurrentPriority, "Block", pxTaskStatusArray[i].ulRunTimeCounter);
		            PRINT_RED(task_buffer);

		          } else if(pxTaskStatusArray[i].eCurrentState == eRunning){

								sprintf(task_buffer, "%s\t%d\t%d\t\t%s\t%ld\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
			                  pxTaskStatusArray[i].uxCurrentPriority, "Running", pxTaskStatusArray[i].ulRunTimeCounter);
		            PRINT_GREEN(task_buffer);

		          } else if(pxTaskStatusArray[i].eCurrentState == eReady){

								sprintf(task_buffer, "%s\t%d\t%d\t\t%s\t%ld\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
			                  pxTaskStatusArray[i].uxCurrentPriority, "Ready", pxTaskStatusArray[i].ulRunTimeCounter);
		            PRINT_YELLOW(task_buffer);

		          } else if(pxTaskStatusArray[i].eCurrentState == eSuspended){

								sprintf(task_buffer, "%s\t%d\t%d\t\t%s\t%ld\t", pxTaskStatusArray[i].pcTaskName, pxTaskStatusArray[i].xTaskNumber,
			                  pxTaskStatusArray[i].uxCurrentPriority, "Suspend", pxTaskStatusArray[i].ulRunTimeCounter);
		            PRINT_BLUE(task_buffer);
		          }
		        }
		      }
		    } else if (xActivatedMember == s4344046_SemaphoreKey) {
            if(xSemaphoreTake( s4344046_SemaphoreKey, 0 ) == pdTRUE){
                int i;
                uint16_t crc_new;
                getPassKey[9] = get_sequence;
                crc_new = calculate_crc(getPassKey);
                getPassKey[30] = crc_new & 0xFF;
                getPassKey[31] = (crc_new >> 8) & 0xFF;
                //add her hamming decoded for payload
                for(i = 0; i < 32; i++){
                   debug_printf("%02x", getPassKey[i]);
                }
                debug_printf("\n");
                s4344046_set_tx_state();
          			s4344046_radio_sendpacket(s4344046_radio_getchan(), address,
          					getPassKey);
                get_sequence++;

                radio_fsm_setstate(0);
                s4344046_radio_setfsmrx();
                s4344046_radio_fsmprocessing();
                while (s4344046_radio_getrxstatus() != 1) {
                    s4344046_radio_fsmprocessing();
                }
                s4344046_radio_getpacket(s4344046_get_pointer_rx());
                uint8_t new_pass;
                rover_radio = s4344046_getpacket();
                for(i = 0; i < 32; i++) {

                    debug_printf("%02x", rover_radio[i]);
                }

                debug_printf("\n");
                new_pass = get_pass_key(rover_radio);
                RecvKey.message = new_pass;
                RecvKey.timer_value = count_interrupt/1000.0;
                debug_printf("PASSKEY: %02X        TIME: %.02f\n",
                    RecvKey.message, RecvKey.timer_value);

                getPassSensor[10] = new_pass;
                getRoverMoved[10] = new_pass;

            }
        } else if (xActivatedMember == s4344046_SemaphoreSensor) {
            if(xSemaphoreTake( s4344046_SemaphoreSensor, 0 ) == pdTRUE){
                int i;
                uint16_t crc_new;
                getPassSensor[9] = get_sequence;
                crc_new = calculate_crc(getPassSensor);
                getPassSensor[30] = crc_new & 0xFF;
                getPassSensor[31] = (crc_new >> 8) & 0xFF;
                //add here hamming decoded for payload
                for(i = 0; i < 32; i++){
                   debug_printf("%02x", getPassSensor[i]);
                }
                debug_printf("\n");
                s4344046_set_tx_state();
          			s4344046_radio_sendpacket(s4344046_radio_getchan(), address,
          					getPassSensor);
                get_sequence++;

                radio_fsm_setstate(0);
                s4344046_radio_setfsmrx();
                s4344046_radio_fsmprocessing();
                while (s4344046_radio_getrxstatus() != 1) {
                    s4344046_radio_fsmprocessing();
                }
                s4344046_radio_getpacket(s4344046_get_pointer_rx());
                uint8_t new_pass;
                rover_radio = s4344046_getpacket();
                for(i = 0; i < 32; i++) {

                    debug_printf("%02x", rover_radio[i]);
                }

                debug_printf("\n");
                new_pass = get_pass_key(rover_radio);
                s4344046_lightbar_write(new_pass);
                RecvSensor.message = new_pass;
                RecvSensor.timer_value = count_interrupt/1000.0;
                debug_printf("SENSOR: %02X        TIME: %.02f\n",
                    RecvSensor.message, RecvSensor.timer_value);

                s4344046_lightbar_write(RecvSensor.message);

            }
        } else if (xActivatedMember == s4344046_SemaphoreTime) {
            if(xSemaphoreTake( s4344046_SemaphoreTime, 0 ) == pdTRUE){
                float get_running_time;
                get_running_time = count_interrupt/1000.0;
                debug_printf("Running time: %.02f\n", get_running_time);

            }
        } else if (xActivatedMember == s4344046_QueueChan) {
            uint8_t message;
            int radio_chan;
            if (xQueueReceive( s4344046_QueueChan, &message, 10 )) {
                int i;
                radio_fsm_setstate(0);
                roverAddress[0] = message;
                getPassKey[1] = message;
                getRoverMoved[1] = message;
                getPassSensor[1] = message;
                radio_chan = (message/16 * 10) + message%16;
                s4344046_radio_settxaddress(&roverAddress);
                s4344046_radio_setchan(radio_chan);
                radio_fsm_buffer_write(NRF24L01P_RX_ADDR_P0, roverAddress, 5); //To send between NP2
                s4344046_radio_gettxaddress(&address);

                get_type = 0x30;
            }
        } else if(xActivatedMember == s4344046_QueueAdjust) {
              if (xQueueReceive( s4344046_QueueAdjust, &track_point, 10 )) {
                  debug_printf("Check Laser:   X: %d   Y: %d\n", track_point.angle_x_axis, track_point.angle_y_axis);
                  show_angle(0, track_point.angle_x_axis);
                  show_angle(1, track_point.angle_y_axis);
              }
        }

		    vTaskDelay(10);
	  }
}



/**
  * @brief  CLI Receive Task.
  * @param  None
  * @retval None
  */
void CLI_Task(void) {

		char cRxedChar;
		char cInputString[100];
		int InputIndex = 0;
		char *pcOutputString;
		BaseType_t xReturned;

		/* Initialise pointer to CLI output buffer. */
		memset(cInputString, 0, sizeof(cInputString));
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		for (;;) {

			/* Receive character */
			cRxedChar = debug_getc();

	    if((int)cRxedChar != 127){
	  		/* Process if chacater if not Null */
	  		if (cRxedChar != '\0') {

	  			/* Put byte into USB buffer */
	  			debug_putc(cRxedChar);

	  			/* Process only if return is received. */
	  			if (cRxedChar == '\r') {

	  				//Put new line and transmit buffer
	  				debug_putc('\n');
	  				debug_flush();

	  				/* Put null character in command input string. */
	  				cInputString[InputIndex] = '\0';

	  				xReturned = pdTRUE;
	  				/* Process command input string. */
	  				while (xReturned != pdFALSE) {

	  					/* Returns pdFALSE, when all strings have been returned */
	  					xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

	  					/* Display CLI output string */
	  					debug_printf("%s\n\r",pcOutputString);
	  				    vTaskDelay(5);	//Must delay between debug_printfs.
	  				}
	  				memset(cInputString, 0, sizeof(cInputString));
	  				InputIndex = 0;
	  			} else {
	  				debug_flush();		//Transmit USB buffer
	  				if( cRxedChar == '\r' ) {

	  					/* Ignore the character. */
	  				} else if( cRxedChar == '\b' ) {

	  					/* Backspace was pressed.  Erase the last character in the
	  					 string - if any.*/
	  					if( InputIndex > 0 ) {
	  						InputIndex--;
	  						cInputString[ InputIndex ] = '\0';
	  					}
	  				} else {

	  					/* A character was entered.  Add it to the string
	  					   entered so far.  When a \n is entered the complete
	  					   string will be passed to the command interpreter. */
	  					if( InputIndex < 20 ) {

	  						cInputString[ InputIndex ] = cRxedChar;
	  						InputIndex++;
	  					}
	  				}
	  			}
	  		}
			/*Fix delete function*/
	    } else{

	      if( InputIndex > 0 ) {

	        InputIndex--;
	        cInputString[ InputIndex ] = '\0';
	      }
	      debug_putc('\b');
	      debug_putc(' ');
	      debug_putc('\b');
	      debug_flush();
	    }
	    vTaskDelay(50);
		}
}




/* Defined in main.c. */
void vConfigureTimerForRunTimeStats( void )
{
		unsigned short PrescalerValue;
		/* Timer 2 clock enable */
		__TIM3_CLK_ENABLE();

		/* Compute the prescaler value */
		PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1;		//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

		/* Time base configuration */
		TIM_Init.Instance = TIM3;				//Enable Timer 2
		TIM_Init.Init.Period = 50000/1000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
		TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
		TIM_Init.Init.ClockDivision = 0;			//Set clock division
		TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
		TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

		/* Initialise Timer */
		HAL_TIM_Base_Init(&TIM_Init);

		/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
		/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
		HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

		/* Enable timer update interrupt and interrupt vector for Timer  */
		NVIC_SetVector(TIM3_IRQn, (uint32_t)&tim2_irqhandler);
		NVIC_EnableIRQ(TIM3_IRQn);

		/* Start Timer */
		HAL_TIM_Base_Start_IT(&TIM_Init);
}


/**
  * @brief  Start counter for calculate the time that a task is running
  * @param  None.
  * @retval None.
  */
void tim2_irqhandler( void )
{
    /* Interrupt handler for TIM 6

    The time base for the run time stats is generated by the 16 bit timer 6.
    Each time the timer overflows ulTIM6_OverflowCount is incremented.
    Therefore, when converting the total run time to a 32 bit number, the most
    significant two bytes are given by ulTIM6_OverflowCount and the least
    significant two bytes are given by the current TIM6 counter value.  Care
    must be taken with data    consistency when combining the two in case a timer
    overflow occurs as the value is being read. */

		//Clear Update Flag
		__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);
		count_interrupt++;		//increment counter, when the interrupt occurs


}


/**
  * @brief  Calculate the y-coordinate given the buffer message and the variable
	*					where the y-coordinate is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					y_coord - variable that will store the value for the y-coordinate
  * @retval the value for the coordinate y.
  */
uint16_t get_coordinate_y(uint8_t* radio_message, uint16_t y_coord) {
		int i;
		int j = 0;
		uint8_t temp_y[4];
		uint16_t get_y_high;
		uint16_t get_y_low;
		j = 0;

		for(i = 18; i < 22; i++) {

				temp_y[j] = radio_message[i];
				j++;
		}

		get_y_high = temp_y[0];
		get_y_high = (get_y_high << 8) | temp_y[1];

		get_y_low = temp_y[2];
		get_y_low = (get_y_low << 8) | temp_y[3];

		y_coord = (s4344046_hamming_byte_decoder((get_y_high & 0xFF00) >>8, (get_y_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_y_low & 0xFF00)>>8, (get_y_low & 0x00FF))<<8);

		return y_coord;

}



/**
  * @brief  Calculate the x-coordinate given the buffer message and the variable
	*					where the x-coordinate is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					x_coord - variable that will store the value for the x-coordinate
  * @retval the value for the coordinate x.
  */
uint16_t get_coordinate_x(uint8_t* radio_message, uint16_t x_coord) {
		int i;
		int j = 0;
		uint8_t temp_x[4];
		uint16_t get_x_high;
		uint16_t get_x_low;

		for (i = 14; i < 18; i++) {

				temp_x[j] = radio_message[i];
				j++;
		}

		get_x_high = temp_x[0];
		get_x_high = (get_x_high << 8) | temp_x[1];

		get_x_low = temp_x[2];
		get_x_low = (get_x_low << 8) | temp_x[3];

		x_coord = (s4344046_hamming_byte_decoder((get_x_high & 0xFF00) >>8, (get_x_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_x_low & 0xFF00)>>8, (get_x_low & 0x00FF))<<8);
		return x_coord;
}


/**
  * @brief  Calculate the height given the buffer message and the variable
	*					where the height is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					height - variable that will store the value for the height
  * @retval the value for the height.
  */
uint16_t get_coordinate_height(uint8_t* radio_message, uint16_t height) {
		int i;
		int j = 0;
		uint8_t temp_height[4];
		uint16_t get_height_high;
		uint16_t get_height_low;

		for (i = 26; i < 30; i++) {

				temp_height[j] = radio_message[i];
				j++;
		}

		get_height_high = temp_height[0];
		get_height_high = (get_height_high << 8) | temp_height[1];

		get_height_low = temp_height[2];
		get_height_low = (get_height_low << 8) | temp_height[3];

		height = (s4344046_hamming_byte_decoder((get_height_high & 0xFF00) >>8, (get_height_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_height_low & 0xFF00)>>8, (get_height_low & 0x00FF))<<8);

	return height;
}


/**
  * @brief  Decode passkey from message.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
  * @retval the passkey value calcuated from message.
  */
uint8_t get_pass_key(uint8_t* radio_message) {
    uint16_t pass_enc;
    uint8_t pass_dec;
    pass_enc = radio_message[11];
		pass_enc = (pass_enc << 8) | radio_message[12];
    pass_dec = (s4344046_hamming_byte_decoder((pass_enc & 0xFF00) >>8, (pass_enc & 0x00FF)));
    return pass_dec;
}



/**
  * @brief  Calculate the width given the buffer message and the variable
	*					where the width is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					width - variable that will store the value for the width
  * @retval the value for the coordinate width.
  */
uint16_t get_coordinate_width(uint8_t* radio_message, uint16_t width) {
		int i;
		int j = 0;
		uint8_t temp_width[4];
		uint16_t get_width_high;
		uint16_t get_width_low;

		for (i = 22; i < 26; i++) {

				temp_width[j] = radio_message[i];
				j++;
		}

		get_width_high = temp_width[0];
		get_width_high = (get_width_high << 8) | temp_width[1];

		get_width_low = temp_width[2];
		get_width_low = (get_width_low << 8) | temp_width[3];

		width = (s4344046_hamming_byte_decoder((get_width_high & 0xFF00) >>8, (get_width_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_width_low & 0xFF00)>>8, (get_width_low & 0x00FF))<<8);

		return width;
}


/**
  * @brief  Calculate the ID given the buffer message and the variable
	*					where the ID is going to be stored.
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
	*					id - variable that will store the value for the ID
  * @retval the value for the ID.
  */
uint16_t get_ID(uint8_t* radio_message, uint16_t id) {
		int i;
		int j = 0;
		uint8_t temp_id[4];
		uint16_t get_id_high;
		uint16_t get_id_low;

		for (i = 10; i < 14; i++) {

				temp_id[j] = radio_message[i];
				j++;
		}

		get_id_high = temp_id[0];
		get_id_high = (get_id_high << 8) | temp_id[1];

		get_id_low = temp_id[2];
		get_id_low = (get_id_low << 8) | temp_id[3];

		id = (s4344046_hamming_byte_decoder((get_id_high & 0xFF00) >>8, (get_id_high & 0x00FF))) |
							(s4344046_hamming_byte_decoder((get_id_low & 0xFF00)>>8, (get_id_low & 0x00FF))<<8);

		return id;
}


/**
  * @brief  Calculate the crc for the entire message (30 bytes, last 2 bytes
	*					are to check if crc is correct or not)
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
  * @retval the value for the crc calculated from message given.
  */
uint16_t calculate_crc(uint8_t* message_radio) {
		uint16_t output_crc = 0x0000;
		uint16_t crc_check = ((message_radio[30]<<8) | message_radio[31]);
		int i;

		for(i = 0; i < 30; i++) {

				output_crc = s4344046_crc_update(output_crc, message_radio[i]);
		}

		return output_crc;
}


/**
  * @brief  Calculate the crc for the entire message (30 bytes, last 2 bytes
	*					are to check if crc is correct or not)
  * @param  radio_message - buffer that contains the entire message read from
	*													the radio.
  * @retval if the crc calculated match the crc from message.
  */
uint8_t check_crc(uint8_t* message_radio) {
		uint16_t output_crc = 0x0000;
		uint16_t crc_check = ((message_radio[31]<<8) | message_radio[30]);
		int i;

		for(i = 0; i < 30; i++) {

				output_crc = s4344046_crc_update(output_crc, message_radio[i]);
		}

		if(output_crc == crc_check) {

				return 1;
		} else {
				return 0;
		}
}



/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
static void Hardware_init( void ) {

		GPIO_InitTypeDef GPIO_InitStructure;

		portDISABLE_INTERRUPTS();	//Disable interrupts
		Hardware_init_Acc();

		BRD_LEDInit();				//Initialise Blue LED
		BRD_LEDOff();				//Turn off Blue LED

    /*CONFIG LASER*/
    __BRD_D1_GPIO_CLK();
    /* Configure the D1 pin as an input */
		GPIO_InitStructure.Pin = BRD_D1_PIN;				/*Pin D1*/
	  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;			/*Output Mode*/
		/*Enable Pull up, down or no pull resister*/
	  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			/*Pin latency*/
		/*Initialise Pin D1*/
	  HAL_GPIO_Init(BRD_D1_GPIO_PORT, &GPIO_InitStructure);


		vConfigureTimerForRunTimeStats();
    s4344046_pantilt_init(pan_tilt_velocity, pan_tilt_velocity);
    show_angle(0, 0);
    show_angle(1, 0);

		portENABLE_INTERRUPTS();	//Enable interrupts

		s4344046_radio_init();
    s4344046_lightbar_init();

		HAL_Delay(5000); //delay for 5 seconds
}

/**
  * @brief  Application Tick Task.
  * @param  None
  * @retval None
  */
void vApplicationTickHook( void ) {

		BRD_LEDOff();
}

/**
  * @brief  Idle Application Task (Disabled)
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
		static portTickType xLastTx = 0;

		BRD_LEDOff();

		for (;;) {
				/* The idle hook simply prints the idle tick count */
				if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {

						xLastTx = xTaskGetTickCount();
				}
		}
}

/**
  * @brief  vApplicationStackOverflowHook
  * @param  Task Handler and Task Name
  * @retval None
  */
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {

		/* This function will get called if a task overflows its stack.   If the
		parameters are corrupt then inspect pxCurrentTCB to find which was the
		offending task. */

		BRD_LEDOff();
		( void ) pxTask;
		( void ) pcTaskName;

		for( ;; );
}
