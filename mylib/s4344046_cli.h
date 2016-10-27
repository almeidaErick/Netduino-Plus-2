
 /* -----------------------------------------------------------------------------
 # Author name: Erick Almeida - 43440461
 # Creation date: 050516
 # Revision date (name): -
 # Changes implemented (date):
 *            6-May-2016 - Fix semafores and queues for Stage 6.
 *						13-May-2016 - Implement the command Top, Suspend, Resume,
 *													Hamdec, hamenc, crc.
 *						15-May-2016 - Implement the command for Acc
 *						16-May-2016 - Implement the command for Tracking.
 *						18-May-2016 - Change the name for Tracking command from "TAKE" to
 *													"RADIO".
 *            28-May-2016 - Implement orb command that takes channel and orb
 *													number to set addres of the ORB.
 *						29-May-2016 - Set commands to move rover
 *						20-May-2016 - Implement calibration for rover.
 *
 #(Comments):
 ------------------------------------------------------------------------------*/


#ifndef S4344046_CLI_H
#define S4344046_CLI_H

/* Scheduler includes. */
#include "FreeRTOS.h"


#include "FreeRTOS_CLI.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define PRINT_RED(X) debug_printf("%s %s %s\n",ANSI_COLOR_RED,X,ANSI_COLOR_RESET)
#define PRINT_GREEN(X) debug_printf("%s %s %s\n",ANSI_COLOR_GREEN,X,ANSI_COLOR_RESET)
#define PRINT_YELLOW(X) debug_printf("%s %s %s\n",ANSI_COLOR_YELLOW,X,ANSI_COLOR_RESET)
#define PRINT_BLUE(X) debug_printf("%s %s %s\n",ANSI_COLOR_BLUE,X,ANSI_COLOR_RESET)

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


SemaphoreHandle_t s4344046_SemaphoreTop;
SemaphoreHandle_t s4344046_SemaphoreAcc;
SemaphoreHandle_t s4344046_SemaphoreAccPl;
SemaphoreHandle_t s4344046_SemaphoreKey;
SemaphoreHandle_t s4344046_SemaphoreSensor;
SemaphoreHandle_t s4344046_SemaphoreTime;
SemaphoreHandle_t s4344046_SemaphoreDistance;

QueueSetHandle_t xQueueSet;
QueueSetHandle_t xQueueAcc;
QueueSetHandle_t xQueueSem;
QueueSetHandle_t xQueueRov;

QueueSetMemberHandle_t xActivatedSem;

QueueHandle_t s4344046_QueueAcc;	/* Queue used */
QueueHandle_t s4344046_QueueAccPl;

QueueHandle_t s4344046_QueueEncoder;
QueueHandle_t s4344046_QueueDecoder;
QueueHandle_t s4344046_QueueResumeTask;
QueueHandle_t s4344046_QueueSuspendTask;
QueueHandle_t s4344046_QueueCrcTask;
QueueHandle_t s4344046_QueueRadio;
QueueHandle_t s4344046_QueueChan;
QueueHandle_t s4344046_QueueForward;
QueueHandle_t s4344046_QueueReverse;
QueueHandle_t s4344046_QueueCalibration;
QueueHandle_t s4344046_QueueAngle;
QueueHandle_t s4344046_QueueAdjust;
QueueHandle_t s4344046_QueueTarget;

int laser_on;
int laser_off;

/* Private variables ---------------------------------------------------------*/
CLI_Command_Definition_t xEcho;

/* Private variables ---------------------------------------------------------*/
CLI_Command_Definition_t xLaser;

CLI_Command_Definition_t xPan;

CLI_Command_Definition_t xTilt;

CLI_Command_Definition_t xBox;

CLI_Command_Definition_t xTop;

CLI_Command_Definition_t xSuspend;

CLI_Command_Definition_t xResume;

CLI_Command_Definition_t xHamenc;

CLI_Command_Definition_t xHamdec;

CLI_Command_Definition_t xCrc;

CLI_Command_Definition_t xAcc;

CLI_Command_Definition_t xTrack;

CLI_Command_Definition_t xRfchanset;

CLI_Command_Definition_t xPassKey;

CLI_Command_Definition_t xSensor;

CLI_Command_Definition_t xGetTime;

CLI_Command_Definition_t xForward;

CLI_Command_Definition_t xReverse;

CLI_Command_Definition_t xAngle;

CLI_Command_Definition_t xCalibration;

CLI_Command_Definition_t xOrb;

CLI_Command_Definition_t xDistance;

CLI_Command_Definition_t xAdjust;


struct pass_msg {
  uint8_t message;
  float timer_value;
};

struct calibration_rov {
  uint8_t left_wheel;
  uint8_t right_wheel;
  uint8_t type;
};

struct angle_calibration {
  uint8_t left_wheel;
  uint8_t right_wheel;
};

struct adjust_laser {
  int angle_x_axis;
  int angle_y_axis;
};




#endif
