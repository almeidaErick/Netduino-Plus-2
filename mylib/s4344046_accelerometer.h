
/* -----------------------------------------------------------------------------
# Author name: Erick Almeida - 43440461
# Creation date: 140516
# Revision date (name): -
# Changes implemented (date):
#								14-May-2016 - Create functions to read and write into registers
#		            15-May-2016 - Simplify all write and read functions into a
#		                          general function.
#(Comments):
------------------------------------------------------------------------------*/


#ifndef S4344046_ACCELEROMETER_H
#define S4344046_ACCELEROMETER_H

/*Struct that will stores the value for coordinates in x y and z as hex and decimal
	values*/
struct Message {
	int16_t x_location;
	int16_t y_location;
  int16_t z_location;
  uint16_t x_hex;
  uint16_t y_hex;
  uint16_t z_hex;
};

#define MMA8452Q_ADDRESS	0x1D << 1
#define HIGH_X 0x01
#define LOW_X 0x02
#define HIGH_Y 0x03
#define LOW_Y 0x04
#define HIGH_Z 0x05
#define LOW_Z 0x06
#define PL_REG 0x10
#define PL_CONFIG 0x11
#define CTRL_REG1 0x2A

//CONTROL MODES OF operation
#define STAND_MODE 0x00
#define ACTIVE_MODE 0x01
#define PL_MODE 0x40

void Hardware_init_Acc();
int16_t get_twos_complement (uint16_t get_number);
void write_register_value(uint8_t reg_number, uint8_t reg_value);
uint8_t read_register(uint8_t register_num);
void s4344046_TaskAcc(void);

#endif
