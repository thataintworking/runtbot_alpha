// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _DualMotorDriver_H_
#define _DualMotorDriver_H_

#include "Arduino.h"

#define I2C_ADDR   0x10
#define I2C_NUM_REGS 16

#define LW_ENA_PIN  7
#define LW_FWD_PIN  6
#define LW_REV_PIN  5
#define LW_ENC_PIN  3
#define RW_ENA_PIN  12
#define RW_FWD_PIN  10
#define RW_REV_PIN  9
#define RW_ENC_PIN  2

#define LW_DIR_REG    0x02
#define LW_SPEED_REG  0x03
#define RW_DIR_REG    0x07
#define RW_SPEED_REG  0x08

#define DEVICE_ID 77
#define SOFTWARE_VERSION 1


//Do not add code below this line
#endif /* _DualMotorDriver_H_ */
