/***
 * DualMotorDriver
 *
 * A dual motor driver implemented on an ATmega328 (aka Arduino UNO)
 * that works with an L293 or similar H-bridge motor controller
 * and simple wheel encoders.  It uses a PID algorithm to keep
 * the wheels moving at the specified speed and provides an I2C
 * register-based interface.
 *
 * Author: Ron Smith
 * Copyright ©2017, That Ain't Working, All Rights Reserved
 *
 * I2C Device Address: 0x10
 *
 * I2C Registers:
 *      0   RO	B	Device ID. Should always be 0x68
 *      1   RO  B   Software Version
 *      2   RW  B   Left Wheel Direction: 1 = forward, 0 = reverse
 *      3   RW  B   Left Wheel Target Speed CPS/10
 *      4   RO  B   Left Wheel Actual Speed CPS/10
 *      5	RO  B	Left Wheel PWM
 *      6   RW  B   Right Wheel Direction: 1 = forward, 0 = reverse
 *      7   RW  B   Right Wheel Target Speed CPS/10
 *      8   RO  B   Right Wheel Actual Speed CPS/10
 *      9	RO  B	Right Wheel PWM
 ***/

#include "Arduino.h"
#include <Wire.h>

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

#define LW_DIR_REG    2
#define LW_SPEED_REG  3
#define LW_CPS_REG    4
#define RW_DIR_REG    6
#define RW_SPEED_REG  7
#define RW_CPS_REG    8
#define REG_NULL      255

volatile byte read_register, write_register, write_value;

struct RegisterValues {
	byte dev_id;
	byte sw_ver;
	byte lw_dir;
	byte lw_speed;
	byte lw_cps;
	byte lw_pwm;
	byte rw_dir;
	byte rw_speed;
	byte rw_cps;
	byte rw_pwm;
};

const int reg_len = sizeof(RegisterValues);

union Registers {
	RegisterValues v;
	byte b[reg_len];
} registers;

volatile unsigned long lw_clicks, rw_clicks;

unsigned long last_lw_clicks, last_rw_clicks;
int lw_pwm, rw_pwm;

unsigned int loop_count;
bool speed_change = false;


void lw_encoder_isr() {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
	lw_clicks++;
}


void rw_encoder_isr() {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
	rw_clicks++;
}

void i2c_receive(int n) {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    read_register = write_register = write_value = REG_NULL;
    if (n == 1) {
        read_register = Wire.read();
    } else if (n >= 2) {
        write_register = Wire.read();
        write_value = Wire.read();
    }
    while (Wire.available()) Wire.read();   // throw away any extra data
}


void i2c_request() {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    if (read_register < reg_len) {
		Wire.write(((byte*)registers.b) + read_register, reg_len - read_register);
    } else {
        Wire.write((byte)0);
    }
    read_register = REG_NULL;
}


void setup() {
	Serial.begin(9600);
	Serial.println();

	Serial.println("Initializing registers");
	registers.v.dev_id 		= 'M';
	registers.v.sw_ver 		= '2';
	registers.v.lw_dir		= 1;
	registers.v.lw_speed	= 0;
	registers.v.lw_cps		= 0;
	registers.v.lw_pwm		= 0;
	registers.v.rw_dir		= 1;
	registers.v.rw_speed	= 0;
	registers.v.rw_cps		= 0;
	registers.v.rw_pwm		= 0;


	Serial.println("Configuring pins");
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
	digitalWrite(5, LOW);
	digitalWrite(6, LOW);
	digitalWrite(9, LOW);
	digitalWrite(10, LOW);

	Serial.println("Configuring encoder");
	lw_clicks = rw_clicks = last_lw_clicks = last_rw_clicks = 0;
    attachInterrupt(digitalPinToInterrupt(3), lw_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), rw_encoder_isr, CHANGE);

    read_register = write_register = write_value = 255;
    Serial.print("Initializing I2C Slave Mode at 0x");
    Serial.println(I2C_ADDR, HEX);
    Wire.begin(I2C_ADDR);
    Wire.onReceive(i2c_receive);
    Wire.onRequest(i2c_request);

    loop_count = 0;
    speed_change = true;
}


void loop() {
	////////////////////////////
	// HANDLE REGISTER CHANGES
	////////////////////////////

	if (write_register < reg_len) {
		switch (write_register) {
			case LW_DIR_REG:
				registers.v.lw_dir = (bool)write_value;
				break;
			case RW_DIR_REG:
				registers.v.rw_dir = (bool)write_value;
				break;
			case LW_SPEED_REG:
				registers.v.lw_speed = write_value;
				if (write_value == 0) registers.v.lw_pwm = 0;
				break;
			case RW_SPEED_REG:
				registers.v.rw_speed = write_value;
				if (write_value == 0) registers.v.rw_pwm = 0;
				break;
		}

		write_register = write_value = REG_NULL;

		Serial.print("Dir/Speed: LEFT ");
		Serial.print(registers.v.lw_dir ? "FWD/" : "REV/");
		Serial.print(registers.v.lw_speed);
		Serial.print(", RIGHT ");
		Serial.print(registers.v.rw_dir ? "FWD/" : "REV/");
		Serial.print(registers.v.rw_speed);
		Serial.println();
	}

	////////////////////////////
	// CALCULATE CPS
	////////////////////////////

	if (lw_clicks > last_lw_clicks) { // this skips the time when the click counter wraps around
		unsigned long cps = lw_clicks - last_lw_clicks;
		registers.v.lw_cps = (byte)(cps < 255 ? cps : 255);
		speed_change = true;
	}
	last_lw_clicks = lw_clicks;

	if (rw_clicks > last_rw_clicks) { // this skips the time when the click counter wraps around
		unsigned long cps = rw_clicks - last_rw_clicks;
		registers.v.rw_cps = (byte)(cps < 255 ? cps : 255);
		speed_change = true;
	}
	last_rw_clicks = rw_clicks;

	/////////////////////////////////////////////////
	// ADJUST PWM TO TRY AND ATTAIN DESIRED SPEED
	////////////////////////////////////////////////

	if (registers.v.lw_cps < registers.v.lw_speed && registers.v.lw_pwm < 255) {
		Serial.println("Increasing PWM");
		registers.v.lw_pwm += 1;
		speed_change = true;
	} else if (registers.v.lw_cps > registers.v.lw_speed && registers.v.lw_pwm > 0) {
		Serial.println("Decreasing PWM");
		registers.v.lw_pwm -= 1;
		speed_change = true;
	}

	if (registers.v.rw_cps < registers.v.rw_speed && registers.v.rw_pwm < 255) {
		registers.v.rw_pwm++;
		speed_change = true;
	} else if (registers.v.rw_cps > registers.v.rw_speed && registers.v.rw_pwm > 0) {
		registers.v.rw_pwm--;
		speed_change = true;
	}

	////////////////////
	// SET PIN VALUES
	////////////////////

	digitalWrite(registers.v.lw_dir ? 6 : 5, LOW);
	if (registers.v.lw_speed > 0)
		analogWrite(registers.v.lw_dir ? 5 : 6, registers.v.lw_pwm);
	else
		digitalWrite(registers.v.lw_dir ? 5 : 6, LOW);

	digitalWrite(registers.v.rw_dir ? 10 : 9, LOW);
	if (registers.v.rw_speed > 0)
		analogWrite(registers.v.rw_dir ? 9 : 10, registers.v.rw_pwm);
	else
		digitalWrite(registers.v.rw_dir ? 9 : 10, LOW);

	///////////////////////////////////////////////////////////
	// LOG ONLY ONCE PER SECOND AND ONLY IF SPEED HAS CHANGED
	///////////////////////////////////////////////////////////

	if (loop_count < 10) {
		loop_count++;
	} else {
		if (speed_change) {
			Serial.print("CPS/PWM: LEFT ");
			Serial.print(registers.v.lw_cps);
			Serial.print("/");
			Serial.print(lw_pwm);
			Serial.print(", RIGHT ");
			Serial.print(registers.v.rw_cps);
			Serial.print("/");
			Serial.print(rw_pwm);
			Serial.println();
			speed_change = false;
		}
		loop_count = 0;
	}

	delay(100);  // thus CPS is actually clicks-per-tenth-of-a-second
}
