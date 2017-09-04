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
 *      0    RO      Device ID. Should always be 0x68
 *      1    RO      Software Version
 *      2    RW      Left Wheel Direction: 0 = stop, 1 = forward, 2 = reverse
 *      3    RW      Left Wheel Target speed clicks-per-second
 *      4    RO      Left Wheel Actual speed clicks-per-second
 *      5    RO      Left Wheel  PWM output
 *      6    RW      Right Wheel Direction: 0 = stop, 1 = forward, 2 = reverse
 *      7    RW      Right Wheel Target speed clicks-per-second
 *      8    RO      Right Wheel Actual speed clicks-per-second
 *      9    RO      Right Wheel PWM output
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
#define LW_CPS_REG    5
#define RW_DIR_REG    7
#define RW_SPEED_REG  8
#define RW_CPS_REG    10
#define REG_NULL      255

volatile byte read_register, write_register, write_value1, write_value2;

struct RegisterValues {
	byte dev_id;
	byte sw_ver;
	byte lw_dir;
	word lw_speed;
	word lw_cps;
	byte rw_dir;
	word rw_speed;
	word rw_cps;
};

const int reg_len = sizeof(RegisterValues);

union Registers {
	RegisterValues v;
	byte b[reg_len];
} registers;

volatile unsigned long lw_clicks, rw_clicks;

unsigned long last_lw_clicks, last_rw_clicks;

unsigned int loop_count;


void lw_encoder_isr() {
	lw_clicks++;
}


void rw_encoder_isr() {
	rw_clicks++;
}

void i2c_receive(int n) {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    read_register = write_register = write_value1 = write_value2 = REG_NULL;
    if (n == 1) {
        read_register = Wire.read();
    } else if (n >= 2) {
        write_register = Wire.read();
        write_value1 = Wire.read();
        if (n > 2) write_value2 = Wire.read();
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

    read_register = write_register = write_value1 = 255;
    Serial.print("Initializing I2C Slave Mode at 0x");
    Serial.println(I2C_ADDR, HEX);
    Wire.begin(I2C_ADDR);
    Wire.onReceive(i2c_receive);
    Wire.onRequest(i2c_request);

    loop_count = 0;
}


void loop() {
	if (write_register < reg_len) {
		switch (write_register) {
			case LW_DIR_REG:
				registers.v.lw_dir = (bool)write_value1;
				break;
			case RW_DIR_REG:
				registers.v.rw_dir = (bool)write_value1;
				break;
			case LW_SPEED_REG:
				registers.v.lw_speed = write_value1 + (write_value2 << 8);
				break;
			case RW_SPEED_REG:
				registers.v.rw_speed = write_value1 + (write_value2 << 8);
				break;
		}

		digitalWrite(registers.v.lw_dir ? 6 : 5, LOW);
		analogWrite(registers.v.lw_dir ? 5 : 6, registers.v.lw_speed);
		digitalWrite(registers.v.rw_dir ? 10 : 9, LOW);
		analogWrite(registers.v.rw_dir ? 9 : 10, registers.v.rw_speed);

		write_register = write_value1 = write_value2 = REG_NULL;

		Serial.print("Dir/Speed: LEFT ");
		Serial.print(registers.v.lw_dir ? "FWD/" : "REV/");
		Serial.print(registers.v.lw_speed);
		Serial.print(", RIGHT ");
		Serial.print(registers.v.rw_dir ? "FWD/" : "REV/");
		Serial.print(registers.v.rw_speed);
		Serial.println();
	}

	if (loop_count < 10) {
		loop_count++;
	} else {
		bool speed_change = false;

		if (lw_clicks > last_lw_clicks) { // this skips the time when the click counter wraps around
			registers.v.lw_cps = lw_clicks - last_lw_clicks;
			speed_change = true;
		}
		last_lw_clicks = lw_clicks;

		if (rw_clicks > last_rw_clicks) { // this skips the time when the click counter wraps around
			registers.v.rw_cps = rw_clicks - last_rw_clicks;
			speed_change = true;
		}
		last_rw_clicks = rw_clicks;

		loop_count = 0;

		if (speed_change) {
			Serial.print("CPS: LEFT ");
			Serial.print(registers.v.lw_cps);
			Serial.print(", RIGHT ");
			Serial.print(registers.v.rw_cps);
		}
	}
	delay(100);
}
