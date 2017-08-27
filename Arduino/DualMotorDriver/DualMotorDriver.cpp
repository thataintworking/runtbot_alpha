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

#define LW_DIR_REG    0x02
#define LW_SPEED_REG  0x03
#define RW_DIR_REG    0x07
#define RW_SPEED_REG  0x08

#define DEVICE_ID 77
#define SOFTWARE_VERSION 1

boolean forward = false;
byte speed = 0;

volatile byte read_register, write_register, write_value;
byte registers[] = { 'X', 1, 0, 0, 0, 0 };
const int reg_len = 6;

volatile unsigned long lw_clicks, rw_clicks;
unsigned long last_lw_clicks, last_rw_clicks;


void lw_encoder_isr() {
	lw_clicks++;
}


void rw_encoder_isr() {
	rw_clicks++;
}

void i2c_receive(int n) {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    read_register = write_register = 0;
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
    	registers[2] = (forward ? 1 : 0);
    	registers[3] = speed;
    	registers[4] = (byte)(lw_clicks - last_lw_clicks);
    	registers[5] = (byte)(rw_clicks - last_rw_clicks);
        Wire.write(registers[read_register]);
    } else {
        Wire.write((byte)0);
    }
    read_register = 255;
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

    read_register = write_register = write_value = 255;
    Serial.println("Initializing I2C");
    Wire.begin(0x20);
    Wire.onReceive(i2c_receive);
    Wire.onRequest(i2c_request);
}


void loop() {
	if (write_register < reg_len) {
		switch (write_register) {
			case 2:
				forward = (boolean)write_value;
				break;
			case 3:
				speed = write_value;
				break;
		}
		write_register = write_value = 255;
	}

	Serial.print("Dir/Speed: ");
	Serial.print(forward ? "FWD/" : "REV/");
	Serial.println(speed);
	digitalWrite(forward ? 6 : 5, LOW);
	analogWrite(forward ? 5 : 6, speed);
	digitalWrite(forward ? 10 : 9, LOW);
	analogWrite(forward ? 9 : 10, speed);

	if (last_lw_clicks != lw_clicks) {
		Serial.print("Left wheel clicks: ");
		Serial.println(lw_clicks - last_lw_clicks);
		last_lw_clicks = lw_clicks;
	}

	if (last_rw_clicks != rw_clicks) {
		Serial.print("Right wheel clicks: ");
		Serial.println(rw_clicks - last_rw_clicks);
		last_rw_clicks = rw_clicks;
	}
	delay(5000);
}
