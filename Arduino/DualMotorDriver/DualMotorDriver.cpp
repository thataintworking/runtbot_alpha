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
 *      0x00    RO      Device ID. Should always be 0x68
 *      0x01    RO      Software Version
 *      0x02    RW      Left Wheel Direction: 0x00 stop, 0x01 forward, 0x02 reverse
 *      0x03    RW      Left Wheel Target speed clicks-per-second
 *      0x04    RO      Left Wheel Actual speed clicks-per-second
 *      0x05    RO      Left Wheel Current PWM output MSB
 *      0x06    RO      Left Wheel Current PWM output LSB
 *      0x07    RW      Right Wheel Direction: 0x00 stop, 0x01 forward, 0x02 reverse
 *      0x08    RW      Right Wheel Target speed clicks-per-second
 *      0x09    RO      Right Wheel Actual speed clicks-per-second
 *      0x0A    RO      Right Wheel Current PWM output MSB
 *      0x0B    RO      Right Wheel Current PWM output LSB
 ***/

#include "DualMotorDriver.h"
#include "Wheel.h"
#include <Wire.h>


volatile unsigned long lw_clicks;
volatile unsigned long rw_clicks;
volatile byte read_register;
volatile byte write_register;
volatile byte write_value;

unsigned long last_millis;
Wheel* left_wheel;
Wheel* right_wheel;

struct RegisterValues {
    byte device_id;
    byte software_verion;
    WheelData left;
    WheelData right;
};

const int rvsize = sizeof(RegisterValues);

union Registers {
    byte data[rvsize];
    RegisterValues vals;
} registers;


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
    registers.vals.left = left_wheel->getWheelData();
    registers.vals.right = right_wheel->getWheelData();
    if (read_register < rvsize) {
        Wire.write(registers.data + read_register, rvsize - read_register);
    } else {
        Wire.write(-1);
    }
    read_register = 0;
}


void lw_encoder_isr() {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    lw_clicks++;
}


void rw_encoder_isr() {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    rw_clicks++;
}


void setup() {
    Serial.begin(9600);

    Serial.println("Initialize global variables");
    read_register = write_register = write_value = 0;
    last_millis = millis();
    left_wheel = new Wheel("Left Wheel", LW_ENA_PIN, LW_FWD_PIN, LW_REV_PIN, LW_ENC_PIN);
    right_wheel = new Wheel("Right Wheel", LW_ENA_PIN, LW_FWD_PIN, LW_REV_PIN, LW_ENC_PIN);
    registers.vals.device_id = DEVICE_ID;
    registers.vals.software_verion = SOFTWARE_VERSION;

    Serial.println("Attaching interrupts");
    attachInterrupt(digitalPinToInterrupt(LW_ENC_PIN), lw_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RW_ENC_PIN), rw_encoder_isr, CHANGE);

    Serial.println("Initialize I2C");
    Wire.begin(I2C_ADDR);
    Wire.onReceive(i2c_receive);
    Wire.onRequest(i2c_request);
}


void loop() {
    switch (write_register) {
        case LW_DIR_REG:
        	if (write_value <= REV) {
        		left_wheel->changeDirection((Dir)write_value);
        	}
            break;
        case LW_SPEED_REG:
			left_wheel->changeSpeed(write_value);
            break;
        case RW_DIR_REG:
        	if (write_value <= REV) {
        		right_wheel->changeDirection((Dir)write_value);
        	}
            break;
        case RW_SPEED_REG:
            right_wheel->changeSpeed(write_value);
            break;
    }
    write_register = write_value = 0;

    if (millis() - last_millis >= 1000) { // it's been a second
        left_wheel->measureSpeed(lw_clicks);
        right_wheel->measureSpeed(rw_clicks);
        last_millis = millis();
    }

    left_wheel->loop();
    right_wheel->loop();

    delay(100);
}

