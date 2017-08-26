/*
 * Wheel.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: ron
 */

#include "Wheel.h"

const char* dir_name[] = { "STOP", "FORWARD", "REVERSE" };



Wheel::Wheel(const char* n, int ena_pin, int fwd_pin, int rev_pin, int enc_pin) :
	name(n), direction(STP), target_speed(0.0), measured_speed(0.0), pwm(0.0),
	enable_pin(ena_pin), forward_pin(fwd_pin), reverse_pin(rev_pin), encoder_pin(enc_pin),
	last_pwm(0), last_clicks(0L),
	pid(&measured_speed, &pwm, &target_speed, 2, 5, 1, P_ON_M, DIRECT)
{
	pid.SetOutputLimits(0, 255);
	Serial.print(name);
	Serial.println(": initializing pins");
	pinMode(enable_pin, OUTPUT);
	pinMode(forward_pin, OUTPUT);
	pinMode(reverse_pin, OUTPUT);
	pinMode(encoder_pin, INPUT);
	digitalWrite(enable_pin, LOW);
	analogWrite(forward_pin, 0);
	analogWrite(reverse_pin, 0);
 }

void Wheel::measureSpeed(unsigned long clicks) {
    double new_measured_speed = clicks - last_clicks;
    if (new_measured_speed != measured_speed) {
        Serial.print(name);
        Serial.print(": measured speed at ");
        measured_speed = new_measured_speed;
        last_clicks = clicks;
        Serial.println(measured_speed);
    }
}

void Wheel::setL293Pins() {
    switch (direction) {
    	case FWD:
			digitalWrite(reverse_pin, LOW);
			analogWrite(forward_pin, (byte)pwm);
	        digitalWrite(enable_pin, HIGH);
			break;
    	case REV:
    		digitalWrite(forward_pin, LOW);
			analogWrite(reverse_pin, (byte)pwm);
	        digitalWrite(enable_pin, HIGH);
			break;
    	case STP:
            digitalWrite(enable_pin, LOW);
    		digitalWrite(forward_pin, LOW);
			digitalWrite(reverse_pin, LOW);
    }
}

void Wheel::changeDirection(Dir d) {
	Serial.print(name);
    if (d == STP) {
    	Serial.print(name);
    	Serial.println(": stopping");
    } else {
    	if (direction != STP && direction != d)
    		digitalWrite(enable_pin, LOW); // temporary stop so we don't kill the motor
		Serial.print(": changing direction to ");
		Serial.println(dir_name[d]);
    }
	direction = d;
    setL293Pins();
}

void Wheel::changeSpeed(unsigned int s) {
	Serial.print(name);
	Serial.print(": changing target speed to ");
	Serial.println(s);
	target_speed = s;
}

void Wheel::stop() {
}

void Wheel::loop() {
	if (direction != STP) {
//		Serial.print(name);
//		Serial.println(": computing PWM");
        pid.Compute();
        if (last_pwm != (byte)pwm) {
            Serial.print(name);
            Serial.print(": Computed PWM value changed: ");
            Serial.println((byte)pwm);
    		setL293Pins();
            last_pwm = (byte)pwm;
        }
	}
}

WheelData Wheel::getWheelData() {
    WheelData wd = {
        (byte)direction,
        (byte)target_speed,
        (byte)measured_speed,
        (word)pwm
    };
    return wd;
}
