/*
 * Wheel.h
 *
 *  Created: Aug 25, 2017
 *  Author: Ron Smith
 *  Copyright ©2017, That Ain't Working, All Rights Reserved
 *
 */

#ifndef WHEEL_H_
#define WHEEL_H_

#include "Arduino.h"
#include <PID_v1.h>

struct WheelData {
	byte direction;
	byte target_speed;
	byte measured_speed;
	word pwm;
};

enum Dir { STP, FWD, REV };

class Wheel {
public:
	Wheel(const char* n, int ena_pin, int fwd_pin, int rev_pin, int enc_pin);
	virtual ~Wheel() {}

	void encoderISR();
	void measureSpeed(unsigned long clicks);
	void changeDirection(Dir d);
	void changeSpeed(unsigned int s);
	void stop();
	void loop();

	WheelData getWheelData();

private:
	void setL293Pins();

	const char* name;
	Dir direction;
	double target_speed;
	double measured_speed;
	double pwm;
	int enable_pin;
	int forward_pin;
	int reverse_pin;
	int encoder_pin;
	byte last_pwm;
	unsigned long last_clicks;
	PID pid;
};

#endif /* WHEEL_H_ */
