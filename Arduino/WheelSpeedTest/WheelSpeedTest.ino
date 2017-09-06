/***
 * WheelSpeedTest
 *
 * Runs through a range of PWM values and takes an average click count.
 ***/

#include "Arduino.h"

#define I2C_ADDR   0x10
#define I2C_NUM_REGS 16

#define LW_ENA_PIN  7
#define LW_FWD_PIN  5
#define LW_REV_PIN  6
#define LW_ENC_PIN  3

#define RW_ENA_PIN  12
#define RW_FWD_PIN  9
#define RW_REV_PIN  10
#define RW_ENC_PIN  2

const int out_pin[] = { LW_ENA_PIN, LW_FWD_PIN, LW_REV_PIN, RW_ENA_PIN, RW_FWD_PIN, RW_REV_PIN };
const int num_out_pins = sizeof(out_pin)/sizeof(out_pin[0]);

int speed;
unsigned long last_millis;

volatile unsigned long lw_clicks, rw_clicks;
unsigned long last_lw_clicks, last_rw_clicks;


void lw_encoder_isr() {
	lw_clicks++;
}


void rw_encoder_isr() {
	rw_clicks++;
}

void setup() {
	Serial.begin(9600);

	Serial.println("Configuring pins");
	for (int p = 0; p < num_out_pins; p++) {
		pinMode(p, OUTPUT);
		digitalWrite(p, LOW);
	}
	pinMode(LW_ENC_PIN, INPUT);
	pinMode(LW_ENC_PIN, INPUT);

	Serial.println("Configuring encoder interrupts");
	lw_clicks = rw_clicks = last_lw_clicks = last_rw_clicks = 0;
    attachInterrupt(digitalPinToInterrupt(3), lw_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), rw_encoder_isr, CHANGE);

    speed = 0;
    last_millis = millis();
}


void loop() {
	if (speed < 255) {
		speed += 32;
		if (speed > 255) speed = 255;
		Serial.print("Speed: ");
		Serial.print(speed);
		digitalWrite(LW_ENA_PIN, HIGH);
		digitalWrite(RW_ENA_PIN, HIGH);
		analogWrite(LW_FWD_PIN, speed);
		analogWrite(RW_FWD_PIN, speed);
		last_lw_clicks = lw_clicks;
		last_rw_clicks = rw_clicks;
		delay(5000);
		int lw_cps = (lw_clicks - last_lw_clicks) / 5;
		int rw_cps = (rw_clicks - last_rw_clicks) / 5;
		Serial.print(", LWCPS: ");
		Serial.print(lw_cps);
		Serial.print(", RWCPS: ");
		Serial.println(rw_cps);
	} else {
		for (int p = 0; p < num_out_pins; p++) {
			digitalWrite(p, LOW);
		}
		delay(10000);
	}
}
