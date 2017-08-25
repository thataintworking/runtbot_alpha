/***
 * Wheel.cpp
 *
 * Author: Ron Smith
 * Copyright ©2017, That Ain't Working, All Rights Reserved
 ***/

#include "Wheel.h"


Wheel::Wheel(const char* n, int ena_pin, int fwd_pin, int rev_pin, int enc_pin) :
        name(n), direction(STOP), target_speed(0), measured_speed(0), pwm(0), prev_dir(STOP),
        enable_pin(ena_pin), forward_pin(fwd_pin), reverse_pin(rev_pin), encoder_pin(enc_pin),
        clicks(0), last_clicks(0), last_pwm(0), last_target_speed(0),
        pid(&measured_speed, &pwm, &target_speed, 2, 5, 1, P_ON_M, DIRECT)
{
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

void Wheel::encoder_isr() { clicks++; }

void Wheel::measure_speed() {
    double new_measured_speed = clicks - last_clicks;
    if (new_measured_speed != measured_speed) {
        Serial.print(name);
        Serial.print(": measured speed at ");
        measured_speed = new_measured_speed;
        last_clicks = clicks;
        Serial.println(measured_speed);
    }
}

void Wheel::change_direction(byte d) {
    if (d >= 0 && d <= 2) {
        prev_dir = direction;
        direction = d;
    }
}

void Wheel::change_speed(byte s) {
    target_speed = s;
}

void Wheel::stop() {
    digitalWrite(enable_pin, LOW);
    analogWrite(forward_pin, 0);
    analogWrite(reverse_pin, 0);
}

WheelData Wheel::wheel_data() {
    WheelData wd = {
        (byte)direction,
        (byte)target_speed,
        (byte)measured_speed,
        (word)pwm
    };
    return wd;
}

void Wheel::loop() {
    if (target_speed != last_target_speed) {
        Serial.print(name);
        Serial.print(": target speed changed to ");
        Serial.println(target_speed);
        last_target_speed = target_speed;
    }
    if (direction == STOP) {
        if (prev_dir != STOP) {
            Serial.print(name);
            Serial.println(": Direction changed to stop. Stopping.");
            stop();
        }
    } else {
        if (prev_dir != direction && prev_dir != STOP) {
            Serial.print(name);
            Serial.print(": Direction changed to ");
            Serial.print(direction == FORWARD ? "FORWARD" : "REVERSE");
            Serial.println(". Stopping.");
            stop();
        }
        pid.Compute();
        if (pwm != last_pwm) {
            Serial.print(name);
            Serial.print(": Computed PWM value changed: ");
            Serial.println(pwm);
            last_pwm = pwm;
        }
        if (direction == FORWARD) {
            analogWrite(reverse_pin, 0);
            analogWrite(forward_pin, (int)pwm);
        } else {
            analogWrite(forward_pin, 0);
            analogWrite(reverse_pin, (int)pwm);
        }
        digitalWrite(enable_pin, HIGH);
    }
    prev_dir = direction;
}
