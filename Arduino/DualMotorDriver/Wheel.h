/***
 * Wheel.h
 *
 * Author: Ron Smith
 * Copyright ©2017, That Ain't Working, All Rights Reserved
 ***/

#ifndef Wheel_h
#define Wheel_h

#include <PID_v1.h>


struct WheelData {
    byte direction;
    byte target_speed;
    byte measured_speed;
    word pwm;
};


class Wheel {

  private:

    const char* name;
    volatile byte direction;
    byte prev_dir;
    volatile double target_speed;
    volatile double measured_speed;
    volatile double pwm;
    double last_pwm;
    double last_target_speed;
    volatile unsigned long clicks;
    unsigned long last_clicks;
    PID pid;
    int enable_pin;
    int forward_pin;
    int reverse_pin;
    int encoder_pin;

  public:

    Wheel(const char* n, int ena_pin, int fwd_pin, int rev_pin, int enc_pin);

    void encoder_isr();

    void measure_speed();

    void change_direction(byte d);

    void change_speed(byte s);

    void stop();

    WheelData wheel_data();

    void loop();
};

#endif
