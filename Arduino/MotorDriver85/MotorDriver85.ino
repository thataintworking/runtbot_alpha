/***
 * MotorDriver85
 * A motor driver implemented on an ATtiny85 that works with
 * an L293 or similar H-bridge motor controller and a simple
 * wheel encoder. It uses a PID algorithm to keep the wheel
 * moving at the specified speed and provides an I2C interface.
 *
 * I2C Device Address: 0x10 if pin 1 ~5v or 0x20 if pin 1 is ~3v
 *
 * NOTE: Be careful not to pull pin 1 to ground. It is the reset pin.
 *
 * I2C Registers:
 *      0x01    RO      Device ID. Should always be 0x66
 *      0x02    RO      Major Version of software
 *      0x03    RO      Minor Version of software
 *      0x04    RW      Direction: 0x00 stop, 0x01 forward, 0x02 reverse
 *      0x05    RW      Target speed clicks-per-second
 *      0x06    RO      Actual speed clicks-per-second
 *      0x07    RO      Click count MSB
 *      0x08    RO      Click count LSB
 *      0x09    RO      Current PWM output MSB
 *      0x0A    RO      Current PWM output LSB
 ***/

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <limits.h>
#include <Wire.h>
#include <PID_v1.h>

#define I2C_ADDR_A 0x10
#define I2C_ADDR_B 0x20
#define I2C_NUM_REGS 10

#define FORWARD_PIN  3
#define BACKWARD_PIN 4
#define ENCODER_PIN  PCINT1
#define ADDR_SEL_PIN A0


union ControlRegisters {
    byte reg[I2C_NUM_REGS];
    struct val {
        byte dev_id;
        byte major_version;
        byte minor_version;
        byte direction;
        byte target_speed;
        byte actual_speed;
        word pwm_output;
        word unused;
    } val;
};

volatile ControlRegisters registers;
volatile unsigned long clicks;

unsigned long last_millis;
unsigned long last_clicks;
byte prev_direction;
int dir_pin;

double pid_in, pid_out, pid_target;

PID motor_pid(&pid_in, &pid_out, &pid_target, 2, 5, 1, P_ON_M, DIRECT);


void setup() {
    pinMode(FORWARD_PIN, OUTPUT);
    pinMode(BACKWARD_PIN, OUTPUT);
    pinMode(ADDR_SEL_PIN, INPUT);
    pinMode(ENCODER_PIN, INPUT);
    delay(100);

    stop_motor();
    delay(100);

    registers.val.dev_id        = 0x66;
    registers.val.major_version = 1;
    registers.val.minor_version = 1;
    registers.val.direction     = 0;
    registers.val.target_speed  = 0;
    registers.val.actual_speed  = 0;
    registers.val.pwm_output    = 0;
    registers.val.unused        = 0;

    last_millis = millis();

    prev_direction = 0;

    pid_in = 0;
    pid_target = 0;
    motor_pid.SetMode(AUTOMATIC);

    GIMSK |= (1 << PCIE);         // pin change interrupt enable
    PCMSK |= (1 << ENCODER_PIN);  // pin change interrupt enabled for ENCODER_PIN
    sei();                        // enable interrupts

    delay(100);
    Wire.begin(analogRead(A0) > 900 ? I2C_ADDR_A : I2C_ADDR_B);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

void loop() {
    if (prev_direction != registers.val.direction) { // direction change received
        if (prev_direction > 0) stop_motor(); // stop before changing direction unless we already were
        if (registers.val.direction > 0) {
            switch (registers.val.direction) {
                case 1:
                    dir_pin = FORWARD_PIN;
                    break;
                case 2:
                    dir_pin = BACKWARD_PIN;
                    break;
                default:
                    dir_pin = -1;
                    registers.val.direction = 0;
            }
        }
        prev_direction = registers.val.direction;
    }

    if (millis() - last_millis >= 1000) { // it's been a second
        int s = clicks - last_clicks;
        if (s >= 0) registers.val.actual_speed = s;
        last_clicks = clicks;
        last_millis = millis();
    }

    if (registers.val.direction > 0 && dir_pin > 0) {
        pid_in = registers.val.actual_speed;
        pid_target = registers.val.target_speed;
        motor_pid.Compute();
        analogWrite(dir_pin, pid_out);
        registers.val.pwm_output = (int)pid_out;
    }
}


void stop_motor() {
    analogWrite(FORWARD_PIN, 0);
    analogWrite(BACKWARD_PIN, 0);
    dir_pin = -1;
}


void receiveEvent(int n) {
    if (n >= 2) {   // receive commands require 2 bytes
        int r = Wire.read();
        int v = Wire.read();
        switch (r) {
            case 0x04:  // set direction
                if (v >= 0 && v <= 2) registers.val.direction = v;
                break;
            case 0x05:  // set speed
                registers.val.target_speed = v;
                break;
        }
    }
    while (Wire.available()) Wire.read();   // throw away any extra data
}


void requestEvent() {
    // just send all the registers, it's only 10 bytes.
    Wire.write(registers.reg, I2C_NUM_REGS);
}


ISR(PCINT0_vect) {
    clicks++;
}
