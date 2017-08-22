/***
 * DualMotorDriver328
 * A dual motor driver implemented on an ATmega328 (aka Arduino UNO)
 * that works with an L293 or similar H-bridge motor controller
 * and simple wheel encoders.  It uses a PID algorithm to keep
 * the wheels moving at the specified speed and provides an I2C
 * register-based interface.
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

#include <stdio.h>
#include <Wire.h>
#include <PID_v1.h>

#define I2C_ADDR   0x10
#define I2C_NUM_REGS 16

#define LW_ENA_PIN  7
#define LW_FWD_PIN  6
#define LW_REV_PIN  5
#define LW_ENC_PIN  3
#define RW_ENA_PIN  12
#define RW_FWD_PIN  11
#define RW_REV_PIN  10
#define RW_ENC_PIN  2

#define LW_DIR_REG    0x02
#define LW_SPEED_REG  0x03
#define RW_DIR_REG    0x07
#define RW_SPEED_REG  0x08

#define STOP     0
#define FORWARD  1
#define REVERSE 2
#define LEFT     0
#define RIGHT    1

#define DEVICE_ID 77
#define SOFTWARE_VERSION 1
#define REG_SIZE 12

struct WheelData {
    byte direction;
    byte target_speed;
    byte measured_speed;
    word pwm;
};

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

class Wheel {

  private:

    const char* name;
    volatile byte direction;
    byte prev_dir;
    volatile double target_speed;
    volatile double measured_speed;
    volatile double pwm;
    double last_pwm;
    volatile unsigned long clicks;
    unsigned long last_clicks;
    PID pid;
    int enable_pin;
    int forward_pin;
    int reverse_pin;
    int encoder_pin;

  public:

    Wheel(const char* n, int ena_pin, int fwd_pin, int rev_pin, int enc_pin) :
            name(n), direction(STOP), target_speed(0), measured_speed(0), pwm(0), prev_dir(STOP), 
            enable_pin(ena_pin), forward_pin(fwd_pin), reverse_pin(rev_pin), encoder_pin(enc_pin), 
            clicks(0), last_clicks(0), last_pwm(0),
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
                
    void encoder_isr() { clicks++; }
    
    void measure_speed() {
        double new_measured_speed = clicks - last_clicks;
        if (new_measured_speed != measured_speed) {
            Serial.print(name);
            Serial.print(": measured speed at ");
            measured_speed = new_measured_speed;
            last_clicks = clicks;
            Serial.println(measured_speed);
        }
    }

void change_direction(byte d, boolean use_serial=false) {
        if (use_serial) Serial.print(name);
        if (d >= 0 && d <= 2) {
            if (use_serial) {
                Serial.print(": changing direction to ");
                Serial.println(d == 0 ? "STOP" : (d == 1 ? "FORWARD" : "REVERSE"));
            }
            prev_dir = direction;
            direction = d;
        } else if (use_serial) {
            Serial.print(": invalid value for change direction: ");
            Serial.println(d);
        }
    }

    void change_speed(byte s, boolean use_serial=false) {
        if (use_serial) Serial.print(name);
        if (s >= 0 && s <= 255) {
            if (use_serial) {
                Serial.print(": changing speed to ");
                Serial.println(s);
            }
            target_speed = s;
        } else if (use_serial) {
            Serial.print(": invalid value for change speed: ");
            Serial.println(s);
        }
    }
    
    void stop(boolean use_serial=false) {
        if (use_serial) {
            Serial.print(name);
            Serial.println(": stopping");
        }
        digitalWrite(enable_pin, LOW);
        analogWrite(forward_pin, 0);
        analogWrite(reverse_pin, 0);
    }

    WheelData regs() {
        WheelData wd = {
            (byte)direction,
            (byte)target_speed,
            (byte)measured_speed,
            (word)pwm
        };
        return wd;
    }
    
    void loop() {
        Serial.print(name);
        Serial.println(": Begin loop.");
        if (direction == STOP && prev_dir != STOP) {
            Serial.print(name);
            Serial.println(": Direction changed to stop. Stopping.");
            stop(true);
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
                analogWrite(forward_pin, pwm);
            } else {
                analogWrite(forward_pin, 0);
                analogWrite(reverse_pin, pwm);
            }
            digitalWrite(enable_pin, HIGH);
        }
        prev_dir = direction;
    }
};

Wheel* left_wheel;
Wheel* right_wheel;

unsigned long last_millis;
volatile int request_register;
byte i2creqbuf[rvsize+1];


void setup() {
    Serial.begin(9600);

    Serial.println("Initialize global variables");
    request_register = 0;
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
    if (millis() - last_millis >= 1000) { // it's been a second
        left_wheel->measure_speed();
        right_wheel->measure_speed();
        last_millis = millis();
    }
    left_wheel->loop();
    right_wheel->loop();
}


void stop_all_motors() {
    left_wheel->stop();
    right_wheel->stop();
}


void serial_print_hex(byte b) {
    char hexbuf[5];
    sprintf(hexbuf, "%02X ", b);
    Serial.print(hexbuf);
}


void i2c_receive(int n) {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    if (n == 1) {
        request_register = Wire.read();
    } else if (n >= 2) {
        int r = Wire.read();
        int v = Wire.read();
        switch (r) {
            case LW_DIR_REG:
                left_wheel->change_direction(v);
                break;
            case LW_SPEED_REG:
                left_wheel->change_speed(v);
                break;
            case RW_DIR_REG:
                right_wheel->change_direction(v);
                break;
            case RW_SPEED_REG:
                right_wheel->change_speed(v);
                break;
        }
    }
    while (Wire.available()) Wire.read();   // throw away any extra data
}

void i2c_request() {
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    registers.vals.left = left_wheel->regs();
    registers.vals.right = right_wheel->regs();
    if (request_register >= 0 && request_register < rvsize) {
        Wire.write(registers.data + request_register, rvsize - request_register);
    } else {
        Wire.write(-1);
    }
}


void lw_encoder_isr() {
    // attachInterrupt won't let you pass an object's method as the ISR
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    left_wheel->encoder_isr();
}


void rw_encoder_isr() {
    // attachInterrupt won't let you pass an object's method as the ISR
    // NOTE: This is an interrupt handler. Do not user Serial in this function.
    right_wheel->encoder_isr();
}
