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

#include <Wire.h>
#include <PID_v1.h>

#define I2C_ADDR   0x10
#define I2C_NUM_REGS 16

#define LW_ENA_PIN  7
#define LW_FWD_PIN  6
#define LW_BWD_PIN  5
#define LW_ENC_PIN  2
#define RW_ENA_PIN  12
#define RW_FWD_PIN  11
#define RW_BWD_PIN  10
#define RW_ENC_PIN  3

#define LW_DIR_REG    0x02
#define LW_SPEED_REG  0x03
#define RW_DIR_REG    0x07
#define RW_SPEED_REG  0x08

#define STOP     0
#define FORWARD  1
#define BACKWARD 2
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
}

struct RegisterValues {
    byte device_id;
    byte software_verion;
    WheelData left;
    WheelData right;
};

union Registers {
    byte data[sizeof(RegisterValues)];
    RegisterValues vals;
} registers;

class Wheel {

  private:

    const char* name;
    byte direction;
    byte prev_dir;
    double target_speed;
    double measured_speed;
    double pwm;
    volatile unsigned long clicks;
    unsigned long last_clicks;
    PID pid;
    int enable_pin;
    int forward_pin;
    int backward_pin;
    int encoder_pin;

  public:

    Wheel(const char* n, int ena_pin, int fwd_pin, int bwd_pin, int enc_pin) :
            name(n), data.direction(STOP), data.target_speed(0), data.measured_speed(0), data.pwm(0), prev_dir(STOP),
            enable_pin(ena_pin), forward_pin(fwd_pin), backward_pin(bwd_pin), encoder_pin(enc_pin),
            pid(&measured_speed, &pwm, &target_speed, 2, 5, 1, P_ON_M, DIRECT), clicks(0), last_clicks(0),
    {
        Serial.print(name);
        Serial.println(": initializing pins");
        pinMode(enable_pin, OUTPUT);
        pinMode(forward_pin, OUTPUT);
        pinMode(backward_pin, OUTPUT);
        pinMode(encoder_pin, INPUT);
        digitalWrite(enable_pin, LOW);
        analogWrite(forward_pin, 0);
        analogWrite(backward_pin, 0);
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

    void change_direction(byte d) {
        Serial.print(name);
        if (d >= 0 && d <= 2) {
            Serial.print(": changing direction to ");
            Serial.println(d == 0 ? "STOP" : (d == 1 ? "FORWARD" : "BACKWARD"));
            prev_dir = direction;
            direction = d;
        } else {
            Serial.print(": invalid value for change direction: ");
            Serial.println(d);
        }
    }

    void change_speed(byte s) {
        Serial.print(name);
        if (s >= 0 && s <= 255) {
            Serial.print(": changing speed to ");
            Serial.println(s);
            target_speed = s;
        } else {
            Serial.print(": invalid value for change speed: ");
            Serial.println(s);
        }
    }
    
    void stop() {
        Serial.print(name);
        Serial.println(": stopping");
        direction = STOP;
        digitalWrite(enable_pin, LOW);
        analogWrite(forward_pin, 0);
        analogWrite(backward_pin, 0);
        prev_dir = STOP;
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
        if (prev_dir != direction) {
            if (prev_dir != STOP) stop();
        }
        if (direction == STOP) {
            stop();
        } else {
            pid.Compute();
            if (direction == FORWARD) {
                analogWrite(backward_pin, 0);
                analogWrite(forward_pin, pwm);
            } else {
                analogWrite(forward_pin, 0);
                analogWrite(backward_pin, pwm);
            }
        }
    }
};

Wheel* left_wheel;
Wheel* right_wheel;

unsigned long last_millis;

void setup() {
    Serial.begin(9600);

    Serial.println("Initialize global variables");
    last_millis = millis();
    left_wheel = new Wheel("Left Wheel", LW_ENA_PIN, LW_FWD_PIN, LW_BWD_PIN, LW_ENC_PIN);
    right_wheel = new Wheel("Right Wheel", LW_ENA_PIN, LW_FWD_PIN, LW_BWD_PIN, LW_ENC_PIN);
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


void i2c_receive(int n) {
    Serial.print("I2C receiving ");
    Serial.print(n);
    Serial.println(" bytes");
    if (n >= 2) {
        int r = Wire.read();
        int v = Wire.read();
        switch (r) {
            case LW_DIR_REG:
                Serial.print("Received left wheel direction change: ");
                Serial.println(v);
                left_wheel->change_direction(v);
                break;
            case LW_SPEED_REG:
                Serial.print("Received left wheel speed change: ");
                Serial.println(v);
                left_wheel->change_speed(v);
                break;
            case RW_DIR_REG:
                Serial.print("Received right wheel direction change: ");
                Serial.println(v);
                right_wheel->change_direction(v);
                break;
            case RW_SPEED_REG:
                Serial.print("Received right wheel speed change: ");
                Serial.println(v);
                right_wheel->change_speed(v);
                break;
            default:
                Serial.print("Cannot update register ");
                Serial.println(r);
        }
    }
    while (Wire.available()) Wire.read();   // throw away any extra data
}


void i2c_request() {
    registers.vals.left = left_wheel->regs();
    registers.vals.right = right_wheel->regs()
    Write.send(registers.data, sizeof(RegisterValues))
}


void lw_encoder_isr() {
    // attachInterrupt won't let you pass an object's method as the ISR
    //Serial.println("Left Encoder");
    left_wheel->encoder_isr();
}


void rw_encoder_isr() {
    // attachInterrupt won't let you pass an object's method as the ISR
    //Serial.println("Right Encoder");
    right_wheel->encoder_isr();
}