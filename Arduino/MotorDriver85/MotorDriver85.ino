#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Wire.h>

#define I2C_ADDR 0x10
#define I2C_NUM_REGS 10

volatile int encoder_counter;                 // counts the "clicks" as the wheel turns
volatile int control_registers[I2C_NUM_REGS]; // values than can be read/written via I2C

void setup() {
  encoder_counter = 0;
  for (int x = 0; x < I2C_NUM_REGS; x++)
    control_registers[x] = 0;
  
  GIMSK |= (1 << PCIE);   // pin change interrupt enable
  PCMSK |= (1 << PCINT4); // pin change interrupt enabled for PCINT4 (aka PB4, physical pin 3)
  sei();                  // enable interrupts

  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}


void loop() {
  delay(100);
  // TODO: monitor the encoder_counter over time and adjust PWM to motor as needed
  // TODO: check for register changes and adjust/control motor as needed
}


void receiveEvent(int n) {
  while (Wire.available()) {
    int reg = Wire.read();
    
  }
}

void requestEvent() {
  // TODO: write me
}

ISR(PCINT0_vect) {
  ++encoder_counter;
}

