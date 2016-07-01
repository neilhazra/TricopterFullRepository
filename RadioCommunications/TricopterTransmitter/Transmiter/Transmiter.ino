#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
int joystick[1];
RF24 radio(9, 10);
const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(pipe);
}

void loop() {
  if (Serial.available())
  {
    joystick[0] = Serial.read();
  }
  radio.write( joystick, sizeof(joystick));
}

