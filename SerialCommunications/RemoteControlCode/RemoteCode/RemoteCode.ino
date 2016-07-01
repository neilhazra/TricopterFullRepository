#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

int joystick[1];
int numericalBuffer;

RF24 radio(9, 10);

const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void)
{
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(pipe);
}
`
void loop(void)
{
  if (Serial.available()) {
    joystick[0] = Serial.read();
  }

  
  radio.write(joystick, sizeof(joystick));
}

