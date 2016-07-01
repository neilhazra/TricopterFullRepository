#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
byte numericalBuffer[2];   
int joystick[4] = {0,0,0,0};

int test[1];
RF24 radio(9, 10);

const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void)
{
  Serial.begin(115200);
  radio.begin();
  //radio.setDataRate(RF24_1MBPS);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setPayloadSize(12);
  //radio.enableDynamicPayloads();
  //radio.setAutoAck(0);
  radio.openWritingPipe(pipe);
  //radio.setStates({DYNPD:0x00,  FEATURE:0x00});
   radio.printDetails();
}

void loop()
{
  
if (Serial.available()>1) {
    numericalBuffer[0] = Serial.read();
    numericalBuffer[1] = Serial.read();
}

  if(numericalBuffer[0]<=100)  {
     joystick[0] = numericalBuffer[0];
  }
  else if (numericalBuffer[0] >= 101) {
    joystick[1] = numericalBuffer[0]-101;
            digitalWrite(13,HIGH);
  }
  ///////////////////////////////////////////////////////////
  if(numericalBuffer[1] <= 100)  {
     joystick[2] = numericalBuffer[1];
  }
  else if (numericalBuffer[1] >= 101) {
    joystick[3] = numericalBuffer[1]-101;
  }
  
  /*
  test[0] = numericalBuffer[1];
  radio.write(test, sizeof(test));
  */

  radio.write(joystick, sizeof(joystick));
}

