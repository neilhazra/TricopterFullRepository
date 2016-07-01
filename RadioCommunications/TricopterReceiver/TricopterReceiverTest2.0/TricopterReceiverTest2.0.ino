#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>
const int RIGHT_MOTOR_PIN = A1;
const int LEFT_MOTOR_PIN = A2;
const int TAIL_MOTOR_PIN = A0;
Servo TAIL_MOTOR;
Servo RIGHT_MOTOR;
Servo LEFT_MOTOR;
int joystick[4];
int test[1];
RF24 radio(40, 53);
const uint64_t pipe = 0xE8E8F0F0E1LL;


void setup(void)
{
  //TAIL_MOTOR.attach(TAIL_MOTOR_PIN);
  RIGHT_MOTOR.attach(RIGHT_MOTOR_PIN);
  //LEFT_MOTOR.attach(LEFT_MOTOR_PIN);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();
}

void loop(void)
{
  if ( radio.available() )
  {
      radio.read( joystick, sizeof(joystick));
      //TAIL_MOTOR.writeMicroseconds(map(joystick[2],0,100,1000,2000));
      //RIGHT_MOTOR.write(joystick[0]);
      //LEFT_MOTOR.write(joystick[1]);
      //TAIL_MOTOR.write(joystick[2]);
      //RIGHT_MOTOR.writeMicroseconds((joystick[0]*5.56)+1000);
      //RIGHT_MOTOR.write(joystick[0]);
      //LEFT_MOTOR.writeMicroseconds(map(joystick[1],0,100,1000,2000));
      
      Serial.println(joystick[1]);
     /*
      radio.read( test, sizeof(test));
      Serial.println(test[0]);
      */
}
  else
  {
    Serial.println("No radio available");
      TAIL_MOTOR.write(30);
      RIGHT_MOTOR.write(30);
      LEFT_MOTOR.write(30);
  }
}
