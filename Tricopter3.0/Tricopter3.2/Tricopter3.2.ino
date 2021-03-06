//Includes
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <EEPROM.h>
////////////////////////////////////////////////////////////////////////
//Board Connections
#define RMOTOR_PIN  A0
#define LMOTOR_PIN  A2
#define TMOTOR_PIN  A1
#define SERVO_PIN  A3
const int MPU_Interrupt_PIN = 1;//This is actually the interrupt number, not the pin number
// PID tunings ****************************************

double kpr = 0.75;      // rollInput
double kir = 0.5;
double kdr = 0.05;

double kpp = 0.75;      // pitchInput
double kip = 0.5;
double kdp = 0.05;

double kpy = 1.5;      // yawInput
double kiy = 0.02;
double kdy = 0.01;


#define PID_PITCH_MIN -15
#define PID_PITCH_MAX 15
#define PID_ROLL_MIN -10
#define PID_ROLL_MAX 10
#define INIT_ROLL 0.0
#define INIT_PITCH 0.0

// initial PID values. tested on stable platform / ground.
double RollSetpoint = INIT_ROLL;
double PitchSetpoint = INIT_PITCH;

// Global Variables ***********************************
double rollInput,pitchInput,yawInput,throttleInput; //PID input
double rollOutputLeft,rollOutputRight,pitchOutputForward,pitchOutputReverse,yawOutput;   // PID output

int tailMotorVal = 0;
int rightMotorVal = 0;
int leftMotorVal = 0;

//Gyro Data Variables
uint8_t mpuIntStatus,devStatus;
uint16_t packetSize,fifoCount;
uint8_t fifoBuffer[64];
VectorFloat gravity;
float ypr[3] = {0.0, 0.0, 0.0};
Quaternion q;
double smoothVal;
volatile boolean mpuInterrupt = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}
////////////////////////////////////////////////////////////////////////////
MPU6050 mpu;
//Servo Objects
Servo rightMotor;
Servo leftMotor;
Servo tailMotor;
Servo yawServo;
////////////////////////////////////////////////////////////////////////////
//PID Instances
PID PID_ROLL_LEFT(&rollInput, &rollOutputLeft, &RollSetpoint, kpr, kir, kdr, DIRECT);
PID PID_ROLL_RIGHT(&rollInput, &rollOutputRight, &RollSetpoint, kpr, kir, kdr, REVERSE);
PID PID_PITCH_FORWARD(&pitchInput, &pitchOutputForward, &PitchSetpoint, kpp, kip, kdp, DIRECT);
PID PID_PITCH_REVERSE(&pitchInput, &pitchOutputReverse, &PitchSetpoint, kpp, kip, kdp, REVERSE);
/////////////////////////////////////////////////////////////////////////////
byte numericalBuffer[2];  
int joystick[4] = {0,0,0,0};
long bluetoothLastUpdated; //Safety Feature Turns Tricopter off if there is a long gap between bluetooth transmissions
const int COM_DELAY_THRESHOLD = 5000;
double Throttle,Yaw,Pitch,Roll;


void setup()
{
   initMotors();
   Serial.begin(9600);//Serial for debug purposes
   Serial1.begin(9600); //Serial for joystick data
   Serial2.begin(9600); //Serial for inflight data / blackbox (not implemented requires additional bluetooth)
   pinMode(19, INPUT_PULLUP); //resister for Bluetooth 1
   //pinMode(21,INPUT_PULLUP); //resister for Bluetooth 2
       
 ///////////////////////////////////////////
 //***TODO: Wait for Serial Ports to connect
 ///////////////////////////////////////////
 
  Wire.begin();
  TWBR = 24;      // i2c bus @ 400KHz for MCU
 
  Serial2.println("Calibrating Motors"); //Although calibrated earlier inform flier now that serial communication has begun
  
  //Initialize Stabalizer
  PID_PITCH_FORWARD.SetMode(AUTOMATIC);
  PID_PITCH_FORWARD.SetOutputLimits(PID_PITCH_MIN,PID_PITCH_MAX);
  PID_PITCH_FORWARD.SetSampleTime(10);
  
  PID_PITCH_REVERSE.SetMode(AUTOMATIC);
  PID_PITCH_REVERSE.SetOutputLimits(PID_PITCH_MIN,PID_PITCH_MAX);
  PID_PITCH_REVERSE.SetSampleTime(10);
  
  PID_ROLL_LEFT.SetMode(AUTOMATIC);
  PID_ROLL_LEFT.SetOutputLimits(PID_ROLL_MIN,PID_ROLL_MAX);
  PID_ROLL_LEFT.SetSampleTime(10);
  
  PID_ROLL_RIGHT.SetMode(AUTOMATIC);
  PID_ROLL_RIGHT.SetOutputLimits(PID_ROLL_MIN,PID_ROLL_MAX);
  PID_ROLL_RIGHT.SetSampleTime(10);
  
  //Connect and Prepare Gyro
   if(!mpu.testConnection())
  {
    Serial2.println("Gyro Connection Lost");
    Serial2.println("Fight Terminated");
    terminateFlight();
  }
  else
  {
    Serial2.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial2.println(F("Testing device connections..."));
    Serial2.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial2.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(-15);
    mpu.setZGyroOffset(-.75);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
    if(devStatus == 0)
    {
      mpu.setDMPEnabled(true);
      attachInterrupt(MPU_Interrupt_PIN, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial2.print("MPU6050 DMP Fatal Error. Error = ");
      Serial2.println(devStatus);
    }
  }

  //Wait for DMP stabalization algorithm takes 45 seconds
  for(int i =0; i<2653; i++)  {
      updateYawPitchRoll();
      Serial2.println(rollInput);
      //Serial2.println(pitchInput);  
}
  Serial2.println("Tricopter Initialized");
}




void loop()
{
  updateYawPitchRoll();
  while (!mpuInterrupt && fifoCount < packetSize) {
    gatherBluetoothData();
    decodeBluetoothData();
    saveBluetoothData();
    computePID();
    updateMotors();
    statusUpdate(); 
    tailMotor.write(tailMotorVal);
    rightMotor.write(rightMotorVal);
    leftMotor.write(leftMotorVal);  
    //Serial2.println(bluetoothLastUpdated);
     
  }
}


//If the bluetooth is sending data-retrieve and save it
void gatherBluetoothData()  {
  if(Serial1.available() > 1)  {
    numericalBuffer[0] = Serial1.read();
    numericalBuffer[1] = Serial1.read();
    bluetoothLastUpdated = millis();
  }
}
void decodeBluetoothData()  {
  //////////////////////////////////////Read from first buffer and extract two values
  if(numericalBuffer[0]<=100)  {
     joystick[0] = numericalBuffer[0];
  }
  else if (numericalBuffer[0] >= 101) {
    joystick[1] = numericalBuffer[0]-101;
  }
  ///////////////////////////////////////Read from second buffer and extract two values
  if(numericalBuffer[1] <= 100)  {
     joystick[2] = numericalBuffer[1];
  }
  else if (numericalBuffer[1] >= 101) {
    joystick[3] = numericalBuffer[1]-101;
  }
/////////////////////////////////////////Check the delay in transmission  
  if(millis()-bluetoothLastUpdated > COM_DELAY_THRESHOLD)  {
    terminateFlight();
  }  
}
void saveBluetoothData()  {
    Throttle = (double)joystick[2];      //Serial2.println(Throttle);    
    Yaw = (double)joystick[3];
    Pitch = (double)joystick[0];
    Roll = (double)joystick[1];
}
    
void terminateFlight()  {
 rightMotor.write(30);
 leftMotor.write(30);
 tailMotor.write(30);
  while(true) {
   Serial2.println("Flight Terminated");
  }
}

void updateYawPitchRoll()  {
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if  ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	mpu.resetFIFO();
	Serial2.println(F("FIFO overflow!")); 
  }else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	mpu.getFIFOBytes(fifoBuffer, packetSize);
	fifoCount -= packetSize;
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr[0] = ypr[0] * 180 / M_PI;
        ypr[1] = ypr[1] * 180 / M_PI;
        ypr[2] = -ypr[2] * 180 / M_PI;
        rollInput = float(ypr[2]);
        pitchInput = float(ypr[1]);
              
        //Serial1.println(ypr[0]);
  }
}

void statusUpdate()  {
 /////////////////////////////////////// 
 //TODO send info every 50 loop cycles
 ///////////////////////////////////////
    Serial2.print(tailMotorVal);
    Serial2.print("\t");
    Serial2.print(rightMotorVal);
    Serial2.print("\t");
    Serial2.print(leftMotorVal);
    Serial2.println("\t");
    /*
    Serial2.print(pitchInput);
    Serial2.print("\t");
    Serial2.println(rollInput);
    */
}
////////////////////////////////////////////////////////////
//Stabalization Algorithm
//
//
//
//
////////////////////////////////////////////////////////////

void computePID()
{ 
  
  if (abs(rollInput) < 1)  {
    rollInput = 0;
  }  
  if (abs(pitchInput) < 1)  {
    pitchInput = 0;  
  }
  
  loadDefaultSetpoints();
  PID_ROLL_LEFT.Compute();
  PID_ROLL_RIGHT.Compute();
  PID_PITCH_FORWARD.Compute();
  PID_PITCH_REVERSE.Compute();
}

void updateMotors()
{
  if(Throttle == 30)  {
     tailMotorVal = 30;
     rightMotorVal = 30;
     leftMotorVal = 30; 
  }
  else  {
   if(abs(rollInput)<=7)  {
     tailMotorVal = constrain(Throttle+pitchOutputForward,30,180);
     rightMotorVal = constrain(Throttle+rollOutputRight+pitchOutputReverse,30,180);
     leftMotorVal = constrain(Throttle+rollOutputLeft+pitchOutputReverse,30,180);
     
     /*
      tailMotor.write(constrain(Throttle+pitchOutputForward,30,180));
      rightMotor.write(constrain(Throttle+rollOutputRight+pitchOutputReverse,30,180));
      leftMotor.write(constrain(Throttle+rollOutputLeft+pitchOutputReverse,30,180));
     */  
   }  

    if (abs(rollInput)>=7)  {
      tailMotorVal = constrain(Throttle+pitchOutputForward,30,100);
      rightMotorVal = constrain(Throttle+rollOutputRight,30,100);
      leftMotorVal = constrain(Throttle+rollOutputLeft,30,100);
      
      /*
      tailMotor.write(constrain(Throttle+pitchOutputForward,30,180));
      rightMotor.write(constrain(Throttle+rollOutputRight,30,180));
      leftMotor.write(constrain(Throttle+rollOutputLeft,30,180));
      */  
    }
  }
}

void loadDefaultSetpoints()
{
  double kpr = 1.35;      // rollInput
  double kir = 0.095;
  double kdr = 0.45;

  double kpp = 1.15;      // pitchInput
  double kip = 0.095;
  double kdp = 0.3;  
  
  double kpy = 1.265;      // yawInput
  double kiy = 0.001;
  double kdy = 0.01;
  
  PID_ROLL_LEFT.SetTunings(kpr,kir,kdr);
  PID_ROLL_RIGHT.SetTunings(kpr,kir,kdr);
  
  PID_PITCH_FORWARD.SetTunings(kpp,kip,kdp);
  PID_PITCH_REVERSE.SetTunings(kpp,kip,kdp);
}



void initMotors()  {
  rightMotor.attach(RMOTOR_PIN);
  leftMotor.attach(LMOTOR_PIN);
  tailMotor.attach(TMOTOR_PIN);
  yawServo.attach(SERVO_PIN);
  tailMotor.write(30);
  rightMotor.write(30);
  leftMotor.write(30);
  yawServo.write(67); 
}


//Needed if using sofware serial on arduino uno instead of Serial1 on arduino mega
void detachMotors()  {
  rightMotor.detach();
  leftMotor.detach();
  tailMotor.detach();
  yawServo.detach();
  
}



/////////////////////////////////////////////////
//Processing Source Code Below
//Make sure Serial outputs are correctly chosen
/////////////////////////////////////////////////



/*
import procontroll.*;
import java.io.*;
import processing.serial.*;

ControllIO controll;
ControllDevice device;
ControllStick stick;
ControllStick stick2;
ControllButton button;
Serial port;
byte out[] = new byte[2];
int control = 0;   
void setup() {
  size(400, 400);
  controll = ControllIO.getInstance(this);
  println(Serial.list());
  port = new Serial(this, Serial.list()[2], 9600);
  device = controll.getDevice("Logitech Dual Action");
  stick2 = device.getStick("X Axis Y Axis");
  stick = device.getStick("Z Axis Z Rotation");
  stick.setTolerance(0.05f);
  stick.setMultiplier(-100);
  stick2.setTolerance(0.05f);
  stick2.setMultiplier(-100);
}

void draw() {  
 
  float a = stick2.getY() + 30;
  float b = -stick2.getX() + 30;
  float c = stick.getY() + 30;
  float d = -stick.getX() + 30;
 
///////////////////////////////////////////////// 
  int w = constrain(int(a),0,100);
  int x = constrain(int(b),0,100); 
 
 if (control % 2 == 0)  {
   out[0] = byte(w);
 }
 if (control % 2 ==1)  {
   out[0] = byte(x+101);
 }
 
  int y = constrain(int(c),0,100);
  int z = constrain(int(d),0,100);    
  
 if (control % 2 == 0)  {
   out[1] = byte(y);
 }
 if (control % 2 ==1)  {
   out[1] = byte(z+101);
 }  
///////////////////////////////////////////////////
  port.write(out);
  //println(z+303);
  println(port.read());
  control++;
}
*/
