//Includes
#include <RF24.h>
#include "nRF24L01.h"
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
const int CE_PIN = 40;
const int CSN_PIN = 53;
const int MPU_Interrupt_PIN = 1;//This is actually the interrupt number, not the pin number
//Radio Variables 
long prevMillis;
int joystick[4];
double Throttle,Yaw,Pitch,Roll;
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe = 0xE8E8F0F0E1LL;
/////////////////////////////////////////////////////////////////////////

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
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  TWBR = 24;      // i2c bus @ 400KHz
  
  //Attach  Servos
  rightMotor.attach(RMOTOR_PIN);
  leftMotor.attach(LMOTOR_PIN);
  tailMotor.attach(TMOTOR_PIN);
  yawServo.attach(SERVO_PIN);
  
  //Calibrate Motors
  Serial.println("Calibrating Motors. Please Wait 5 Seconds");
  tailMotor.write(30);
  rightMotor.write(30);
  leftMotor.write(30);
  delay(5000);
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
  //Connect to Radio
  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  //Connect and Prepare Gyro
   if(!mpu.testConnection())
  {
    Serial.println("Gyro Connection Lost");
    Serial.println("Fight Terminated");
    terminateFlight();
  }
  else
  {
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    /*
    mpu.setFullScaleGyroRange(3);
    mpu.setFullScaleAccelRange(3);
    mpu.setXGyroOffset(-17);
    mpu.setYGyroOffset(-17);
    mpu.setZGyroOffset(-15);
    mpu.setXAccelOffset(-2205-285-39-37);
    mpu.setYAccelOffset(1500+820+95+10);
    mpu.setZAccelOffset(1688);
    */
    ///*
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(-15);
    mpu.setZGyroOffset(-.75);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    //*/
    if(devStatus == 0)
    {
      mpu.setDMPEnabled(true);
      attachInterrupt(MPU_Interrupt_PIN, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.print("MPU6050 DMP Fatal Error. Error = ");
      Serial.println(devStatus);
    }
  }

  //Wait for DMP stabalization algorithm takes 45 seconds
  
  for(int i =0; i<2653; i++)  {
      updateYawPitchRoll();
      Serial.println(rollInput);
  }
  Serial.println("Tricopter Initialized");
}

void loop()
{
  updateYawPitchRoll();
  while (!mpuInterrupt && fifoCount < packetSize) {
    updateRadioVariables();//should be ok
    computePID();
    //updateMotors();
    /*
    Serial.print(pitchInput);   
    Serial.print("\t");
    Serial.println(pitchOutputForward);
    /*
    leftMotor.write(Throttle);
    rightMotor.write(30);
    tailMotoavrdude: ser_drain(): read error: The handle is invalid.r.write(30);
    */

  tailMotor.write(constrain(Throttle+pitchOutputForward,30,180));
  rightMotor.write(constrain(Throttle+rollOutputRight+pitchOutputReverse,30,180));
  leftMotor.write(constrain(Throttle+rollOutputLeft+pitchOutputReverse,30,180));
  
  //leftMotor.write(rollOutputLeft);
 
  }
}


void updateRadioVariables()
{
  if(radio.available())
  {
    //Read Data
    radio.read(joystick, sizeof(joystick));
    //Save Data
    Throttle = (double)joystick[2];      //Serial.println(Throttle);    
    Yaw = (double)joystick[3];
    Pitch = (double)joystick[0];
    Roll = (double)joystick[1];
    //Safety Shutdown
    prevMillis = millis();
  }
   else
  {
    //Error Message
    if((millis()-prevMillis) > 50)
    {
      Serial.println("No radio available");
    }
    //Safety Shutdown
    if ((millis() - prevMillis) > 2000)  {
      terminateFlight();
    }
  }
}

void terminateFlight()  {
 rightMotor.write(30);
 leftMotor.write(30);
 tailMotor.write(30);
 
while(true) {
   Serial.println("Flight Terminated");
  }
}

void updateYawPitchRoll()  {
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if  ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	mpu.resetFIFO();
	Serial.println(F("FIFO overflow!")); 
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
        //Serial.println(ypr[0]);
  }
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
/*
if(Pitch > 50)          // change pitch setpoint according to RC TX
  {
    PitchSetpoint = 15;
    kpp = 1.9;
    kip = 0.3;
    kdp = 0.85;
    PID_PITCH_FORWARD.SetTunings(kpp,kip,kdp);
    PID_PITCH_REVERSE.SetTunings(kpp,kip,kdp);
  }
  else if(Pitch < 50)
  {
    PitchSetpoint = -15;
    kpp = 1.9;
    kip = 0.3;
    kdp = 0.85;
    PID_PITCH_FORWARD.SetTunings(kpp,kip,kdp);
    PID_PITCH_REVERSE.SetTunings(kpp,kip,kdp);
  }
  else if(Pitch == 50)
  {
    PitchSetpoint = INIT_PITCH;
    loadDefaultSetpoints();
  }
  
  if(Roll > 50)          // change roll setpoint according to RC TX
  {
    RollSetpoint = 15;
    kpr = 1.8;
    kir = 0.45;
    kdr = 0.85;
    PID_ROLL_LEFT.SetTunings(kpr,kir,kdr);
    PID_ROLL_RIGHT.SetTunings(kpr,kir,kdr);
  }
  else if(Roll < 50)
  {
    RollSetpoint = -15;
    kpr = 1.8;
    kir = 0.45;
    kdr = 0.85;
    PID_ROLL_LEFT.SetTunings(kpr,kir,kdr);
    PID_ROLL_RIGHT.SetTunings(kpr,kir,kdr);
  }
  else if(Roll == 50)
  {
    RollSetpoint = INIT_ROLL;
    loadDefaultSetpoints();
  }
  
  else
  {
    loadDefaultSetpoints();
  }
  
  PID_ROLL_LEFT.Compute();
  PID_ROLL_RIGHT.Compute();
  PID_PITCH_FORWARD.Compute();
  PID_PITCH_REVERSE.Compute();
*/
  loadDefaultSetpoints();
  PID_ROLL_LEFT.Compute();
  PID_ROLL_RIGHT.Compute();
  PID_PITCH_FORWARD.Compute();
  PID_PITCH_REVERSE.Compute();
}

void updateMotors()
{
  /*  
  if((Throttle >= 40) && (Throttle <= 100))
  {
    //leftMotor.write(Throttle + pitchOutputForward + rollOutputLeft);
    Serial.println(Throttle + pitchOutputForward + rollOutputLeft);
    Serial.print("\t"); 
    //rightMotor.write(Throttle + pitchOutputForward + rollOutputRight);
    Serial.print(Throttle + pitchOutputForward + rollOutputRight);
    Serial.print("\t"); 
    //tailMotor.write(Throttle + pitchOutputReverse);
    Serial.print(Throttle + pitchOutputReverse);  
}
  else if((Throttle < 40) && (Throttle > 30))
  {
    leftMotor.write(Throttle);
    rightMotor.write(Throttle);
    tailMotor.write(Throttle);
  }
  else
  {
    leftMotor.write(30);
    rightMotor.write(30);
    tailMotor.write(30);
  }
  */
}

void loadDefaultSetpoints()
{
  double kpr = 1.35;      // rollInput
  double kir = 0.095;
  double kdr = 0.45;

  double kpp = 1.15;      // pitchInput
  double kip = 0.095;
  double kdp = 0.3;
  
  /*double kpp = 2.6;      // pitchInput
  double kip = 0.55;
  double kdp = 1.23;*/

  double kpy = 1.265;      // yawInput
  double kiy = 0.001;
  double kdy = 0.01;
  
  PID_ROLL_LEFT.SetTunings(kpr,kir,kdr);
  PID_ROLL_RIGHT.SetTunings(kpr,kir,kdr);
  
  PID_PITCH_FORWARD.SetTunings(kpp,kip,kdp);
  PID_PITCH_REVERSE.SetTunings(kpp,kip,kdp);
}


