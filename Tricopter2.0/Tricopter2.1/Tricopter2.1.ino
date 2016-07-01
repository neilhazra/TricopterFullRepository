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
#define RMOTOR_PIN  A1
#define LMOTOR_PIN  A2
#define TMOTOR_PIN  A0
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
#define ESC_MIN 1000
#define ESC_MAX 2000

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


#define PID_PITCH_MIN -50
#define PID_PITCH_MAX 50
#define PID_ROLL_MIN -30
#define PID_ROLL_MAX 30
#define INIT_ROLL 0.0
#define INIT_PITCH 0.0

// initial PID values. tested on stable platform / ground.
double initRoll = INIT_ROLL;
double initPitch = INIT_PITCH;

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
PID pid_roll_left(&rollInput, &rollOutputLeft, &initRoll, kpr, kir, kdr, DIRECT);
PID pid_roll_right(&rollInput, &rollOutputRight, &initRoll, kpr, kir, kdr, REVERSE);
PID pid_pitch_fwd(&pitchInput, &pitchOutputForward, &initPitch, kpp, kip, kdp, DIRECT);
PID pid_pitch_rev(&pitchInput, &pitchOutputReverse, &initPitch, kpp, kip, kdp, REVERSE);

/////////////////////////////////////////////////////////////////////////////
void setup()
{
  Wire.begin();
  TWBR = 24;      // i2c bus @ 400KHz
  //Attach  Servos
  rightMotor.attach(RMOTOR_PIN);
  leftMotor.attach(LMOTOR_PIN);
  tailMotor.attach(TMOTOR_PIN);
  yawServo.attach(SERVO_PIN);
  //Initialize Stabalizer
  pid_pitch_fwd.SetMode(AUTOMATIC);
  pid_pitch_fwd.SetOutputLimits(PID_PITCH_MIN,PID_PITCH_MAX);
  pid_pitch_fwd.SetSampleTime(14);
  
  pid_pitch_rev.SetMode(AUTOMATIC);
  pid_pitch_rev.SetOutputLimits(PID_PITCH_MIN,PID_PITCH_MAX);
  pid_pitch_rev.SetSampleTime(14);
  
  pid_roll_left.SetMode(AUTOMATIC);
  pid_roll_left.SetOutputLimits(PID_ROLL_MIN,PID_ROLL_MAX);
  pid_roll_left.SetSampleTime(14);
  
  pid_roll_right.SetMode(AUTOMATIC);
  pid_roll_right.SetOutputLimits(PID_ROLL_MIN,PID_ROLL_MAX);
  pid_roll_right.SetSampleTime(14);
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
    mpu.setFullScaleGyroRange(3);
    mpu.setFullScaleAccelRange(3);
    mpu.setXGyroOffset(-17);
    mpu.setYGyroOffset(-17);
    mpu.setZGyroOffset(-15);
    mpu.setXAccelOffset(-2205-285-39-37);
    mpu.setYAccelOffset(1500+820+95+10);
    mpu.setZAccelOffset(1688);
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
  //Wait for the DMP to initialize
  updateYawPitchRoll();
  
  while((ypr[1] > 1.0 || ypr[1] < -1.0) || (ypr[2] > 1.0 || ypr[2] < -1.0))
  {
    updateYawPitchRoll();
  }
}

void loop()
{
  updateRadioVariables();//should be ok
  updateYawPitchRoll();// should be ok
  
  computePID();
  updateMotors();
}


void updateRadioVariables()
{
  if(radio.available())
  {
    //Read Data
    radio.read(joystick, sizeof(joystick));
    //Save Data
    Throttle = (double)joystick[0];
    Yaw = (double)joystick[1];
    Pitch = (double)joystick[2];
    Roll = (double)joystick[3];
    //Safety Shutdown
    prevMillis = millis();
  }
   else
  {
    //Error Message
    Serial.println("No radio available");
    //Safety Shutdown
    if ((millis() - prevMillis) > 1000)  {
      terminateFlight();
    }
  }
}
void terminateFlight()  {
 rightMotor.write(30);
 leftMotor.write(30);
 tailMotor.write(30);
 
 while(true) {
   ;
  }
}
void updateYawPitchRoll()  {
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
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
  
if(Pitch > 50)          // change pitch setpoint according to RC TX
  {
    initPitch = 15;
    kpp = 1.9;
    kip = 0.3;
    kdp = 0.85;
    pid_pitch_fwd.SetTunings(kpp,kip,kdp);
    pid_pitch_rev.SetTunings(kpp,kip,kdp);
  }
  else if(Pitch < 50)
  {
    initPitch = -15;
    kpp = 1.9;
    kip = 0.3;
    kdp = 0.85;
    pid_pitch_fwd.SetTunings(kpp,kip,kdp);
    pid_pitch_rev.SetTunings(kpp,kip,kdp);
  }
  else if(Pitch == 50)
  {
    initPitch = INIT_PITCH;
    loadDefaultSetpoints();
  }
  
  if(Roll > 50)          // change roll setpoint according to RC TX
  {
    initRoll = 15;
    kpr = 1.8;
    kir = 0.45;
    kdr = 0.85;
    pid_roll_left.SetTunings(kpr,kir,kdr);
    pid_roll_right.SetTunings(kpr,kir,kdr);
  }
  else if(Roll < 50)
  {
    initRoll = -15;
    kpr = 1.8;
    kir = 0.45;
    kdr = 0.85;
    pid_roll_left.SetTunings(kpr,kir,kdr);
    pid_roll_right.SetTunings(kpr,kir,kdr);
  }
  else if(Roll == 50)
  {
    initRoll = INIT_ROLL;
    loadDefaultSetpoints();
  }
  
  else
  {
    loadDefaultSetpoints();
  }
  
  pid_roll_left.Compute();
  pid_roll_right.Compute();
  pid_pitch_fwd.Compute();
  pid_pitch_rev.Compute();
}

void updateMotors()
{
  #define START_PID_CONTROLLERS_AT 1040
    
  if((Throttle > START_PID_CONTROLLERS_AT) && (Throttle < 2002))
  {
    leftMotor.writeMicroseconds(Throttle + pitchOutputForward + rollOutputLeft);
    rightMotor.writeMicroseconds(Throttle + pitchOutputForward + rollOutputRight);
    tailMotor.writeMicroseconds(Throttle + pitchOutputReverse);
  }
  else if(Throttle < START_PID_CONTROLLERS_AT && Throttle > 1034)
  {
    leftMotor.writeMicroseconds(Throttle);
    rightMotor.writeMicroseconds(Throttle);
    tailMotor.writeMicroseconds(Throttle);
  }
  else
  {
    leftMotor.writeMicroseconds(1005);
    rightMotor.writeMicroseconds(1005);
    tailMotor.writeMicroseconds(1005);
  }
}s

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
  
  pid_roll_left.SetTunings(kpr,kir,kdr);
  pid_roll_right.SetTunings(kpr,kir,kdr);
  
  pid_pitch_fwd.SetTunings(kpp,kip,kdp);
  pid_pitch_rev.SetTunings(kpp,kip,kdp);
}


