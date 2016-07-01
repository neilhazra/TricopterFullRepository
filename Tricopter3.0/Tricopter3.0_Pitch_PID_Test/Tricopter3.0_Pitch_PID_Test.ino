//Includes
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
////////////////////////////////////////////////////////////////////////
//Board Connections
#define RMOTOR_PIN  A0
#define LMOTOR_PIN  A2
#define TMOTOR_PIN  A1
#define SERVO_PIN  A3
const int MPU_Interrupt_PIN = 1;//This is actually the interrupt number, not the pin number
// PID tunings ****************************************

double kpr = 0.95;      // rollInput
double kir = 0.7;
double kdr = 0.05;

double kpp = 1;      // pitchInput
double kip = 0.0;
double kdp = 0.0;

#define PID_ROLL_MIN 0
#define PID_ROLL_MAX 100
#define PID_PITCH_MIN 0
#define PID_PITCH_MAX 100
#define INIT_ROLL 0.0
#define INIT_PITCH 0.0

// initial PID values. tested on stable platform / ground.
double RollSetpoint = INIT_ROLL;
double PitchSetpoint = INIT_PITCH;
// Global Variables ***********************************
double rollInput,pitchInput; //PID input
double rollOutputLeft,rollOutputRight,pitchOutputForward,pitchOutputReverse;   // PID output

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
const int COM_DELAY_THRESHOLD = 1000;


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
 //////////////////////////////
 /////////////
 
  Wire.begin();
  TWBR = 24;      // i2c bus @ 400KHz for MCU
 
  Serial2.println("Calibrating Motors"); //Although calibrated earlier inform flier now that serial communication has begun
  
  //Initialize Stabalizer  
  PID_ROLL_LEFT.SetMode(AUTOMATIC);
  PID_ROLL_LEFT.SetOutputLimits(PID_ROLL_MIN,PID_ROLL_MAX);
  PID_ROLL_LEFT.SetSampleTime(15);
  
  PID_ROLL_RIGHT.SetMode(AUTOMATIC);
  PID_ROLL_RIGHT.SetOutputLimits(PID_ROLL_MIN,PID_ROLL_MAX);
  PID_ROLL_RIGHT.SetSampleTime(15);
  
  
  PID_PITCH_FORWARD.SetMode(AUTOMATIC);
  PID_PITCH_FORWARD.SetOutputLimits(PID_PITCH_MIN,PID_PITCH_MAX);
  PID_PITCH_FORWARD.SetSampleTime(10);
  
  PID_PITCH_REVERSE.SetMode(AUTOMATIC);
  PID_PITCH_REVERSE.SetOutputLimits(PID_PITCH_MIN,PID_PITCH_MAX);
  PID_PITCH_REVERSE.SetSampleTime(10);
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
}
  Serial2.println("Tricopter Initialized");
}




void loop()
{
  updateYawPitchRoll();
  while (!mpuInterrupt && fifoCount < packetSize) {    
    //gatherBluetoothData();
    //decodeBluetoothData();
    /*
    if(joystick[3] == 30)  {
       joystick[3] = 7;
    }   
   
    if(joystick[1] == 30)  {
       joystick[1] = 7;
    }   
   
    if(joystick[0] == 30)  {
       joystick[0] = 7;
    }   
   */
    //kpr += (joystick[3]-7)/100;
    //kir += (joystick[1]-7)/100;
    //kdr += (joystick[0]-7)/100;
    PID_PITCH_FORWARD.SetTunings(kpp,kip,kdp);
    PID_PITCH_REVERSE.SetTunings(kpp,kip,kdp);
    PID_ROLL_LEFT.SetTunings(kpr,kir,kdr);
    PID_ROLL_RIGHT.SetTunings(kpr,kir,kdr);
    PID_PITCH_FORWARD.Compute();
    PID_PITCH_REVERSE.Compute();
    PID_ROLL_LEFT.Compute();
    PID_ROLL_RIGHT.Compute();
    
    Serial2.print(rollInput);
    Serial2.print("\t");
    Serial2.print(kpr);
    Serial2.print("\t");
    Serial2.print(kir);
    Serial2.print("\t");
    Serial2.print(kdr);
    Serial2.print("\t");
    Serial2.print(rollOutputRight);
    Serial2.print("\t");
    Serial2.println(rollOutputLeft);

  /*  
    if(abs(rollInput)<=7)  {    
    rightMotor.write(constrain(30 + map(rollOutputRight+pitchOutputReverse,0,100,0,10),30,100));
    leftMotor.write(constrain(30 + map(rollOutputLeft+pitchOutputReverse,0,100, 0,10),30,100));  
    tailMotor.write(constrain(30 + map(pitchOutputForward,0,100,0,10),30,100));  
    }  
    else  {            
    rightMotor.write(constrain(30 + map(rollOutputRight,0,100,0,10),30,100));
    leftMotor.write(constrain(30 + map(rollOutputLeft,0,100, 0,10),30,100));  
    tailMotor.write(constrain(30 + map(pitchOutputForward,0,100, 0,10),30,100));
    }
  */
    
    //rightMotor.write(constrain(30 + map(pitchOutputReverse,0,100,0,10),30,100));
    //leftMotor.write(constrain(30 + map(pitchOutputReverse,0,100, 0,10),30,100));  
    rightMotor.write(30);
    leftMotor.write(30);  
    //tailMotor.write(constrain(30 + map(pitchOutputForward,0,100,0,10),30,100));  
    tailMotor.write(45);
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
        rollInput = 2*float(ypr[2]);
        pitchInput = 2*float(ypr[1]);     
        //Serial1.println(ypr[0]);
  }
}
void initMotors()  {
  rightMotor.attach(RMOTOR_PIN);
  leftMotor.attach(LMOTOR_PIN);
  tailMotor.attach(TMOTOR_PIN);
  yawServo.attach(SERVO_PIN);
  tailMotor.write(30);
  rightMotor.write(30);
  leftMotor.write(30);
  yawServo.write(68); 
}







