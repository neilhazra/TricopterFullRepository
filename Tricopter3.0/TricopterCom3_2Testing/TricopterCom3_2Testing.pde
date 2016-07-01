import procontroll.*;
import java.io.*;
import processing.serial.*;

int leftMotor = 0;
int rightMotor= 0;
int tailMotor = 0;

ControllIO controll;
ControllDevice device;
ControllStick stick;
ControllStick stick2;
ControllButton button;
Serial port;
byte out[] = new byte[2];
int control = 0;   
boolean altiLock;
float a;
float b;
float c;
float d;

void setup() {
  size(400, 400);
  controll = ControllIO.getInstance(this);
  println(Serial.list());
  port = new Serial(this, Serial.list()[2], 9600);
   
  device = controll.getDevice("Logitech Dual Action");
  
  device.plug(this, "incrementLeftMotor", ControllIO.ON_PRESS, 1);//x
  device.plug(this, "incrementTailMotor", ControllIO.ON_PRESS, 2);//y
  device.plug(this, "incrementRightMotor", ControllIO.ON_PRESS, 3);//z
  
  device.plug(this, "decrementLeftMotor", ControllIO.ON_PRESS, 5);//x
  device.plug(this, "decrementTailMotor", ControllIO.ON_PRESS, 7);//y
  device.plug(this, "decrementRightMotor", ControllIO.ON_PRESS, 6);//z
  
  
  stick2 = device.getStick("X Axis Y Axis");
  stick = device.getStick("Z Axis Z Rotation");
  stick.setTolerance(0.05f);
  stick.setMultiplier(-100);
  stick2.setTolerance(0.05f);
  stick2.setMultiplier(-100);
}

void draw() {
  a = stick2.getY() ;

  int w =   int(map(int(a),-100,100,0,60));//Throttle
  int x =   int(constrain(int(leftMotor),0,50));//leftMotor 
  int y =   int(constrain(int(rightMotor),0,50)); //rightMotor
  int z =   int(constrain(int(tailMotor),0,50)); //tailMotor
  
   
 if (control % 2 == 0)  {
   out[0] = byte(w);
 }
 if (control % 2 ==1)  {
   out[0] = byte(x+101);
 }
 
 if (control % 2 == 0)  {
   out[1] = byte(y);
 }
 if (control % 2 ==1)  {
   out[1] = byte(z+101);
 }  


  port.write(out);
  //delay(20);
  //port.write(out);
  
  leftMotor = 1;
  rightMotor = 1;
  tailMotor = 1;
  
  print(x);
  print("\t");
  print(z);
  print("\t");
  println(y);
  
  control++;
}


void incrementLeftMotor()  {
  leftMotor= 2;
}

void incrementRightMotor()  {
  rightMotor=2;
}

void incrementTailMotor()  {
  tailMotor=2;
}


void decrementLeftMotor()  {
  leftMotor=0;
}

void decrementRightMotor()  {
  rightMotor=0;
}

void decrementTailMotor()  {
  tailMotor=0;
}



