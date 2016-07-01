import procontroll.*;
import java.io.*;
import processing.serial.*;

ControllIO controll;
ControllDevice device;
ControllStick stick;
ControllStick stick2;
ControllButton button;
Serial port;
PFont f;

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
  port = new Serial(this, Serial.list()[1], 9600);
  
  f = createFont("Arial",16,true);
  
  device = controll.getDevice("Logitech Dual Action");
  
  
  
  device.plug(this, "altiLockEnable", ControllIO.ON_PRESS, 1);
  device.plug(this, "altiLockDisable", ControllIO.ON_PRESS, 2);
  

  
  stick2 = device.getStick("X Axis Y Axis");
  stick = device.getStick("Z Axis Z Rotation");
  stick.setTolerance(0.05f);
  stick.setMultiplier(-100);
  stick2.setTolerance(0.05f);
  stick2.setMultiplier(-100);
}

void draw() {   

if(altiLock == false)  {  
  a = stick2.getY() ;
  b = -stick2.getX();
  c = stick.getY();
  d = -stick.getX();
}

if (altiLock==true)  {
  a = a;
  b = b;
  c = c;
  d = d;
}

///////////////////////////////////////////////// 
  int w =   int(map(int(a),-100,100,30,100));
  int x =   int(map(int(b),-100,100,30,100)); 
 
 if (control % 2 == 0)  {
   out[0] = byte(w);
 }
 if (control % 2 ==1)  {
   out[0] = byte(x+101);
 }
 
  int y =int(map(int(c),-100,100,30,100));
  int z = int(map(int(d),-100,100,30,100)); 
  
 if (control % 2 == 0)  {
   out[1] = byte(y);
 }
 if (control % 2 ==1)  {
   out[1] = byte(z+101);
 }  
///////////////////////////////////////////////////
  textFont(f,16);
  //fill(280);
  background(204);
  text(w,10,100);  
  //clear(); 
  port.write(out);
  println(w);
  control++;
}


void altiLockEnable(){

  println("altiLock Set");
  altiLock = true;
}

void  altiLockDisable(){
  println("altiLock Off");
  altiLock = false;
}
