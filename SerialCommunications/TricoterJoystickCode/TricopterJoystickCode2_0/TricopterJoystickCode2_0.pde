import procontroll.*;
import java.io.*;
import processing.serial.*;

ControllIO controll;
ControllDevice device;
ControllStick stick;
ControllStick stick2;
ControllButton button;
Serial port;

void setup() {
  size(400, 400);
  controll = ControllIO.getInstance(this);
  port = new Serial(this, Serial.list()[0], 115200);
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
  
  int w = constrain(int(a),0,100);
  port.write(w);
  int x = constrain(int(b),0,100); 
 //port.write(x+100);
  int y = constrain(int(c),0,100);
  //port.write(-y);
  int z = constrain(int(d),0,100);    
  //port.write(-(z+100));
  println(w);
}

