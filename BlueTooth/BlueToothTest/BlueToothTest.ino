char blueToothVal;           //value sent over via bluetooth
char lastValue;              //stores last state of device (on/off)
void setup()
{
 Serial1.begin(9600);
 pinMode(19,INPUT_PULLUP);
 pinMode(13,OUTPUT);
}
 
 
void loop()
{
  if(Serial1.available())
  {//if there is data being recieved
    blueToothVal=Serial1.read(); //read it
  }
  if (blueToothVal=='n')
  {//if value from bluetooth mySerial is n
    digitalWrite(13,HIGH);            //switch on LED
    if (lastValue!='n')
      Serial1.println(F("LED is on")); //print LED is on
    lastValue=blueToothVal;
  }
  else if (blueToothVal=='f')
  {//if value from bluetooth mySerial is n
    digitalWrite(13,LOW);             //turn off LED
    if (lastValue!='f')
      Serial1.println(F("LED is off")); //print LED is on
    lastValue=blueToothVal;
  }
  delay(10);
}
