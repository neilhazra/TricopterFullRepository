// Hall A
int dir1PinA = 12;
int dir2PinA = 13;
int speedPinA = 10; // Needs to be a PWM pin to be able to control motor speed

// Hall B
int dir1PinB = 7;
int dir2PinB =8;
int speedPinB = 9; // Needs to be a PWM pin to be able to control motor speed

//Hall C
int dir1PinC = 2;
int dir2PinC =4;
int speedPinC = 3; // Needs to be a PWM pin to be able to control motor speed


void setup() {  // Setup runs once per reset
Serial.begin(9300);
  pinMode(dir1PinA,OUTPUT);
  pinMode(dir2PinA,OUTPUT);
  pinMode(speedPinA,OUTPUT);
  pinMode(dir1PinB,OUTPUT);
  pinMode(dir2PinB,OUTPUT);
  pinMode(speedPinB,OUTPUT); 
}

void loop() {
  hallBHigh();
  hallCHigh(); 
  hallALow();
  delay(300);
  hallCHigh();
  hallALow();
  hallBOff();
  delay(300);
  hallCHigh();
  hallALow();
  hallBOff();
  delay(300);
  hallCHigh();
  hallBLow();
  hallAOff();
  delay(300);
  hallAHigh();
  hallCHigh();
  hallBLow();
  delay(300);
  hallAHigh();
  hallBLow();
  hallCOff();
  delay(300);
  hallAHigh();
  hallBLow();
  hallCLow();
  delay(300);
  hallAHigh();
  hallBOff();
  hallCLow();
  delay(300);
  hallAHigh();
  hallBHigh();
  hallCLow();
  delay(300);
  hallAOff();
  hallBHigh();
  hallCLow();
  delay(300);
  hallALow();
  hallBHigh();
  hallCLow();
  delay(300);
  hallALow();
  hallBHigh();
  hallCOff();
  delay(300);
}

void hallAHigh()  {
  analogWrite(speedPinA, 255);
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);  
}

void hallBHigh()  {
  analogWrite(speedPinB, 255);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);  
  
}

void hallCHigh()  {
  analogWrite(speedPinC, 255);
  digitalWrite(dir1PinC, LOW);
  digitalWrite(dir2PinC, HIGH);  
}


void hallALow()  {
  analogWrite(speedPinA, 255);
  digitalWrite(dir2PinA, LOW);
  digitalWrite(dir1PinA, HIGH); 
}

void hallBLow()  {
  analogWrite(speedPinB, 255); 
  digitalWrite(dir2PinB, LOW);
  digitalWrite(dir1PinB, HIGH);  
}

void hallCLow()  {
    analogWrite(speedPinC, 255);
  digitalWrite(dir2PinC, LOW);
  digitalWrite(dir1PinC, HIGH);
}


void hallAOff()  {
  analogWrite(speedPinA, 0);
 
}

void hallBOff()  {
  analogWrite(speedPinB, 0); 
 
}

void hallCOff()  {
  analogWrite(speedPinC, 0);

}

void allOff()  {
   hallAOff();
   hallBOff();
   hallCOff();
  
}








