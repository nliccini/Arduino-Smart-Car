#include <Servo.h>
Servo eyeMount;

/* define logic control pins */
/* NOTE: The motor control board L298N was used in this project */
int LeftRev = 9;         // IN1
int LeftFor = 7;         // IN2
int RightFor = 8;        // IN3
int RightRev = 10;       // IN4
int Echo = A4;           // Ultrasonic sensor input pin
int Trig = A5;           // Ultrasonic sensor output pin
int rightLineSensor = 2; // Rightmost Line-Tracing sensor pin
int midLineSensor = 4;   // Middle Line-Tracing sensor pin
int leftLineSensor = 11; // Leftmost Line-Tracing sensor pin

/* define channel enable pins, variable speed, and motion boolean */
int ENA = 5;          // enable Left side
int ENB = 6;          // enable Right side
int motorSpeed = 115; // You can change this to change the motor speed
int turnSpeed = 170;
boolean stopped = true;  // if the robot is stopped or not

/* define bluetooth input and mode state variables */
char getstr;             // input from serial monitor, it is a single character
int state = 0;           // State determines what mode you're in

/* define distance variables for Ultrasonic Sensor */
int RightDistance = 0, LeftDistance = 0, detectedDistance = 0;

/* define forward function */
void forward()
{ 
  stopped = false;
  analogWrite(ENA,motorSpeed); // Enable Left side variable speed
  analogWrite(ENB,motorSpeed); // Enable Right side variable speed
  //digitalWrite(ENA,HIGH);        // Enable Left side full speed
  //digitalWrite(ENB,HIGH);        // Enable Right side full speed
  digitalWrite(LeftRev,LOW);     // HIGH = Left side reverse
  digitalWrite(LeftFor,HIGH);    // HIGH = Left side forward
  digitalWrite(RightFor,HIGH);   // HIGH = Right side forward
  digitalWrite(RightRev,LOW);    // HIGH = Right side reverse
  Serial.println("Forward");
}

/* define back function */
void back()
{
  stopped = false;
  analogWrite(ENA,motorSpeed);
  analogWrite(ENB,motorSpeed);
  //digitalWrite(ENA,HIGH);
  //digitalWrite(ENB,HIGH);
  digitalWrite(LeftRev,HIGH);
  digitalWrite(LeftFor,LOW);
  digitalWrite(RightFor,LOW);
  digitalWrite(RightRev,HIGH);
  Serial.println("Back");
}

/* define turn left function */
void turnLeft()
{
  stopped = false;
  analogWrite(ENA,turnSpeed);
  analogWrite(ENB,turnSpeed);
  //digitalWrite(ENA,HIGH);
  //digitalWrite(ENB,HIGH);
  digitalWrite(LeftRev,HIGH);
  digitalWrite(LeftFor,LOW);
  digitalWrite(RightFor,HIGH);
  digitalWrite(RightRev,LOW);
  Serial.println("Turn left");
}

/* define turn right function */
void turnRight()
{
  stopped = false;
  analogWrite(ENA,turnSpeed);
  analogWrite(ENB,turnSpeed);
  //digitalWrite(ENA,HIGH);
  //digitalWrite(ENB,HIGH);
  digitalWrite(LeftRev,LOW);
  digitalWrite(LeftFor,HIGH);
  digitalWrite(RightFor,LOW);
  digitalWrite(RightRev,HIGH);
  Serial.println("Turn right");
}

/* define stop function */
void stopMovement() 
{
  stopped = true;
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  digitalWrite(13, HIGH);
  Serial.println("Stop");
}

/* define function for finding and calculating distance */
int distanceTest()
{
  digitalWrite(Trig,LOW);
  delayMicroseconds(2);
  digitalWrite(Trig,HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig,LOW);
  float distance = pulseIn(Echo, HIGH);
  distance = distance/58; //58 is cm, 148 is inches
  return (int)distance;
}

/* define Bluetooth Control from device
 * THIS FUNCTION CAN BE CALLED IN LOOP()
 * NOTE: btControl() has the functions stateControl() and speedControl() already 
 * built in 
 */
void btControl() 
{
  getstr = Serial.read();
  switch(getstr) {
    case 'f':
      forward();
      break;
    case 'b':
      back();
      break;
   case 'l':
      turnLeft();
      break;
   case 'r':
      turnRight();
      break; 
   case 's':
      stopMovement();
      break;
    case 'x':         // x, y, z are the different modes
      state = 2;
      break;
    case 'y':
      state = 1;
      break;
    case 'z':
      state = 0;
      break;
    case '+': 
      if(motorSpeed < 245 && turnSpeed < 245) {
        motorSpeed+=10;
        turnSpeed++; 
        break; 
      }
      break;
    case '-':         
      if(motorSpeed > 75 && turnSpeed > 75) {
        motorSpeed-=10;
        turnSpeed--; 
        break;  
      }
      break;
  }
}

/* define function to switch between car modes. Mode X is auto, Mode Y is line tracing
 * Mode Z is Bluetooth control.
 */
void stateControl() {
  getstr = Serial.read();
  switch(getstr) {        // This allows you to change modes any time.
        case 'x':         // x is automatic driving with obstacle awareness
          state = 2;
          break;
        case 'y':         // y is line following
          state = 1;
          break;
        case 'z':         // z is bt control with obstacle awareness
          state = 0;
          break;
  }
}

/* define function to increase or decrease motor and turning speed
 */
void speedControl() {
  getstr = Serial.read();
  switch(getstr) {        // This allows you to change speed any time.
        case '+': 
        if(motorSpeed < 245 && turnSpeed < 245) {
          motorSpeed+=10;
          turnSpeed++; 
          break;  
        }
          break;
        case '-':         
          if(motorSpeed > 75 && turnSpeed > 75) {
          motorSpeed-=10;
          turnSpeed--; 
          break;  
        }
          break;
  }
}

/* define obstacle recognition and logic function for automatic driving */
void detectObstaclesAUTO()
{
  //getstr = Serial.read();
  if(detectedDistance <= 15) { // if distance detected is <= 20 cm
    stopMovement();
    
    delay(250);
    eyeMount.write(15);
    delay(1000);
    RightDistance = distanceTest();
    
    delay(250);
    eyeMount.write(135);
    delay(1000);
    LeftDistance = distanceTest();

    delay(250);
    eyeMount.write(75);
    
    if(RightDistance > LeftDistance) {
      turnRight();
      delay(500);
    }
    else if(RightDistance < LeftDistance) {
      turnLeft();
      delay(500);
    }
    else if((RightDistance <= 30) || (LeftDistance <= 30)) {
      back();
      delay(500);
    }
    else {
      forward();       // If object was detected by mistake, then continue forward.
      stateControl();
      speedControl();
    }
  }
  else {
    forward();         // If no object is detected, then continue forward.
    stateControl();
    speedControl();
  }
}

/* define obstacle recognition and logic function for bluetooth control */
void detectObstaclesBT()
{
  if(detectedDistance <= 15) {      // if distance detected is <= 15 cm
    stopMovement();
    
    delay(250);
    eyeMount.write(15);
    delay(1000);
    RightDistance = distanceTest();
    
    delay(250);
    eyeMount.write(135);
    delay(1000);
    LeftDistance = distanceTest();

    delay(250);
    eyeMount.write(75);
    
    if(RightDistance > LeftDistance) {
      turnRight();
      delay(500);
      stopMovement();
    }
    else if(RightDistance < LeftDistance) {
      turnLeft();
      delay(500);
      stopMovement();
    }
    else if((RightDistance <= 30) || (LeftDistance <= 30)) {
      back();
      delay(500);
    }
    else {
      btControl();  // If object was detected by mistake, then continue with BT control
    }
  }
  else {
    btControl();    // If no object is detected, then continue with BT control
  }
}

/* define automatic obstacle avoidance and driving function
 * THIS FUNCTION CAN BE CALLED IN LOOP()
 */
void avoidObstaclesAUTO() 
{
    for(int pos = 25; pos < 135; pos+= 5) {
      eyeMount.write(pos);
      detectedDistance = distanceTest();
      Serial.print("Distance: ");
      Serial.println(detectedDistance);
      detectObstaclesAUTO();
      delay(20);
    }
    delay(20);
    for(int pos = 135; pos > 25; pos-= 5) {
      eyeMount.write(pos); 
      detectedDistance = distanceTest();
      Serial.println(detectedDistance);
      detectObstaclesAUTO();
      delay(20);
    }
    delay(20);
}

/* define bluetooth obstacle avoidance and control function
 * THIS FUNCTION CAN BE CALLED IN LOOP()
 */
void avoidObstaclesBT() 
{
  if(stopped == false) {
    for(int pos = 25; pos < 135; pos+= 5) {
      eyeMount.write(pos);
      detectedDistance = distanceTest();
      Serial.print("Distance: ");
      Serial.println(detectedDistance);
      detectObstaclesBT();
      delay(20);
    }
    delay(20);
    for(int pos = 135; pos > 25; pos-= 5) {
      eyeMount.write(pos); 
      detectedDistance = distanceTest();
      Serial.println(detectedDistance);
      detectObstaclesBT();
      delay(20);
    }
    delay(20);
  } 
  else {
    btControl();
  }
}

/* define follow line function
 * THIS FUNCTION CAN BE CALLED IN LOOP()
 */
void followLine() 
{
  //getstr = Serial.read();
  int rightLine = digitalRead(rightLineSensor); // 1 means black, 0 means another color
  int midLine = digitalRead(midLineSensor);
  int leftLine = digitalRead(leftLineSensor);

  if((rightLine == 0) && midLine && leftLine) { // just var in if() means non-zero int
    turnLeft();
    delay(2);
    while(1) {
      midLine = digitalRead(leftLineSensor);
      if(midLine == 1) {
        turnLeft();
        delay(2);
      }
      else {
        break;
      }
    }
  } 
  else if(rightLine && midLine && (leftLine == 0)) {
    turnRight();
    delay(2);
    while(1) {
      midLine = digitalRead(rightLineSensor);
      if(midLine == 1) {
        turnRight();
        delay(50);
      }
      else {
        break;
      }
    }
  } 
  else if(rightLine ==0 && leftLine == 0 && midLine ==0) {
    back();
    delay(2);
    stateControl();
    speedControl();
  }
  else {
    forward();
    delay(2);
    stateControl();
    speedControl();
  }
}

/* put your setup code here, to run once */
void setup() {
 Serial.begin(9600);
/* Set the defined pins to the output */
  pinMode(LeftRev,OUTPUT);
  pinMode(LeftFor,OUTPUT);
  pinMode(RightFor,OUTPUT);
  pinMode(RightRev,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(Echo,INPUT);
  pinMode(Trig,OUTPUT);
  //pinMode(rightLineSensor,INPUT); // not sure if necessary to declare pinMode
  eyeMount.attach(3);
  eyeMount.write(75);
}

/* There are 3 Modes as of now. Mode X is automatic driving and obstacle detection.
 * Mode Y is line following. Mode Z is Bluetooth control and obstacle detection. 
 * Alternatively, you can comment out the Mode Switcher code (the if(state==_) blocks)
 * and call in one of the following methods to control the robot in a single mode.
 */

 /* put your main code here, to run repeatedly */
void loop() {
  
  if(state == 2) {
    avoidObstaclesAUTO(); // x is auto
  }
  else if(state == 1) {
    followLine();         // y is follow line
  }
  else if(state == 0) {
    avoidObstaclesBT();   // z is bt
  }

  //btControl();
  //avoidObstaclesAUTO();
  //avoidObstaclesBT();
  //followLine();

  /*
  if(getstr == 'q') {
    digitalWrite(TRIG, LOW);
    eyeMount.write(75);
  }
  */
}
