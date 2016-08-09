#include <Servo.h>
Servo eyeMount;

/*define logic control pins*/
int LeftRev = 9;   // IN1
int LeftFor = 7;   // IN2
int RightFor = 8;  // IN3
int RightRev = 10; // IN4
int Echo = A4;
int Trig = A5;

/*define channel enable pins and variable speed*/
int ENA = 5;
int ENB = 6;
int motorSpeed = 110; // You can change this to change the motor speed
char getstr;
boolean stopped = true;

/*define distance variables for Ultrasonic Sensor*/
int RightDistance = 0, LeftDistance = 0, detectedDistance = 0;

/*define forward function*/
void forward()
{ 
  stopped = false;
  //analogWrite(ENA,motorSpeed); // Enable Left side variable speed
  //analogWrite(ENB,motorSpeed); // Enable Right side variable speed
  digitalWrite(ENA,HIGH);        // Enable Left side full speed
  digitalWrite(ENB,HIGH);        // Enable Right side full speed
  digitalWrite(LeftRev,LOW);     // HIGH = Left side reverse
  digitalWrite(LeftFor,HIGH);    // HIGH = Left side forward
  digitalWrite(RightFor,HIGH);   // HIGH = Right side forward
  digitalWrite(RightRev,LOW);    // HIGH = Right side reverse
  Serial.println("Forward");
}

/*define back function*/
void back()
{
  stopped = false;
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(LeftRev,HIGH);
  digitalWrite(LeftFor,LOW);
  digitalWrite(RightFor,LOW);
  digitalWrite(RightRev,HIGH);
  Serial.println("Back");
}

/*define turnLeft function*/
void turnLeft()
{
  stopped = false;
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,LOW);
  digitalWrite(LeftRev,HIGH);
  digitalWrite(LeftFor,LOW);
  digitalWrite(RightFor,LOW);
  digitalWrite(RightRev,LOW);
  Serial.println("Turn left");
}

/*define turnRight function*/
void turnRight()
{
  stopped = false;
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,HIGH);
  digitalWrite(LeftRev,LOW);
  digitalWrite(LeftFor,LOW);
  digitalWrite(RightFor,LOW);
  digitalWrite(RightRev,HIGH);
  Serial.println("Turn right");
}

/*define stop function*/
void stopMovement() 
{
  stopped = true;
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  digitalWrite(13, HIGH);
  Serial.println("Stop");
}

/*define function for finding distance*/
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

/*define obstacle recognition and logic function*/
void detectObstacles()
{
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
      //forward();
      //delay(500);
      btControl();
    }
  }
  else {
    //forward();
    btControl();

    // FEATURE: if key pressed then switch between forward() or btControl()
  }
}

/*define obstacle avoidance total function
 * THIS FUNCTION IS THE ONE YOU CALL IN LOOP()
 */
void avoidObstacles() 
{
  if(stopped == false) {
    for(int pos = 25; pos < 135; pos+= 5) {
      eyeMount.write(pos);
      detectedDistance = distanceTest();
      Serial.print("Distance: ");
      Serial.println(detectedDistance);
      detectObstacles();
      delay(20);
    }
    delay(20);
    for(int pos = 135; pos > 25; pos-= 5) {
      eyeMount.write(pos); 
      detectedDistance = distanceTest();
      Serial.println(detectedDistance);
      detectObstacles();
      delay(20);
    }
    delay(20);
  } 
  else {
    btControl();
  }
}

/*define Bluetooth Control from device
 * THIS FUNCTION CAN BE CALLED IN LOOP()
 */
void btControl() {
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
 }
}

/*put your setup code here, to run once*/
void setup() {
 Serial.begin(9600);
/*Set the defined pins to the output*/
  pinMode(LeftRev,OUTPUT);
  pinMode(LeftFor,OUTPUT);
  pinMode(RightFor,OUTPUT);
  pinMode(RightRev,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(Echo,INPUT);
  pinMode(Trig,OUTPUT);
  eyeMount.attach(3);
  eyeMount.write(75);
}

/*put your main code here, to run repeatedly*/
void loop() {
  //btControl();
  avoidObstacles();

  /* if(getstr == 'o') {
    avoidObstacles();
  }

  if(getstr == 'q') {
    digitalWrite(TRIG, LOW);
    eyeMount.write(75);
  }
  */
}
