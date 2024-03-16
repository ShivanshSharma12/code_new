#include <AFMotor.h>
#include <Wire.h>
#include <Servo.h>





Servo myservo;  // create servo object to control a servo
Servo constarm;
Servo grip;
Servo base;

  #define in1 8
  #define in2 7
  #define in3 13
  #define in4 12
  #define enA 6
  #define enB 5



#define trig 4
#define echo 2  

 int M1_Speed = 80; // speed of motor 1
 int M2_Speed = 80; // speed of motor 2
 int LeftRotationSpeed = 200;  // Left Rotation Speed
 int RightRotationSpeed = 200; // Right Rotation Speed


 void setup() {

  pinMode(in1,OUTPUT); //motor pins
  pinMode(in2,OUTPUT); //motor pins
  pinMode(in3,OUTPUT); //motor pins
  pinMode(in4,OUTPUT); //motor pins

    pinMode(enA,OUTPUT); //motor pins
    pinMode(enB,OUTPUT); //motor pins

    pinMode(A0, INPUT); // initialize Left sensor as an input
    pinMode(A1, INPUT); // initialize Right sensor as an input

  pinMode(trig, OUTPUT); //Ultrasonic sensor pins
  pinMode(echo, INPUT); //Ultrasonic sensor pins

  myservo.attach(9);        //right servo
  constarm.attach(3);       //left servo
  grip.attach(10);          //grip
  base.attach(11);          //base
  Serial.begin(9600);

  }

  bool robotArmExecuted = false;
void loop() {
  constarm.write(135);// left servo fixed
  int distance = obstacle();
  Serial.println(distance);
  if (distance > 0 && distance <= 14.8) {
 
    if (!robotArmExecuted) {
      Stop();
      delay(500);
      //backward();
      //delay(300);
      //Stop();
      //delay(500);
      robotarm();
      robotArmExecuted = true;
      Serial.println("Here");
      delay(500);
    }
    // Do not move forward when an obstacle is detected
  
  } else {
    // Reset the flag when no obstacle is detected
    robotArmExecuted = false;
    forward(); // Move forward when no obstacle is detected
    linefollower();
  }
}
void linefollower() 
{

  int LEFT_SENSOR = digitalRead(A0);
  int RIGHT_SENSOR = digitalRead(A1);

if(RIGHT_SENSOR==0 && LEFT_SENSOR==0) 
{
    forward(); //FORWARD
}

  else if(RIGHT_SENSOR==0 && LEFT_SENSOR==1)
 {
    right(); //Move Right
 }

  else if(RIGHT_SENSOR==1 && LEFT_SENSOR==0) 
{
    left(); //Move Left
}

  else if(RIGHT_SENSOR==1 && LEFT_SENSOR==1) 
 {
    Stop();  //STOP
 }
}



void forward()
{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(enA, M1_Speed);
    analogWrite(enB, M2_Speed);
}

void backward()
{
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);

            analogWrite(enA, M1_Speed);
            analogWrite(enB, M2_Speed);
            delay(200);
                
}

void right()
{
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(enA, LeftRotationSpeed);
            analogWrite(enB, RightRotationSpeed);
}

void left()
{
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);

                analogWrite(enA, LeftRotationSpeed);
                analogWrite(enB, RightRotationSpeed);
}

void Stop()
{
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);
}

int obstacle() 
{
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long t = pulseIn(echo, HIGH);
  int cm = (t / 2)/29.1;
  return cm;
}

/////////////////////////////////////////////////////////************************************//////////////////////////////////////////

void robotarm(){
grip.write(90);
base.write(90);
based();
gripc();
armc();
basel();
gripo();
baser();
}

void based(){
int a;
for(a=90;a<180;a++) {
  myservo.write(a);                  // sets the servo position according to the scaled value
  delay(15);
  } //opens the arm/ extension
}

void armc(){
int a;
for(a=170;a>40;a--) {
  myservo.write(a);                  // sets the servo position according to the scaled value
  delay(15); //folds the arm/ contraction
}
}

void gripc(){
int a;
for(a=90;a<200;a++) {
  grip.write(a);                  // sets the servo position according to the scaled value
  delay(15);
  } //opens the arm/ extension
}


void gripo(){
int a;
for(a=230;a>90;a--) {
  grip.write(a);                  // sets the servo position according to the scaled value
  delay(15); //
}
}

void basel(){
int a;
for(a=90;a<180;a++) {
  base.write(a);                  // sets the servo position according to the scaled value
  delay(15);
  } //opens the arm/ extension
}

void baser(){
int a;
for(a=180;a>90;a--) {
  base.write(a);                  // sets the servo position according to the scaled value
  delay(15); //
}
}



