#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ps5Controller.h>
/////  MOTOR PINS  \\\\\\

const int motorApwm = 13; //left
const int motorAdir = 12;
const int motorBpwm = 27; //left
const int motorBdir = 26;
const int motorCpwm = 33; //right
const int motorCdir = 32;
const int motorDpwm = 18; //right
const int motorDdir = 19;

float leftMotorSpeed = 0.0;
float rightMotorSpeed = 0.0;

////  MOTOR ADJUSTMENT END \\\\

MPU6050 mpu6050(Wire);
///   PID CONSTANTS  \\\

float prerror = 0.0;
float error = 0.0;
long timer = 0;
float pidOutput = 0.0;
const float Kp = 2.0;
const float Ki = 0.05;
const float Kd = 0.1;
float basespeed = 80;
float maxspeed=0.0;
////    PID ENDS  \\\\

void setup() {
  Serial.begin(115200);
  ps5.begin("D0:BC:C1:98:2E:F3"); //replace
  Serial.println("Ready.");
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(motorApwm, OUTPUT);
  pinMode(motorAdir, OUTPUT);
  pinMode(motorBpwm, OUTPUT);
  pinMode(motorBdir, OUTPUT);
  pinMode(motorCpwm, OUTPUT);
  pinMode(motorCdir, OUTPUT);
  pinMode(motorDpwm, OUTPUT);
  pinMode(motorDdir, OUTPUT);
  stop();
}

void loop() {
  if (ps5.isConnected() == false)  {
    Serial.println("PS5 controller not found");
    delay(300);
  }
  mpu6050.update();
  float angle = mpu6050.getAngleZ();
  if (millis() - timer > 1000) {
    Serial.print("\tangleZ : ");
    Serial.print(angle);

    error = 0.0 - angle;
    if ( -1 < error && error < 1) {
      error = 0;
    }
    else if ( 89 < error && error < 91) {
      error = 90;
    }
    else if (-91 < error && error < -89) {
      error = -90;
    }
    else {
      error = 0.0 - angle;
    }

    pid();
    timer = millis();
  }


  ////////////////////////    PS5 FUNCTIONS         /////////////////////////////////////////
  while (ps5.isConnected() == true) {
    if (ps5.Right()){
      buttonRight();
      Serial.println("Right Button");
    }
    if (ps5.Down()) {
      backward();
      Serial.println("Down Button");
    }
    if (ps5.Up()) {
      pid();
      forward();
      Serial.println("Up Button");
    }
    if (ps5.Left()){
      buttonLeft();
      Serial.println("Left Button");
    }

    if (ps5.Square()) {
      error = 90;
      pid();
      if (millis() - timer > 1000) {
        forward();
        timer = millis();
      }
      Serial.println("Square Button");
    }
    if (ps5.Cross()) Serial.println("Cross Button");
    if (ps5.Circle()) Serial.println("Circle Button");
    if (ps5.Triangle()) Serial.println("Triangle Button");

    if (ps5.UpRight()) {
      Serial.println("Up Right");
    }
    if (ps5.DownRight()) {
       maxspeed=ps5.DownRight();
      Serial.println("Down Right");
    }
    if (ps5.UpLeft()) Serial.println("Up Left");
    if (ps5.DownLeft()) Serial.println("Down Left");

    if (ps5.L1()) Serial.println("L1 Button");
    if (ps5.R1()) Serial.println("R1 Button");

    if (ps5.Share()) Serial.println("Share Button");
    if (ps5.Options()) Serial.println("Options Button");
    if (ps5.L3()) Serial.println("L3 Button");
    if (ps5.R3()) Serial.println("R3 Button");

    if (ps5.PSButton()) Serial.println("PS Button");
    if (ps5.Touchpad()) Serial.println("Touch Pad Button");

    if (ps5.L2()) {
      Serial.printf("L2 button at %d\n", ps5.L2Value());
    }
    if (ps5.R2()) {
      Serial.printf("R2 button at %d\n", ps5.R2Value());
    }

    if (ps5.LStickX()) {
      forward();
      Serial.printf("Left Stick x at %d\n", ps5.LStickX());
    }
    if (ps5.LStickY()) {
      Serial.printf("Left Stick y at %d\n", ps5.LStickY());
    }
    if (ps5.RStickX()) {
      Serial.printf("Right Stick x at %d\n", ps5.RStickX());
    }
    if (ps5.RStickY()) {
      Serial.printf("Right Stick y at %d\n", ps5.RStickY());
    }
    //
    ////    if (ps5.Charging()) Serial.println("The controller is charging"); //doesn't work
    ////    if (ps5.Audio()) Serial.println("The controller has headphones attached"); //doesn't work
    ////    if (ps5.Mic()) Serial.println("The controller has a mic attached"); //doesn't work
    //
    //    Serial.printf("Battery Level : %d\n", ps5.Battery()); //doesn't work

    Serial.println();

    delay(300);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////

///////////   PID  ///////////////
void pid() {
  float derivative = error - prerror;

  prerror = error;

  // Calculate PID output
  float pidOutput = Kp * error + Kd * derivative;
  if (millis() - timer > 1000) {
    Serial.print("\tError: ");
    Serial.print(error);
    Serial.print("\tPID Output: ");
    Serial.println(pidOutput);  /////////NEED CHECKING<<<<<<<<<<<
    motorspeed(pidOutput);
    timer = millis();
  }

}
////////MOTOR FUNCTIONS\\\\\\\\\\\\\\\\\

void forward()
{
  digitalWrite(motorApwm, HIGH);
  digitalWrite(motorBpwm, HIGH);
  digitalWrite(motorCpwm, HIGH);
  digitalWrite(motorDpwm, HIGH);
  digitalWrite(motorAdir, LOW);
  digitalWrite(motorBdir, LOW);
  digitalWrite(motorCdir, LOW);
  digitalWrite(motorDdir, LOW);
  analogWrite(motorApwm, leftMotorSpeed);
  analogWrite(motorBpwm, leftMotorSpeed);
  analogWrite(motorCpwm, rightMotorSpeed);
  analogWrite(motorDpwm, rightMotorSpeed);
  Serial.println("Forward");
}

void backward()
{
  digitalWrite(motorApwm, LOW);
  digitalWrite(motorBpwm, LOW);
  digitalWrite(motorCpwm, LOW);
  digitalWrite(motorDpwm, LOW);
  digitalWrite(motorAdir, HIGH);
  digitalWrite(motorBdir, HIGH);
  digitalWrite(motorCdir, HIGH);
  digitalWrite(motorDdir, HIGH);
  analogWrite(motorApwm, leftMotorSpeed);
  analogWrite(motorBpwm, leftMotorSpeed);
  analogWrite(motorCpwm, rightMotorSpeed);
  analogWrite(motorDpwm, rightMotorSpeed);
  Serial.println("Back");
}
void left()
{
  digitalWrite(motorApwm, HIGH);
  digitalWrite(motorBpwm, HIGH);
  digitalWrite(motorCpwm, LOW);
  digitalWrite(motorDpwm, LOW);
  digitalWrite(motorAdir, LOW);
  digitalWrite(motorBdir, LOW);
  digitalWrite(motorCdir, HIGH);
  digitalWrite(motorDdir, HIGH);
  analogWrite(motorApwm, leftMotorSpeed);
  analogWrite(motorBpwm, leftMotorSpeed);
  analogWrite(motorCpwm, rightMotorSpeed);
  analogWrite(motorDpwm, rightMotorSpeed);
  Serial.println("Left");
}
void right()
{
  digitalWrite(motorApwm, LOW);
  digitalWrite(motorBpwm, LOW);
  digitalWrite(motorCpwm, HIGH);
  digitalWrite(motorDpwm, HIGH);
  digitalWrite(motorAdir, HIGH);
  digitalWrite(motorBdir, HIGH);
  digitalWrite(motorCdir, LOW);
  digitalWrite(motorDdir, LOW);
  analogWrite(motorApwm, leftMotorSpeed);
  analogWrite(motorBpwm, leftMotorSpeed);
  analogWrite(motorCpwm, rightMotorSpeed);
  analogWrite(motorDpwm, rightMotorSpeed);
  Serial.println("Right");
}

void stop()
{
  digitalWrite(motorApwm, LOW);
  digitalWrite(motorBpwm, LOW);
  digitalWrite(motorCpwm, LOW);
  digitalWrite(motorDpwm, LOW);
  digitalWrite(motorAdir, LOW);
  digitalWrite(motorBdir, LOW);
  digitalWrite(motorCdir, LOW);
  digitalWrite(motorDdir, LOW);
  analogWrite(motorApwm, leftMotorSpeed);
  analogWrite(motorBpwm, leftMotorSpeed);
  analogWrite(motorCpwm, rightMotorSpeed);
  analogWrite(motorDpwm, rightMotorSpeed);
    Serial.println("STOP");
}

void buttonRight(){
  digitalWrite(motorApwm, HIGH);
  digitalWrite(motorBpwm, HIGH);
  digitalWrite(motorCpwm, LOW);
  digitalWrite(motorDpwm, LOW);
  digitalWrite(motorAdir, LOW);
  digitalWrite(motorBdir, LOW);
  digitalWrite(motorCdir, HIGH);
  digitalWrite(motorDdir, HIGH);
  analogWrite(motorApwm, leftMotorSpeed);
  analogWrite(motorBpwm, leftMotorSpeed);
  analogWrite(motorCpwm, rightMotorSpeed);
  analogWrite(motorDpwm, rightMotorSpeed);
    Serial.println("BUTTON-RIGHT");
}

void buttonLeft(){
  digitalWrite(motorApwm, LOW);
  digitalWrite(motorBpwm, LOW);
  digitalWrite(motorCpwm, HIGH);
  digitalWrite(motorDpwm, HIGH);
  digitalWrite(motorAdir, HIGH);
  digitalWrite(motorBdir, HIGH);
  digitalWrite(motorCdir, LOW);
  digitalWrite(motorDdir, LOW);
  analogWrite(motorApwm, leftMotorSpeed);
  analogWrite(motorBpwm, leftMotorSpeed);
  analogWrite(motorCpwm, rightMotorSpeed);
  analogWrite(motorDpwm, rightMotorSpeed);
     Serial.println("BUTTON-LEFT");
}

/*
    /////////MOTOR FUNCTIONS END\\\\\\\\\\\


*/
////////////////   PID   \\\\\\\\\\\\\\\\\\\\\

void motorspeed(float pidOutput) {
  leftMotorSpeed = constrain(basespeed + pidOutput, 50, maxspeed);
  rightMotorSpeed = constrain(basespeed - pidOutput, 50, maxspeed);

  analogWrite(motorApwm, leftMotorSpeed);
  analogWrite(motorBpwm, leftMotorSpeed);
  analogWrite(motorCpwm, rightMotorSpeed);
  analogWrite(motorDpwm, rightMotorSpeed);
}
