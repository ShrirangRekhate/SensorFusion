#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

/////  MOTOR PINS  \\\\\\

const int motorApwm=13;//left
const int motorAdir=12;
const int motorBpwm=27;//left
const int motorBdir=26;
const int motorCpwm=33;//right
const int motorCdir=32;
const int motorDpwm=18;//right
const int motorDdir=19;

float leftMotorSpeed=0.0;
float rightMotorSpeed=0.0;

////  MOTOR ADJUSTMENT END \\\\

MPU6050 mpu6050(Wire);
///   PID CONSTANTS  \\\

float prerror=0.0;
float error=0.0;
long timer = 0;
float pidOutput=0.0;
const float Kp = 2.0;  
const float Ki = 0.05; 
const float Kd = 0.1;  

////    PID ENDS  \\\\

void setup() {
  Serial.begin(9600);
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
  mpu6050.update();
  
  if(millis() - timer > 1000){
        float angle=mpu6050.getAngleZ();
//        Serial.print("\tgyroAngleZ : ");Serial.print(mpu6050.getGyroAngleZ());
//        Serial.print("\tangleZ : ");Serial.print(angle);
//        Serial.print("\n");
         timer = millis();

  error=0.0-angle;
  
    float derivative = error - prerror;

    prerror = error;

    // Calculate PID output
    float pidOutput = Kp * error + Kd * derivative;
    Serial.print("\tError: ");
    Serial.print(error);
    Serial.print("\tPID Output: ");
    Serial.println(pidOutput);  /////////NEED CHECKING<<<<<<<<<<<
  }
  delay(1000);
  motorspeed(pidOutput);

  
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
}
/*
 *  /////////MOTOR FUNCTIONS END\\\\\\\\\\\ 
 * 
 * 
 */
void motorspeed(float pidOutput){
   leftMotorSpeed = constrain(255 + pidOutput, 0, 255);
   rightMotorSpeed = constrain(255 - pidOutput, 0, 255);
  if (pidOutput > 0) {
digitalWrite(motorApwm, LOW);
digitalWrite(motorBpwm, LOW);
digitalWrite(motorCpwm, HIGH);
digitalWrite(motorDpwm, HIGH);
digitalWrite(motorAdir, HIGH);
digitalWrite(motorBdir, HIGH);
digitalWrite(motorCdir, LOW);
digitalWrite(motorDdir, LOW);
    Serial.println("Right");
  }
  if(pidOutput < 0){
digitalWrite(motorApwm, HIGH);
digitalWrite(motorBpwm, HIGH);
digitalWrite(motorCpwm, LOW);
digitalWrite(motorDpwm, LOW);
digitalWrite(motorAdir, LOW);
digitalWrite(motorBdir, LOW);
digitalWrite(motorCdir, HIGH);
digitalWrite(motorDdir, HIGH);
    Serial.println("left");
  }
analogWrite(motorApwm, leftMotorSpeed);
analogWrite(motorBpwm, leftMotorSpeed);
analogWrite(motorCpwm, rightMotorSpeed);
analogWrite(motorDpwm, rightMotorSpeed);
}
