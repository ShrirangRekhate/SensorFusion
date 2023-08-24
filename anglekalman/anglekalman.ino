#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);
float prerror=0.0;
float error=0.0;
long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
  

        float angle=mpu6050.getAngleZ();
//        Serial.print("\tgyroAngleZ : ");Serial.print(mpu6050.getGyroAngleZ());
        Serial.print("\tangleZ : ");Serial.print(angle);
        Serial.print("\n");

  
  
}
