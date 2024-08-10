#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <vector>
#include "define.h"
#include "encoder.h"
#include "tofSensor.h"
#include "motorPid.h"
// #include "mazeSolve.h"
// #include "position.h"
void print() {
  Serial.println();  // Print a newline at the end of the line
}

// Variadic template function to handle multiple arguments
template<typename T, typename... Args>
void print(T first, Args... args) {
  Serial.print(first);
  Serial.print(" ");
  print(args...);  // Recursive call to handle the next argument
}


int rotationSpeed = 150;
int encoderCountPerRotation = 150;
float tyreDiameter = 40;  // in mm
float botDiameter = 110;    // in mm
float encoderToDistanceRatio = (3.14159 * tyreDiameter) / encoderCountPerRotation;
float degreeToEncoderRatio = (encoderCountPerRotation * botDiameter) / (360.0 * tyreDiameter);
float targetA, targetB;

void rotate(int deg) {
  float requiredCount = abs(deg) * degreeToEncoderRatio;
  int flagA = 1, flagB = 1;
  print("Attempting rotation by deg: ", deg);

  if (deg > 0) {
    targetA = rightEncoder + requiredCount;
    targetB = leftEncoder + requiredCount;
    mspeed(-rotationSpeed, rotationSpeed);
    while (flagA || flagB) {
      print(rightEncoder, targetA, leftEncoder, targetB);
      if (flagA && rightEncoder >= targetA) {
        flagA = 0;
        motor1.drive(0);
      }
      if (flagB && leftEncoder >= targetB) {
        flagB = 0;
        motor2.drive(0);
      }
    }
  } else {
    targetA = rightEncoder + requiredCount;
    targetB = leftEncoder + requiredCount;
    mspeed(rotationSpeed, -rotationSpeed);
    while (flagA || flagB) {
      print(rightEncoder, targetA, leftEncoder, targetB);
      if (flagA && rightEncoder >= targetA) {
        flagA = 0;
        motor1.drive(0);
      }
      if (flagB && leftEncoder >= targetB) {
        flagB = 0;
        motor2.drive(0);
      }
    }
  }
}
void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(15), countLeftOut1, RISING);
  attachInterrupt(digitalPinToInterrupt(19), countRightOut1, RISING);
  // mspeed(0, 0);
  // delay(5000);
//   delay(1000);
// rotate(90);
// delay(1000);
// rotate(-90);
// delay(1000);
// rotate(180);
// delay(1000);
// rotate(-180);
  // setID();
  
}

void loop() {
  while(digitalRead(14)){
    delay(200);
    rotate(90);
  }
  // PID_wall();
  // delay(1000);
  // if(sensor1.readRangeSingleMillimeters()*m1+c1>300)rotate(-90);
  // else if(sensor2.readRangeSingleMillimeters()*m3+c3>300)rotate(90);



}
