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

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(15), countLeftOut1, RISING);
  attachInterrupt(digitalPinToInterrupt(19), countRightOut1, RISING);
  mspeed(0, 0);
  delay(5000);

  setID();
  
}

void loop() {
  PID_wall();
  delay(1000);
// Serial.print("A : ");
// Serial.print(leftEncoder);
// Serial.print("\t");
// Serial.print("B : ");
// Serial.println(rightEncoder);
}