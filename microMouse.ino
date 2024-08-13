#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <vector>
#include "define.h"
// #include "wifi_logger.h"
#include "tofSensor.h"
#include "motorPid.h"
#include "encoder.h"
#include "position.h"
#include "mazeSolve.h"

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(15), countLeftOut1, RISING);
  attachInterrupt(digitalPinToInterrupt(19), countRightOut1, RISING);
  pinMode(14, INPUT);

  
  // wifi_init();
  // wifi_log("Starting the log");
  while (1) {
    mspeed(0, 0);
    int s1 = digitalRead(14);
    if (s1 == HIGH) {
      break;  // Here we have to choose the Rule which will follow by the bot
      // S1 switch for LHS and s2 switch for RHS
    }

  }setID();
}

void loop() {
    // wifi_log("Entering void loop");
    // // wifi_log(to_string(pos.x) + " " + to_string(pos.y) + " " + to_string(pos.orient));
    mspeed(0, 0);
    identifyBlockType();
    floodfill();
    int next = nextBlock();
    // wifi_log("At block: ("+String(pos.x)+", "+String(pos.y)+") orient: "+String(pos.orient));
    // wifi_log("Block identified as type: " + String(maze[pos.x][pos.y]));
    // wifi_log("next block towards: "+String(next));
    // floodfill();
    if (next == -1) {
      floodfill();
      next = nextBlock();
      if (next == -1) {
        while (1) {
        digitalWrite(LED, HIGH);
        delay(1000);
        digitalWrite(LED, LOW);
        delay(1000);
        }
      }
    }
    // wifi_log("Rotating");
    pos.moveTowards(next);
    pos.forward();

  leftEncoder = 0;
  rightEncoder = 0;

    while(rightEncoder<300 || leftEncoder<300){
      wallFollow();
    }

  // int flagA = 1, flagB = 1;
  // targetA = rightEncoder + blockSize;
  // targetB = leftEncoder + blockSize;
  //   mspeed(rotationSpeed, rotationSpeed);
  //   while (flagA || flagB) {
  //     print(rightEncoder, targetA, flagA, leftEncoder, targetB, flagB);
  //     if (flagA && rightEncoder >= targetA) {
  //       flagA = 0;
  //       motor2.drive(0);
  //     }
  //     if (flagB && leftEncoder >= targetB) {
  //       flagB = 0;
  //       motor1.drive(0);
  //     }
  //   }
    // wifi_log("one_block_travelled");
    // delay(1000);

}


// void loop(){
//   rotate(90);
//   delay(2000);
//   rotate(-90);
//   delay(2000);
//   rotate(180);
//   delay(2000);
//   rotate(-180);
//   delay(2000);
// }
