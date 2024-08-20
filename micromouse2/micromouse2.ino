#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "define.h"
#include <vector>
#include "tofSensor.h"
#include "motorPid.h"
#include "encoder.h"
#include "position.h"
#include "mazeSolve.h"

// int tofThresh[] = {0, 0, 0};
// void calibrate() {
//   int times = 50;
//   for (int i = 0; i < times; i++) {
//     Serial.println("Doing");
//     tofThresh[0] += sensor1.readRangeSingleMillimeters();
//     tofThresh[1] += sensor2.readRangeSingleMillimeters();
//     tofThresh[2] += sensor3.readRangeSingleMillimeters();
//   }
//   for (int i = 0; i < 3; i++) {
//     tofThresh[i] /= times;
//   }
// }

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(15), countLeftOut1, RISING);
  attachInterrupt(digitalPinToInterrupt(19), countRightOut1, RISING);
  pinMode(14, INPUT);
  pinMode(35, INPUT);


  // wifi_init();
  // wifi_log("Starting the log");
  while (1) {
    mspeed(0, 0);
    int s1 = digitalRead(14);
    int s2 = digitalRead(35);
    if (s1 == HIGH) {
      maxSpeed = 160;
      basespeeda = 120;
      basespeedb = 120;
      break;  // Here we have to choose the Rule which will follow by the bot
      // S1 switch for LHS and s2 switch for RHS
    }
    if (s2 == HIGH) {
      maxSpeed = 200;
      basespeeda = 150;
      basespeedb = 150;
      break;  // Here we have to choose the Rule which will follow by the bot
      // S1 switch for LHS and s2 switch for RHS
    }
  }
  delay(1000);
  setID();
}

void loop() {
  mspeed(0, 0);
  identifyBlockType();
  floodfill();
  int next = nextBlock();
  if (next == -1) {
    floodfill();
    next = nextBlock();
    if (next == -1) {
      while (1) {
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
      }
    }
  }

  int diff = next - pos.orient;
  if (diff == 1 || diff == -3) {
    pos.orient = (pos.orient + 1) % 4;
    rotate(-90);
    turnFlag = diff;
  } else if (diff == -1 || diff == 3) {
    pos.orient = (pos.orient + 3) % 4;
    rotate(90);
    turnFlag = diff;
  } else if (abs(diff) == 2) {
    pos.orient = (pos.orient + 2) % 4;
    rotate(180);
  }

  if (pos.orient == 0 && pos.x > 0) pos.x--;
  if (pos.orient == 1 && pos.y > 0) pos.y--;
  if (pos.orient == 2 && pos.x < pos._MAZE_SIZE - 1) pos.x++;
  if (pos.orient == 3 && pos.y < pos._MAZE_SIZE - 1) pos.y++;

  if (abs(turnFlag) == 1 || abs(turnFlag) == 3) {
    leftEncoder = 10;
    rightEncoder = 10;
  } else {
    leftEncoder = 0;
    rightEncoder = 0;
  }

  tof[2] = sensor3.readRangeSingleMillimeters() ;

  while ((rightEncoder < 300 || leftEncoder < 300) && tof[2] > 90) {
    // tof[0] = m1 * sensor1.readRangeSingleMillimeters() + c1;
    // tof[2] = m5 * sensor3.readRangeSingleMillimeters() + c5;
    // tof[4] = m3 * sensor2.readRangeSingleMillimeters() + c3;
    tof[0] = sensor1.readRangeSingleMillimeters() ;
    tof[2] = sensor3.readRangeSingleMillimeters() ;
    tof[4] = sensor2.readRangeSingleMillimeters() ;

    if (tof[0] < tofThreshold && tof[4] < tofThreshold) {
      float position = tof[0] - tof[4];

      int error = position;
      PL = error;
      IL = IL + error;
      DL = error - lastError;
      lastError = error;
      int motorspeed = PL * Kp + IL * Ki + DL * Kd;

      int motorspeeda = basespeeda + motorspeed;
      int motorspeedb = basespeedb - motorspeed;

      if (motorspeeda > maxSpeed) {
        motorspeeda = maxSpeed;
      }
      if (motorspeedb > maxSpeed) {
        motorspeedb = maxSpeed;
      }
      if (motorspeeda < 0) {
        motorspeeda = 0;
      }
      if (motorspeedb < 0) {
        motorspeedb = 0;
      }
      mspeed(motorspeeda, motorspeedb);
    }

    else if (tof[0] > tofThreshold && tof[4] < tofThreshold) {
      rightError = tof[4] - 100;
      rightDiff = rightError - rightLastError;

      int correction = (rightError * rightP) + (rightDiff * rightD);
      rightLastError = rightError;

      if (correction < -40) {
        correction = -20;
      }

      int motorspeeda = basespeeda - correction;
      int motorspeedb = basespeedb + correction;
      if (motorspeeda > maxSpeed) {
        motorspeeda = maxSpeed;
      }
      if (motorspeedb > maxSpeed) {
        motorspeedb = maxSpeed;
      }
      if (motorspeeda < 0) {
        motorspeeda = 0;
      }
      if (motorspeedb < 0) {
        motorspeedb = 0;
      }
      // else if (correction < -20)
      // {
      //     correction = -10;
      // }
      mspeed(motorspeeda, motorspeedb);
    }

    else if (tof[0] < tofThreshold && tof[4] > tofThreshold) {
      leftError = 90 - tof[0];
      leftDiff = leftError - leftLastError;

      int correction = (leftError * leftP) + (leftDiff * leftD);
      leftLastError = leftError;
      if (correction < -40) {
        correction = -20;
      }
      int motorspeeda = basespeeda - correction;
      int motorspeedb = basespeedb + correction;
      if (motorspeeda > maxSpeed) {
        motorspeeda = maxSpeed;
      }
      if (motorspeedb > maxSpeed) {
        motorspeedb = maxSpeed;
      }
      if (motorspeeda < 0) {
        motorspeeda = 0;
      }
      if (motorspeedb < 60) {
        motorspeedb = 60;
      }

      mspeed(motorspeeda, motorspeedb);
    }

    else if (tof[0] > tofThreshold && tof[4] > tofThreshold) {
      encoderError = leftEncoder - rightEncoder;
      // if (encoderError > 50) {
      //   encoderError = 10;
      // } else if (encoderError < -50) {
      //   encoderError = -10;
      // }
      encoderCorrection = float(encoderError * encoderP) + float(encoderLastError * encoderD);

      mspeed(basespeeda - encoderCorrection, basespeedb + encoderCorrection);
    }
  }
}



// void loop(){
//   // encoderPid();
// }
