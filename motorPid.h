Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, STBY);

void mspeed(int posa, int posb) {
  motor1.drive(posa);
  motor2.drive(posb);
}

void wallPid() {
    // tof[0] = sensor1.readRangeSingleMillimeters();
    // tof[4] = sensor2.readRangeSingleMillimeters();
    // tof[0] = m1 * tof[0] + c1;
    // tof[4] = m3 * tof[4] + c3;
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

void leftPid() {
  tof[0] = sensor1.readRangeSingleMillimeters();
  tof[0] = m1 * tof[0] + c1;
  leftError = 100 - tof[0];
  leftDiff = leftError - leftLastError;

  int correction = (leftError * leftP) + (leftDiff * leftD);
  leftLastError = leftError;

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

void rightPid() {
  tof[4] = sensor2.readRangeSingleMillimeters();
  tof[4] = m3 * tof[4] + c3;
  Serial.println(tof[4]);
  rightError = tof[4] - 100;
  rightDiff = rightError - rightLastError;

  int correction = (rightError * rightP) + (rightDiff * rightD);
  rightLastError = rightError;

  int motorspeeda = basespeeda - correction;
  int motorspeedb = basespeedb + correction;
  if (motorspeeda > maxSpeed) {
    motorspeeda = maxSpeed;
  }
  if (motorspeedb > maxSpeed) {
    motorspeedb = maxSpeed;
  }
  if (motorspeeda < 60) {
    motorspeeda = 60;
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
void encoderPid() {
  encoderError = leftEncoder - rightEncoder;
  if (encoderError > 50) {
    encoderError = 10;
  } else if (encoderError < -50) {
    encoderError = -10;
  }
  encoderCorrection = float(encoderError * encoderP) + float(encoderLastError * encoderD);

  mspeed(basespeeda - encoderCorrection, basespeedb + encoderCorrection);
}
void wallFollow() {
  tof[0] = m1 * sensor1.readRangeSingleMillimeters() + c1;
  tof[4] = m3 * sensor2.readRangeSingleMillimeters() + c3;
  if (tof[0] < tofThreshold && tof[4] < tofThreshold) {
    wallPid();
  }

  else if (tof[0] > tofThreshold && tof[4] < tofThreshold) {
    rightPid();
  }

  else if (tof[0] < tofThreshold && tof[4] > tofThreshold) {
    leftPid();
  }

  else if (tof[0] > tofThreshold && tof[4] > tofThreshold) {
    encoderPid();
  }
}
