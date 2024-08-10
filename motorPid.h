Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, STBY);

void mspeed(int posa, int posb) {
  motor1.drive(posa);
  motor2.drive(posb);
}

void PID_wall() {
  while (1) {
    float v1 = sensor1.readRangeSingleMillimeters();
    float v2 = sensor2.readRangeSingleMillimeters();
    v1 = m1 * v1 + c1;
    v2 = m3 * v2 + c3;
    float position = v1 - v2;

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
    if( leftEncoder>300 || rightEncoder >300){
      leftEncoder = rightEncoder = 0;
      mspeed(0, 0);
      return;
    }
  }
  
}