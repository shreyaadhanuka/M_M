VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

// void tofRead()
// {
//     tof[0] = m1 * sensor1.readRangeSingleMillimeters() + c1;
//     tof[4] = m3 * sensor2.readRangeSingleMillimeters() + c3;
// }

void setID() {
  Wire.begin();

  pinMode(XSHUT_PIN_1, OUTPUT);
  pinMode(XSHUT_PIN_2, OUTPUT);
  pinMode(XSHUT_PIN_3, OUTPUT);

  digitalWrite(XSHUT_PIN_1, LOW);  
  digitalWrite(XSHUT_PIN_2, LOW);  
  digitalWrite(XSHUT_PIN_3, LOW);  

  delay(10);
  digitalWrite(XSHUT_PIN_1, HIGH);  
  delay(10);

  sensor1.setTimeout(500);
  if (!sensor1.init()) {
    Serial.println("Failed to detect and initialize sensor 1!");
    while (1);
  }
  sensor1.setAddress(0x30);  

  digitalWrite(XSHUT_PIN_2, HIGH);  
  delay(10);

  sensor2.setTimeout(500);
  if (!sensor2.init()) {
    Serial.println("Failed to detect and initialize sensor 2!");
    while (1);
  }
  sensor2.setAddress(0x31);  

  digitalWrite(XSHUT_PIN_3, HIGH);  
  delay(10);

  sensor3.setTimeout(500);
  if (!sensor3.init()) {
    Serial.println("Failed to detect and initialize sensor 3!");
    while (1);
  }
  sensor3.setAddress(0x32);  
}

// void read_sensors() {
  
//   //Serial.println("Initiated ... ");
//   lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
//   lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
//   lox3.rangingTest(&measure3, false);
//   // print sensor one reading
//   Serial.print(F("1: "));
//   if(measure1.RangeStatus != 4) {     // if not out of range
//     Serial.print(measure1.RangeMilliMeter);
//   } else {
//     Serial.print(F("Out of range"));
//   }
  
//   Serial.print(F(" "));

//   // print sensor two reading
//   Serial.print(F("2: "));
//   if(measure2.RangeStatus != 4) {
//     Serial.print(measure2.RangeMilliMeter);
//   } else {
//     Serial.print(F("Out of range"));
//   }

//   Serial.print(F(" "));

//   // print sensor 3 reading
//   Serial.print(F("3: "));
//   if(measure3.RangeStatus != 4) {
//     Serial.print(measure3.RangeMilliMeter);
//   } else {
//     Serial.print(F("Out of range"));
//   }
  
//   Serial.println();
// }
