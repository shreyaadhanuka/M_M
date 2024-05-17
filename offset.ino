#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include "TB6612_ESP32.h"  
#define PWMA 13
#define AIN2 12
#define AIN1 14
#define STBY 27
#define BIN1 26
#define BIN2 25
#define PWMB 33

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY,5000 ,8,1 );
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY,5000 ,8,2 );

#define LED 2

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 15 //right 15
#define SHT_LOX2 23 //left 23
#define SHT_LOX3 4 //center 4

#define maxSpeed 250
#define basespeeda 220
#define basespeedb 220

int motorspeeda;
int motorspeedb;

int offset = 0;

uint16_t lastPosition = 0;
float Kp = 0.2; 
float Ki = 0;  
float Kd = 0.0;
int PL;
int IL;
int DL;
int error = 0; 
int lastError = 0;


// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// this holds the measurement
int measure1;
int measure2;
int measure3;

VL53L0X_RangingMeasurementData_t m1;
VL53L0X_RangingMeasurementData_t m2;
VL53L0X_RangingMeasurementData_t m3;

void mspeed(int posa, int posb) {
  motor1.drive(posa);
  motor2.drive(posb);
}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  Serial.println("ALl reset ");
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  Serial.println("ALl unreset ");

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);

  //Serial.println("activating 1st ... ");
  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);
  
  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);


  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }

  delay(10);

  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX2
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }
}

void avg() {
  int sum=0;

  for(int i = 0; i < 400; i++){
    lox1.rangingTest(&m1, false); // pass in 'true' to get debug data printout!
    lox2.rangingTest(&m2, false); // pass in 'true' to get debug data printout!
    if(m1.RangeStatus != 4 && m2.RangeStatus != 4) {
      digitalWrite(LED,HIGH);
      delay(5);
      digitalWrite(LED,LOW);
      delay(5);
      sum = sum +  m1.RangeMilliMeter - m2.RangeMilliMeter;
    }
  }

  // Serial.print(F("1: "));
  // Serial.print(avg1);
  // Serial.print(" ");
  // Serial.print(F("2: "));
  // Serial.print(avg2);
  // Serial.println();
  
  offset = sum/1000;
  //Serial.println(offset);

}



void PID_wall() {
  lox1.rangingTest(&m1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&m2, false);
  lox3.rangingTest(&m3, false);
  if(m3.RangeMilliMeter > 75){
    int position = m1.RangeMilliMeter - m2.RangeMilliMeter;
    if(position + 5 > offset || position > offset){
      Serial.print("In Left ");
    }else if(position - 5 < offset || position < offset){
      Serial.print("In right ");
    }else{
      Serial.print("In center ");
    }
    Serial.println(m3.RangeMilliMeter);
    int error = position - offset;
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
    // Serial.print(F("left motorspeed : "));
    // Serial.print(motorspeeda);
    // Serial.print("\t");
    // Serial.print(F("right motorspeed : "));
    // Serial.println(motorspeedb);
    mspeed(motorspeeda, motorspeedb);
  }else{
    mspeed(0,0);
  }
    

}


void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  Serial.println(F("Both in reset mode...(pins are low)"));
  
  pinMode(LED, OUTPUT);
  Serial.println(F("Starting..."));
  setID();
  avg();

  // Attach encoder pins


  // Set motor speed to 0 initially
 
}

void loop() {
   
  PID_wall();
  //delay(50);

  // Print encoder position

  // Map encoder position to motor speed


  // Set motor speed

  // If encoder position exceeds a certain value, reverse motor direction


  // Add a delay to control loop execution rate

}