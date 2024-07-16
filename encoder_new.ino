#include <SparkFun_TB6612.h>

#define PWMA 13
#define AIN2 12
#define AIN1 14
#define STBY 27
#define BIN1 26
#define BIN2 25
#define PWMB 33

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Encoder pins
#define pinA 18
#define pinB 17

int stateA = 0;
int stateB = 0;

volatile long positionA = 0;
volatile long positionB = 0;

// Variables to store the previous state of the encoder pins
// int prevEncoderAState1 = LOW;
// int prevEncoderAState2 = LOW;
// int prevEncoderBState1 = LOW;
// int prevEncoderBState2 = LOW;

void updateEncoderA() {
  uint8_t s = stateA & 3; // Preserve the last state of the lower 2 bits
  if (digitalRead(pinA)) s |= 4; // Read the current state of pin1
  switch (s) {
        case 0: case 5: case 10: case 15:
            // No change in position
            break;
        case 1: case 7: case 8: case 14:
        case 2: case 4: case 11: case 13:
        case 3: case 12:
        default:
            positionA++;
            break;
    }
  stateA = (s >> 2); // Update the state with the current reading
}
void updateEncoderB() {
  uint8_t s = stateB & 3; // Preserve the last state of the lower 2 bits
  if (digitalRead(pinB)) s |= 4; // Read the current state of pin1
  switch (s) {
        case 0: case 5: case 10: case 15:
            // No change in position
            break;
        case 1: case 7: case 8: case 14:
        case 2: case 4: case 11: case 13:
        case 3: case 12:
        default:
            positionB++;
            break;
    }
  stateB = (s >> 2); // Update the state with the current reading
}

void printEncoder() {
  Serial.print(positionA);
  Serial.print("\t");
  Serial.print(positionB);
  Serial.print("\n");
  delay(50);
}

void mspeed(int posa, int posb) {
  motor1.drive(posa);
  motor2.drive(posb);
}

int rotationSpeed = 150;
int encoderCountPerRotation = 150;
float tyreDiameter = 43.1; // in mm
float botDiameter = 120; // in mm
float degreeToEncoderRatio = (encoderCountPerRotation * botDiameter) / (360.0 * tyreDiameter);

// void rotate(int deg) {
//   int initialCountA = encoderCountA;
//   int initialCountB = encoderCountB;
//   float requiredCount = abs(deg) * degreeToEncoderRatio;

//   if (deg > 0) {
//     while ((encoderCountA < (initialCountA + requiredCount)) && (encoderCountB < (initialCountB + requiredCount))) {
//       mspeed(rotationSpeed, -rotationSpeed);
//       updateEncoderA();
//       updateEncoderB();
//     }
//   } else {
//     while ((encoderCountA > (initialCountA - requiredCount)) && (encoderCountB > (initialCountB - requiredCount))) {
//       mspeed(-rotationSpeed, rotationSpeed);
//       updateEncoderA();
//       updateEncoderB();
//     }
//   }
//   motor1.brake();
//   motor2.brake();
// }

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  Serial.begin(115200);
  Serial.println("Basic Encoder Test:");
}

void loop() {
  printEncoder();
  updateEncoderA();
  updateEncoderB();
}
