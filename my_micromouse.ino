#include <SparkFun_TB6612.h>  
#include <Wire.h>             
#include <VL53L0X.h>          
#include <cstring>            
#include <utility>            
#include <queue>              
#include <vector>             
using namespace std;

class Position {
private:
  uint8_t _MAZE_SIZE;
public:
  uint8_t x;
  uint8_t y;
  uint8_t orient;

  Position();
  Position(uint8_t x, uint8_t y, uint8_t orient);
  void rotateLeft();
  void rotateRight();
  void rotate180();
  void moveTowards(int newOrient);
  void forward();
  void state() const;
};

#define PWMA 13
#define AIN1 14
#define AIN2 12
#define STBY 27
#define BIN1 26
#define BIN2 25
#define PWMB 33

#define ENCODER_A_PIN1 18
#define ENCODER_A_PIN2 5
#define ENCODER_B_PIN1 17
#define ENCODER_B_PIN2 16

#define LED 2

#define BLOCK_LENGTH 100 
#define MAZE_SIZE 16
#define DEFAULT_ORIENTATION 2  
#define START_X 0              
#define START_Y 0

#define maxSpeed 250
#define basespeeda 220
#define basespeedb 220

#define XSHUT_PIN_1 15  // GPIO pin for XSHUT of sensor 1
#define XSHUT_PIN_2 23  // GPIO pin for XSHUT of sensor 2
#define XSHUT_PIN_3 4   // GPIO pin for XSHUT of sensor 3

#define pinA 18
#define pinB 17


int stateA = 0;
int stateB = 0;

volatile long positionA = 0;
volatile long positionB = 0;

// Variables to store the previous state of the encoder pins
int prevEncoderAState1 = LOW;
int prevEncoderAState2 = LOW;
int prevEncoderBState1 = LOW;
int prevEncoderBState2 = LOW;

const int offsetA = 1;
const int offsetB = 1;

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

int limit1 = 0;
int limit2 = 0; 
int limit3 = 0;
int offset = 0;

int motorspeeda;
int motorspeedb;


uint16_t lastPosition = 0;
float Kp = 0.2;
float Ki = 0;
float Kd = 0.0;
int PL;
int IL;
int DL;
int error = 0;
int lastError = 0;

int rotationSpeed = 150;
int encoderCountPerRotation = 150;
float tyreDiameter = 43.1;  // in mm
float botDiameter = 135;    // in mm
float encoderToDistanceRatio = (3.14159 * tyreDiameter) / encoderCountPerRotation;
float degreeToEncoderRatio = (encoderCountPerRotation * botDiameter) / (360.0 * tyreDiameter);
float targetA, targetB;



VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

void updateEncoderA();
void updateEncoderB();
void printEncoder();
void encoderInit();
void PID_wall();
void setID();
void calibrateSensors();
void mspeed();
void maze_init();
void identifyBlockType();
void floodfill();
int nextBlock();
void printMatrix(uint8_t mat[][MAZE_SIZE]);

Position pos(START_X, START_Y, DEFAULT_ORIENTATION);
uint8_t maze[MAZE_SIZE][MAZE_SIZE];
uint8_t flood[MAZE_SIZE][MAZE_SIZE];

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Initialising the maze, sensors and encoders
void setup() {
  int start = millis();
  Serial.begin(115200);
  mazeInit();
  print("Maze setup completed at ", millis() - start, " ms");
  setID();
  print("Setup completed at ", millis() - start, " ms");
  calibrateSensors();
  print("calibration completed at ", millis() - start, " ms");


  //new
  // pinMode(ENCODER_A_PIN1, INPUT);
  // pinMode(ENCODER_A_PIN2, INPUT);
  // pinMode(ENCODER_B_PIN1, INPUT);
  // pinMode(ENCODER_B_PIN2, INPUT);

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN1), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN1), updateEncoderB, RISING);

  //new

  // encoderInit();
  // print("Encoder setup completed at ", millis()-start, " ms");
  // floodfill();
  // printMatrix(flood);
  // print("Rotating soon...");
  // delay(3000);
  // pos.moveTowards(3);
  // print("Rotation completed");
}

// Main algorithm
void loop() {
  mspeed(0,0);
  // identifyBlockType();

  mspeed(100,100);
  int i = 0;
  while(positionA < 200 && positionB < 200){
    Serial.print(i++);
    Serial.print("  ");
    printEncoder();
    updateEncoderA();
    updateEncoderB(); 
  }
  


  /*
    Sample algorithm to be followed at the end
    identifyBlock();
    int next = nextBlock();
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
    pos.moveTowards(next);
    resetEncoders();
    while (encoderDistance() < BLOCK_LENGTH) {
      pid_wall();
    }

  */
}

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
  // delay(50);
}



void mspeed(int posa, int posb) {
  motor1.drive(posa);
  motor2.drive(posb);
}

// For debug
// Base case for the variadic template
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

void PID_wall() {
  int m1 = sensor1.readRangeSingleMillimeters();
  int m2 = sensor2.readRangeSingleMillimeters();
  int position = m1 - m2;
  if (position + 5 > offset || position > offset) {
    Serial.print("In Left ");
  } else if (position - 5 < offset || position < offset) {
    Serial.print("In right ");
  } else {
    Serial.print("In center ");
  }
  // Serial.println(m3.RangeMilliMeter);
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
  // print("Left motor:", motorspeeda, "Right Motor:", motorspeedb);
  mspeed(motorspeeda, motorspeedb);
}

void setID() {
  Wire.begin();

  // Initialize the first sensor
  pinMode(XSHUT_PIN_1, OUTPUT);
  pinMode(XSHUT_PIN_2, OUTPUT);
  pinMode(XSHUT_PIN_3, OUTPUT);

  digitalWrite(XSHUT_PIN_1, LOW);  // Keep sensor 1 in shutdown
  digitalWrite(XSHUT_PIN_2, LOW);  // Keep sensor 2 in shutdown
  digitalWrite(XSHUT_PIN_3, LOW);  // Keep sensor 3 in shutdown

  delay(10);
  digitalWrite(XSHUT_PIN_1, HIGH);  // Turn on sensor 1
  delay(10);

  sensor1.setTimeout(500);
  if (!sensor1.init()) {
    Serial.println("Failed to detect and initialize sensor 1!");
    while (1)
      ;
  }
  sensor1.setAddress(0x30);  // Change I2C address of sensor 1 to 0x30

  // Initialize the second sensor
  digitalWrite(XSHUT_PIN_2, HIGH);  // Turn on sensor 2
  delay(10);

  sensor2.setTimeout(500);
  if (!sensor2.init()) {
    Serial.println("Failed to detect and initialize sensor 2!");
    while (1)
      ;
  }
  sensor2.setAddress(0x31);  // Change I2C address of sensor 2 to 0x31

  // Initialize the third sensor
  digitalWrite(XSHUT_PIN_3, HIGH);  // Turn on sensor 3
  delay(10);

  sensor3.setTimeout(500);
  if (!sensor3.init()) {
    Serial.println("Failed to detect and initialize sensor 3!");
    while (1)
      ;
  }
  sensor3.setAddress(0x32);  // Change I2C address of sensor 3 to 0x32

  // Now all sensors have unique addresses and can be used simultaneously
}

void calibrateSensors() {
  int times = 50;
  digitalWrite(LED, HIGH);
  for (int i = 0; i < times; i++) {
    limit1 += sensor1.readRangeSingleMillimeters();
    limit2 += sensor2.readRangeSingleMillimeters();
    limit3 += sensor3.readRangeSingleMillimeters();
  }
  limit1 /= times;
  limit2 /= times;
  limit3 /= times;

  offset = limit1 - limit2;
}

void resetEncoders() {
  encoderCountA = 0;
  encoderCountB = 0;
}

void encoderInit() {
  pinMode(ENCODER_A_PIN1, INPUT);
  pinMode(ENCODER_A_PIN2, INPUT);
  pinMode(ENCODER_B_PIN1, INPUT);
  pinMode(ENCODER_B_PIN2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN1), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN1), updateEncoderB, RISING);
}

void rotate(int deg) {
  float requiredCount = abs(deg) * degreeToEncoderRatio;
  int flagA = 1, flagB = 1;
  print("Attempting rotation by deg: ", deg);

  if (deg > 0) {
    targetA = encoderCountA + requiredCount;
    targetB = encoderCountB - requiredCount;
    mspeed(rotationSpeed, -rotationSpeed);
    while (flagA || flagB) {
      print(encoderCountA, targetA, encoderCountB, targetB);
      if (flagA && encoderCountA >= targetA) {
        flagA = 0;
        motor1.drive(0);
      }
      if (flagB && encoderCountB <= targetB) {
        flagB = 0;
        motor2.drive(0);
      }
    }
  } else {
    targetA = encoderCountA - requiredMotor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);Count;
    targetB = encoderCountB + requiredCount;
    mspeed(-rotationSpeed, rotationSpeed);
    while (flagA || flagB) {
      print(encoderCountA, targetA, encoderCountB, targetB);
      if (flagA && encoderCountA <= targetA) {
        flagA = 0;
        motor1.drive(0);
      }
      if (flagB && encoderCountB >= targetB) {
        flagB = 0;
        motor2.drive(0);
      }
    }
  }
}


void printMatrix(uint8_t mat[][MAZE_SIZE]) {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      printf("%2d ", mat[i][j]);
    }
    Serial.println();
  }
}

void mazeInit() {
  // After using flash memory, we will be reading maze from flash memory
  memset(maze, 0, MAZE_SIZE * MAZE_SIZE * sizeof(uint8_t));
}

void identifyBlockType() {
  uint8_t a[4], b[4];
  int error = 30;
  a[0] = sensor3.readRangeSingleMillimeters() < limit3 + error;
  a[1] = sensor1.readRangeSingleMillimeters() < limit1 + error;
  a[3] = sensor2.readRangeSingleMillimeters() < limit2 + error;
  a[2] = 0;

  for (int i = 0; i < 4; i++) {
    b[i] = a[(i + pos.orient) % 4];
  }

  uint8_t type = 8 * b[0] + 4 * b[1] + 2 * b[2] + b[3];
  print("Block (", pos.x, ", ", pos.y, ") is of type: ", type);
  maze[pos.x][pos.y] = type;
}

vector<pair<uint8_t, uint8_t>> neighbours(uint8_t i, uint8_t j) {
  vector<pair<uint8_t, uint8_t>> nb;
  if (i > 0 && flood[i - 1][j] == 255) {
    if (((maze[i - 1][j] >> 1) & 1 || (maze[i][j] >> 3) & 1) == 0)
      nb.push_back({ i - 1, j });
  }
  if (i < MAZE_SIZE - 1 && flood[i + 1][j] == 255) {
    if (((maze[i + 1][j] >> 3) & 1 || (maze[i][j] >> 1) & 1) == 0)
      nb.push_back({ i + 1, j });
  }
  if (j > 0 && flood[i][j - 1] == 255) {
    if (((maze[i][j - 1] >> 2) & 1 || (maze[i][j]) & 1) == 0)
      nb.push_back({ i, j - 1 });
  }
  if (j < MAZE_SIZE - 1 && flood[i][j + 1] == 255) {
    if (((maze[i][j + 1]) & 1 || (maze[i][j] >> 2) & 1) == 0)
      nb.push_back({ i, j + 1 });
  }
  return nb;
}


void floodfill() {
  int b = MAZE_SIZE / 2;
  int a = b - 1;
  memset(flood, -1, MAZE_SIZE * MAZE_SIZE * sizeof(uint8_t));
  memset(&flood[a][a], 0, 2 * sizeof(uint8_t));
  memset(&flood[b][a], 0, 2 * sizeof(uint8_t));

  queue<pair<uint8_t, uint8_t>> q;
  // printMatrix(flood);

  q.push({ a, a });
  q.push({ a, b });
  q.push({ b, a });
  q.push({ b, b });

  while (!q.empty()) {
    auto [i, j] = q.front();
    q.pop();

    for (auto [a, b] : neighbours(i, j)) {
      if (flood[a][b] == 255) {
        flood[a][b] = flood[i][j] + 1;
        q.push({ a, b });
      }
    }
  }
}

int nextBlock() {
  int i = pos.x, j = pos.y;
  if (i > 0 && flood[i - 1][j] < flood[i][j]) {
    if (((maze[i - 1][j] >> 1) & 1 || (maze[i][j] >> 3) & 1) == 0)
      return 0;
  }
  if (i < MAZE_SIZE && flood[i + 1][j] < flood[i][j]) {
    if (((maze[i + 1][j] >> 3) & 1 || (maze[i][j] >> 1) & 1) == 0)
      return 2;
  }
  if (j > 0 && flood[i][j - 1] < flood[i][j]) {
    if (((maze[i][j - 1] >> 2) & 1 || (maze[i][j]) & 1) == 0)
      return 1;
  }
  if (j < MAZE_SIZE && flood[i][j + 1] < flood[i][j]) {
    if (((maze[i][j + 1]) & 1 || (maze[i][j] >> 2) & 1) == 0)
      return 3;
  }
  return -1;
}

Position::Position() {
  x = 0;
  y = 0;
  orient = 0;
  _MAZE_SIZE = 10;  // Default MAZE_SIZE
}

Position::Position(uint8_t x, uint8_t y, uint8_t orient) {
  this->x = x;
  this->y = y;
  this->orient = orient;
  this->_MAZE_SIZE = MAZE_SIZE;
}

void Position::rotateLeft() {
  print("rotating left");
  orient = (orient + 1) % 4;
  rotate(-90);
}

void Position::rotateRight() {
  print("rotating right");
  orient = (orient + 3) % 4;
  rotate(90);
}

void Position::rotate180() {
  print("rotating 180");
  orient = (orient + 2) % 4;
  rotate(180);
}

void Position::moveTowards(int newOrient) {
  int diff = newOrient - orient;
  if (diff == 1 || diff == -3) rotateLeft();
  else if (diff == -1 || diff == 3) rotateRight();
  else if (abs(diff) == 2) rotate180();
}

void Position::forward() {
  if (orient == 0 && x > 0) x--;
  if (orient == 1 && y > 0) y--;
  if (orient == 2 && x < _MAZE_SIZE - 1) x++;
  if (orient == 3 && y < _MAZE_SIZE - 1) y++;
}
