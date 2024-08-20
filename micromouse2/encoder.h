#define PWMA 26
#define AIN1 25
#define AIN2 4
#define STBY 16
#define BIN1 17
#define BIN2 5
#define PWMB 18

#define XSHUT_PIN_1 13  
#define XSHUT_PIN_2 32  
#define XSHUT_PIN_3 27  

#define pinA 15
#define pinB 19

int maxSpeed; //160/200
int basespeeda; //120/150
int basespeedb; //120/150

// #define m1 0.977767
// #define m3 0.964964
// #define m5 1.00739

// #define c1 2.40135
// #define c3 -13.458
// #define c5 -3.31112

#define tofThreshold 150


int stateA = 0;
int stateB = 0;

int turnFlag = 0;

volatile long rightEncoder = 0;
volatile long leftEncoder = 0;

// float Kp = 1.5;
// float Ki = 0;
// float Kd = 1.1;
// float encoderP = 5;
// float encoderD = 0; 
// float leftP = 1.6;
// float leftD = 2;//1.2
// float rightP = 1.6;
// float rightD = 2;

float Kp = 1.1; //1.5;
float Ki = 0;
float Kd = 1.7; //1.1;
float encoderP = 0.2 ;
float encoderD = 0; 
float leftP = 1.3;
float leftD = 5;//1.2
float rightP = 1.3;
float rightD = 8;

float tof[5]={0,0,0,0,0};


float encoderError = 0;
float encoderLastError = 0;
float encoderDiff = 0;
float encoderCorrection = 0;
float leftError = 0;
float leftLastError = 0;
float leftDiff = 0;
float rightError = 0;
float rightLastError = 0;
float rightDiff = 0;
int PL;
int IL;
int DL;
int lastError = 0;

#define LED 2
#define MAZE_SIZE 5

uint8_t maze[MAZE_SIZE][MAZE_SIZE];
uint8_t flood[MAZE_SIZE][MAZE_SIZE];

