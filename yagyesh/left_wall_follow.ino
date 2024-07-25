#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <VL53L0X.h>

#define PWMA 13
#define AIN1 14
#define AIN2 12
#define STBY 27
#define BIN1 26
#define BIN2 25
#define PWMB 33

#define LED 2

#define FRONT_SENSOR 15
#define LEFT_SENSOR 23
#define RIGHT_SENSOR 4

#define WALL_THRESHOLD 100 // Adjust based on your maze dimensions
#define TURN_DELAY 500     // Adjust based on your robot's turning speed
#define MOVE_SPEED 150

VL53L0X frontSensor;
VL53L0X leftSensor;
VL53L0X rightSensor;

Motor leftMotor = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, 1, STBY);

void setup()
{
    Serial.begin(115200);

    Wire.begin();
    setupSensors();

    pinMode(LED, OUTPUT);

    delay(2000); // Give time for everything to initialize
}

void loop()
{
    leftWallFollow();
}

void setupSensors()
{
    pinMode(FRONT_SENSOR, OUTPUT);
    pinMode(LEFT_SENSOR, OUTPUT);
    pinMode(RIGHT_SENSOR, OUTPUT);

    digitalWrite(FRONT_SENSOR, LOW);
    digitalWrite(LEFT_SENSOR, LOW);
    digitalWrite(RIGHT_SENSOR, LOW);

    delay(10);

    // Setup front sensor
    digitalWrite(FRONT_SENSOR, HIGH);
    delay(10);
    frontSensor.init();
    frontSensor.setAddress((uint8_t)0x30);

    // Setup left sensor
    digitalWrite(LEFT_SENSOR, HIGH);
    delay(10);
    leftSensor.init();
    leftSensor.setAddress((uint8_t)0x31);

    // Setup right sensor
    digitalWrite(RIGHT_SENSOR, HIGH);
    delay(10);
    rightSensor.init();
    rightSensor.setAddress((uint8_t)0x32);
}

void leftWallFollow()
{
    int frontDistance = frontSensor.readRangeSingleMillimeters();
    int leftDistance = leftSensor.readRangeSingleMillimeters();
    int rightDistance = rightSensor.readRangeSingleMillimeters();

    Serial.print("Front: ");
    Serial.print(frontDistance);
    Serial.print(" Left: ");
    Serial.print(leftDistance);
    Serial.print(" Right: ");
    Serial.println(rightDistance);

    if (leftDistance > WALL_THRESHOLD)
    {
        // No wall on the left, turn left
        turnLeft();
    }
    else if (frontDistance < WALL_THRESHOLD)
    {
        // Wall in front, turn right
        turnRight();
    }
    else
    {
        // Wall on the left and no wall in front, move forward
        moveForward();
    }
}

void moveForward()
{
    leftMotor.drive(MOVE_SPEED);
    rightMotor.drive(MOVE_SPEED);
    delay(100); // Adjust this delay as needed
}

void turnLeft()
{
    leftMotor.drive(-MOVE_SPEED);
    rightMotor.drive(MOVE_SPEED);
    delay(TURN_DELAY);
    stopMotors();
}

void turnRight()
{
    leftMotor.drive(MOVE_SPEED);
    rightMotor.drive(-MOVE_SPEED);
    delay(TURN_DELAY);
    stopMotors();
}

void stopMotors()
{
    leftMotor.brake();
    rightMotor.brake();
}