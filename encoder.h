
void countLeftOut1(){
    leftEncoder = leftEncoder + 1;  
}

void countRightOut1(){
    rightEncoder = rightEncoder + 1;
}


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


int rotationSpeed = 150;
int encoderCountPerRotation = 150;
float tyreDiameter = 39.6;  // in mm
float botDiameter = 108;    // in mm
float encoderToDistanceRatio = (3.14159 * tyreDiameter) / encoderCountPerRotation;
float distanceToEncoderRatio = 1.0/encoderToDistanceRatio;
float degreeToEncoderRatio = (encoderCountPerRotation * botDiameter) / (360.0 * tyreDiameter);

float blockLength = 1260/5.0; // in mm
int blockSize = blockLength*distanceToEncoderRatio;
float targetA, targetB;

void rotate(int deg) {
  float requiredCount = abs(deg) * degreeToEncoderRatio;
  int flagA = 1, flagB = 1;
  print("Attempting rotation by deg: ", deg);

  if (deg > 0) {
    targetA = rightEncoder + requiredCount;
    targetB = leftEncoder + requiredCount;
    mspeed(-rotationSpeed, rotationSpeed);
    while (flagA || flagB) {
      print(rightEncoder, targetA, leftEncoder, targetB);
      if (flagA && rightEncoder >= targetA) {
        flagA = 0;
        motor2.drive(0);
      }
      if (flagB && leftEncoder >= targetB) {
        flagB = 0;
        motor1.drive(0);
      }
    }
  } else {
    targetA = rightEncoder + requiredCount;
    targetB = leftEncoder + requiredCount;
    mspeed(rotationSpeed, -rotationSpeed);
    while (flagA || flagB) {
      print(rightEncoder, targetA, leftEncoder, targetB);
      if (flagA && rightEncoder >= targetA) {
        flagA = 0;
        motor2.drive(0);
      }
      if (flagB && leftEncoder >= targetB) {
        flagB = 0;
        motor1.drive(0);
      }
    }
  }
}

