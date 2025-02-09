/*******************************************************
 *  TAB 3: MotorControl.ino
 *******************************************************/
#ifndef MOTOR_CONTROL_INO
#define MOTOR_CONTROL_INO

// Pin Definitions (adjust as needed)
const int M1_IN1 = 14;
const int M1_IN2 = 27;
const int M2_IN1 = 25;
const int M2_IN2 = 26;
const int M3_IN1 = 33;
const int M3_IN2 = 32;

/**
 * MotorControl_Init()
 *   Initializes all motor driver pins.
 */
void MotorControl_Init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  Serial.println("Motor control initialized");
}

/**
 * Motor1(float speed)
 *  speed in range [-255..+255]
 */
void Motor1(float speed) {
  if (speed > 0) {
    int pwmVal = constrain(speed, 0, 255);
    analogWrite(M1_IN1, pwmVal);
    analogWrite(M1_IN2, 0);
  } else if (speed < 0) {
    int pwmVal = constrain(-speed, 0, 255);
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, pwmVal);
  } else {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, 0);
  }
}

/**
 * Motor2(float speed)
 *  speed in range [-255..+255]
 */
void Motor2(float speed) {
  if (speed > 0) {
    int pwmVal = constrain(speed, 0, 255);
    analogWrite(M2_IN1, pwmVal);
    analogWrite(M2_IN2, 0);
  } else if (speed < 0) {
    int pwmVal = constrain(-speed, 0, 255);
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, pwmVal);
  } else {
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, 0);
  }
}

/**
 * Motor3(float speed)
 *  speed in range [-255..+255]
 */
void Motor3(float speed) {
  if (speed > 0) {
    int pwmVal = constrain(speed, 0, 255);
    analogWrite(M3_IN1, pwmVal);
    analogWrite(M3_IN2, 0);
  } else if (speed < 0) {
    int pwmVal = constrain(-speed, 0, 255);
    analogWrite(M3_IN1, 0);
    analogWrite(M3_IN2, pwmVal);
  } else {
    analogWrite(M3_IN1, 0);
    analogWrite(M3_IN2, 0);
  }
}

#endif // MOTOR_CONTROL_INO
