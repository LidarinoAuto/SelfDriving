#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Pin Definisjoner brukes direkte fra Fane 1
const int PWM_MAX = 255; // Maksimal PWM verdi
const int PWM_MIN = 0;   // Minimal PWM verdi

void motorSetup() {
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);

  Serial.println("Motor control initialized");
}

void controlMotor(int in1Pin, int in2Pin, float speed) {
  int pwmVal = constrain(speed, PWM_MIN, PWM_MAX);
  if (speed > 0) {
    analogWrite(in1Pin, pwmVal);
    analogWrite(in2Pin, PWM_MIN);
  } 
  else if (speed < 0) {
    pwmVal = constrain(-speed, PWM_MIN, PWM_MAX);
    analogWrite(in1Pin, PWM_MIN);
    analogWrite(in2Pin, pwmVal);
  } 
  else {
    analogWrite(in1Pin, PWM_MIN);
    analogWrite(in2Pin, PWM_MIN);
  }
}

void Motor1(float speed) {
  controlMotor(MOTOR1_IN1, MOTOR1_IN2, speed);
}

void Motor2(float speed) {
  controlMotor(MOTOR2_IN1, MOTOR2_IN2, speed);
}

void Motor3(float speed) {
  controlMotor(MOTOR3_IN1, MOTOR3_IN2, speed);
}

#endif // MOTOR_CONTROL_H
