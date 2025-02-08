#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void motorSetup() {
  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
}

void setMotorSpeed(int speed) {
  if (speed > 0) {
    digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    analogWrite(MOTOR_ENABLE_PIN, speed);
  } 
  else if (speed < 0) {
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
    analogWrite(MOTOR_ENABLE_PIN, abs(speed));
  } 
  else {
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    analogWrite(MOTOR_ENABLE_PIN, 0);
  }
}

#endif
