#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "PinConfig.h"

void motorSetup() {
  // Sett opp pinner for alle tre motorene
  pinMode(MOTOR_1_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_1_BACKWARD_PIN, OUTPUT);

  pinMode(MOTOR_2_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_2_BACKWARD_PIN, OUTPUT);

  pinMode(MOTOR_3_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_3_BACKWARD_PIN, OUTPUT);

  // Hvis du har egne ENABLE-pinner, sett dem opp her
  // pinMode(MOTOR_1_ENABLE_PIN, OUTPUT);
  // pinMode(MOTOR_2_ENABLE_PIN, OUTPUT);
  // pinMode(MOTOR_3_ENABLE_PIN, OUTPUT);
}

void setMotorSpeed(int forwardPin, int backwardPin, int speed) {
  // speed > 0 => fremover
  // speed < 0 => bakover
  // speed = 0 => stopp
  if (speed > 0) {
    int pwmVal = constrain(speed, 0, 255);
    analogWrite(forwardPin, pwmVal);
    analogWrite(backwardPin, 0);
  } 
  else if (speed < 0) {
    int pwmVal = constrain(-speed, 0, 255);
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, pwmVal);
  } 
  else {
    // Stopp
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, 0);
  }
}

#endif // MOTOR_CONTROL_H
