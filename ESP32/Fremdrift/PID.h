#ifndef PID_H
#define PID_H

#include <Arduino.h>

// Disse deklareres i .ino-filen
extern float pid_kp[3];
extern float pid_ki[3];
extern float pid_kd[3];

extern float pid_integral[3];
extern float pid_lastError[3];

// Funksjon med anti-windup
inline float computePID_withAntiWindup(byte i, float setpoint, float actual, float dt) {
  float error = setpoint - actual;

  // Oppdater integral
  pid_integral[i] += error * dt;

  // Begrens integral (anti-windup del 1)
  const float integralLimit = 100.0;
  if (pid_integral[i] > integralLimit)  pid_integral[i] = integralLimit;
  if (pid_integral[i] < -integralLimit) pid_integral[i] = -integralLimit;

  float derivative = (error - pid_lastError[i]) / dt;

  // Regn ut PID:
  float output = (pid_kp[i] * error)
               + (pid_ki[i] * pid_integral[i])
               - (pid_kd[i] * derivative);

  // Begrens til PWM-område
  output = constrain(output, -255, 255);

  // Hvis utgangen er mettet, "rull tilbake" litt (anti-windup del 2)
  if (output == 255.0f || output == -255.0f) {
    // Fjern forrige integral-endring
    pid_integral[i] -= error * dt;
  }

  pid_lastError[i] = error;
  return output;
}

#endif // PID_H
