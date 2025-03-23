#ifndef DEBUGGING_H
#define DEBUGGING_H

#include <Arduino.h>
#include "PID.h"

// Toggle debugging output
bool debugStatusEnabled = false;
bool debugPIDEnabled    = false;


/**
 * Enkel statusutskrift av overordnede variabler
 */
inline void debugPrintStatus(int speedX, int speedY, int rotation, bool pwmDisabled) {
  if (!debugStatusEnabled) return;
  Serial.print("[DEBUG] speedX="); Serial.print(speedX);
  Serial.print(" speedY="); Serial.print(speedY);
  Serial.print(" rotation="); Serial.print(rotation);
  Serial.print(" pwmDisabled="); Serial.println(pwmDisabled ? "YES" : "NO");
}

/**
 * Skriver ut PID-status for hvert hjul:
 * - Setpoint
 * - Measured speed
 * - Error
 * - P-, I-, D-bidrag
 * - Total PWM
 */
inline void debugPrintPID(const float setpoint[], const float actual[], const float pwm[], float dt) {
  if (!debugPIDEnabled) return;
  Serial.println("=== PID DEBUG ===");
  for (uint8_t i = 0; i < 3; i++) {
    float error = setpoint[i] - actual[i];
    float p = pid_kp[i] * error;
    float i_term = pid_ki[i] * pid_integral[i];
    float d = pid_kd[i] * ((error - pid_lastError[i]) / dt);
    Serial.print("Wheel "); Serial.print(i);
    Serial.print(" | SP="); Serial.print(setpoint[i], 2);
    Serial.print(" Act="); Serial.print(actual[i], 2);
    Serial.print(" Err="); Serial.print(error, 2);
    Serial.print(" P="); Serial.print(p, 2);
    Serial.print(" I="); Serial.print(i_term, 2);
    Serial.print(" D="); Serial.print(d, 2);
    Serial.print(" PWM="); Serial.println(pwm[i], 1);
  }
  Serial.println();
}

#endif // DEBUGGING_H
