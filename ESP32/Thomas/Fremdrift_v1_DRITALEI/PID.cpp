#include <Arduino.h>
#include "PID.h"

float computePID(float setpoint, float actual_speed, float &filtered_speed, float &integral, float &last_error, float interval) {
  // Bruk lavpassfilter til å jevne ut hastighetsmålingen
  filtered_speed = (ALPHA * filtered_speed) + ((1 - ALPHA) * actual_speed);
  
  // Beregn feil basert på filtrert hastighet
  float error = setpoint - filtered_speed;
  
  // Oppdater integral- og derivatelementene
  integral += error * (interval / 1000.0);
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT); // Begrens integral for å unngå "windup"
  
  float derivative = (error - last_error) / (interval / 1000.0); // Beregn derivat
  float control_signal = kp * error + ki * integral + kd * derivative; // Beregn kontrollsignal
  
  // Påfør dødsone for å unngå små bevegelser
  if (abs(control_signal) < DEAD_ZONE) {
    control_signal = 0;
  }

  // Lagre siste feilverdi
  last_error = error;
  
  return control_signal;
}
