#ifndef KIWI_DRIVE_H
#define KIWI_DRIVE_H

// Globale settpunkter
extern float setpoint1, setpoint2, setpoint3;

// Konstanter
const float ROTATION_RADIUS = 75.0; // mm fra senter til hjul

/**
 * KiwiDrive(vx_mm, vy_mm, omega_rad)
 *
 *  vx_mm, vy_mm: ønskede lineære hastigheter i mm/s langs robotens X- og Y-akser.
 *  omega_rad:    ønsket rotasjonshastighet i rad/s (positiv = CCW).
 *
 * Denne funksjonen beregner ønskede hjulhastigheter (i mm/s)
 * for en 3-hjuls Kiwi/omni-plattform og lagrer dem i
 * setpoint1, setpoint2, setpoint3.
 */
void KiwiDrive(float vx_mm, float vy_mm, float omega_rad)
{
  // Hjulvinkler (forutsetter 3-hjul på 120° mellomrom)
  float angle1 = radians(60 - 90);  // = -30 grader
  float angle2 = radians(180 - 90); // =  90 grader
  float angle3 = radians(300 - 90); // = 210 grader

  // Beregn den translasjonelle komponenten for hvert hjul
  float t1 = -sin(angle1) * vx_mm + cos(angle1) * -vy_mm;
  float t2 = -sin(angle2) * vx_mm + cos(angle2) * -vy_mm;
  float t3 = -sin(angle3) * vx_mm + cos(angle3) * -vy_mm;

  // Konverter rotasjonshastighet (rad/s) til lineær hastighet (mm/s)
  // ved hvert hjulets kontaktpunkt
  float rotational_linear_speed = omega_rad * ROTATION_RADIUS;

  // Legg til samme rotasjonskomponent til hvert hjul
  float w1 = t1 + rotational_linear_speed;
  float w2 = t2 + rotational_linear_speed;
  float w3 = t3 + rotational_linear_speed;

  // Tildel globale PID settpunkter (i mm/s)
  setpoint1 = w1;
  setpoint2 = w2;
  setpoint3 = w3;
}

#endif // KIWI_DRIVE_H
