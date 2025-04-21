#ifndef KIWI_DRIVE_H
#define KIWI_DRIVE_H

#include <Arduino.h>

// Dette arrayet vil lagre m�lhastigheten (mm/s) for hvert hjul
extern float setpointSpeed[3];

// Skaleringskonstanter
static const float KIWI_SCALE_LINEAR   = 1.0f;
static const float KIWI_SCALE_ROTATION = 1.0f;

// Hjulvinkler (i radianer)
static const float ANGLE_1 = (150.0f - 90.0f) * DEG_TO_RAD; // -30�
static const float ANGLE_2 = (270.0f - 90.0f) * DEG_TO_RAD; //  90�
static const float ANGLE_3 = (390.0f - 90.0f) * DEG_TO_RAD; // 210�

static const float ROTATION_RADIUS = 155.0f; // mm

inline void kiwiSetup() {
  // Ingen init n�dvendig per n�
}

/**
 * setKiwiDrive(speedX, speedY, rotation_dps)
 *
 *  speedX, speedY: translasjon i mm/s  (float gir finere kontroll)
 *  rotation_dps:  rotasjon i grader/s (float, 0?360+)
 */
inline void setKiwiDrive(float speedX, float speedY, float rotation_dps)
{
  // 1) Skaler line�rhastigheter
  float vx = speedX * KIWI_SCALE_LINEAR;
  float vy = speedY * KIWI_SCALE_LINEAR;

  // 2) Konverter grader/s til rad/s
  float vr = rotation_dps * DEG_TO_RAD * KIWI_SCALE_ROTATION;

  // 3) Translasjonskomponenter per hjul
  float t1 = -sin(ANGLE_1) * vx + cos(ANGLE_1) * -vy;
  float t2 = -sin(ANGLE_2) * vx + cos(ANGLE_2) * -vy;
  float t3 = -sin(ANGLE_3) * vx + cos(ANGLE_3) * -vy;

  // 4) Omregn rotasjon (rad/s) ? line�r hastighet (mm/s)
  float rotation_linear = vr * ROTATION_RADIUS;

  // 5) Sum translasjon + rotasjon
  setpointSpeed[0] = t1 - rotation_linear;
  setpointSpeed[1] = t2 - rotation_linear;
  setpointSpeed[2] = t3 - rotation_linear;
}

#endif // KIWI_DRIVE_H
