#ifndef KIWI_DRIVE_H
#define KIWI_DRIVE_H

#include <Arduino.h>

// Dette arrayet vil lagre målhastigheten (mm/s) for hvert hjul
// f.eks. setpointSpeed[0] = hastighet for hjul 1
//       setpointSpeed[1] = hastighet for hjul 2
//       setpointSpeed[2] = hastighet for hjul 3
extern float setpointSpeed[3];

// Evt. justeringskonstanter
static const float KIWI_SCALE_LINEAR   = 1.0;  
static const float KIWI_SCALE_ROTATION = 1.0;

// 3 hjul, 120° mellom hver. Her er en vanlig definisjon:
// (60°, 180°, 300°) justert -90° for orientering.
static const float ANGLE_1 = (150.0  - 90.0) * DEG_TO_RAD; // = -30° i rad
static const float ANGLE_2 = (270.0 - 90.0) * DEG_TO_RAD; // =  90° i rad
static const float ANGLE_3 = (390.0 - 90.0) * DEG_TO_RAD; // = 210° i rad;

// Hvis du vil bruke en radius for rotasjon (mm):
// [Du kan justere til 75 mm, 100 mm e.l. avhengig av robotens geometri]
static const float ROTATION_RADIUS = 50.0;

inline void kiwiSetup() {
  // Evt. spesielle init-rutiner for KiwiDrive. 
  // Akkurat nå: ingen.
}

/**
 * setKiwiDrive(speedX, speedY, rotation)
 *
 *  speedX, speedY: translasjon i mm/s (eller en vilkårlig skala)
 *  rotation:       rotasjon i rad/s (eller en vilkårlig skala)
 *
 * Resultat: setter setpointSpeed[] for hvert av de 3 hjulene.
 */
inline void setKiwiDrive(int speedX, int speedY, int rotation)
{
  // Konverter til float og ev. skaler
  float vx = speedX   * KIWI_SCALE_LINEAR;
  float vy = speedY   * KIWI_SCALE_LINEAR;
  float vr = rotation * KIWI_SCALE_ROTATION; // rotasjon

  // Translasjonskomponent for hvert hjul
  float t1 = (-sin(ANGLE_1) * vx) + (cos(ANGLE_1) * -vy);
  float t2 = (-sin(ANGLE_2) * vx) + (cos(ANGLE_2) * -vy);
  float t3 = (-sin(ANGLE_3) * vx) + (cos(ANGLE_3) * -vy);

  // Regn om rotasjon (rad/s) til lineær hastighet (mm/s)
  float rotation_linear = vr * ROTATION_RADIUS;

  // Summér translasjon + rotasjon
  float w1 = t1 - rotation_linear;
  float w2 = t2 - rotation_linear;
  float w3 = t3 - rotation_linear;

  // Disse hastighetene (mm/s) vil vi at hvert hjul skal oppnå:
  setpointSpeed[0] = w1;
  setpointSpeed[1] = w2;
  setpointSpeed[2] = w3;
}

#endif // KIWI_DRIVE_H
