/*******************************************************
 *  TAB 2: KiwiDrive.ino
 *******************************************************/
#ifndef KIWI_DRIVE_INO
#define KIWI_DRIVE_INO

// We need to use the global setpoints declared in the main tab
extern float setpoint1, setpoint2, setpoint3;

/**
 * KiwiDrive(vx, vy, omega)
 *  vx, vy: dimensionless translational commands (-1..+1)
 *  omega:  dimensionless rotational command  (-1..+1)
 *
 *  This function calculates the desired wheel speeds (mm/s)
 *  for each of the three omni wheels and stores them
 *  in setpoint1, setpoint2, setpoint3.
 */
void KiwiDrive(float vx, float vy, float omega)
{
  // For your project, set the maximum translation/rotation speeds:
  const float MAX_TRANS_SPEED = 300.0; // mm/s for vx or vy = ±1.0
  const float MAX_ROT_SPEED   = 1.0;   // "1.0" = 1 rad/s, for example

  // Convert dimensionless [-1..+1] to mm/s or rad/s
  float vx_mm = vx * MAX_TRANS_SPEED;
  float vy_mm = vy * MAX_TRANS_SPEED;
  float w_rad = omega * MAX_ROT_SPEED;

  // Adjust direction if needed: 
  // For some Kiwi layouts, forward might be -vy instead of vy
  // vy_mm = -vy_mm;  // (uncomment if needed)

  // Wheel angles (assuming 3-wheel kiwi at 120° each)
  float angle1 = radians(  60 - 90 ); // = -30 deg
  float angle2 = radians( 180 - 90 ); // =  90 deg
  float angle3 = radians( 300 - 90 ); // = 210 deg

  // Translational components
  float t1 = -sin(angle1)*vx_mm + cos(angle1)*vy_mm;
  float t2 = -sin(angle2)*vx_mm + cos(angle2)*vy_mm;
  float t3 = -sin(angle3)*vx_mm + cos(angle3)*vy_mm;

  // Convert rotation to linear speed at the wheel.
  // This depends on your geometry (distance from center, etc).
  // For a quick example, define some constant:
  const float ROTATION_GAIN_MM = 50.0;
  float w1 = t1 + (ROTATION_GAIN_MM * w_rad);
  float w2 = t2 + (ROTATION_GAIN_MM * w_rad);
  float w3 = t3 + (ROTATION_GAIN_MM * w_rad);

  // Assign to global PID setpoints
  setpoint1 = w1;
  setpoint2 = w2;
  setpoint3 = w3;

  // (Optional) Debug:
  /*
  Serial.print("KiwiDrive => w1: "); Serial.print(w1);
  Serial.print(", w2: "); Serial.print(w2);
  Serial.print(", w3: "); Serial.print(w3);
  Serial.println(" mm/s");
  */
}

#endif // KIWI_DRIVE_INO
