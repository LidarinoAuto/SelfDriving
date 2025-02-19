/*******************************************************
 *  TAB 2: KiwiDrive.ino
 *******************************************************/
#ifndef KIWI_DRIVE_INO
#define KIWI_DRIVE_INO

// We need to use the global setpoints declared in the main tab
extern float setpoint1, setpoint2, setpoint3;

/**
 * KiwiDrive(vx_mm, vy_mm, omega_rad)
 *
 *  vx_mm, vy_mm: desired linear speeds in mm/s along the robot’s X and Y axes.
 *  omega_rad:    desired rotational speed in rad/s (positive = CCW, for example).
 *
 * This function calculates the desired per-wheel speeds (in mm/s)
 * for a 3-wheel Kiwi/omni platform and stores them in
 * setpoint1, setpoint2, setpoint3.
 */
void KiwiDrive(float vx_mm, float vy_mm, float omega_rad)
{
  // Wheel angles (assuming 3-wheel at 120° spacing).
  // We often do (60°, 180°, 300°) or (0°, 120°, 240°), etc.
  // Subtract 90 to align your coordinate system if needed.
  float angle1 = radians(  60 - 90 ); // = -30 deg
  float angle2 = radians( 180 - 90 ); // =  90 deg
  float angle3 = radians( 300 - 90 ); // = 210 deg

  // Calculate the translational component for each wheel
  float t1 = -sin(angle1)*vx_mm + cos(angle1)*-vy_mm;
  float t2 = -sin(angle2)*vx_mm + cos(angle2)*-vy_mm;
  float t3 = -sin(angle3)*vx_mm + cos(angle3)*-vy_mm;

  // We need to convert rotational speed (rad/s) into
  // a linear speed (mm/s) at each wheel's contact point.
  // That depends on the distance from the center to each wheel.
  // Let's define a constant "ROTATION_RADIUS" (in mm):
  float ROTATION_RADIUS = 75.0; // example: 75 mm from center to wheel
  // Then linear speed at the wheel = omega_rad * ROTATION_RADIUS
  // However, if your geometry differs (some Kiwi designs
  // incorporate different angles), you can tweak further.
  float rotational_linear_speed = omega_rad * ROTATION_RADIUS;

  // We add the same rotational component to each wheel
  // but note that direction might differ in some designs.
  // For a typical Kiwi, it’s each wheel the same sign.
  float w1 = t1 + rotational_linear_speed;
  float w2 = t2 + rotational_linear_speed;
  float w3 = t3 + rotational_linear_speed;

  // Assign to global PID setpoints (in mm/s)
  setpoint1 = w1;
  setpoint2 = w2;
  setpoint3 = w3;

  // Optional Debug:
  /*
  Serial.print("KiwiDrive => w1: "); Serial.print(w1);
  Serial.print(", w2: "); Serial.print(w2);
  Serial.print(", w3: "); Serial.print(w3);
  Serial.println(" mm/s");
  */
}

#endif // KIWI_DRIVE_INO
