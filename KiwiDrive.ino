// KiwiDrive.ino

// Define the wheel mounting angles in degrees
const float WHEEL_ANGLE_1 = 60.0;
const float WHEEL_ANGLE_2 = 180.0;
const float WHEEL_ANGLE_3 = 300.0;

// You can adjust this constant to fine-tune how strongly rotation affects
// each wheel’s speed. Essentially, it converts your desired rotational velocity
// into a PWM contribution.
const float ROTATION_GAIN = 1.0;

/**************************************************************
 * KiwiDrive()
 *   vx, vy: Desired translational velocity components (e.g. -1.0..+1.0)
 *   omega: Desired rotational speed (positive for, say, counterclockwise)
 *   maxPWM: Maximum PWM value (0..255)
 **************************************************************/
void KiwiDrive(float vx, float vy, float omega, float maxPWM) {
  // (Optional) Invert vy if needed to fix forward/backward reversal
  vy = -vy;
  
  // Apply a 90° correction to the wheel geometry if necessary:
  float rad1 = radians(WHEEL_ANGLE_1 - 90);
  float rad2 = radians(WHEEL_ANGLE_2 - 90);
  float rad3 = radians(WHEEL_ANGLE_3 - 90);

  // Compute the translational component for each wheel.
  // These equations are based on projecting the desired velocity vector 
  // onto the axis perpendicular to each wheel's roller axis.
  float t1 = -sin(rad1) * vx + cos(rad1) * vy;
  float t2 = -sin(rad2) * vx + cos(rad2) * vy;
  float t3 = -sin(rad3) * vx + cos(rad3) * vy;

  // Mix in rotation:
  // Here we simply add a term based on the desired rotation (omega) scaled by a gain.
  float w1 = t1 + ROTATION_GAIN * omega;
  float w2 = t2 + ROTATION_GAIN * omega;
  float w3 = t3 + ROTATION_GAIN * omega;

  // Normalize the wheel speeds so that none exceed the range after mixing.
  float maxVal = max(max(abs(w1), abs(w2)), abs(w3));

  if (maxVal < 0.0001) {
    // When the maximum value is near zero, there’s no movement.
    w1 = 0;
    w2 = 0;
    w3 = 0;
  } else {
    w1 /= maxVal;
    w2 /= maxVal;
    w3 /= maxVal;
  }

  // Scale speeds to the maximum PWM value.
  w1 *= maxPWM;
  w2 *= maxPWM;
  w3 *= maxPWM;

  // -----------------------------------------------------------------------
  // Set Motor 1
  int pwm1 = (int)abs(w1);
  if (w1 >= 0.0) {
    M1_Forward();
    M1_SetSpeed(pwm1);
  } else {
    M1_Backward();
    M1_SetSpeed(pwm1);
  }

  // Set Motor 2
  int pwm2 = (int)abs(w2);
  if (w2 >= 0.0) {
    M2_Forward();
    M2_SetSpeed(pwm2);
  } else {
    M2_Backward();
    M2_SetSpeed(pwm2);
  }

  // Set Motor 3
  int pwm3 = (int)abs(w3);
  if (w3 >= 0.0) {
    M3_Forward();
    M3_SetSpeed(pwm3);
  } else {
    M3_Backward();
    M3_SetSpeed(pwm3);
  }

  // (Optional) Debug output:
  Serial.print("Motor1 raw=");
  Serial.print(w1);
  Serial.print(", pwm=");
  Serial.println(pwm1);

  Serial.print("Motor2 raw=");
  Serial.print(w2);
  Serial.print(", pwm=");
  Serial.println(pwm2);

  Serial.print("Motor3 raw=");
  Serial.print(w3);
  Serial.print(", pwm=");
  Serial.println(pwm3);

  Serial.println("---------------");
}
