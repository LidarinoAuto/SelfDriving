
// Hjulene står plassert med 120° mellom seg.
// Oppsettet avhenger av hvordan du faktisk har montert hjulene.
// La oss si hjul 1 står i vinkel 0°, hjul 2 = 120°, hjul 3 = 240°:
const float WHEEL_ANGLE_1 = 90.0;
const float WHEEL_ANGLE_2 = 210.0;
const float WHEEL_ANGLE_3 = 330.0;

/**************************************************************
 * KiwiDrive()
 *   vx, vy: Desired velocity components (e.g. -1.0..+1.0)
 *   maxPWM: Maximum PWM value (0..255)
 **************************************************************/
void KiwiDrive(float vx, float vy, float maxPWM) {
  // Convert degrees -> radians
  float rad1 = radians(WHEEL_ANGLE_1);
  float rad2 = radians(WHEEL_ANGLE_2);
  float rad3 = radians(WHEEL_ANGLE_3);

  // Calculate raw wheel speeds (could be negative or positive)
  float w1 = -sin(rad1) * vx + cos(rad1) * vy;
  float w2 = -sin(rad2) * vx + cos(rad2) * vy;
  float w3 = -sin(rad3) * vx + cos(rad3) * vy;

  // Find max absolute value for normalization
  float maxVal = max(max(abs(w1), abs(w2)), abs(w3));

  // If maxVal is near zero, everything goes to zero => robot stands still
  if (maxVal < 0.0001) {
    w1 = 0;
    w2 = 0;
    w3 = 0;
  } else {
    // Normalize so that the largest magnitude becomes 1
    w1 /= maxVal;
    w2 /= maxVal;
    w3 /= maxVal;
  }

  // Scale to the desired maxPWM range
  w1 *= maxPWM;
  w2 *= maxPWM;
  w3 *= maxPWM;

  // -----------------------------------------------------------------------
  // Motor 1
  int pwm1 = (int)abs(w1);
  bool m1Forward = (w1 >= 0.0);
  if (m1Forward) {
    M1_Forward();
    M1_SetSpeed(pwm1);
  } else {
    M1_Backward();
    M1_SetSpeed(pwm1);
  }

  // Motor 2
  int pwm2 = (int)abs(w2);
  bool m2Forward = (w2 >= 0.0);
  if (m2Forward) {
    M2_Forward();
    M2_SetSpeed(pwm2);
  } else {
    M2_Backward();
    M2_SetSpeed(pwm2);
  }

  // Motor 3
  int pwm3 = (int)abs(w3);
  bool m3Forward = (w3 >= 0.0);
  if (m3Forward) {
    M3_Forward();
    M3_SetSpeed(pwm3);
  } else {
    M3_Backward();
    M3_SetSpeed(pwm3);
  }

  // -----------------------------------------------------------------------
  // Print out debug information: raw float, direction, final PWM
  // Motor 1
  Serial.print("Motor1 raw=");
  Serial.print(w1);
  Serial.print(", dir=");
  Serial.print(m1Forward ? "Fwd" : "Bwd");
  Serial.print(", pwm=");
  Serial.println(pwm1);

  // Motor 2
  Serial.print("Motor2 raw=");
  Serial.print(w2);
  Serial.print(", dir=");
  Serial.print(m2Forward ? "Fwd" : "Bwd");
  Serial.print(", pwm=");
  Serial.println(pwm2);

  // Motor 3
  Serial.print("Motor3 raw=");
  Serial.print(w3);
  Serial.print(", dir=");
  Serial.print(m3Forward ? "Fwd" : "Bwd");
  Serial.print(", pwm=");
  Serial.println(pwm3);

  Serial.println("---------------");

}
