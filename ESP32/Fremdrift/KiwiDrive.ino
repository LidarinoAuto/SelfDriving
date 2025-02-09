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
  vy = -vy;  // (Optional) Invert vy if needed to fix forward/backward reversal
  
  float rad1 = radians(WHEEL_ANGLE_1 - 90);
  float rad2 = radians(WHEEL_ANGLE_2 - 90);
  float rad3 = radians(WHEEL_ANGLE_3 - 90);

  float t1 = -sin(rad1) * vx + cos(rad1) * vy;
  float t2 = -sin(rad2) * vx + cos(rad2) * vy;
  float t3 = -sin(rad3) * vx + cos(rad3) * vy;

  float w1 = t1 + ROTATION_GAIN * omega;
  float w2 = t2 + ROTATION_GAIN * omega;
  float w3 = t3 + ROTATION_GAIN * omega;

  float maxVal = max(max(abs(w1), abs(w2)), abs(w3));

  if (maxVal < 0.0001) {
    w1 = 0;
    w2 = 0;
    w3 = 0;

    M1_Stop();
    M2_Stop();
    M3_Stop();
    return;
  } else {
    w1 /= maxVal;
    w2 /= maxVal;
    w3 /= maxVal;
  }

  w1 *= maxPWM;
  w2 *= maxPWM;
  w3 *= maxPWM;

  int pwm1 = (int)abs(w1);
  int pwm2 = (int)abs(w2);
  int pwm3 = (int)abs(w3);

  // Motor 1
  if (w1 >= 0.0) {
    M1_Forward(pwm1);  // Send PWM-verdi til funksjonen
  } else {
    M1_Backward(pwm1);
  }

  // Motor 2
  if (w2 >= 0.0) {
    M2_Forward(pwm2);
  } else {
    M2_Backward(pwm2);
  }

  // Motor 3
  if (w3 >= 0.0) {
    M3_Forward(pwm3);
  } else {
    M3_Backward(pwm3);
  }

  // Debug output
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
