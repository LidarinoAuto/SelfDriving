// ==================== PIN DEFINITIONS ====================
const int M1_IN1 = 14;
const int M1_IN2 = 27;

const int M2_IN1 = 25;
const int M2_IN2 = 26;

const int M3_IN1 = 33;
const int M3_IN2 = 32;

// ==================== INITIALIZATION ====================
void MotorControl_Init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  // Debugging info
  Serial.begin(115200);
  Serial.println("Motor control initialized");
}

// ==================== MOTOR 1 CONTROL ====================
void Motor1(float speed) {
  if (speed > 0) {
      int constrainedSpeed = constrain(speed, 0, 255);
      analogWrite(M1_IN1, constrainedSpeed);
      analogWrite(M1_IN2, 0);
      //Serial.print("Motor 1 Forward Speed: ");
      //Serial.println(constrainedSpeed);
  } else if (speed < 0) {
      int constrainedSpeed = constrain(-speed, 0, 255);
      analogWrite(M1_IN1, 0);
      analogWrite(M1_IN2, constrainedSpeed);
      //Serial.print("Motor 1 Backward Speed: ");
      //Serial.println(constrainedSpeed);
  } else {
      analogWrite(M1_IN1, 0);
      analogWrite(M1_IN2, 0);
      //Serial.println("Motor 1 Stopped");
  }
}

// ==================== MOTOR 2 CONTROL ====================
void Motor2(float speed) {
  if (speed > 0) {
      int constrainedSpeed = constrain(speed, 0, 255);
      analogWrite(M2_IN1, constrainedSpeed);
      analogWrite(M2_IN2, 0);
      //Serial.print("Motor 2 Forward Speed: ");
      //Serial.println(constrainedSpeed);
  } else if (speed < 0) {
      int constrainedSpeed = constrain(-speed, 0, 255);
      analogWrite(M2_IN1, 0);
      analogWrite(M2_IN2, constrainedSpeed);
      //Serial.print("Motor 2 Backward Speed: ");
      //Serial.println(constrainedSpeed);
  } else {
      analogWrite(M2_IN1, 0);
      analogWrite(M2_IN2, 0);
      //Serial.println("Motor 2 Stopped");
  }
}

// ==================== MOTOR 3 CONTROL ====================
void Motor3(float speed) {
  if (speed > 0) {
      int constrainedSpeed = constrain(speed, 0, 255);
      analogWrite(M3_IN1, constrainedSpeed);
      analogWrite(M3_IN2, 0);
      //Serial.print("Motor 3 Forward Speed: ");
      //Serial.println(constrainedSpeed);
  } else if (speed < 0) {
      int constrainedSpeed = constrain(-speed, 0, 255);
      analogWrite(M3_IN1, 0);
      analogWrite(M3_IN2, constrainedSpeed);
      //Serial.print("Motor 3 Backward Speed: ");
      //Serial.println(constrainedSpeed);
  } else {
      analogWrite(M3_IN1, 0);
      analogWrite(M3_IN2, 0);
      //Serial.println("Motor 3 Stopped");
  }
}
