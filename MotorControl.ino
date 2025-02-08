// ==================== PIN DEFINITIONS ====================
const int M1_IN1 = 3;
const int M1_IN2 = 5;

const int M2_IN1 = 6;
const int M2_IN2 = 9;

const int M3_IN1 = 10;
const int M3_IN2 = 11;

// ==================== INITIALIZATION ====================
void MotorControl_Init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  // Debugging info
  Serial.begin(9600);
  Serial.println("Motor control initialized");
}

// ==================== MOTOR 1 FUNCTIONS ====================
void M1_Forward(float speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(M1_IN1, speed);
  analogWrite(M1_IN2, 0);
  Serial.print("M1 Forward: ");
  Serial.println(speed);
}

void M1_Backward(float speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, speed);
  Serial.print("M1 Backward: ");
  Serial.println(speed);
}

void M1_Stop() {
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  Serial.println("M1 Stopped");
}

// ==================== MOTOR 2 FUNCTIONS ====================
void M2_Forward(float speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(M2_IN1, speed);
  analogWrite(M2_IN2, 0);
  Serial.print("M2 Forward: ");
  Serial.println(speed);
}

void M2_Backward(float speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, speed);
  Serial.print("M2 Backward: ");
  Serial.println(speed);
}

void M2_Stop() {
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
  Serial.println("M2 Stopped");
}

// ==================== MOTOR 3 FUNCTIONS ====================
void M3_Forward(float speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(M3_IN1, speed);
  analogWrite(M3_IN2, 0);
  Serial.print("M3 Forward: ");
  Serial.println(speed);
}

void M3_Backward(float speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(M3_IN1, 0);
  analogWrite(M3_IN2, speed);
  Serial.print("M3 Backward: ");
  Serial.println(speed);
}

void M3_Stop() {
  analogWrite(M3_IN1, 0);
  analogWrite(M3_IN2, 0);
  Serial.println("M3 Stopped");
}
