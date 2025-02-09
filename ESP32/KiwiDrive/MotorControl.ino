// ==================== PIN DEFINITIONS ====================
const int M1_IN1 = 5;
const int M1_IN2 = 4;
const int M1_EN  = 14; // Must be PWM-capable pin for speed control

const int M2_IN1 = 16;
const int M2_IN2 = 0;
const int M2_EN  = 12; // Must be PWM-capable pin for speed control

const int M3_IN1 = 15;
const int M3_IN2 = 2;
const int M3_EN  = 13; // Must be PWM-capable pin for speed control

// ==================== INITIALIZATION ====================
void MotorControl_Init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_EN,  OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_EN,  OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_EN,  OUTPUT);

  // If you have special setup for your driver (like setting up brake pins),
  // do it here.
}

// ==================== MOTOR 1 FUNCTIONS ====================
void M1_Forward() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
}

void M1_Backward() {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
}

void M1_Stop() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_EN, 0);
}

void M1_SetSpeed(int speedVal) {
  speedVal = constrain(speedVal, 0, 255);
  analogWrite(M1_EN, speedVal);
}

// ==================== MOTOR 2 FUNCTIONS ====================
void M2_Forward() {
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
}

void M2_Backward() {
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
}

void M2_Stop() {
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  analogWrite(M2_EN, 0);
}

void M2_SetSpeed(int speedVal) {
  speedVal = constrain(speedVal, 0, 255);
  analogWrite(M2_EN, speedVal);
}

// ==================== MOTOR 3 FUNCTIONS ====================
void M3_Forward() {
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, HIGH);
}

void M3_Backward() {
  digitalWrite(M3_IN1, HIGH);
  digitalWrite(M3_IN2, LOW);
}

void M3_Stop() {
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, LOW);
  analogWrite(M3_EN, 0);
}

void M3_SetSpeed(int speedVal) {
  speedVal = constrain(speedVal, 0, 255);
  analogWrite(M3_EN, speedVal);
}
