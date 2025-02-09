#include <IRremote.h>

// Define the pin for the IR receiver
const int IR_RECEIVE_PIN = 10; // Adjust to match your setup

// ==================== PIN DEFINITIONS ====================
const int M1_IN1 = 5;
const int M1_IN2 = 4;
const int M1_EN = 14; // Must be PWM-capable pin for speed control

const int M2_IN1 = 16;
const int M2_IN2 = 0;
const int M2_EN = 12; // Must be PWM-capable pin for speed control

const int M3_IN1 = 15;
const int M3_IN2 = 2;
const int M3_EN = 13; // Must be PWM-capable pin for speed control

// ==================== INITIALIZATION ====================
void MotorControl_Init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_EN, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_EN, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_EN, OUTPUT);

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

// Hjulene står plassert med 120° mellom seg.
// Oppsettet avhenger av hvordan du faktisk har montert hjulene.
// La oss si hjul 1 står i vinkel 0°, hjul 2 = 120°, hjul 3 = 240°:
const float WHEEL_ANGLE_1 = 90.0;
const float WHEEL_ANGLE_2 = 210.0;
const float WHEEL_ANGLE_3 = 330.0;

/**************************************************************
 * KiwiDrive()
 *   vx, vy: Desired velocity components (e.g. -1.0..+1.0)
 *   maxPWM: Maximum PWM value (0..255)
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
}

void setup() {
  Serial.begin(115200);
  Serial.println("IR Receiver is starting...");

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  MotorControl_Init(); 
}

void loop() {
  if (IrReceiver.decode()) {
    Serial.print("Received IR signal: ");
    Serial.println(IrReceiver.decodedIRData.command, HEX); 

    switch (IrReceiver.decodedIRData.command) {
      case 0x18: 
        Serial.println("Framover");
        KiwiDrive(0.0, 1.0, 255); // Fremover
        break;
      case 0x52:
        Serial.println("Bakover");
        KiwiDrive(0.0, -1.0, 255); // Bakover
        break;
      case 0x8:
        Serial.println("Venstre");
        KiwiDrive(-1.0, 0.0, 255); // Venstre
        break;
      case 0x5A:
        Serial.println("Høyre");
        KiwiDrive(1.0, 0.0, 255); // Høyre
        break;
      case 0x1C:
        Serial.println("Stopp");
        KiwiDrive(0.0, 0.0, 0); // Stopp
        break;
      default:
        Serial.println("Unknown command");
        break;
    }

    IrReceiver.resume(); 
  }
}