// ==================== PIN DEFINITIONS ====================
const int IR_RECEIVE_PIN = 15;
const int ENCODER1_A_PIN = 18;
const int ENCODER1_B_PIN = 17;
const int ENCODER2_A_PIN = 22;
const int ENCODER2_B_PIN = 23;
const int ENCODER3_A_PIN = 21;
const int ENCODER3_B_PIN = 19;
const int MOTOR1_IN1 = 14;
const int MOTOR1_IN2 = 27;
const int MOTOR2_IN1 = 25;
const int MOTOR2_IN2 = 26;
const int MOTOR3_IN1 = 33;
const int MOTOR3_IN2 = 32;

// ==================== INCLUDES ====================
#include "MotorControl.h"
#include "PID.h"
#include "IRControl.h"
#include "KiwiDrive.h"

// ==================== VARIABLES ====================
volatile long encoderCount1 = 0, encoderCount2 = 0, encoderCount3 = 0;
unsigned long last_time = 0;

const int ENCODER_RESOLUTION = 360;
const float WHEEL_DIAMETER_MM = 68.0;
float CONTROL_INTERVAL = 100.0; // Endret til float uten const

const float INTEGRAL_LIMIT = 200.0;
const int DEAD_ZONE = 1;
float setpoint1 = 0, setpoint2 = 0, setpoint3 = 0;
float kp = 3.1, ki = 0.5, kd = 1.4;

float integral1 = 0, integral2 = 0, integral3 = 0;
float last_error1 = 0, last_error2 = 0, last_error3 = 0;

float filtered_speed1 = 0, filtered_speed2 = 0, filtered_speed3 = 0;

// ==================== INTERRUPT HANDLER ====================
void IRAM_ATTR readEncoder1() {
  bool encoderA = digitalRead(ENCODER1_A_PIN);
  bool encoderB = digitalRead(ENCODER1_B_PIN);
  encoderCount1 += (encoderA == encoderB) ? 1 : -1;
}

void IRAM_ATTR readEncoder2() {
  bool encoderA = digitalRead(ENCODER2_A_PIN);
  bool encoderB = digitalRead(ENCODER2_B_PIN);
  encoderCount2 += (encoderA == encoderB) ? 1 : -1;
}

void IRAM_ATTR readEncoder3() {
  bool encoderA = digitalRead(ENCODER3_A_PIN);
  bool encoderB = digitalRead(ENCODER3_B_PIN);
  encoderCount3 += (encoderA == encoderB) ? 1 : -1;
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  motorSetup();
  IRSetup(); // Initialiser IR-mottakeren

  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), readEncoder1, CHANGE);

  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), readEncoder2, CHANGE);

  pinMode(ENCODER3_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A_PIN), readEncoder3, CHANGE);
}

// ==================== LOOP ====================
void loop() {
  static unsigned long last_control_time = 0;
  static long last_encoderCount1 = 0, last_encoderCount2 = 0, last_encoderCount3 = 0;

  // Håndter IR-innganger
  handleIRInput();

  if (millis() - last_control_time >= CONTROL_INTERVAL) {
    last_control_time = millis();

    long deltaCount1 = encoderCount1 - last_encoderCount1;
    long deltaCount2 = encoderCount2 - last_encoderCount2;
    long deltaCount3 = encoderCount3 - last_encoderCount3;
    last_encoderCount1 = encoderCount1;
    last_encoderCount2 = encoderCount2;
    last_encoderCount3 = encoderCount3;

    const float alpha = 0.5; // Juster etter behov
    filtered_speed1 = alpha * ((deltaCount1 / (float)ENCODER_RESOLUTION) * (PI * WHEEL_DIAMETER_MM)) + (1 - alpha) * filtered_speed1;
    filtered_speed2 = alpha * ((deltaCount2 / (float)ENCODER_RESOLUTION) * (PI * WHEEL_DIAMETER_MM)) + (1 - alpha) * filtered_speed2;
    filtered_speed3 = alpha * ((deltaCount3 / (float)ENCODER_RESOLUTION) * (PI * WHEEL_DIAMETER_MM)) + (1 - alpha) * filtered_speed3;

    // Riktig antall argumenter skal være 6
    float control_signal1 = computePID(setpoint1, filtered_speed1, integral1, last_error1, filtered_speed1, CONTROL_INTERVAL);
    float control_signal2 = computePID(setpoint2, filtered_speed2, integral2, last_error2, filtered_speed2, CONTROL_INTERVAL);
    float control_signal3 = computePID(setpoint3, filtered_speed3, integral3, last_error3, filtered_speed3, CONTROL_INTERVAL);

    integral1 = constrain(integral1, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    integral2 = constrain(integral2, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    integral3 = constrain(integral3, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    control_signal1 = constrain(control_signal1, -255, 255);
    control_signal2 = constrain(control_signal2, -255, 255);
    control_signal3 = constrain(control_signal3, -255, 255);

    Motor1(control_signal1);
    Motor2(control_signal2);
    Motor3(control_signal3);

    if (abs(control_signal1) >= 10) {
      Serial.print("Setpoint1:"); Serial.print(setpoint1);
      Serial.print("\tActualSpeed1:"); Serial.print(filtered_speed1);
      Serial.print("\tControl1:"); Serial.print(control_signal1);
      Serial.print("\tP1:"); Serial.print(kp * abs(setpoint1 - filtered_speed1));
      Serial.print("\tI1:"); Serial.print(ki * integral1);
      Serial.print("\tD1:"); Serial.print(kd * (filtered_speed1 - last_error1));
      Serial.print("\tPWM1:"); Serial.println(control_signal1);
    }

    if (abs(control_signal2) >= 10) {
      Serial.print("Setpoint2:"); Serial.print(setpoint2);
      Serial.print("\tActualSpeed2:"); Serial.print(filtered_speed2);
      Serial.print("\tControl2:"); Serial.print(control_signal2);
      Serial.print("\tP2:"); Serial.print(kp * abs(setpoint2 - filtered_speed2));
      Serial.print("\tI2:"); Serial.print(ki * integral2);
      Serial.print("\tD2:"); Serial.print(kd * (filtered_speed2 - last_error2));
      Serial.print("\tPWM2:"); Serial.println(control_signal2);
    }

    if (abs(control_signal3) >= 10) {
      Serial.print("Setpoint3:"); Serial.print(setpoint3);
      Serial.print("\tActualSpeed3:"); Serial.print(filtered_speed3);
      Serial.print("\tControl3:"); Serial.print(control_signal3);
      Serial.print("\tP3:"); Serial.print(kp * abs(setpoint3 - filtered_speed3));
      Serial.print("\tI3:"); Serial.print(ki * integral3);
      Serial.print("\tD3:"); Serial.print(kd * (filtered_speed3 - last_error3));
      Serial.print("\tPWM3:"); Serial.println(control_signal3);
    }
  }
}
