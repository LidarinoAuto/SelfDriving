// ==================== PIN DEFINITIONS ====================
#define ENCODER_A_PIN 5
#define ENCODER_B_PIN 4
#define MOTOR_FORWARD_PIN 13
#define MOTOR_BACKWARD_PIN 12
#define MOTOR_ENABLE_PIN 14
#define IR_RECEIVE_PIN 15  // IR mottaker

// ==================== INCLUDES ====================
#include "MotorControl.h"
#include "PID.h"
#include "IRControl.h" // Ny fane for IR-styring

// ==================== VARIABLES ====================
volatile int encoder_ticks = 0;
unsigned long last_time = 0;

const int ENCODER_RESOLUTION = 360;
const float WHEEL_DIAMETER_MM = 68.0;
const float GEAR_RATIO = 0.5;
const int CONTROL_INTERVAL = 100;

const float INTEGRAL_LIMIT = 200.0;
const int DEAD_ZONE = 1;
float target_speed_mms = 350.0;
float kp = 3.1, ki = 0.5, kd = 1.4;

float last_error = 0.0, integral = 0.0, derivative = 0.0;
int control_signal = 0;

// ==================== INTERRUPT HANDLER ====================
void IRAM_ATTR encoderInterrupt() {
  int encoder_A = digitalRead(ENCODER_A_PIN);
  int encoder_B = digitalRead(ENCODER_B_PIN);
  encoder_ticks += (encoder_A == encoder_B) ? 1 : -1;
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  motorSetup();
  IRSetup(); // Initialiser IR-mottakeren

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderInterrupt, CHANGE);
}

// ==================== LOOP ====================
void loop() {
  static unsigned long last_control_time = 0;

  // Håndter IR-innganger
  handleIRInput();

  if (millis() - last_control_time >= CONTROL_INTERVAL) {
    last_control_time = millis();

    float actual_speed_mms = ((encoder_ticks / (float)ENCODER_RESOLUTION) * 
                              (3.14159 * WHEEL_DIAMETER_MM) * GEAR_RATIO) / 
                              (CONTROL_INTERVAL / 1000.0);
    float rpm = (encoder_ticks / (float)ENCODER_RESOLUTION) * (60.0 / (CONTROL_INTERVAL / 1000.0));

    encoder_ticks = 0;

    control_signal = computePID(target_speed_mms, actual_speed_mms, CONTROL_INTERVAL);
    control_signal = constrain(control_signal, -255, 255);
    setMotorSpeed(control_signal);

    if (abs(control_signal) >= 10) {
      Serial.print("Setpoint:"); Serial.print(target_speed_mms);
      Serial.print("\tActualSpeed:"); Serial.print(actual_speed_mms);
      Serial.print("\tRPM:"); Serial.print(rpm);
      Serial.print("\tError:"); Serial.print(target_speed_mms - actual_speed_mms);
      Serial.print("\tP:"); Serial.print(kp * abs(target_speed_mms - actual_speed_mms));
      Serial.print("\tI:"); Serial.print(ki * integral);
      Serial.print("\tD:"); Serial.print(kd * derivative);
      Serial.print("\tPWM:"); Serial.println(control_signal);
    }
  }
}
