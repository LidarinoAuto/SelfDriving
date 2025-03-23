/*******************************************************
    HOVEDFIL: Fremdrift_3.ino
 *******************************************************/
#include <Arduino.h>
#include "PinConfig.h"
#include "MotorControl.h"
#include "PID.h"
#include "IRControl.h"
#include "Kiwidrive.h"
#include "SerielControl.h"
#include "Debugging.h"


// === Globale variabler for bevegelse ===
int speedX = 0;
int speedY = 0;
int rotation = 0;

// === PID-arrayer ===
float pid_kp[3] = {1.7, 1.7, 1.7};
float pid_ki[3] = {2.5, 2.5, 2.5};
float pid_kd[3] = {0.00001, 0.00001, 0.00001};
float pid_integral[3] = {0};
float pid_lastError[3] = {0};

// === Kiwi setpoints ===
float setpointSpeed[3] = {0};

// === Encoder-teller ===
volatile long encoderTicks[3] = {0};

// === Encoder-konfig ===
static const int   ENCODER_COUNTS_PER_REV = 720;
static const float GEAR_RATIO             = 75.8;
static const float WHEEL_DIAMETER_MM      = 58.0;
static const float WHEEL_CIRCUM_MM        = WHEEL_DIAMETER_MM * PI;

// === Tidsstyring ===
static const unsigned long CONTROL_INTERVAL = 100; // PID-loop
static const unsigned long STOP_PWM_DELAY   = 500; // ms idle før PWM kuttes
unsigned long lastControlTime = 0;
unsigned long lastMotionTime  = 0;
bool pwmDisabled = false;

// === Interrupt-rutiner ===
void IRAM_ATTR encoderInterrupt1() {
  bool A = digitalRead(ENCODER_A1_PIN), B = digitalRead(ENCODER_B1_PIN);
  encoderTicks[0] += (A == B) ? 1 : -1;
}
void IRAM_ATTR encoderInterrupt2() {
  bool A = digitalRead(ENCODER_A2_PIN), B = digitalRead(ENCODER_B2_PIN);
  encoderTicks[1] += (A == B) ? 1 : -1;
}
void IRAM_ATTR encoderInterrupt3() {
  bool A = digitalRead(ENCODER_A3_PIN), B = digitalRead(ENCODER_B3_PIN);
  encoderTicks[2] += (A == B) ? 1 : -1;
}

void setup() {
  Serial.begin(115200);
  motorSetup();
  kiwiSetup();
  IRSetup();

  // Encoder interrupts
  pinMode(ENCODER_A1_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A2_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A3_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1_PIN), encoderInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2_PIN), encoderInterrupt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A3_PIN), encoderInterrupt3, CHANGE);

  Serial.println("Setup complete!");
}

void loop() {
  handleIRInput();
  readSerialCommands();
  // Etter handleIRInput() og readSerialCommands()
  debugPrintStatus(speedX, speedY, rotation, pwmDisabled);

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    // Kalkuler ønsket hjulhastighet
    setKiwiDrive(speedX, speedY, rotation);

    // Hvis alle bevegelser er null → vurder idle-timer
    if (speedX == 0 && speedY == 0 && rotation == 0) {
      if (lastMotionTime == 0) lastMotionTime = now;
      if (!pwmDisabled && now - lastMotionTime >= STOP_PWM_DELAY) {
        // Kutt PWM
        for (int i=0; i<3; i++) {
          setMotorSpeed((i==0?MOTOR_1_FORWARD_PIN:(i==1?MOTOR_2_FORWARD_PIN:MOTOR_3_FORWARD_PIN)),
                        (i==0?MOTOR_1_BACKWARD_PIN:(i==1?MOTOR_2_BACKWARD_PIN:MOTOR_3_BACKWARD_PIN)), 0);
        }
        pwmDisabled = true;
      }
    } else {
      // Ny kommando → reset og kjør PID
      lastMotionTime = 0;
      pwmDisabled = false;
      runPIDLoop(now - lastControlTime);
    }
    lastControlTime = now;
  }
}

void runPIDLoop(unsigned long deltaMs) {
  float dt = deltaMs / 1000.0;
  float actualSpeed[3];
  float pwmVals[3];

  for (byte i = 0; i < 3; i++) {
    noInterrupts();
    long ticks = encoderTicks[i];
    encoderTicks[i] = 0;
    interrupts();

    float revsWheel = (ticks / (float)ENCODER_COUNTS_PER_REV) / GEAR_RATIO;
    float speed_mms = (revsWheel / dt) * (WHEEL_CIRCUM_MM / 60.0);
    actualSpeed[i] = speed_mms;

    float pwm = computePID_withAntiWindup(i, setpointSpeed[i], speed_mms, dt);
    pwmVals[i] = pwm;

    int fwdPin = (i==0?MOTOR_1_FORWARD_PIN:(i==1?MOTOR_2_FORWARD_PIN:MOTOR_3_FORWARD_PIN));
    int bwdPin = (i==0?MOTOR_1_BACKWARD_PIN:(i==1?MOTOR_2_BACKWARD_PIN:MOTOR_3_BACKWARD_PIN));
    setMotorSpeed(fwdPin, bwdPin, (int)pwm);
  }

  // Debug PID for alle hjul
  debugPrintPID(setpointSpeed, actualSpeed, pwmVals, dt);
    
  }

