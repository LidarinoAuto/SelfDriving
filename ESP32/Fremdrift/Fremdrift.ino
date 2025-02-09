/*******************************************************
 *  HOVEDFIL: Fremdrift_3.ino
 *******************************************************/
#include <Arduino.h>
#include "PinConfig.h"
#include "MotorControl.h"
#include "PID.h"
#include "IRControl.h"
#include "Kiwidrive.h"

// === Globale variabler for IR-styrt bevegelse ===
int speedX = 0;
int speedY = 0;
int rotation = 0;

// === PID-relaterte arrayer (ett sett for hvert hjul) ===
float pid_kp[3] = {1.8, 1.8, 1.8};
float pid_ki[3] = {1.0, 1.0, 1.0};
float pid_kd[3] = {0.0000001, 0.0000001, 0.0000001};

float pid_integral[3]   = {0, 0, 0};
float pid_lastError[3]  = {0, 0, 0};

// === Kiwi setpoints (beregnes i setKiwiDrive) ===
float setpointSpeed[3] = {0, 0, 0}; // mm/s

// === Encoderteller for hvert hjul ===
volatile long encoderTicks[3] = {0, 0, 0};

// === Konstanter for konvertering av encoder-ticks ===
//  (tilpass til dine motorer/encodere/hjul)
static const int   ENCODER_COUNTS_PER_REV = 720; // encoder "pulses" per full omdreining
static const float GEAR_RATIO             = 75.8; // f.eks. 0.5 hvis du har 2:1 utveksling
static const float WHEEL_DIAMETER_MM      = 58.0;
static const float WHEEL_CIRCUM_MM        = WHEEL_DIAMETER_MM * PI;

// === Tidsstyring av PID-løkka ===
static const unsigned long CONTROL_INTERVAL = 100; // ms

// ==========================================================
//  INTERRUPT-FUNKSJONER for hver av de 3 encoderne
// ==========================================================
void IRAM_ATTR encoderInterrupt1() {
  bool A = digitalRead(ENCODER_A1_PIN);
  bool B = digitalRead(ENCODER_B1_PIN);
  encoderTicks[0] += (A == B) ? 1 : -1;
}

void IRAM_ATTR encoderInterrupt2() {
  bool A = digitalRead(ENCODER_A2_PIN);
  bool B = digitalRead(ENCODER_B2_PIN);
  encoderTicks[1] += (A == B) ? 1 : -1;
}

void IRAM_ATTR encoderInterrupt3() {
  bool A = digitalRead(ENCODER_A3_PIN);
  bool B = digitalRead(ENCODER_B3_PIN);
  encoderTicks[2] += (A == B) ? 1 : -1;
}

// ----------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Motorpinner
  motorSetup();

  // Kiwi init (hvis du trenger noe her)
  kiwiSetup();

  // IR init
  IRSetup();

  // Encoderpinner + interrupts
  pinMode(ENCODER_A1_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1_PIN), encoderInterrupt1, CHANGE);

  pinMode(ENCODER_A2_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2_PIN), encoderInterrupt2, CHANGE);

  pinMode(ENCODER_A3_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B3_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A3_PIN), encoderInterrupt3, CHANGE);

  Serial.println("Setup complete!");
}

// ----------------------------------------------------------
void loop() {
  // 1) Les IR for å bestemme speedX, speedY, rotation
  handleIRInput();

  // 2) Kjør PID hver CONTROL_INTERVAL ms
  static unsigned long lastControlTime = 0;
  unsigned long now = millis();
  unsigned long deltaTime = now - lastControlTime;

  if (deltaTime >= CONTROL_INTERVAL) {
    // a) Bruk KiwiDrive for å finne setpoint-hastighet (mm/s) for hvert hjul
    setKiwiDrive(speedX, speedY, rotation);

    // b) For hver av de 3 hjulene: Mål hastighet, beregn PID, kjør motor
    float dt_seconds = (float)deltaTime / 1000.0;
    for (byte i = 0; i < 3; i++) {
      // ---- Beskyttet avlesning av encoderTicks ----
      noInterrupts();
      long count = encoderTicks[i];
      encoderTicks[i] = 0; // nullstill for neste periode
      interrupts();

      // ---- Beregn faktisk hastighet i mm/s ----
      //  'count' = antall "pulses" denne perioden
      float revs = (float)count / ENCODER_COUNTS_PER_REV; // antall omdreininger
      // Ta hensyn til giring (om encoder sitter på motoraksling).
      float revsWheel = revs / GEAR_RATIO; 
      // RPM = revsWheel / dt (sek) * 60 
      float rpm = (revsWheel / dt_seconds) * 60.0;
      // mm/s = (RPM * hjulomkrets) / 60
      float speed_mms = (rpm * WHEEL_CIRCUM_MM) / 60.0;

      // ---- Kjør PID-regning (med anti-windup) ----
      float pwm = computePID_withAntiWindup(i, setpointSpeed[i], speed_mms, dt_seconds);

      // ---- Kjør motoren ----
      switch (i) {
        case 0:
          setMotorSpeed(MOTOR_1_FORWARD_PIN, MOTOR_1_BACKWARD_PIN, (int)pwm);
          break;
        case 1:
          setMotorSpeed(MOTOR_2_FORWARD_PIN, MOTOR_2_BACKWARD_PIN, (int)pwm);
          break;
        case 2:
          setMotorSpeed(MOTOR_3_FORWARD_PIN, MOTOR_3_BACKWARD_PIN, (int)pwm);
          break;
      }

      // ---- (Valgfritt) litt debug for motor 0: ----
      if (i == 0) {
        Serial.print("[M1] SP=");  Serial.print(setpointSpeed[0], 1);
        Serial.print("  Act=");    Serial.print(speed_mms, 1);
        Serial.print("  Err=");    Serial.print(setpointSpeed[0] - speed_mms, 1);
        Serial.print("  PWM=");    Serial.println(pwm, 1);
      }
    }

    lastControlTime = now;
  }
}
