/*******************************************************
    HOVEDFIL: Fremdrift_3.ino (Oppdatert for mm/s)
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
// 720 teller per omdreining (A+B i kvadratur gir ofte 4x 720, pass på!)
static const int   ENCODER_COUNTS_PER_REV = 720;
// Hvis enkoderen sitter på motorakselen med gir 75.8:1,
// må du dele antall motoromdreininger på 75.8 for å få hjulomdreininger.
static const float GEAR_RATIO             = 75.8;
static const float WHEEL_DIAMETER_MM      = 58.0;
static const float WHEEL_CIRCUM_MM        = WHEEL_DIAMETER_MM * PI;

// === Tidsstyring ===
static const unsigned long CONTROL_INTERVAL = 100; // ms, PID-loop
static const unsigned long STOP_PWM_DELAY   = 500; // ms idle før PWM kuttes
unsigned long lastControlTime = 0;
unsigned long lastMotionTime  = 0;
bool pwmDisabled = false;

// === Interrupt-rutiner ===
void IRAM_ATTR encoderInterrupt1() {
  bool A = digitalRead(ENCODER_A1_PIN);
  bool B = digitalRead(ENCODER_B1_PIN);
  // Opp/ned avhengig av A vs B:
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

  // (Valgfritt) Skru på debug av PID:
  debugPIDEnabled = true;   // Skriver ut PID-detaljer
  debugStatusEnabled = false;  // Ekstra status om speedX, speedY, rotation
  
  Serial.println("Setup complete!");
}

void loop() {
  handleIRInput();
  readSerialCommands();

  // Eventuell debugging av overordnede kommandoer
  debugPrintStatus(speedX, speedY, rotation, pwmDisabled);

  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    // Oppdater Kiwi-setpoints
    setKiwiDrive(speedX, speedY, rotation);

    // Hvis speedX=0, speedY=0 og rotation=0 → idle
    if (speedX == 0 && speedY == 0 && rotation == 0) {
      if (lastMotionTime == 0) {
        lastMotionTime = now;
      }
      // Hvis vi har stått i ro lenge nok → kutt PWM
      if (!pwmDisabled && now - lastMotionTime >= STOP_PWM_DELAY) {
        for (int i = 0; i < 3; i++) {
          int fwdPin = (i==0?MOTOR_1_FORWARD_PIN : (i==1?MOTOR_2_FORWARD_PIN : MOTOR_3_FORWARD_PIN));
          int bwdPin = (i==0?MOTOR_1_BACKWARD_PIN: (i==1?MOTOR_2_BACKWARD_PIN: MOTOR_3_BACKWARD_PIN));
          setMotorSpeed(fwdPin, bwdPin, 0);
        }
        pwmDisabled = true;
      }
    } else {
      // Ny bevegelseskommando → kjør PID
      lastMotionTime = 0;
      pwmDisabled = false;
      runPIDLoop(now - lastControlTime);
    }

    lastControlTime = now;
  }
}

/**
 * Kjører PID-regulering for alle 3 hjul.
 *  - Deler ut tid deltaMs
 *  - Leser av encoderTicks[i] og konverterer til mm/s
 *  - Kaller computePID_withAntiWindup()
 *  - Oppdaterer motorene med PWM
 */
void runPIDLoop(unsigned long deltaMs) {
  float dt = deltaMs / 1000.0;
  float actualSpeed[3];
  float pwmVals[3];

  for (byte i = 0; i < 3; i++) {
    // Les og nullstill pulstellere
    noInterrupts();
    long ticks = encoderTicks[i];
    encoderTicks[i] = 0;
    interrupts();

    // 1) Finn antall hjul-omdreininger i løpet av dt
    //    OBS: ENCODER_COUNTS_PER_REV er ofte 4x "CPR" om du bruker A/B begge kanaler.
    //    Deretter deler vi på girforhold for å få hjulomdreininger.
    float wheelRevs = (ticks / (float)ENCODER_COUNTS_PER_REV) / GEAR_RATIO;

    // 2) Konverter omdreininger -> distanse (mm)
    float dist_mm = wheelRevs * WHEEL_CIRCUM_MM;

    // 3) mm/s = dist_mm / dt
    //    Fjern "/ 60.0" for å faktisk få mm/s (ikke mm/min).
    float speed_mms = dist_mm / dt;
    actualSpeed[i] = speed_mms;

    // 4) Beregn PID-output
    float pwm = computePID_withAntiWindup(i, setpointSpeed[i], speed_mms, dt);
    pwmVals[i] = pwm;

    // 5) Skriv PWM til motor
    int fwdPin = (i==0 ? MOTOR_1_FORWARD_PIN  : (i==1 ? MOTOR_2_FORWARD_PIN  : MOTOR_3_FORWARD_PIN));
    int bwdPin = (i==0 ? MOTOR_1_BACKWARD_PIN : (i==1 ? MOTOR_2_BACKWARD_PIN : MOTOR_3_BACKWARD_PIN));
    setMotorSpeed(fwdPin, bwdPin, (int)pwm);
  }

  // Valgfri debugging av PID
  debugPrintPID(setpointSpeed, actualSpeed, pwmVals, dt);
}
