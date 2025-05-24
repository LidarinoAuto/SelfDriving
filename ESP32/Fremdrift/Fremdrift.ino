/*******************************************************
    HOVEDFIL: Fremdrift_3.ino (Kombinasjon av A-kode og B-kode)
    - Måler hastighet med "A-kodens" rpm-logikk
    - Stopper PWM etter 500 ms inaktivitet (B-kodens idle-stopp)
 *******************************************************/
#include <Arduino.h>
#include "PinConfig.h"
#include "MotorControl.h"
#include "PID.h"
#include "IRControl.h"
#include "Kiwidrive.h"
#include "SerielControl.h"

// === Globale variabler for IR-styrt bevegelse ===
int speedX = 0;
int speedY = 0;
int rotation = 0;

// === PID-relaterte arrayer (ett sett for hvert hjul) ===
float pid_kp[3] = {1.32, 1.32, 1.32};
float pid_ki[3] = {3.5, 3.5, 3.5};
float pid_kd[3] = {0.028, 0.028, 0.028};

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

// === Idle-stopp fra B-kode ===
static const unsigned long STOP_PWM_DELAY   = 100; // ms idle før PWM kuttes
unsigned long lastMotionTime  = 0;
bool pwmDisabled = false;

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
    while (!Serial);
  // Kun de fem kolonnene vi skal plotte:
  //Serial.println("SP0,Err0,P0,I0,D0");

  // ? resten av setup() uendret ?

  // Motorpinner
  motorSetup();


  // Kiwi init
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
  readSerialCommands();

  // 2) Kjør PID (eller stopp) hver CONTROL_INTERVAL ms
  static unsigned long lastControlTime = 0;
  unsigned long now = millis();
  unsigned long deltaTime = now - lastControlTime;

  if (deltaTime >= CONTROL_INTERVAL) {
    // a) Oppdater Kiwi-setpoints
    setKiwiDrive(speedX, speedY, rotation);

    // =============== IDLE-STOPP-LOGIKK (fra B-koden) ===============
    if (speedX == 0 && speedY == 0 && rotation == 0) {
      // Robot "inaktiv" (ingen bevegelseskommando)
      if (lastMotionTime == 0) {
        lastMotionTime = now;  // start teller
      }
      // Hvis vi ikke allerede har deaktivert PWM,
      // og det har gått STOP_PWM_DELAY ms, stopp motorene helt
      if (!pwmDisabled && (now - lastMotionTime >= STOP_PWM_DELAY)) {
        // Kutt PWM
        for (int i = 0; i < 3; i++) {
          setMotorSpeed(
            (i == 0 ? MOTOR_1_FORWARD_PIN : (i == 1 ? MOTOR_2_FORWARD_PIN : MOTOR_3_FORWARD_PIN)),
            (i == 0 ? MOTOR_1_BACKWARD_PIN : (i == 1 ? MOTOR_2_BACKWARD_PIN : MOTOR_3_BACKWARD_PIN)),
            0
          );
        }
        pwmDisabled = true;
        Serial.println("[INFO] PWM disabled after idle");
      }
    } else {
      // Vi har en ny bevegelseskommando
      lastMotionTime = 0;  // nullstill idle-teller
      pwmDisabled = false; // re-aktiver PWM
      // ============== KJØR PID-LØKKA (fra A-koden) ==============
      float dt_seconds = deltaTime / 1000.0;
      for (byte i = 0; i < 3; i++) {
        noInterrupts();
        long count = encoderTicks[i];
        encoderTicks[i] = 0;
        interrupts();

        // Beregn antall hjul-omdreininger
        float revs = (float)count / ENCODER_COUNTS_PER_REV;
        float revsWheel = revs / GEAR_RATIO;

        // RPM = (revsWheel / dt) * 60
        float rpm = (revsWheel / dt_seconds) * 60.0;

        // speed_mms = (rpm * WHEEL_CIRCUM_MM) / 60 => mm/s
        float speed_mms = (rpm * WHEEL_CIRCUM_MM) / 60.0;

        // Kall PID
        float pwm = computePID_withAntiWindup(i, setpointSpeed[i], speed_mms, dt_seconds);

        // Ta vare p� gammel feil for D-term:
        long oldErr0 = pid_lastError[0];
        
        // Kjør motor
        int fwdPin = (i == 0 ? MOTOR_1_FORWARD_PIN
                     : (i == 1 ? MOTOR_2_FORWARD_PIN : MOTOR_3_FORWARD_PIN));
        int bwdPin = (i == 0 ? MOTOR_1_BACKWARD_PIN
                     : (i == 1 ? MOTOR_2_BACKWARD_PIN : MOTOR_3_BACKWARD_PIN));
        setMotorSpeed(fwdPin, bwdPin, (int)pwm);

        /* (Valgfritt) enkel debug for hjul 0
        if (i == 0) {
          Serial.print("[M1] SP=");  Serial.print(setpointSpeed[0], 1);
          Serial.print("  Act=");    Serial.print(speed_mms, 1);
          Serial.print("  Err=");    Serial.print(setpointSpeed[0] - speed_mms, 1);
          Serial.print("  PWM=");    Serial.println(pwm, 1);

          */
        // Inne i l�kka der du itererer gjennom hjulene (i = 0, 1, 2)
        // Kun for hjul 0: CSV-utskrift for Serial Plotter
        if (i == 0) {
          float err0 = setpointSpeed[0] - speed_mms;
          float p0   = pid_kp[0] * err0;
          float i0   = pid_ki[0] * pid_integral[0];
          float d0   = pid_kd[0] * (err0 - pid_lastError[0]) / dt_seconds;
        
          // �n linje med space-separerte label:value-par
          Serial.print("SP0:"); Serial.print(setpointSpeed[0], 1); Serial.print(" ");
          Serial.print("Err0:");Serial.print(err0,            1); Serial.print(" ");
          Serial.print("P0:");  Serial.print(p0,              1); Serial.print(" ");
          Serial.print("I0:");  Serial.print(i0,              1); Serial.print(" ");
          Serial.println("D0:");Serial.print(d0,              1);
        }

      }
    }

    lastControlTime = now;
  }
}
