/*******************************************************
*  3MotorPIDloopKiwiSR_withIR_withCircle.ino
*******************************************************/
#include <Arduino.h>
#include <IRremote.h>  // Legg til IRremote-biblioteket

// Inkluder "tabs" (filer) i samme mappe:
#include "KiwiDrive.ino"
#include "MotorControl.ino"

// ------------------ IR-konfigurasjon ------------------
const int IR_RECEIVE_PIN = 13;
decode_results results;

// Timeout for å nullstille hvis ingen IR-kommando:
const unsigned long IR_TIMEOUT = 280; // ms
unsigned long lastIRReceiveTime = 0;

// ------------------ Encoder Pins --------------------
#define EncoderA1 18
#define EncoderB1 17
#define EncoderA2 16
#define EncoderB2 23
#define EncoderA3 4
#define EncoderB3 19

// ------------------ Motor / Gear Constants ----------
volatile long encoderCount1 = 0, encoderCount2 = 0, encoderCount3 = 0;
const float gearRatio = 75.8;
const int encoderCountsPerRevolution = 720;
const float wheelDiameter = 58.0; // mm
const float wheelCircumference = wheelDiameter * PI;

// ------------------ PID Global Variables ------------
float setpoint1 = 0, setpoint2 = 0, setpoint3 = 0;
float actualSpeed1 = 0, actualSpeed2 = 0, actualSpeed3 = 0;

double kp = 1.8, ki = 2.5, kd = 0.005;
double integral1 = 0, integral2 = 0, integral3 = 0;
double previousError1 = 0, previousError2 = 0, previousError3 = 0;

// Fremdriftsvariabler for KiwiDrive
float vx_mm = 0.0;     // Hastighet i X-retning (mm/s)
float vy_mm = 0.0;     // Hastighet i Y-retning (mm/s)
float omega_rad = 0.0; // Rotasjonshastighet (rad/s)

// Tidskontroll for sirkeltest
unsigned long startTime;
const unsigned long CIRCLE_RUN_TIME = 5000; // Kjør i sirkel i 5 sek

// ------------------ Funksjonsprototyper -------------
void readSerialCommands();
void IR_Check();
void updatePID(int motorID,
               volatile long &encoderCount,
               long &lastCount,
               volatile float &actualSpeed,
               float &setpoint,
               double &integral,
               double &previousError,
               void (*motorControl)(float),
               unsigned long deltaTime);
void runCircleTest(); // Ny funksjon for sirkelbevegelse

// ------------------ Encoder Interrupts --------------
void IRAM_ATTR readEncoder1() {
  bool encoderA = digitalRead(EncoderA1);
  bool encoderB = digitalRead(EncoderB1);
  encoderCount1 += (encoderA == encoderB) ? 1 : -1;
}
void IRAM_ATTR readEncoder2() {
  bool encoderA = digitalRead(EncoderA2);
  bool encoderB = digitalRead(EncoderB2);
  encoderCount2 += (encoderA == encoderB) ? 1 : -1;
}
void IRAM_ATTR readEncoder3() {
  bool encoderA = digitalRead(EncoderA3);
  bool encoderB = digitalRead(EncoderB3);
  encoderCount3 += (encoderA == encoderB) ? 1 : -1;
}

// -----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // IR-oppstart
  IrReceiver.enableIRIn();
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  lastIRReceiveTime = millis();

  // Motor & driver pins
  MotorControl_Init();

  // Encoders
  pinMode(EncoderA1, INPUT_PULLUP);
  pinMode(EncoderB1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderA1), readEncoder1, CHANGE);

  pinMode(EncoderA2, INPUT_PULLUP);
  pinMode(EncoderB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderA2), readEncoder2, CHANGE);

  pinMode(EncoderA3, INPUT_PULLUP);
  pinMode(EncoderB3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderA3), readEncoder3, CHANGE);

  Serial.println("Setup complete! (Serial + IR KiwiDrive)");

  // Start sirkeltest
  runCircleTest();
}

// -----------------------------------------------------
void loop() {
  // Kjør PID hver ~100ms
  static long lastCount1 = 0, lastCount2 = 0, lastCount3 = 0;
  static unsigned long lastTime = 0;

  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime >= 100) {
    updatePID(1, encoderCount1, lastCount1, actualSpeed1, setpoint1,
              integral1, previousError1, Motor1, deltaTime);

    updatePID(2, encoderCount2, lastCount2, actualSpeed2, setpoint2,
              integral2, previousError2, Motor2, deltaTime);

    updatePID(3, encoderCount3, lastCount3, actualSpeed3, setpoint3,
              integral3, previousError3, Motor3, deltaTime);

    lastTime = currentTime;
  }

  delay(20); // liten delay
}

/*******************************************************
* runCircleTest()
* - Setter opp roboten til å kjøre en sirkel
*******************************************************/
void runCircleTest() {
  Serial.println("Starter sirkeltest!");

  // Definer hastigheter for sirkelbevegelse
  vx_mm = 200.0;   // Fremoverbevegelse (mm/s)
  vy_mm = 0.0;     // Ingen sidelengs bevegelse
  omega_rad = 1.0; // Rotasjonshastighet (rad/s)

  startTime = millis();
  
  while (millis() - startTime < CIRCLE_RUN_TIME) {
    KiwiDrive(vx_mm, vy_mm, omega_rad);
  }

  // Stopp etter 5 sekunder
  vx_mm = 0.0;
  vy_mm = 0.0;
  omega_rad = 0.0;
  KiwiDrive(vx_mm, vy_mm, omega_rad);
  Serial.println("Sirkeltest fullført. Robot stoppet.");
}
