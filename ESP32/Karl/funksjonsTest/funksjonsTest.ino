/*******************************************************
*  3MotorPIDloopKiwiSR_withIR.ino
*******************************************************/
#include <Arduino.h>
#include <IRremote.h>  // IRremote-biblioteket

// Inkluder "tabs" (filer) i samme mappe:
#include "KiwiDrive.ino"
#include "MotorControl.ino"

// ------------------ IR-konfigurasjon ------------------
// *Økt* IR_TIMEOUT for å unngå at hastigheter nullstilles mens vi tester:
const unsigned long IR_TIMEOUT = 9999999; // midlertidig svært stor verdi
const int IR_RECEIVE_PIN = 13;
decode_results results;
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

// ------------------ Funksjonsprototyper -------------
void readSerialCommands();
void IR_Check();  // IR-funksjon
void updatePID(int motorID,
               volatile long &encoderCount,
               long &lastCount,
               volatile float &actualSpeed,
               float &setpoint,
               double &integral,
               double &previousError,
               void (*motorControl)(float),
               unsigned long deltaTime);

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
  IrReceiver.enableIRIn(); // Aktiver mottak
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
}

// -----------------------------------------------------
void loop() {
  // 1) Les kommandoer fra Serial (hvis du ønsker)
  readSerialCommands();

  // 2) Sjekk IR-kommandoer (hvis du ønsker)
  IR_Check();

  // 3) Fram/bak-løkke (veksler hvert 1000 ms = 1 sekund)
  static unsigned long lastChange = 0;
  static bool forward = true;

  if (millis() - lastChange > 1000) {
    lastChange = millis();
    forward = !forward;

    if (forward) {
      // Kjør framover
      vx_mm = 0.0;
      vy_mm = 250.0;   // endre verdi om du vil ha annen fart
      omega_rad = 0.0;
      Serial.println("** Framover i 1 sekund **");
    } else {
      // Kjør bakover
      vx_mm = 0.0;
      vy_mm = -250.0;  // endre verdi om du vil ha annen fart
      omega_rad = 0.0;
      Serial.println("** Bakover i 1 sekund **");
    }
  }

  // 4) Oppdater KiwiDrive-settpunkter
  KiwiDrive(vx_mm, vy_mm, omega_rad);

  // 5) Kjør PID hver ~100ms
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
* readSerialCommands()
* - Les en linje fra Serial (f.eks. "100 150 0.5")
* - Oppdaterer vx_mm, vy_mm, omega_rad
*******************************************************/
void readSerialCommands() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      float x, y, w;
      int count = sscanf(line.c_str(), "%f %f %f", &x, &y, &w);
      if (count == 3) {
        vx_mm = x;
        vy_mm = y;
        omega_rad = w;
        Serial.print("[DEBUG] Parsed from Serial: vx=");
        Serial.print(vx_mm);
        Serial.print(", vy=");
        Serial.print(vy_mm);
        Serial.print(", omega=");
        Serial.println(omega_rad);
      } else {
        Serial.print("[ERROR] Could not parse: ");
        Serial.println(line);
      }
    }
  }
}

/*******************************************************
* IR_Check()
* - Les IR-kommandoer og sett vx_mm, vy_mm, omega_rad.
* - NB: IR_TIMEOUT er satt svært høyt, så vi unngår 
*   å nullstille bevegelse i denne demoen.
*******************************************************/
void IR_Check() {
  if (IrReceiver.decode()) {
    lastIRReceiveTime = millis();
    unsigned long cmd = IrReceiver.decodedIRData.command;

    Serial.print("Received IR command: 0x");
    Serial.println(cmd, HEX);

    switch (cmd) {
      case 0x18: // Forward
        vy_mm = 250.0;
        vx_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("IR: Framover");
        break;
      case 0x52: // Backward
        vy_mm = -250.0;
        vx_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("IR: Bakover");
        break;
      case 0x8:  // Left
        vx_mm = -250.0;
        vy_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("IR: Venstre");
        break;
      case 0x5A: // Right
        vx_mm = 250.0;
        vy_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("IR: Høyre");
        break;
      case 0x1C: // Stop
        vx_mm = 0.0;
        vy_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("IR: Stopp");
        break;
      case 0xD:  // Rotate Right
        vx_mm = 0.0;
        vy_mm = 0.0;
        omega_rad = 2.0; 
        Serial.println("IR: Rotate right");
        break;
      case 0x16: // Rotate Left
        vx_mm = 0.0;
        vy_mm = 0.0;
        omega_rad = -2.0;
        Serial.println("IR: Rotate left");
        break;
      default:
        Serial.println("IR: Unknown command");
        break;
    }
    IrReceiver.resume(); // Klar for neste mottak
  }

  // Normalt ville vi nullstille hastigheter her hvis
  // IR_TIMEOUT passeres, men den er satt så høy at
  // det ikke skjer i praksis.
  if (millis() - lastIRReceiveTime > IR_TIMEOUT) {
    // Vanligvis ville vi nullstille, men her
    // hopper vi over / kommenterer ut:
    // vx_mm = 0.0;
    // vy_mm = 0.0;
    // omega_rad = 0.0;
  }
}

/*******************************************************
* updatePID()
*******************************************************/
void updatePID(int motorID,
               volatile long &encoderCount,
               long &lastCount,
               volatile float &actualSpeed,
               float &setpoint,
               double &integral,
               double &previousError,
               void (*motorControl)(float),
               unsigned long deltaTime)
{
  noInterrupts();
  long count = encoderCount - lastCount;
  lastCount = encoderCount;
  interrupts();

  // Calculate actual speed (mm/s)
  float rpm = (count / (float)encoderCountsPerRevolution)
              / (deltaTime / 60000.0)
              / gearRatio;
  actualSpeed = (rpm * wheelCircumference / 60.0);

  // PID
  float error = setpoint - actualSpeed;
  integral += error * (deltaTime / 1000.0);

  // Anti-windup limit
  const float integralLimit = 200.0;
  if (integral > integralLimit)  integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;

  float derivative = (error - previousError) / (deltaTime / 1000.0);
  double pidOutput = (kp * error) + (ki * integral) + (kd * derivative);

  // Constrain to [-255, 255]
  pidOutput = constrain(pidOutput, -255, 255);

  // Hvis saturert, "unwind" integral litt
  if (abs(pidOutput) >= 255) {
    integral -= error * (deltaTime / 1000.0);
  }

  // Kjør motor
  motorControl(pidOutput);

  // Debug (valgfritt)
  /*
  Serial.print("Motor "); Serial.print(motorID);
  Serial.print(" | SP="); Serial.print(setpoint);
  Serial.print(" | Act="); Serial.print(actualSpeed);
  Serial.print(" | Err="); Serial.print(error);
  Serial.print(" | PID="); Serial.println(pidOutput);
  */

  previousError = error;
}
