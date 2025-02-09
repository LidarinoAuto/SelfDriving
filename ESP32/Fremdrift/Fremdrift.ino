/*******************************************************
 *  TAB 1: 3MotorPIDloopKiwiIRTest.ino (Main Code)
 *******************************************************/
#include <IRremote.h>

// Include the KiwiDrive and MotorControl "tabs" (files):
// (Not strictly necessary to `#include` .ino files if they're in the same sketch folder,
// but this can help IntelliSense in some editors.)
#include "KiwiDrive.ino"
#include "MotorControl.ino"

// ------------------ IR and Timeout ------------------
const int IR_RECEIVE_PIN = 13;
//IRrecv IrReceiver(IR_RECEIVE_PIN);
decode_results results;

const unsigned long IR_TIMEOUT = 280; // ms
unsigned long lastIRReceiveTime = 0;

// ------------------ Encoder Pins --------------------
#define EncoderA1 18
#define EncoderB1 17
#define EncoderA2 22
#define EncoderB2 23
#define EncoderA3 21
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

double kp = 3.0, ki = 2.5, kd = 0.005;
double integral1 = 0, integral2 = 0, integral3 = 0;
double previousError1 = 0, previousError2 = 0, previousError3 = 0;

// Forward declarations for our local functions
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

  // IR setup
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

  Serial.println("Setup complete!");
}

void loop() {
  // 1) Check IR commands
  IR_Check();

  // 2) Run PID every ~100ms
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

  delay(20); // small delay
}

/*******************************************************
 * IR_Check()
 * - Handles reading IR commands
 * - Updates (vx, vy, omega)
 * - Calls KiwiDrive() to update the motor setpoints
 *******************************************************/
void IR_Check() {
  static float vx = 0.0;
  static float vy = 0.0;
  static float omega = 0.0;

  // Check for new IR data
  if (IrReceiver.decode()) {
    lastIRReceiveTime = millis();
    unsigned long cmd = IrReceiver.decodedIRData.command;

    Serial.print("Received IR: 0x");
    Serial.println(cmd, HEX);

    // Adjust the command cases to your remote's codes:
    switch (cmd) {
      case 0x18: // Forward
        vy = 1.0;  vx = 0.0; omega = 0.0;
        Serial.println("Framover");
        break;
      case 0x52: // Backward
        vy = -1.0; vx = 0.0; omega = 0.0;
        Serial.println("Bakover");
        break;
      case 0x8:  // Left
        vx = -1.0; vy = 0.0; omega = 0.0;
        Serial.println("Venstre");
        break;
      case 0x5A: // Right
        vx = 1.0;  vy = 0.0; omega = 0.0;
        Serial.println("Høyre");
        break;
      case 0x1C: // Stop
        vx = 0.0; vy = 0.0; omega = 0.0;
        Serial.println("Stopp");
        break;
      case 0xD:  // Rotate Right
        vx = 0.0; vy = 0.0; omega = 1.0;
        Serial.println("Rotate right");
        break;
      case 0x16: // Rotate Left
        vx = 0.0; vy = 0.0; omega = -1.0;
        Serial.println("Rotate left");
        break;
      default:
        Serial.println("Unknown command");
        break;
    }

    // Prepare for next IR reception
    IrReceiver.resume();
  }

  // Check timeout -> stop if no IR for too long
  if (millis() - lastIRReceiveTime > IR_TIMEOUT) {
    vx = 0.0; vy = 0.0; omega = 0.0;
  }

  // Update the desired wheel speeds via KiwiDrive
  KiwiDrive(vx, vy, omega);
}

/*******************************************************
 * updatePID()
 * - Reads encoder difference
 * - Computes actual speed in mm/s
 * - Runs PID
 * - Sends PWM to the motor driver
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

  // If saturated, "unwind" integral a bit
  if (pidOutput == 255 || pidOutput == -255) {
    integral -= error * (deltaTime / 1000.0);
  }

  // Drive the motor
  motorControl(pidOutput);

  // Prepare for next iteration
  previousError = error;

  // Optional debugging:
  Serial.print("Motor "); Serial.print(motorID);
  Serial.print(" | Setpoint="); Serial.print(setpoint);
  Serial.print(" | Actual=");   Serial.print(actualSpeed);
  Serial.print(" | Error=");    Serial.print(error);
  Serial.print(" | PIDOut=");   Serial.println(pidOutput);
}
