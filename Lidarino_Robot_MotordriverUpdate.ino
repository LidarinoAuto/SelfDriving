// Lidarino_Robot_Test.ino

#include <IRremote.h>

// Tell the compiler "KiwiDrive" will exist somewhere:
void KiwiDrive(float vx, float vy, float omega, float maxPWM = 255.0);

const int IR_RECIEVE_PIN = 4;
decode_results results;

// Define a timeout (in milliseconds) after which the robot should stop 
// if no command is received.
const unsigned long IR_TIMEOUT = 280;   // adjust as needed

// Variable to store the last time an IR command was received:
unsigned long lastIRReceiveTime = 0;

void setup() {
  Serial.begin(115200);
  MotorControl_Init();
}

void loop() {
  // 1. Move forward for 2 seconds
  KiwiDrive(0.0, 1.0, 0.0);  // (vx=0, vy=+1) => forward
  delay(2000);

  // 2. Move right for 2 seconds
  KiwiDrive(1.0, 0.0, 0.0);  // (vx=+1, vy=0) => right
  delay(2000);

  // 3. Move backward for 2 seconds
  KiwiDrive(0.0, -1.0, 0.0); // (vx=0, vy=-1) => backward
  delay(2000);

  // 4. Move left for 2 seconds
  KiwiDrive(-1.0, 0.0, 0.0); // (vx=-1, vy=0) => left
  delay(2000);

  // 5. Stop for 1 second
  KiwiDrive(0.0, 0.0, 0.0);
  delay(1000);

  // Then it loops back to step 1, repeating the square forever
}
