// Lidarino_Robot_Test.ino

//#include <Arduino.h>

// Tell the compiler "KiwiDrive" will exist somewhere:
void KiwiDrive(float vx, float vy, float maxPWM = 255.0);

void setup() {
  Serial.begin(9600);
  MotorControl_Init();
}

void loop() {
  // 1. Move forward for 2 seconds
  KiwiDrive(0.0, 1.0);  // (vx=0, vy=+1) => forward
  delay(2000);

  // 2. Move right for 2 seconds
  KiwiDrive(1.0, 0.0);  // (vx=+1, vy=0) => right
  delay(2000);

  // 3. Move backward for 2 seconds
  KiwiDrive(0.0, -1.0); // (vx=0, vy=-1) => backward
  delay(2000);

  // 4. Move left for 2 seconds
  KiwiDrive(-1.0, 0.0); // (vx=-1, vy=0) => left
  delay(2000);

  // 5. Stop for 1 second
  KiwiDrive(0.0, 0.0);
  delay(1000);

  // Then it loops back to step 1, repeating the square forever
}
