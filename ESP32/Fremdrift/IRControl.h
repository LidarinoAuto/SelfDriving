#ifndef IR_CONTROL_H
#define IR_CONTROL_H
 
#include "PinConfig.h"
#include <IRremote.h>
 
// Extern variables shared with main code
extern int speedX;
extern int speedY;
extern int rotation;
 
// Variabler for å styre robotens start/stopp
bool robotStarted = false;
bool lidarStarted = false;
 
// Timeout i ms
static unsigned long lastIRReceiveTime = 0;
 
inline void IRSetup() {
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  lastIRReceiveTime = millis(); // Initialize at startup
  Serial.begin(115200); // Seriell kommunikasjon til debugging
  Serial2.begin(115200); // Seriell til Raspberry Pi (Lidar)
}
 
inline void handleIRInput() {
  if (IrReceiver.decode()) {
    lastIRReceiveTime = millis(); // Oppdater tidspunkt for siste signal
 
    Serial.print("IR cmd: ");
    Serial.println(IrReceiver.decodedIRData.command, HEX);
 
    switch (IrReceiver.decodedIRData.command) {
      case 0x18: // Forward
        if (robotStarted) {
          speedX = 50;    
          speedY = 0;  
          rotation = 0;
        }
        break;
      case 0x52: // Backward
        if (robotStarted) {
          speedX = -50;
          speedY = 0;
          rotation = 0;
        }
        break;
      case 0x8:  // Left
        if (robotStarted) {
          speedX = 0;
          speedY = 50;
          rotation = 0;
        }
        break;
      case 0x5A: // Right
        if (robotStarted) {
          speedX = 0;
          speedY = -50;
          rotation = 0;
        }
        break;
      case 0x16: // Rotate left
        if (robotStarted) {
          speedX = 0;
          speedY = 0;
          rotation = -2;
        }
        break;
      case 0xD: // Rotate right
        if (robotStarted) {
          speedX = 0;
          speedY = 0;
          rotation = 2;
        }
        break;
      case 0x1C: // Start (med 2 sek delay)
        Serial.println("Starting robot in 2 sec...");
        delay(2000);
        robotStarted = true;
        Serial.println("Robot started!");
        break;
      case 0x19: // Stop (stopper også Lidar)
        robotStarted = false;
        speedX = 0;
        speedY = 0;
        rotation = 0;
        Serial.println("Robot stopped!");
 
        // Stopp også Lidar hvis den var på
        if (lidarStarted) {
          Serial.println("Stopping Lidar...");
          Serial.println("STOP_LIDAR");
          lidarStarted = false;
        }
        break;
      case 0x45: // Start Lidar
        if (!lidarStarted) {
          Serial.println("Starting Lidar...");
          Serial.println("START_LIDAR");
          lidarStarted = true;
        }
        break;
      case 0x46: // Stop Lidar
        if (lidarStarted) {
          Serial.println("Stopping Lidar...");
          Serial.println("STOP_LIDAR");
          lidarStarted = false;
        }
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
 
    IrReceiver.resume();
  }

  // ---- CHECK TIMEOUT ----
  // If too long has passed since last IR command, stop the robot
  /*if (millis() - lastIRReceiveTime > IR_TIMEOUT) {
    speedX = 0;
    speedY = 0;
    rotation = 0;
  }*/
}

#endif // IR_CONTROL_H
