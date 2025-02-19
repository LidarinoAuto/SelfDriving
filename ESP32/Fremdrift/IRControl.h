#ifndef IR_CONTROL_H
#define IR_CONTROL_H

#include "PinConfig.h"
#include <IRremote.h>

// Extern variables shared with main code
extern int speedX;
extern int speedY;
extern int rotation;

// Timeout in ms
//static const unsigned long IR_TIMEOUT = 500; // e.g. 500 ms

// We store the last time we received any IR signal
static unsigned long lastIRReceiveTime = 0;

inline void IRSetup() {
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  lastIRReceiveTime = millis(); // Initialize at startup
}

inline void handleIRInput() {
  // Check if there's a new IR command
  if (IrReceiver.decode()) {
    lastIRReceiveTime = millis(); // We got a new IR signal right now

    Serial.print("IR cmd: ");
    Serial.println(IrReceiver.decodedIRData.command, HEX);

    switch (IrReceiver.decodedIRData.command) {
      case 0x18: // Forward
        speedX = 0;    
        speedY = 300;  
        rotation = 0;
        break;
      case 0x52: // Backward
        speedX = 0;
        speedY = -300;
        rotation = 0;
        break;
      case 0x8:  // Left
        speedX = -300;
        speedY = 0;
        rotation = 0;
        break;
      case 0x5A: // Right
        speedX = 300;
        speedY = 0;
        rotation = 0;
        break;
      case 0x1C: // Stop
        speedX = 0;
        speedY = 0;
        rotation = 0;
        break;
      case 0x16: // Rotate left
        speedX = 0;
        speedY = 0;
        rotation = -2;
        break;
      case 0xD: // Rotate right
        speedX = 0;
        speedY = 0;
        rotation = 2;
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
