#ifndef IR_CONTROL_H
#define IR_CONTROL_H

#include "PinConfig.h"
#include <IRremote.h>

// Extern variables shared with main code
extern int speedX;
extern int speedY;
extern int rotation;

// Timeout in ms
static unsigned long lastIRReceiveTime = 0;

inline void IRSetup() {
  Serial.begin(115200);               // Seriell kommunikasjon til debugging
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  lastIRReceiveTime = millis();       // Initialize at startup
}

inline void handleIRInput() {
  if (!IrReceiver.decode()) return;   // Hvis ingen ny kode, hopp ut

  lastIRReceiveTime = millis();       // Oppdater tidspunkt for siste signal
  uint32_t cmd = IrReceiver.decodedIRData.command;
  Serial.print("IR cmd: 0x");
  Serial.println(cmd, HEX);

  switch (cmd) {
    case 0x18:  // Forward
      speedX   = 200;
      speedY   = 0;
      rotation = 0;
      break;

    case 0x52:  // Backward
      speedX   = -200;
      speedY   = 0;
      rotation = 0;
      break;

    case 0x08:  // Left  (merk at 0x8 skal skrives som 0x08)
      speedX   = 0;
      speedY   = 200;
      rotation = 0;
      break;

    case 0x5A:  // Right
      speedX   = 0;
      speedY   = -200;
      rotation = 0;
      break;

    case 0x16:  // Rotate left
      speedX   = 0;
      speedY   = 0;
      rotation = 50;
      break;

    case 0x0D:  // Rotate right (0xD = 0x0D)
      speedX   = 0;
      speedY   = 0;
      rotation = -50;
      break;

    case 0x1C:  // Start (med 2 sek delay)
      Serial.println("START_LIDAR");
      delay(2000);
      Serial.println("ROBOT_START");
      break;

    case 0x19:  // Stop (stopper ogs� Lidar)
      speedX   = 0;
      speedY   = 0;
      rotation = 0;
      Serial.println("ROBOT_STOP");
      Serial.println("STOP_LIDAR");
      break;

    case 0x45:  // Start Lidar
      Serial.println("START_LIDAR");
      break;

    case 0x46:  // Stop Lidar
      Serial.println("STOP_LIDAR");
      break;

    default:
      Serial.println("Unknown command");
      break;
  }

  IrReceiver.resume();  // Klargj�r for neste melding
}

#endif // IR_CONTROL_H
