#ifndef IR_CONTROL_H
#define IR_CONTROL_H

#include <IRremote.h>
#include "KiwiDrive.h"

// Fjernet duplisert definisjon av IR_RECEIVE_PIN
// const int IR_RECEIVE_PIN = 15;

void IRSetup() {
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void handleIRInput() {
  static float vx_mm = 0.0;
  static float vy_mm = 0.0;
  static float omega_rad = 0.0;

  if (IrReceiver.decode()) {
    Serial.print("Received IR signal: ");
    Serial.println(IrReceiver.decodedIRData.command, HEX);

    switch (IrReceiver.decodedIRData.command) {
      case 0x18: // Framover
        vy_mm = 200.0;  // +Y = fremover
        vx_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("Framover");
        break;
      case 0x52: // Bakover
        vy_mm = -200.0;
        vx_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("Bakover");
        break;
      case 0x8:  // Venstre
        vx_mm = -200.0;
        vy_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("Venstre");
        break;
      case 0x5A: // Høyre
        vx_mm = 200.0;
        vy_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("Høyre");
        break;
      case 0x1C: // Stopp
        vx_mm = 0.0;
        vy_mm = 0.0;
        omega_rad = 0.0;
        Serial.println("Stopp");
        break;
      case 0xD:  // Roter høyre
        vx_mm = 0.0;
        vy_mm = 0.0;
        omega_rad = 5.0;  // eksempel: 1 rad/s
        Serial.println("Roter høyre");
        break;
      case 0x16: // Roter venstre
        vx_mm = 0.0;
        vy_mm = 0.0;
        omega_rad = -5.0; // eksempel: -1 rad/s
        Serial.println("Roter venstre");
        break;
      default:
        Serial.println("Ukjent kommando");
        break;
    }

    // Oppdater ønskede hjulhastigheter med mm/s og rad/s
    KiwiDrive(vx_mm, vy_mm, omega_rad);

    // Nullstill variabler etter bruk
    vx_mm = 0.0;
    vy_mm = 0.0;
    omega_rad = 0.0;

    IrReceiver.resume();
  }
}

#endif // IR_CONTROL_H
