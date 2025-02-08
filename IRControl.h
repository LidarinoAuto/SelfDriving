#ifndef IR_CONTROL_H
#define IR_CONTROL_H

#include <IRremote.h>

const int IR_RECEIVE_PIN = 15;

void IRSetup() {
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void handleIRInput() {
  if (IrReceiver.decode()) {
    Serial.print("Received IR signal: ");
    Serial.println(IrReceiver.decodedIRData.command, HEX);

    switch (IrReceiver.decodedIRData.command) {
      case 0x18:
        Serial.println("Framover");
        target_speed_mms = 350.0;
        break;
      case 0x52:
        Serial.println("Bakover");
        target_speed_mms = -350.0;
        break;
      case 0x8:
        Serial.println("Venstre");
        // Implementer venstre-styring her
        break;
      case 0x5A:
        Serial.println("Høyre");
        // Implementer høyre-styring her
        break;
      case 0x1C:
        Serial.println("Stopp");
        target_speed_mms = 0;
        break;
      default:
        Serial.println("Unknown command");
        break;
    }

    IrReceiver.resume();
  }
}

#endif
