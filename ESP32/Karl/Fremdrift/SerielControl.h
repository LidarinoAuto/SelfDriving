#ifndef SERIAL_CONTROL_H
#define SERIAL_CONTROL_H

#include <Arduino.h>

// Disse finnes allerede som globale variabler i hovedkoden din
extern int speedX;
extern int speedY;
extern int rotation;

/**
 * readSerialCommands()
 *
 * Leser én linje fra Serial, forsøker å parse tre flyttall:
 *   "vx vy omega"
 * og konverterer dem til int for speedX, speedY, rotation.
 *
 * Eksempel på kommando over serielt:
 *   100 150 0.5
 *
 * Da blir:
 *   speedX   = 100
 *   speedY   = 150
 *   rotation = 0
 * (siden 0.5 blir til (int)0)
 */
inline void readSerialCommands() {
  if (Serial.available() > 0) {
    // Les en linje fram til '\n'
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      float x, y, w;
      // Søk etter tre flyttall i teksten
      int count = sscanf(line.c_str(), "%f %f %f", &x, &y, &w);

      if (count == 3) {
        // Konverter til int (hvis du vil bevare float, kan du endre speedX/rotation til float)
        speedX   = (int)x;
        speedY   = (int)y;
        rotation = (int)w;

        Serial.print("[DEBUG] Parsed: vx=");
        Serial.print(x);
        Serial.print(", vy=");
        Serial.print(y);
        Serial.print(", omega=");
        Serial.println(w);
      } else {
        // Hvis vi ikke klarer å parse 3 tall, gi feilmelding
        Serial.print("[ERROR] Could not parse: ");
        Serial.println(line);
      }
    }
  }
}

#endif // SERIAL_CONTROL_H
