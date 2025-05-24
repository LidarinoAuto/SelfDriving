#include <IRremote.hpp>  // Merk: nyere versjoner bruker IRremote.hpp
 
const int IR_RECEIVE_PIN = 13;  // Bruk en pin som st�tter digital inn (ikke 0, 2 eller 12)
 
void setup() {

  Serial.begin(115200);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // Starter mottakeren

}
 
void loop() {

  if (IrReceiver.decode()) {

    Serial.print("IR-kode mottatt: ");

    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);  // 32-bit r�verdi

    IrReceiver.resume();  // Klargj�r for neste signal

  }

}
 
