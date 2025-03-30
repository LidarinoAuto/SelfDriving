#include <IRremote.h>

const int RECV_PIN = 13;  // IR mottakerens signalpinne
IRrecv irrecv(RECV_PIN);
decode_results results;

void setup() {
    Serial.begin(9600);
    irrecv.enableIRIn();  // Start IR-mottakeren
    Serial.println("Venter p� IR-signal...");
}

void loop() {
    if (irrecv.decode(&results)) {
        Serial.print("Kode mottatt: ");
        Serial.println(results.value, HEX);  // Skriv ut koden i heksadesimal
        irrecv.resume();  // Klargj�r for neste signal
    }
}
