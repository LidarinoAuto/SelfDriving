// Definerer pinner for de fire HC-SR04-sensorene
 
// Front Venstre
int trigFrontVenstre = 9;
int echoFrontVenstre = 8;
 
// Front Høyre
int trigFrontHoyre = 7;
int echoFrontHoyre = 6;
 
// Bak Venstre
int trigBakVenstre = 5;
int echoBakVenstre = 4;
 
// Bak Høyre
int trigBakHoyre = 3;
int echoBakHoyre = 2;
 
float meter;
 
// Funksjon for å lese avstand fra en HC-SR04-sensor
// Returnerer avstanden i cm. Returnerer -1 dersom avlesningen er utenfor rekkevidde.
float lesAvstand(int trig, int echo) {
  // Sørg for at trig-pinnen er lav
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  // Send en 10 µs puls
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Mål varigheten av echo-pulsen med timeout (30 ms)
  int duration = pulseIn(echo, HIGH, 30000);
  // Dersom varigheten er for lang, regnes den som "Out range"
  if (duration >= 38000) {
    return -1;
  }
  else {
    // Avstand i cm (lydens hastighet ca. 0,0343 cm/µs, tur/retur => del på 2)
    float distance = duration / 58.0;
    return distance;
  }
}
 
void setup() {
  Serial.begin(9600);
  // Sett opp sensorpinnene
  pinMode(trigFrontVenstre, OUTPUT);
  pinMode(echoFrontVenstre, INPUT);
  pinMode(trigFrontHoyre, OUTPUT);
  pinMode(echoFrontHoyre, INPUT);
  pinMode(trigBakVenstre, OUTPUT);
  pinMode(echoBakVenstre, INPUT);
  pinMode(trigBakHoyre, OUTPUT);
  pinMode(echoBakHoyre, INPUT);
  // Sørg for at alle trig-pinner starter lavt
  digitalWrite(trigFrontVenstre, LOW);
  digitalWrite(trigFrontHoyre, LOW);
  digitalWrite(trigBakVenstre, LOW);
  digitalWrite(trigBakHoyre, LOW);
  // Vent litt før første måling
  delay(6000);
  Serial.println("Avstandsmåling:");
}
 
void loop() {
  // Les avstand for hver sensor med korte forsinkelser mellom
  float frontVenstre = lesAvstand(trigFrontVenstre, echoFrontVenstre);
  delay(50);
  float frontHoyre = lesAvstand(trigFrontHoyre, echoFrontHoyre);
  delay(50);
  float bakVenstre = lesAvstand(trigBakVenstre, echoBakVenstre);
  delay(50);
  float bakHoyre = lesAvstand(trigBakHoyre, echoBakHoyre);
  delay(50);
  // Skriv ut målingene til seriemonitoren
  Serial.print("Front Venstre: ");
  if (frontVenstre < 0) {
    Serial.print("Out range");
  } else {
    Serial.print(frontVenstre);
    Serial.print(" cm");
    meter = frontVenstre / 100.0;
    Serial.print("  (");
    Serial.print(meter);
    Serial.print(" m)");
  }
  Serial.println();
  Serial.print("Front Høyre: ");
  if (frontHoyre < 0) {
    Serial.print("Out range");
  } else {
    Serial.print(frontHoyre);
    Serial.print(" cm");
    meter = frontHoyre / 100.0;
    Serial.print("  (");
    Serial.print(meter);
    Serial.print(" m)");
  }
  Serial.println();
  Serial.print("Bak Venstre: ");
  if (bakVenstre < 0) {
    Serial.print("Out range");
  } else {
    Serial.print(bakVenstre);
    Serial.print(" cm");
    meter = bakVenstre / 100.0;
    Serial.print("  (");
    Serial.print(meter);
    Serial.print(" m)");
  }
  Serial.println();
  Serial.print("Bak Høyre: ");
  if (bakHoyre < 0) {
    Serial.print("Out range");
  } else {
    Serial.print(bakHoyre);
    Serial.print(" cm");
    meter = bakHoyre / 100.0;
    Serial.print("  (");
    Serial.print(meter);
    Serial.print(" m)");
  }
  Serial.println();
  Serial.println("-----------------------------");
  delay(1000); // Hovedløkke-forsinkelse
}
