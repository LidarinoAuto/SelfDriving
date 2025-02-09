// Pin-definisjoner
const int in1Pin = 14;      // PWM-pin for motorens retning/hastighet
const int in2Pin = 27;      // PWM-pin for motorens retning/hastighet
const int encoderAPin = 18; // Encoder A-pin
const int encoderBPin = 17; // Encoder B-pin

// Variabler for encoder
volatile long encoderTicks = 0;
long previousTicks = 0;
unsigned long previousMillis = 0;
double currentRPM = 0.0;

// PID-variabler
double setpointRPM = 25.0; // Ønsket hastighet i RPM
double inputRPM = 0.0;      // Målt hastighet i RPM
double outputPWM = 0.0;     // PWM-utgang

// PID-konstanter (juster etter behov)
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
double integral = 0.0;
double previousError = 0.0;

// Konstant for girforhold og encoder ticks per rotasjon
const double gearRatio = 74.8;
const int ticksPerRevolution = 360;
const int interval = 100; // Oppdateringsintervall i ms

void setup() {
  // Sett pin-modus
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);

  // Sett opp interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(encoderAPin), encoderISR, RISING);

  // Sett opp seriell kommunikasjon
  Serial.begin(9600);

  // Sett motoren til å være av som start
  analogWrite(in1Pin, 0);
  analogWrite(in2Pin, 0);

  previousMillis = millis();
}

// Interrupt Service Routine for encoder
void encoderISR() {
  if (digitalRead(encoderBPin) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

// Funksjon for å beregne RPM basert på encoder-ticks
void calculateRPM() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    long ticks = encoderTicks - previousTicks;
    double timeElapsed = (currentMillis - previousMillis) / 1000.0; // Tid i sekunder

    currentRPM = (ticks / (ticksPerRevolution * gearRatio)) * (60.0 / timeElapsed);

    previousTicks = encoderTicks;
    previousMillis = currentMillis;
  }
}

// Funksjon for å kjøre motoren basert på PWM-verdi
void driveMotor(int pwmValue) {
  if (pwmValue > 255) pwmValue = 255;
  if (pwmValue < 0) pwmValue = 0;

  analogWrite(in1Pin, pwmValue);
  analogWrite(in2Pin, 0);
}

// Enkel PID-beregning uten bibliotek
double calculatePID(double setpoint, double input) {
  double error = setpoint - input;

  integral += error * interval / 1000.0;
  if (integral > 255) integral = 255;
  if (integral < 0) integral = 0;

  double derivative = (error - previousError) / (interval / 1000.0);
  previousError = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void loop() {
  // Beregn nåværende RPM
  calculateRPM();

  // Oppdater PID-input og beregn PID-output
  inputRPM = currentRPM;
  outputPWM = calculatePID(setpointRPM, inputRPM);

  // Begrens PWM-utgangen og kjør motoren
  if (outputPWM > 255) outputPWM = 255;
  if (outputPWM < 0) outputPWM = 0;

  driveMotor((int)outputPWM);

  // Debugging-utdata
  Serial.print("Encoder Ticks: ");
  Serial.print(encoderTicks);
  Serial.print(" | Setpoint RPM: ");
  Serial.print(setpointRPM, 2);
  Serial.print(" | Current RPM: ");
  Serial.print(currentRPM, 2);
  Serial.print(" | Output PWM: ");
  Serial.println(outputPWM, 2);

  delay(10); // Kort forsinkelse for stabil oppdatering
}
