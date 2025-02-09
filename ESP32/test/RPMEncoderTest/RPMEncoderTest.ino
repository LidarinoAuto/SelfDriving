// Pin definitions for encoders
#define EncoderA1 18  // Encoder 1 A-pin
#define EncoderB1 17  // Encoder 1 B-pin
#define EncoderA2 19  // Encoder 2 A-pin
#define EncoderB2 21  // Encoder 2 B-pin
#define EncoderA3 22  // Encoder 3 A-pin
#define EncoderB3 23  // Encoder 3 B-pin

// Variables for encoder counts
volatile long encoderCount1 = 0; // Encoder 1 count
volatile long encoderCount2 = 0; // Encoder 2 count
volatile long encoderCount3 = 0; // Encoder 3 count

const float gearRatio = 75.8; // Adjustable gear ratio (common for all motors)
const int encoderCountsPerRevolution = 720; // Encoder counts per revolution (common for all motors)

// Interrupt Service Routine (ISR) for Encoder 1
void IRAM_ATTR readEncoder1() {
    bool encoderA1 = digitalRead(EncoderA1);
    bool encoderB1 = digitalRead(EncoderB1);
    encoderCount1 += (encoderA1 == encoderB1) ? 1 : -1;
}

// ISR for Encoder 2
void IRAM_ATTR readEncoder2() {
    bool encoderA2 = digitalRead(EncoderA2);
    bool encoderB2 = digitalRead(EncoderB2);
    encoderCount2 += (encoderA2 == encoderB2) ? 1 : -1;
}

// ISR for Encoder 3
void IRAM_ATTR readEncoder3() {
    bool encoderA3 = digitalRead(EncoderA3);
    bool encoderB3 = digitalRead(EncoderB3);
    encoderCount3 += (encoderA3 == encoderB3) ? 1 : -1;
}

void setup() {
    MotorControl_Init();
    Serial.begin(115200);

    // Set encoder pins as inputs with pull-ups
    pinMode(EncoderA1, INPUT_PULLUP);
    pinMode(EncoderB1, INPUT_PULLUP);
    pinMode(EncoderA2, INPUT_PULLUP);
    pinMode(EncoderB2, INPUT_PULLUP);
    pinMode(EncoderA3, INPUT_PULLUP);
    pinMode(EncoderB3, INPUT_PULLUP);

    // Attach interrupts for all encoders
    attachInterrupt(digitalPinToInterrupt(EncoderA1), readEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderA2), readEncoder2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderA3), readEncoder3, CHANGE);
}

void loop() {
    static long lastCount1 = 0, lastCount2 = 0, lastCount3 = 0;
    static unsigned long lastTime = 0;

    unsigned long currentTime = millis();
    unsigned long timeIntervalMillis = currentTime - lastTime;

    if (timeIntervalMillis >= 100) { // Update every 100ms
        noInterrupts(); // Temporarily disable interrupts to access shared variables
        long count1 = encoderCount1 - lastCount1;
        long count2 = encoderCount2 - lastCount2;
        long count3 = encoderCount3 - lastCount3;
        lastCount1 = encoderCount1;
        lastCount2 = encoderCount2;
        lastCount3 = encoderCount3;
        interrupts(); // Re-enable interrupts

        // Calculate RPM for each motor
        float rpm1 = (count1 / (float)encoderCountsPerRevolution) / (timeIntervalMillis / 60000.0) / gearRatio;
        float rpm2 = (count2 / (float)encoderCountsPerRevolution) / (timeIntervalMillis / 60000.0) / gearRatio;
        float rpm3 = (count3 / (float)encoderCountsPerRevolution) / (timeIntervalMillis / 60000.0) / gearRatio;

        // Print RPM for each motor
        Serial.print("Motor 1 RPM: ");
        Serial.println(rpm1);
        Serial.print("Motor 2 RPM: ");
        Serial.println(rpm2);
        Serial.print("Motor 3 RPM: ");
        Serial.println(rpm3);

        // Update last time
        lastTime = currentTime;
    }

    delay(50); // Small delay for stability
}
