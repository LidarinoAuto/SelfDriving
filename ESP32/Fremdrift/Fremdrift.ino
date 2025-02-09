// Lidarino_Robot_Test.ino

#include <IRremote.h>

// Tell the compiler "KiwiDrive" will exist somewhere:
void KiwiDrive(float vx, float vy, float omega, float maxPWM = 255.0);

const int IR_RECIEVE_PIN = 13;
decode_results results;

// Define a timeout (in milliseconds) after which the robot should stop 
// if no command is received.
const unsigned long IR_TIMEOUT = 280;   // adjust as needed

// Variable to store the last time an IR command was received:
unsigned long lastIRReceiveTime = 0;

void setup() {
  Serial.begin(9600);
  MotorControl_Init();
  IrReceiver.enableIRIn(); // Start IR receiver
  IrReceiver.begin(IR_RECIEVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver

  // Initialize lastIRReceiveTime to the current time.
  lastIRReceiveTime = millis();
}

void loop() {
  // Check if an IR command has been received:
  if (IrReceiver.decode()) {
    lastIRReceiveTime = millis();   // Update timestamp upon receiving a signal

    Serial.print("Received IR signal: ");
    Serial.println(IrReceiver.decodedIRData.command, HEX);

    switch (IrReceiver.decodedIRData.command) {
      case 0x18:
        Serial.println("Framover");
        KiwiDrive(0.0, 1.0, 0.0); // Forward (adjust vy inversion if needed)
        break;
      case 0x52:
        Serial.println("Bakover");
        KiwiDrive(0.0, -1.0, 0.0); // Backward
        break;
      case 0x8:
        Serial.println("Venstre");
        KiwiDrive(-1.0, 0.0, 0.0); // Left
        break;
      case 0x5A:
        Serial.println("Høyre");
        KiwiDrive(1.0, 0.0, 0.0); // Right
        break;
      case 0x1C:
        Serial.println("Stopp");
        KiwiDrive(0.0, 0.0, 0.0); // Stop immediately on command
        break;
      case 0xD:
        Serial.println("rotate rigth");
        KiwiDrive(0.0, 0.0, 1.0); // Stop immediately on command
        break;
      case 0x16:
        Serial.println("rotate left");
        KiwiDrive(0.0, 0.0, -1.0); // Stop immediately on command
        break;
      default:
        Serial.println("Unknown command");
        break;
    }

    IrReceiver.resume(); // Prepare for the next signal
  }

  // If no IR command has been received for IR_TIMEOUT milliseconds, stop the robot:
  if (millis() - lastIRReceiveTime > IR_TIMEOUT) {
    // If you want to see when the timeout occurs, uncomment the next line:
    // Serial.println("No IR signal received: Stopping robot");
    KiwiDrive(0.0, 0.0, 0.0);
  }
}
