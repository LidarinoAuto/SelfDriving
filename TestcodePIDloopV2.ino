// Pin definitions for encoders
#define EncoderA1 18
#define EncoderB1 17
#define EncoderA2 22
#define EncoderB2 23
#define EncoderA3 21
#define EncoderB3 19

// Variables for encoders and motors
volatile long encoderCount1 = 0, encoderCount2 = 0, encoderCount3 = 0;
const float gearRatio = 75.8;
const int encoderCountsPerRevolution = 720;
const float wheelDiameter = 58.0;
const float wheelCircumference = wheelDiameter * PI;

float actualSpeed1 = 0, actualSpeed2 = 0, actualSpeed3 = 0;
float setpoint1 = 200.0, setpoint2 = -100.0, setpoint3 = -250.0;

// PID variables
double kp = 1.8, ki = 2.5, kd = 0.005;
double integral1 = 0, integral2 = 0, integral3 = 0;
double previousError1 = 0, previousError2 = 0, previousError3 = 0;

// Interrupt Service Routines (ISRs) for encoders
void IRAM_ATTR readEncoder1() {
    bool encoderA = digitalRead(EncoderA1);
    bool encoderB = digitalRead(EncoderB1);
    encoderCount1 += (encoderA == encoderB) ? 1 : -1;
}

void IRAM_ATTR readEncoder2() {
    bool encoderA = digitalRead(EncoderA2);
    bool encoderB = digitalRead(EncoderB2);
    encoderCount2 += (encoderA == encoderB) ? 1 : -1;
}

void IRAM_ATTR readEncoder3() {
    bool encoderA = digitalRead(EncoderA3);
    bool encoderB = digitalRead(EncoderB3);
    encoderCount3 += (encoderA == encoderB) ? 1 : -1;
}

void setup() {
    // Encoder setup
    pinMode(EncoderA1, INPUT_PULLUP);
    pinMode(EncoderB1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EncoderA1), readEncoder1, CHANGE);

    pinMode(EncoderA2, INPUT_PULLUP);
    pinMode(EncoderB2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EncoderA2), readEncoder2, CHANGE);

    pinMode(EncoderA3, INPUT_PULLUP);
    pinMode(EncoderB3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EncoderA3), readEncoder3, CHANGE);

    MotorControl_Init(); // Initialize motor control
}

void loop() {
    static long lastCount1 = 0, lastCount2 = 0, lastCount3 = 0;
    static unsigned long lastTime = 0;

    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime;

    if (deltaTime >= 100) {  // Update every 100ms
        // Update speeds for all motors
        updatePID(1, encoderCount1, lastCount1, actualSpeed1, setpoint1, integral1, previousError1, Motor1, deltaTime);
        updatePID(2, encoderCount2, lastCount2, actualSpeed2, setpoint2, integral2, previousError2, Motor2, deltaTime);
        updatePID(3, encoderCount3, lastCount3, actualSpeed3, setpoint3, integral3, previousError3, Motor3, deltaTime);

        lastTime = currentTime;
    }

    delay(50);  // Small delay for stability
}

void updatePID(int motorID, volatile long &encoderCount, long &lastCount, volatile float &actualSpeed, float setpoint, double &integral, double &previousError, void (*motorControl)(float), unsigned long deltaTime) {
    noInterrupts();
    long count = encoderCount - lastCount;
    lastCount = encoderCount;
    interrupts();

    // Calculate actual speed (mm/s)
    float rpm = (count / (float)encoderCountsPerRevolution) 
                / (deltaTime / 60000.0) 
                / gearRatio;
    actualSpeed = (rpm * wheelCircumference / 60);

    // PID control with anti-windup
    float error = setpoint - actualSpeed;

    // Limit the integral to prevent windup
    const float integralLimit = 200.0;
    integral += error * (deltaTime / 1000.0);
    if (integral > integralLimit) {
        integral = integralLimit;
    } else if (integral < -integralLimit) {
        integral = -integralLimit;
    }

    // Derivative term
    float derivative = (error - previousError) / (deltaTime / 1000.0);
    double pidOutput = (kp * error) + (ki * integral) + (kd * derivative);

    // Saturate PID output to motor range (-255 to 255)
    pidOutput = constrain(pidOutput, -255, 255);

    // Prevent integral windup if motor is saturated
    if (pidOutput == 255 || pidOutput == -255) {
        integral -= error * (deltaTime / 1000.0);
    }

    // Update motor control
    motorControl(pidOutput);

    // Update for the next loop
    previousError = error;

    // Debug output
    Serial.print("Motor "); Serial.print(motorID);
    Serial.print(" - P:"); Serial.print(kp * error);
    Serial.print(" I:"); Serial.print(ki * integral);
    Serial.print(" D:"); Serial.print(kd * derivative);
    Serial.print(" Error:"); Serial.print(error);
    Serial.print(" Setpoint:"); Serial.print(setpoint);
    Serial.print(" ActualSpeed:"); Serial.println(actualSpeed);
}
