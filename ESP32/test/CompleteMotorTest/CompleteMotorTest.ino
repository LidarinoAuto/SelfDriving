
void setup() {
  // Initialize serial if you like
  Serial.begin(9600);
  
  // Initialize the motor pins
  MotorControl_Init();
}
  void Forward() {
  M1_Forward();
  M2_Backward();
  M3_Stop();
  

}

void loop() {
  Forward();
}
