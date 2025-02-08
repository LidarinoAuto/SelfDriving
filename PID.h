#ifndef PID_H
#define PID_H

extern volatile int encoder_ticks;
extern const int ENCODER_RESOLUTION;
extern const float WHEEL_DIAMETER_MM;
extern const int CONTROL_INTERVAL;
extern const float INTEGRAL_LIMIT;
extern const int DEAD_ZONE;
extern float target_speed_mms;
extern float kp, ki, kd;
extern float last_error, integral, derivative;
extern int control_signal;

float computePID(float target, float current, int interval) {
  float error = target - current;
  integral += error * interval;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  derivative = (error - last_error) / interval;
  control_signal = kp * abs(error) + ki * integral + kd * derivative;

  if (abs(control_signal) < DEAD_ZONE) {
    control_signal = 0;
  }

  last_error = error;
  return control_signal;
}

#endif
