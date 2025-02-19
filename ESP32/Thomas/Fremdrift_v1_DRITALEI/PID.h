#ifndef PID_H
#define PID_H

// Eksterne variabler deklarert i andre deler av programmet
extern volatile long encoderCount1, encoderCount2, encoderCount3;
extern const int ENCODER_RESOLUTION;
extern const float WHEEL_DIAMETER_MM;
extern float CONTROL_INTERVAL; // Endret til float uten const
extern const float INTEGRAL_LIMIT;
extern const int DEAD_ZONE;
extern float setpoint1, setpoint2, setpoint3;
extern float kp, ki, kd;
extern float integral1, integral2, integral3;
extern float last_error1, last_error2, last_error3;

// Definer en lavpassfilterkonstant (alpha) mellom 0 og 1
const float ALPHA = 0.8;

// Eksterne variabler for filtrert hastighet
extern float filtered_speed1, filtered_speed2, filtered_speed3;

/**
 * Beregn PID kontrollsignal basert på settpunkt og faktisk hastighet.
 * 
 * @param setpoint Ønsket hastighet (mm/s)
 * @param actual_speed Faktisk hastighet (mm/s)
 * @param filtered_speed Henviser til filtrert hastighet (for minne i funksjonskall)
 * @param integral Henviser til integralvariabelen (for minne i funksjonskall)
 * @param last_error Henviser til forrige feilverdi (for minne i funksjonskall)
 * @param interval Oppdateringsintervall (ms)
 * @return Kontrollsignal for motoren (PWM-verdi)
 */
float computePID(float setpoint, float actual_speed, float &filtered_speed, float &integral, float &last_error, float interval);

#endif // PID_H
