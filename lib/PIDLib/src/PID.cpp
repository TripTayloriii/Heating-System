#include "PID.h"
PID::PID(float Kp, float Ki, float Kd){
    setK(Kp, Ki, Kd);
}

void PID::setK(float newKp, float newKi, float newKd){
    Kp = newKp;
    Ki = newKi;
    Kd = newKd;
}

float PID::update(float setpoint, float measurement, float dt){
    error = setpoint - measurement;
    // Derivative on measurement  (resistant to setpoint changes)
    float deltaMeasurement = (measurement - prevMeasurement) / dt;
    derivativeMeasurement = 0.9 * derivativeMeasurement + 0.1 * deltaMeasurement; //low pass filter

    // Pre-integral output
    float output = Kp * error - Kd * derivativeMeasurement;

    // Integration
    bool heating = (output > 0);
    bool nearSetpoint = abs(error) < max(2.0, 0.2 * setpoint);     // Celsius window
    bool stable = abs(derivativeMeasurement) < 0.5;      // Celsius/s

    if (heating && nearSetpoint && stable) { //only integrate if conditions are met
        integral += error * dt;
    }
    integral = constrain(integral, 0, 2000); //clamp integral
    output += Ki * integral;

    prevMeasurement = measurement;
    return constrain(output, 0, 100);
}