#include "PID.h"
PID::PID(float Kp, float Ki, float Kd){
    setK(Kp, Ki, Kd);
}

void PID::setK(float newKp, float newKi, float newKd){
    Kp = newKp;
    Ki = newKi;
    Kd = newKd;
}

int derivativeThreshold = 1;

float PID::update(float setpoint, float measurement, float dt){
    error = setpoint - measurement; //update error, derivative, integral
    derivative = 0.9 * derivative + 0.1 * (error - prevError) / dt;
    float output = Kp*error + Kd*derivative;
    //conditional integration
    if(!((output >= 100 && error > 0) || (output <= 0 && error < 0))){
        if(abs(derivative) < derivativeThreshold){
            integral += error * dt;
        }
    }
    integral = constrain(integral, -300, 300);
    //derivativeMeasurement = 0.9 * derivativeMeasurement + 0.1 * (measurement - prevMeasurement) / dt;
    output = output + Ki * integral;
    prevError = error; //update prevError
    //prevMeasurement = measurement;
    return constrain(output, 0, 100);
}