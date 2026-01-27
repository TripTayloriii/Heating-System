#include "PID.h"
PID::PID(float Kp, float Kd, float Ki){
    setK(Kp, Kd, Ki);
}

void PID::setK(float newKp, float newKd, float newKi){
    Kp = newKp;
    Kd = newKd;
    Ki = newKi;
}

float PID::update(float setpoint, float measurement, float dt){
    error = setpoint - measurement; //update error, derivative, integral
    derivative = 0.9 * derivative + 0.1 * (error - prevError) / dt;
    integral += error * dt;

    prevError = error; //update prevError

    return Kp*error + Kd*derivative + Ki*integral;
}