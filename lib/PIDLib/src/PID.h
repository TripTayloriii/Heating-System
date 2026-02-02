#ifndef PID_H
#define PID_H
#include <Arduino.h>

class PID{
    public:
        PID(float Kp, float Ki, float Kd);
        float update(float setpoint, float measurement, float dt);
        void setK(float newKp, float newKi, float newKd);
    private:
        float Kp;
        float Kd;
        float Ki;
        float dt;
        float integral = 0;
        float derivative = 0;
        float derivativeMeasurement = 0;
        float filteredDerivativeMeasurement;
        float prevError = 0;
        float prevMeasurement;
        float error;

};
#endif