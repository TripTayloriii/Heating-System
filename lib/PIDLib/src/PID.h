#ifndef PID_H
#define PID_H
#include <Arduino.h>

class PID{
    public:
        PID(float Kp, float Kd, float Ki);
        float update(float setpoint, float measurement, float dt);
        void setK(float newKp, float newKd, float newKi);
    private:
        float Kp;
        float Kd;
        float Ki;
        float dt;
        float integral = 0;
        float prevError = 0;
        float error;

};
#endif