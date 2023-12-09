#ifndef PID_HPP
#define PID_HPP

#include <Arduino.h>

class PID
{
private:
    float kp;
    float ki;
    float kd;
    float integral;
    float derivative;
    float previousError;
    float setPoint;
    float output;
    float error;
    float maxOutput;
    float maxIntegral;
    float lastTime;

public:
    PID(float kp, float ki, float kd, float maxIntegral, float maxOutput);
    float calculate(float setPoint, float feedback);

};

PID::PID(float kp, float ki, float kd, float maxIntegral, float maxOutput)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxIntegral = maxIntegral;
    this->maxOutput = maxOutput;
    this->lastTime = millis();
}

float PID::calculate(float setPoint, float feedBack)
{
    float dt = (millis() - lastTime) / 1000;
    this->setPoint = setPoint;
    this->error = setPoint - feedBack;
    this->integral += error * dt;

    if (integral > maxIntegral)
    {
        integral = maxIntegral;
    }
    else if (integral < -maxIntegral)
    {
        integral = -maxIntegral;
    }

    this->derivative = (error - previousError) / dt;

    this->output = kp * error + ki * integral + kd * derivative;
    if (output > maxOutput)
    {
        output = maxOutput;
    }
    else if (output < -maxOutput)
    {
        output = -maxOutput;
    }
    previousError = error;
    return output;
}

#endif // PID_HPP