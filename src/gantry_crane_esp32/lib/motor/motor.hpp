#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class Motor {
    private:
    u_int8_t forwardPin;
    u_int8_t reversePin;
    u_int8_t pwmPin;
    int8_t pwmCurrentSign = 0;
    int16_t maxPWM = 255;
    int16_t minPWM = 0;

    public:
    Motor(u_int8_t forwardPin, u_int8_t reversePin, u_int8_t pwmPin,  int16_t minPWM, int16_t maxPWM);
    void begin();
    void setPWM(int16_t PWM);
    void brake();
};

#endif // MOTOR_HPP