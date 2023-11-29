#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
    private:
    u_int8_t forwardPin;
    u_int8_t reversePin;
    u_int8_t pwmPin;
    int8_t speedCurrentSign = 0;

    public:
    Motor(u_int8_t forwardPin, u_int8_t reversePin, u_int8_t pwmPin);
    void begin();
    void speed(int16_t speed);
    void brake();
};

#endif // MOTOR_H