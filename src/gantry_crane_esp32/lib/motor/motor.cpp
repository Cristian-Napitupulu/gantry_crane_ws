#include "motor.hpp"

Motor::Motor(u_int8_t forwardPin, u_int8_t reversePin, u_int8_t pwmPin,  int16_t minPWM, int16_t maxPWM)
{
    this->forwardPin = forwardPin;
    this->reversePin = reversePin;
    this->pwmPin = pwmPin;
    this->maxPWM = maxPWM;
    this->minPWM = minPWM;
}

void Motor::begin()
{
    pinMode(this->forwardPin, OUTPUT);
    pinMode(this->reversePin, OUTPUT);
    pinMode(this->pwmPin, OUTPUT);
    analogWriteResolution(PWM_RESOLUTION);
    analogWriteFrequency(PWM_FREQUENCY);

    digitalWrite(this->forwardPin, LOW);
    digitalWrite(this->reversePin, LOW);
    analogWrite(this->pwmPin, 0);
}

void Motor::setPWM(int16_t PWM)
{
    PWM = static_cast<int16_t>(PWM);

    if (PWM > this->maxPWM)
    {
        PWM = this->maxPWM;
    }
    else if (PWM < -this->maxPWM)
    {
        PWM = -this->maxPWM;
    }

    if (PWM < this->minPWM && PWM > 0)
    {
        PWM = this->minPWM;
    }
    else if (PWM > -this->minPWM && PWM < 0)
    {
        PWM = -this->minPWM;
    }

    this->currentPWM = PWM;

    if (PWM > 0)
    {
        digitalWrite(this->forwardPin, HIGH);
        digitalWrite(this->reversePin, LOW);
        analogWrite(this->pwmPin, PWM);
    }
    else if (PWM < 0)
    {
        digitalWrite(this->forwardPin, LOW);
        digitalWrite(this->reversePin, HIGH);
        analogWrite(this->pwmPin, -PWM);
    }
    else if (PWM == 0)
    {
        digitalWrite(this->forwardPin, LOW);
        digitalWrite(this->reversePin, LOW);
        analogWrite(this->pwmPin, 0);
    }
}

void Motor::brake()
{
    this->setPWM(0);
    digitalWrite(this->forwardPin, LOW);
    digitalWrite(this->reversePin, LOW);
    analogWrite(this->pwmPin, this->maxPWM);
}