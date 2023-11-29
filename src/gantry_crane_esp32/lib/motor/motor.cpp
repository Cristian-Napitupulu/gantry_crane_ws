#include "motor.hpp"

Motor::Motor(uint8_t forwardPin, uint8_t reversePin, uint8_t pwmPin){
    this->forwardPin = forwardPin;
    this->reversePin = reversePin;
    this->pwmPin = pwmPin;
}

void Motor::begin(){
    pinMode(this->forwardPin, OUTPUT);
    pinMode(this->reversePin, OUTPUT);
    pinMode(this->pwmPin, OUTPUT);

    digitalWrite(this->forwardPin, LOW);
    digitalWrite(this->reversePin, LOW);
    analogWrite(this->pwmPin, 0);
}

void Motor::speed(int16_t speed){
    // Determine the sign of the speed
    int8_t newSign = (speed > 0) ? 1 : ((speed < 0) ? -1 : 0);

    if (newSign != this->speedCurrentSign) {
        // Reset the direction pins if sign is changed
        digitalWrite(this->forwardPin, LOW);
        digitalWrite(this->reversePin, LOW);
        // delay(1);  // Delay for stability, adjust if needed
        this->speedCurrentSign = newSign;
    }

    if (speed > 0){
        digitalWrite(this->forwardPin, HIGH);
        digitalWrite(this->reversePin, LOW);
        analogWrite(this->pwmPin, speed);
    } else if (speed < 0){
        digitalWrite(this->forwardPin, LOW);
        digitalWrite(this->reversePin, HIGH);
        analogWrite(this->pwmPin, speed);
    } else {
        digitalWrite(this->forwardPin, LOW);
        digitalWrite(this->reversePin, LOW);
        analogWrite(this->pwmPin, 0);
    }
}

void Motor::brake(){
    digitalWrite(this->forwardPin, HIGH);
    digitalWrite(this->reversePin, HIGH);
    analogWrite(this->pwmPin, 255);
}