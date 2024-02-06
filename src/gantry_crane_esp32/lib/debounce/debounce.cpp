#include "debounce.hpp"

Debounce::Debounce(int pin, u_int32_t debounceDelay)
{
    this->pin = pin;
    this->debounceDelay = debounceDelay;
}

void Debounce::begin()
{
    pinMode(pin, INPUT_PULLUP);
    state = digitalRead(pin);
    lastState = state;
    lastDebounceTime = 0;
}

int Debounce::getState()
{
    int reading = digitalRead(pin);

    if (reading != lastState)
    {
        lastDebounceTime = micros() / 1000;
    }

    if ((micros() / 1000 - lastDebounceTime) > debounceDelay)
    {
        if (reading != state)
        {
            state = reading;
        }
    }

    lastState = reading;
    return state;
}

uint8_t Debounce::getPin()
{
    return pin;
}