#include "debounce.hpp"

Debounce::Debounce(int pin, unsigned long debounceDelay)
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
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay)
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