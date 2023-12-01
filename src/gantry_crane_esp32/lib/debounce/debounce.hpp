#ifndef DEBOUNCE_HPP
#define DEBOUNCE_HPP

#include <Arduino.h>

class Debounce
{
private:
  uint8_t pin;
  int state;
  int lastState;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;

public:
  Debounce(int pin, unsigned long debounceDelay = 3);
  void begin();
  int getState();
  uint8_t getPin();
};

#endif // DEBOUNCE_HPP