#ifndef DEBOUNCE_HPP
#define DEBOUNCE_HPP

#include <Arduino.h>

class Debounce
{
private:
  uint8_t pin;
  int state;
  int lastState;
  u_int32_t lastDebounceTime;
  u_int32_t debounceDelay;

public:
  Debounce(int pin, u_int32_t debounceDelay = 3);
  void begin();
  int getState();
  uint8_t getPin();
};

#endif // DEBOUNCE_HPP