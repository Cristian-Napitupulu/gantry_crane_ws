#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>

void splitInt32toInt16(int32_t inputInt32, int16_t &lowerInt16, int16_t &upperInt16)
{
  // Extract the lower 16 bits (first int16)
  lowerInt16 = static_cast<int16_t>(inputInt32 & 0xFFFF);

  // Right shift the input by 16 bits to get the upper 16 bits
  upperInt16 = static_cast<int16_t>((inputInt32 >> 16) & 0xFFFF);
}

float map_value(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // UTILITY_H