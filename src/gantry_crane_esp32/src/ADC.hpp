#ifndef ADC_HPP
#define ADC_HPP

#include <Arduino.h>
#include <parameter.h>
#include <variable.h>

void analogToDigitalConverterInit()
{
  Wire.begin();

  // Initialize analog to digital converter
  if (!analogToDigitalConverter.init())
  {
    Serial.println("ADS1115 not connected!");
  }
  analogToDigitalConverter.setVoltageRange_mV(ADS1115_RANGE_2048);
  analogToDigitalConverter.setCompareChannels(ADS1115_COMP_0_GND);
  analogToDigitalConverter.setCompareChannels(ADS1115_COMP_1_GND);
  analogToDigitalConverter.setMeasureMode(ADS1115_CONTINUOUS);
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  analogToDigitalConverter.setCompareChannels(channel);
  voltage = analogToDigitalConverter.getResult_V(); // alternative: getResult_mV for Millivolt
  if (voltage > 0.035) {
    voltage = voltage / VOLTAGE_DIVIDER_RESISTOR_RATIO + FULL_BRIDGE_RECTIFIER_DIODE_VOLTAGE_DROP;
  }
  return voltage;
}

#endif  // ADC_HPP