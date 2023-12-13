#ifndef ADC_HPP
#define ADC_HPP

#include <Arduino.h>
#include <parameter.h>
#include <variable.h>

class MovingAverage {
private:
  float* buffer;      // Array to store the data
  int bufferSize;   // Size of the buffer
  int currentIndex; // Index to keep track of the current position in the buffer
  float sum;          // Running sum of the values in the buffer

public:
  // Constructor
  MovingAverage(int size) {
    bufferSize = size;
    buffer = new float[size];
    currentIndex = 0;
    sum = 0;

    // Initialize buffer values to 0
    for (int i = 0; i < bufferSize; i++) {
      buffer[i] = 0;
    }
  }

  // Destructor to free memory
  ~MovingAverage() {
    delete[] buffer;
  }

  // Add a new value to the buffer and update the sum
  void addValue(float value) {
    sum -= buffer[currentIndex]; // Subtract the oldest value
    buffer[currentIndex] = value; // Store the new value
    sum += value; // Add the new value
    currentIndex = (currentIndex + 1) % bufferSize; // Move to the next position
  }

  // Calculate and return the moving average
  float getMovingAverage() {
    return sum / bufferSize;
  }
};

MovingAverage trolleyMotorVoltageMovingAverage(TROLLEY_MOTOR_VOLTAGE_MOVING_AVERAGE_BUFFER_SIZE);
MovingAverage hoistMotorVoltageMovingAverage(HOIST_MOTOR_VOLTAGE_MOVING_AVERAGE_BUFFER_SIZE);

void analogToDigitalConverterInit()
{
  Wire.begin();

  // Initialize analog to digital converter
  if (!analogToDigitalConverter.init())
  {
    Serial.println("ADS1115 not connected!");
  }
  analogToDigitalConverter.setVoltageRange_mV(ADS1115_RANGE_6144);
  analogToDigitalConverter.setCompareChannels(ADS1115_COMP_0_GND);
  analogToDigitalConverter.setCompareChannels(ADS1115_COMP_1_GND);
  analogToDigitalConverter.setMeasureMode(ADS1115_CONTINUOUS);
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  analogToDigitalConverter.setCompareChannels(channel);
  voltage = analogToDigitalConverter.getResult_V(); // alternative: getResult_mV for Millivolt
  // if (voltage > 0.1) { // If voltage is less than 100 mV, it is considered as noise
  //   voltage = voltage / VOLTAGE_DIVIDER_RESISTOR_RATIO + FULL_BRIDGE_RECTIFIER_DIODE_VOLTAGE_DROP;
  // }
  // else {
  //   voltage = 0.0;
  // }
  return voltage;
}

#endif  // ADC_HPP