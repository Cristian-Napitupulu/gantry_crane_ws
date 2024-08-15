#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
    public:
        Encoder(uint8_t channelA, uint8_t channelB, int32_t minPulseValue, int32_t maxPulseValue, float minPositionValue, float maxPositionValue);
        void begin();
        void update();
        int32_t getPulse();
        void reset();
        void setPulse(int32_t pulse);
        uint8_t getChannelA();

        float getPosition();
        
    private:
        uint8_t channelA;
        uint8_t channelB;
        volatile int32_t pulses;

        int32_t minPulseValue;
        int32_t maxPulseValue;
        float minPositionValue;
        float maxPositionValue;
};

#endif // ENCODER_HPP