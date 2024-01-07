#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
    public:
        Encoder(uint8_t channelA, uint8_t channelB);
        void begin();
        void update();
        int32_t getPulse();
        int32_t getPulsePerSecond();
        void reset();
        void setPulse(int32_t pulse);
        uint8_t getChannelA();
        
    private:
        uint8_t channelA;
        uint8_t channelB;
        volatile int32_t pulses;
        volatile int32_t lastPulses;
        volatile int32_t pulsePerSecond;
        volatile unsigned long lastUpdateTime;
};


// #include "parameter.h"

// int32_t pulses = 0;

// void encoderUpdate(){
//     if (digitalRead(ENCODER_CHANNEL_A) == digitalRead(ENCODER_CHANNEL_B)){
//         pulses++;
//     } else {
//         pulses--;
//     }
// }

// int32_t encoderGetPulse(){
//     return pulses;
// }

// void encoderReset(){
//     pulses = 0;
// }

// void encoderBegin(){
//     pinMode(ENCODER_CHANNEL_A, INPUT_PULLUP);
//     pinMode(ENCODER_CHANNEL_B, INPUT_PULLUP);
//     attachInterrupt(digitalPinToInterrupt(ENCODER_CHANNEL_A), encoderUpdate, CHANGE);
// }

#endif // ENCODER_HPP