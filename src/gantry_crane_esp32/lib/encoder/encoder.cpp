#include "encoder.hpp"

Encoder::Encoder(uint8_t channelA, uint8_t channelB){
    this->channelA = channelA;
    this->channelB = channelB;
}

void Encoder::begin(){
    pinMode(this->channelA, INPUT_PULLUP);
    pinMode(this->channelB, INPUT_PULLUP);
}

void Encoder::update(){
    volatile unsigned long currentTime = micros();
    if (digitalRead(this->channelA) == digitalRead(this->channelB)){
        this->pulses++;
    } else {
        this->pulses--;
    }
    int32_t pulsePerSecond = (this->pulses - this->lastPulses) / (currentTime - this->lastUpdateTime) * 1000000;
    this->lastUpdateTime = micros();
    this->lastPulses = this->pulses;
}

int32_t Encoder::getPulse(){
    return this->pulses;
}

int32_t Encoder::getPulsePerSecond(){
    return this->pulsePerSecond;
}

void Encoder::reset(){
    this->pulses = 0;
}

void Encoder::setPulse(int32_t pulse){
    this->pulses = pulse;
}

u_int8_t Encoder::getChannelA(){
    return this->channelA;
}