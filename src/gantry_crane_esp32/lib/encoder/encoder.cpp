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
    if (digitalRead(this->channelA) == digitalRead(this->channelB)){
        this->pulses++;
    } else {
        this->pulses--;
    }
}

int32_t Encoder::getPulse(){
    return this->pulses;
}

void Encoder::reset(){
    this->pulses = 0;
}

int8_t Encoder::getChannelA(){
    return this->channelA;
}