#include "encoder.hpp"

Encoder::Encoder(uint8_t channelA, uint8_t channelB, int32_t minPulseValue, int32_t maxPulseValue, float minPositionValue, float maxPositionValue)
{
    this->channelA = channelA;
    this->channelB = channelB;
    this->minPulseValue = minPulseValue;
    this->maxPulseValue = maxPulseValue;
    this->minPositionValue = minPositionValue;
    this->maxPositionValue = maxPositionValue;
}

void Encoder::begin()
{
    pinMode(this->channelA, INPUT_PULLUP);
    pinMode(this->channelB, INPUT_PULLUP);
}

void Encoder::update()
{
    if (digitalRead(this->channelA) == digitalRead(this->channelB))
    {
        this->pulses++;
    }
    else
    {
        this->pulses--;
    }
}

int32_t Encoder::getPulse()
{
    return this->pulses;
}

void Encoder::reset()
{
    this->pulses = 0;
}

void Encoder::setPulse(int32_t pulse)
{
    this->pulses = pulse;
}

u_int8_t Encoder::getChannelA()
{
    return this->channelA;
}

float Encoder::getPosition()
{
    return (this->pulses - this->minPulseValue) * (this->maxPositionValue - this->minPositionValue) / (this->maxPulseValue - this->minPulseValue) + this->minPositionValue;
}