#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"
#include "FreeRTOS_task.hpp"

void setup()
{
    // Initialize notification LED
    ledBuiltIn.begin();
    ledBuiltIn.turnOn();

    // Initialize motor
    trolleyMotor.begin();
    hoistMotor.begin();

    // Initialize encoder
    encoderTrolley.begin();
    attachInterrupt(encoderTrolley.getChannelA(), encoderTrolleyCallback, CHANGE);

    // Initialize limit switches
    limitSwitchEncoderSide.begin();
    limitSwitchTrolleyMotorSide.begin();

    // Initialize ADC
    if (PUBLISH_VOLTAGE)
    {
        analogToDigitalConverterInit();
    }
    delay(1000);

    xTaskCreatePinnedToCore(
        spinMicroROS,
        "microROS spin task",
        40000,
        NULL,
        1 | portPRIVILEGE_BIT,
        &spinMicroROSTaskHandle,
        0);

    delay(1000);

    xTaskCreatePinnedToCore(
        controllerCommandTask,
        "Controller command task",
        30000,
        NULL,
        1 | portPRIVILEGE_BIT,
        &controllerCommandTaskHandle,
        1);
}
void loop()
{
}