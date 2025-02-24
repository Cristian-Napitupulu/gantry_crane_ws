#ifndef FREERTOS_TASK_HPP
#define FREERTOS_TASK_HPP

#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"

bool microROSinitialized = false;
volatile unsigned long positionTime = 0;
volatile unsigned long lastPositionTime = 0;
void spinMicroROS(void *parameter)
{
    microROSInit();

    positionTime = millis();
    for (;;)
    {
        checkLimitSwitch();

        positionTime = millis();
        trolleyPosition = encoderTrolley.getPosition();
        trolleySpeed = (trolleyPosition - lastTrolleyPosition) / (positionTime - lastPositionTime) * 1000.0f;
        lastTrolleyPosition = trolleyPosition;
        lastPositionTime = millis();

        RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
        RCSOFTCHECK(rclc_executor_spin_some(&controllerCommandExecutor, RCL_MS_TO_NS(CONTROLLER_COMMAND_SUBSCRIBER_TIMEOUT_MS)));

        if (PUBLISH_VOLTAGE)
        {
            RCSOFTCHECK(rclc_executor_spin_some(&trolleyMotorVoltageExecutor, RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
            RCSOFTCHECK(rclc_executor_spin_some(&hoistMotorVoltageExecutor, RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
        }

        microROSinitialized = true;
    }
}

uint64_t findOriginTimer = 0;
void findOrigin()
{
    findOriginTimer = millis();
    trolleyMotorPWM = -TROLLEY_MOTOR_FIND_ORIGIN_PWM;
    while (limitSwitchEncoderSideState == false)
    {
        if (millis() - findOriginTimer > CONTROLLER_SAMPLING_TIME_MS)
        {
            if (fabs(trolleySpeed) < 0.5 * TROLLEY_MAX_SPEED)
            {
                trolleyMotorPWM += -5;
            }
            else if (fabs(trolleySpeed) > TROLLEY_MAX_SPEED)
            {
                trolleyMotorPWM += 10;
            }

            trolleyMotor.setPWM(safetyCheck(trolleyMotorPWM));
            findOriginTimer = millis();
        }

        // Instead of Serial.println(""), add a small delay
        vTaskDelay(pdMS_TO_TICKS(1)); // Ensures FreeRTOS can switch tasks
    }

    trolleyMotorPWM = 0;
    trolleyMotor.setPWM(trolleyMotorPWM);
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < 2; i++)
    {
        for (int i = -0.9 * TROLLEY_MOTOR_FIND_ORIGIN_PWM; i > -1.1 * TROLLEY_MOTOR_FIND_ORIGIN_PWM; i--)
        {
            trolleyMotorPWM = i;
            trolleyMotor.setPWM(trolleyMotorPWM);
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        trolleyMotorPWM = 0;
        trolleyMotor.setPWM(trolleyMotorPWM);
        encoderTrolley.reset();
    }
}

bool isControllerCommandTimeout()
{
    if (millis() - controllerCommandLastCallTime > CONTROLLER_COMMAND_TIMEOUT_MS)
    {
        gantryMode = IDLE_MODE;

        return true;
    }

    return false;
}

bool stated_once = false;
uint64_t controllerCommandTimer = 0;
void controllerCommandTask(void *parameter)
{
    while (microROSinitialized == false)
    {
        // Wait for microROS to be initialized
        ledBuiltIn.blink(100);
    }

    findOrigin();

    gantryMode = IDLE_MODE;

    controllerCommandTimer = millis();
    for (;;)
    {
        ledBuiltIn.blink(300);
        vTaskDelay(pdMS_TO_TICKS(1));

        if (millis() - controllerCommandTimer > CONTROLLER_SAMPLING_TIME_MS)
        {
            // Brake gantry
            if (brakeTrolleyMotor == true)
            {
                trolleyMotorPWM = 0;
                trolleyMotor.setPWM(trolleyMotorPWM);
                trolleyMotor.brake();
                if (millis() - trolleyBrakeCommandTimer > BRAKE_COMMAND_TIMEOUT_MS)
                {
                    brakeTrolleyMotor = false;
                }
            }

            if (brakeHoistMotor == true)
            {
                hoistMotorPWM = 0;
                hoistMotor.setPWM(hoistMotorPWM);
                hoistMotor.brake();
                if (millis() - hoistBrakeCommandTimer > BRAKE_COMMAND_TIMEOUT_MS)
                {
                    brakeHoistMotor = false;
                }
            }

            if (gantryMode == IDLE_MODE)
            {
                // Do nothing
                trolleyMotorPWM = 0;
                hoistMotorPWM = 0;
                lastTrolleyMotorVoltage = 0;
                lastHoistMotorVoltage = 0;
            }

            else if (gantryMode == CONTROL_MODE && isControllerCommandTimeout() == false)
            {
                // Control gantry
            }

            else if (gantryMode == MOVE_TO_ORIGIN_MODE && isControllerCommandTimeout() == false)
            {
                if (limitSwitchEncoderSideState == false || fabs(trolleyPosition) > 0.005)
                {
                    if (!stated_once)
                    {
                        trolleyMotorPWM = -TROLLEY_MOTOR_FIND_ORIGIN_PWM;
                        hoistMotorPWM = 0;
                        stated_once = true;
                    }

                    if (fabs(trolleySpeed) < 0.5 * TROLLEY_MAX_SPEED)
                    {
                        trolleyMotorPWM += -5;
                    }

                    else if (fabs(trolleySpeed) > TROLLEY_MAX_SPEED)
                    {
                        trolleyMotorPWM += 10;
                    }
                }
                
                else
                {
                    trolleyMotorPWM = 0;
                    gantryMode = IDLE_MODE;
                    stated_once = false;
                }
            }
            else if (gantryMode == MOVE_TO_MIDDLE_MODE && isControllerCommandTimeout() == false)
            {
                trolleyMotorPWM = TROLLEY_MOTOR_FIND_ORIGIN_PWM * get_sign(ENCODER_MAX_VALUE / 2 - encoderTrolley.getPulse());
                hoistMotorPWM = 0;

                if (fabs(encoderTrolley.getPulse() - ENCODER_MAX_VALUE / 2) < 0.01 * ENCODER_MAX_VALUE)
                {
                    trolleyMotorPWM = 0;
                    gantryMode = IDLE_MODE;
                }
            }

            else if (gantryMode == MOVE_TO_END_MODE && isControllerCommandTimeout() == false)
            {
                if (limitSwitchTrolleyMotorSideState == false)
                {
                    if (!stated_once)
                    {
                        trolleyMotorPWM = TROLLEY_MOTOR_FIND_ORIGIN_PWM;
                        hoistMotorPWM = 0;
                        stated_once = true;
                    }

                    if (fabs(trolleySpeed) < 0.5 * TROLLEY_MAX_SPEED)
                    {
                        trolleyMotorPWM += 5;
                    }

                    else if (fabs(trolleySpeed) > TROLLEY_MAX_SPEED)
                    {
                        trolleyMotorPWM += -10;
                    }
                }

                else
                {
                    trolleyMotorPWM = 0;
                    gantryMode = IDLE_MODE;
                    stated_once = false;
                }
            }

            else
            {
                gantryMode = IDLE_MODE;
            }

            trolleyMotor.setPWM(safetyCheck(trolleyMotorPWM));
            hoistMotor.setPWM(hoistMotorPWM);

            controllerCommandTimer = millis();
        }

        if (PUBLISH_VOLTAGE)
        {
            trolleyMotorVoltage = readChannel(ADS1115_COMP_0_GND);
            hoistMotorVoltage = readChannel(ADS1115_COMP_1_GND);
        }
    }
}

#endif // FREERTOS_TASK_HPP