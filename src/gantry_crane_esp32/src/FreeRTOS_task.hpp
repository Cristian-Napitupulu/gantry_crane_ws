#ifndef FREERTOS_TASK_HPP
#define FREERTOS_TASK_HPP

#include <Arduino.h>

#include "parameter.h"
#include "variable.h"
#include "utility.h"
#include "microROS.hpp"
#include "ADC.hpp"

volatile unsigned long lastPositionTime = 0;
void spinMicroROS(void *parameter)
{
    initmicroROSSemaphore = xSemaphoreCreateBinary();

    microROSInit();

    xSemaphoreGive(initmicroROSSemaphore);

    for (;;)
    {
        ledBuiltIn.blink(300);

        checkLimitSwitch();

        volatile unsigned long positionTime = millis();
        trolleyPosition = encoderTrolley.getPosition();
        trolleySpeed = (trolleyPosition - lastTrolleyPosition) / (positionTime - lastPositionTime) * 1000.0f;

        lastTrolleyPosition = trolleyPosition;
        lastPositionTime = positionTime;

        RCSOFTCHECK(rclc_executor_spin_some(&positionPubExecutor, RCL_MS_TO_NS(POSITION_PUBLISH_TIMEOUT_MS)));
        RCSOFTCHECK(rclc_executor_spin_some(&controllerCommandExecutor, RCL_MS_TO_NS(CONTROLLER_COMMAND_SUBSCRIBER_TIMEOUT_MS)));
        RCSOFTCHECK(rclc_executor_spin_some(&trolleyMotorVoltageExecutor, RCL_MS_TO_NS(TROLLEY_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
        RCSOFTCHECK(rclc_executor_spin_some(&hoistMotorVoltageExecutor, RCL_MS_TO_NS(HOIST_MOTOR_VOLTAGE_PUBLISH_TIMEOUT_MS)));
    }
}

void findOrigin(void *parameter)
{
    
    while (limitSwitchEncoderSide.getState() != LOW)
    {
        ledBuiltIn.blink(100);
        Serial.println("Finding origin");
        trolleyMotorPWM = -TROLLEY_MOTOR_FIND_ORIGIN_PWM;
        trolleyMotor.setPWM(trolleyMotorPWM);
    }

    trolleyMotorPWM = 0;
    trolleyMotor.setPWM(trolleyMotorPWM);

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 0; i < 2; i++)
    {
        for (int i = -0.8 * TROLLEY_MOTOR_FIND_ORIGIN_PWM; i > -(TROLLEY_MOTOR_FIND_ORIGIN_PWM + 50); i--)
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

void controllerCommandTask(void *parameter)
{
    while (xSemaphoreTake(initmicroROSSemaphore, portMAX_DELAY) != pdTRUE)
    {
        // Wait for microROS to be initialized
    }

    findOrigin(NULL);

    gantryMode = IDLE_MODE;

    for (;;)
    {
        if (brakeTrolleyMotor)
        {
            trolleyMotorPWM = 0;
            trolleyMotor.setPWM(trolleyMotorPWM);
            trolleyMotor.brake();
            brakeTrolleyMotor = false;
        }

        if (brakeHoistMotor)
        {
            hoistMotorPWM = 0;
            hoistMotor.setPWM(hoistMotorPWM);
            hoistMotor.brake();
            brakeHoistMotor = false;
        }

        if (gantryMode == IDLE_MODE)
        {
            // Do nothing
            trolleyMotorPWM = 0;
            hoistMotorPWM = 0;
            lastTrolleyMotorVoltage = 0;
            lastHoistMotorVoltage = 0;
        }

        else if (gantryMode == CONTROL_MODE)
        {
            // Control gantry
            u_int32_t current_time = millis();
            if (current_time - controller_command_last_call_time > CONTROLLER_COMMAND_TIMEOUT_MS)
            {
                gantryMode = IDLE_MODE;
            }
        }
        else if (gantryMode == LOCK_CONTAINER_MODE)
        {
            // Lock container
            // Add code specific to LOCK_CONTAINER_MODE
        }
        else if (gantryMode == UNLOCK_CONTAINER_MODE)
        {
            // Unlock container
            // Add code specific to UNLOCK_CONTAINER_MODE
        }
        else if (gantryMode == MOVE_TO_ORIGIN_MODE)
        {
            if (limitSwitchEncoderSide.getState() != LOW)
            {
                trolleyMotorPWM = -TROLLEY_MOTOR_FIND_ORIGIN_PWM;
                hoistMotorPWM = 0;
            }
            else
            {
                trolleyMotorPWM = 0;
                gantryMode = IDLE_MODE;
            }
        }
        else if (gantryMode == MOVE_TO_MIDDLE_MODE)
        {
            trolleyMotorPWM = TROLLEY_MOTOR_FIND_ORIGIN_PWM * get_sign(ENCODER_MAX_VALUE / 2 - encoderTrolley.getPulse());
            hoistMotorPWM = 0;

            if (fabs(encoderTrolley.getPulse() - ENCODER_MAX_VALUE / 2) < 0.01 * ENCODER_MAX_VALUE)
            {
                trolleyMotorPWM = 0;
                gantryMode = IDLE_MODE;
            }
        }

        else if (gantryMode == MOVE_TO_END_MODE)
        {
            if (limitSwitchTrolleyMotorSide.getState() != LOW)
            {
                trolleyMotorPWM = TROLLEY_MOTOR_FIND_ORIGIN_PWM;
                hoistMotorPWM = 0;
            }
            else
            {
                trolleyMotorPWM = 0;
                gantryMode = IDLE_MODE;
            }
        }
        else
        {
            gantryMode = IDLE_MODE;
        }

        trolleyMotor.setPWM(trolleyMotorPWM);
        hoistMotor.setPWM(hoistMotorPWM);

        trolleyMotorVoltage = readChannel(ADS1115_COMP_0_GND);
        hoistMotorVoltage = readChannel(ADS1115_COMP_1_GND);
    }
}

#endif // FREERTOS_TASK_HPP