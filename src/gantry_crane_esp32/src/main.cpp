#include <Arduino.h>

#include "parameter.h"
#include "component.h"
#include "microROS.h"

void setup()
{
  trolleyMotor.begin();
  hoistMotor.begin();

  encoderTrolley.begin();
  attachInterrupt(encoderTrolley.getChannelA(), encoderTrolleyCallback, CHANGE);

  microROSInit();
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&encoderPubExecutor, RCL_MS_TO_NS(ENCODER_PUBLISH_TIMEOUT_NS)));
  RCSOFTCHECK(rclc_executor_spin_some(&trolleyPWMExecutor, RCL_MS_TO_NS(TROLLEY_PWM_SUBSCRIBER_TIMEOUT_NS)));
  RCSOFTCHECK(rclc_executor_spin_some(&hoistPWMExecutor, RCL_MS_TO_NS(HOIST_PWM_SUBSCRIBER_TIMEOUT_NS)));
}