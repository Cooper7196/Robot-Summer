#pragma once

#include "pwm_expander.h"
#include <Arduino.h>

class Motor {
public:
  Motor(PwmExpander &pwmExpander, uint8_t forwardChannel,
        uint8_t reverseChannel);

  void setSpeedPercent(float percent);
  void forward(float percent);
  void reverse(float percent);
  void stop();

private:
  void drive(uint8_t activeChannel, uint8_t inactiveChannel, float percent);

  PwmExpander *pwmExpander_;
  uint8_t forwardChannel_;
  uint8_t reverseChannel_;
};
