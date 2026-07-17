#include "motor.h"

Motor::Motor(PwmExpander &pwmExpander, uint8_t forwardChannel,
             uint8_t reverseChannel)
    : pwmExpander_(&pwmExpander), forwardChannel_(forwardChannel),
      reverseChannel_(reverseChannel) {}

void Motor::setSpeedPercent(float percent) {
  if (percent > 0.0f) {
    forward(percent);
    return;
  }

  if (percent < 0.0f) {
    reverse(-percent);
    return;
  }

  stop();
}

void Motor::forward(float percent) {
  drive(forwardChannel_, reverseChannel_, percent);
}

void Motor::reverse(float percent) {
  drive(reverseChannel_, forwardChannel_, percent);
}

void Motor::stop() {
  pwmExpander_->setChannelOff(forwardChannel_);
  pwmExpander_->setChannelOff(reverseChannel_);
}

void Motor::drive(uint8_t activeChannel, uint8_t inactiveChannel,
                  float percent) {
  if (percent <= 0.0f) {
    stop();
    return;
  }

  if (percent > 100.0f) {
    percent = 100.0f;
  }

  pwmExpander_->setChannelOff(activeChannel);
  pwmExpander_->setChannelOff(inactiveChannel);
  pwmExpander_->setChannelPercent(activeChannel, percent);
}
