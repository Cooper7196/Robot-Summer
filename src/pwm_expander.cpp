#include "pwm_expander.h"

PwmExpander::PwmExpander(uint8_t address, TwoWire &wire)
    : address_(address), wire_(&wire), driver_(address, wire),
      initialized_(false) {}

bool PwmExpander::begin(uint8_t sdaPin, uint8_t sclPin, float pwmFrequencyHz,
                        uint32_t i2cClockHz) {
  initialized_ = false;
  wire_->begin(sdaPin, sclPin);
  wire_->setClock(i2cClockHz);

  if (!isConnected()) {
    return false;
  }

  if (!driver_.begin()) {
    return false;
  }

  initialized_ = true;
  driver_.setPWMFreq(pwmFrequencyHz);
  setAllOff();
  return true;
}

bool PwmExpander::isConnected() const {
  wire_->beginTransmission(address_);
  return wire_->endTransmission() == 0;
}

void PwmExpander::setChannel(uint8_t channel, uint16_t value) {
  if (!initialized_) {
    return;
  }

  if (value >= MAX_PWM) {
    driver_.setPWM(channel, 4096, 0);
    return;
  }

  if (value == 0) {
    setChannelOff(channel);
    return;
  }

  driver_.setPWM(channel, 0, value);
}

void PwmExpander::setChannelPercent(uint8_t channel, float percent) {
  if (!initialized_) {
    return;
  }

  if (percent <= 0.0f) {
    setChannelOff(channel);
    return;
  }

  if (percent >= 100.0f) {
    setChannel(channel, MAX_PWM);
    return;
  }

  const uint16_t value = static_cast<uint16_t>((percent / 100.0f) * MAX_PWM);
  setChannel(channel, value);
}

void PwmExpander::setChannelOff(uint8_t channel) {
  if (!initialized_) {
    return;
  }

  driver_.setPWM(channel, 0, 0);
}

void PwmExpander::setAllOff() {
  if (!initialized_) {
    return;
  }

  for (uint8_t channel = 0; channel < 16; ++channel) {
    setChannelOff(channel);
  }
}
