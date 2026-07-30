#include "pwm_expander.h"

PwmExpander::PwmExpander(uint8_t address, TwoWire &wire)
    : address_(address), wire_(&wire), driver_(address, wire),
      initialized_(false), voltageNormalizationEnabled_(false),
      voltageSensePin_(0), voltageDividerRatio_(1.0f), targetVoltage_(0.0f),
      supplyVoltage_(0.0f), lastVoltageSampleMs_(0), voltageMutex_(nullptr) {}

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

void PwmExpander::enableVoltageNormalization(uint8_t sensePin,
                                             float dividerRatio,
                                             float targetVoltage) {
  if (dividerRatio <= 0.0f || targetVoltage <= 0.0f) {
    voltageNormalizationEnabled_ = false;
    return;
  }

  if (voltageMutex_ == nullptr) {
    voltageMutex_ = xSemaphoreCreateMutex();
  }
  if (voltageMutex_ == nullptr) {
    voltageNormalizationEnabled_ = false;
    return;
  }

  xSemaphoreTake(voltageMutex_, portMAX_DELAY);
  voltageSensePin_ = sensePin;
  voltageDividerRatio_ = dividerRatio;
  targetVoltage_ = targetVoltage;
  supplyVoltage_ = 0.0f;
  lastVoltageSampleMs_ = 0;
  pinMode(voltageSensePin_, INPUT);
  analogSetPinAttenuation(voltageSensePin_, ADC_11db);
  voltageNormalizationEnabled_ = true;
  xSemaphoreGive(voltageMutex_);
  updateSupplyVoltage();
}

float PwmExpander::supplyVoltage() { return updateSupplyVoltage(); }

float PwmExpander::updateSupplyVoltage() {
  if (voltageMutex_ == nullptr) {
    return 0.0f;
  }

  xSemaphoreTake(voltageMutex_, portMAX_DELAY);
  if (!voltageNormalizationEnabled_) {
    const float voltage = supplyVoltage_;
    xSemaphoreGive(voltageMutex_);
    return voltage;
  }

  constexpr uint32_t kSampleIntervalMs = 20;
  constexpr uint8_t kSamplesPerReading = 4;
  const uint32_t nowMs = millis();
  if (supplyVoltage_ > 0.0f &&
      nowMs - lastVoltageSampleMs_ < kSampleIntervalMs) {
    const float voltage = supplyVoltage_;
    xSemaphoreGive(voltageMutex_);
    return voltage;
  }

  uint32_t senseMillivolts = 0;
  for (uint8_t sample = 0; sample < kSamplesPerReading; ++sample) {
    senseMillivolts += analogReadMilliVolts(voltageSensePin_);
  }
  senseMillivolts /= kSamplesPerReading;
  lastVoltageSampleMs_ = nowMs;

  // Retain the previous valid reading through an isolated bad ADC sample.
  if (senseMillivolts > 0) {
    supplyVoltage_ =
        (static_cast<float>(senseMillivolts) / 1000.0f) * voltageDividerRatio_;
  }
  const float voltage = supplyVoltage_;
  xSemaphoreGive(voltageMutex_);
  return voltage;
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

  const float voltage = updateSupplyVoltage();
  if (voltageNormalizationEnabled_ && voltage > 0.0f) {
    // A requested percentage represents that fraction of the target voltage.
    // Duty is capped below when the battery cannot produce the target voltage.
    percent *= targetVoltage_ / voltage;
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
