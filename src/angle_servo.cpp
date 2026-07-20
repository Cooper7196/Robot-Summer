#include "angle_servo.h"

AngleServo::AngleServo(uint8_t pin, uint8_t ledcChannel, const Config &config)
    : pin_(pin), ledcChannel_(ledcChannel), config_(config),
      commandedAngleDeg_(0.0f), initialized_(false), outputEnabled_(false) {}

bool AngleServo::begin(float initialAngleDeg) {
  if (config_.minAngleDeg >= config_.maxAngleDeg ||
      config_.minPulseUs >= config_.maxPulseUs || config_.frequencyHz == 0 ||
      config_.resolutionBits == 0 || config_.resolutionBits > 14) {
    return false;
  }

  if (ledcSetup(ledcChannel_, config_.frequencyHz, config_.resolutionBits) <=
      0.0) {
    return false;
  }

  ledcAttachPin(pin_, ledcChannel_);
  initialized_ = true;
  outputEnabled_ = true;
  return setAngle(initialAngleDeg);
}

bool AngleServo::setAngle(float angleDeg) {
  if (!initialized_ || !outputEnabled_ || !isfinite(angleDeg)) {
    return false;
  }

  commandedAngleDeg_ =
      clamp(angleDeg, config_.minAngleDeg, config_.maxAngleDeg);
  const float fraction = (commandedAngleDeg_ - config_.minAngleDeg) /
                         (config_.maxAngleDeg - config_.minAngleDeg);
  const float pulseUs = config_.minPulseUs +
                        (fraction * (config_.maxPulseUs - config_.minPulseUs));
  const uint32_t maxDuty = (1UL << config_.resolutionBits) - 1UL;
  const uint32_t duty = static_cast<uint32_t>(
      ((pulseUs * config_.frequencyHz) / 1000000.0f) * maxDuty + 0.5f);
  ledcWrite(ledcChannel_, duty);
  return true;
}

bool AngleServo::enable() {
  if (!initialized_) {
    return false;
  }

  if (!outputEnabled_) {
    ledcAttachPin(pin_, ledcChannel_);
    outputEnabled_ = true;
  }
  return setAngle(commandedAngleDeg_);
}

void AngleServo::disable() {
  if (initialized_ && outputEnabled_) {
    // Stop the pulse train before disconnecting LEDC from the physical pin.
    ledcWrite(ledcChannel_, 0);
    ledcDetachPin(pin_);
    pinMode(pin_, INPUT);
    outputEnabled_ = false;
  }
}

float AngleServo::commandedAngle() const { return commandedAngleDeg_; }

bool AngleServo::initialized() const { return initialized_; }

bool AngleServo::outputEnabled() const { return outputEnabled_; }

float AngleServo::clamp(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}
