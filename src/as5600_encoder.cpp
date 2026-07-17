#include "as5600_encoder.h"

As5600Encoder::As5600Encoder(uint8_t muxChannel, uint8_t address,
                             uint8_t muxAddress, TwoWire &wire)
    : wire_(&wire), address_(address), muxAddress_(muxAddress),
      muxChannel_(muxChannel), initialized_(false) {}

bool As5600Encoder::begin(uint8_t sdaPin, uint8_t sclPin,
                          uint32_t i2cClockHz) {
  wire_->begin(sdaPin, sclPin);
  wire_->setClock(i2cClockHz);
  initialized_ = isConnected();
  return initialized_;
}

bool As5600Encoder::isConnected() {
  if (!selectMuxChannel()) {
    return false;
  }

  wire_->beginTransmission(address_);
  return wire_->endTransmission() == 0;
}

bool As5600Encoder::muxIsConnected() {
  if (!usesMux()) {
    return false;
  }

  wire_->beginTransmission(muxAddress_);
  return wire_->endTransmission() == 0;
}

bool As5600Encoder::magnetDetected() {
  uint8_t status = 0;
  return readRegister(STATUS_REGISTER, &status) &&
         ((status & MAGNET_DETECTED_MASK) != 0);
}

bool As5600Encoder::readRawAngle(uint16_t *angle) {
  if (angle == nullptr) {
    return false;
  }

  uint16_t raw = 0;
  if (!readRegister16(RAW_ANGLE_REGISTER, &raw)) {
    return false;
  }

  *angle = raw & 0x0FFF;
  return true;
}

uint16_t As5600Encoder::readRawAngle() {
  uint16_t angle = 0;
  readRawAngle(&angle);
  return angle;
}

float As5600Encoder::readAngleDegrees() {
  uint16_t angle = 0;
  if (!readRawAngle(&angle)) {
    return NAN;
  }

  return (static_cast<float>(angle) * 360.0f) /
         static_cast<float>(COUNTS_PER_REVOLUTION);
}

float As5600Encoder::readAngleRadians() {
  uint16_t angle = 0;
  if (!readRawAngle(&angle)) {
    return NAN;
  }

  return (static_cast<float>(angle) * TWO_PI) /
         static_cast<float>(COUNTS_PER_REVOLUTION);
}

uint8_t As5600Encoder::muxChannel() const { return muxChannel_; }

bool As5600Encoder::usesMux() const { return muxChannel_ != NO_MUX_CHANNEL; }

bool As5600Encoder::selectMuxChannel() {
  if (!usesMux()) {
    return true;
  }

  if (muxChannel_ >= 8) {
    return false;
  }

  wire_->beginTransmission(muxAddress_);
  wire_->write(static_cast<uint8_t>(1U << muxChannel_));
  return wire_->endTransmission() == 0;
}

bool As5600Encoder::readRegister(uint8_t reg, uint8_t *value) {
  if (value == nullptr || !selectMuxChannel()) {
    return false;
  }

  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(true) != 0) {
    return false;
  }

  if (wire_->requestFrom(address_, static_cast<uint8_t>(1)) != 1) {
    return false;
  }

  *value = wire_->read();
  return true;
}

bool As5600Encoder::readRegister16(uint8_t reg, uint16_t *value) {
  if (value == nullptr || !selectMuxChannel()) {
    return false;
  }

  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(true) != 0) {
    return false;
  }

  if (wire_->requestFrom(address_, static_cast<uint8_t>(2)) != 2) {
    return false;
  }

  const uint8_t highByte = wire_->read();
  const uint8_t lowByte = wire_->read();
  *value = (static_cast<uint16_t>(highByte) << 8) | lowByte;
  return true;
}
