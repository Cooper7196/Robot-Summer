#pragma once

#include "pins.h"
#include <Arduino.h>
#include <Wire.h>

class As5600Encoder {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x36;
  static constexpr uint8_t DEFAULT_MUX_ADDRESS = 0x70;
  static constexpr uint8_t NO_MUX_CHANNEL = 0xFF;
  static constexpr uint16_t COUNTS_PER_REVOLUTION = 4096;

  explicit As5600Encoder(uint8_t muxChannel = NO_MUX_CHANNEL,
                         uint8_t address = DEFAULT_ADDRESS,
                         uint8_t muxAddress = DEFAULT_MUX_ADDRESS,
                         TwoWire &wire = Wire1);

  bool begin(uint8_t sdaPin = pins::ENCODER_I2C_SDA_PIN,
             uint8_t sclPin = pins::ENCODER_I2C_SCL_PIN,
             uint32_t i2cClockHz = 100000);
  bool isConnected();
  bool muxIsConnected();
  bool magnetDetected();

  bool readRawAngle(uint16_t *angle);
  uint16_t readRawAngle();
  float readAngleDegrees();
  float readAngleRadians();

  uint8_t muxChannel() const;
  bool usesMux() const;

private:
  static constexpr uint8_t RAW_ANGLE_REGISTER = 0x0C;
  static constexpr uint8_t STATUS_REGISTER = 0x0B;
  static constexpr uint8_t MAGNET_DETECTED_MASK = 0x20;

  TwoWire *wire_;
  uint8_t address_;
  uint8_t muxAddress_;
  uint8_t muxChannel_;
  bool initialized_;

  bool selectMuxChannel();
  bool readRegister(uint8_t reg, uint8_t *value);
  bool readRegister16(uint8_t reg, uint16_t *value);
};
