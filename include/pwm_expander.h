#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

class PwmExpander {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x40;
  static constexpr uint16_t MAX_PWM = 4095;

  explicit PwmExpander(uint8_t address = DEFAULT_ADDRESS, TwoWire &wire = Wire);

  bool begin(uint8_t sdaPin, uint8_t sclPin, float pwmFrequencyHz = 1600.0f,
             uint32_t i2cClockHz = 100000);
  bool isConnected() const;

  void setChannel(uint8_t channel, uint16_t value);
  void setChannelPercent(uint8_t channel, float percent);
  void setChannelOff(uint8_t channel);
  void setAllOff();

private:
  uint8_t address_;
  TwoWire *wire_;
  Adafruit_PWMServoDriver driver_;
  bool initialized_;
};
