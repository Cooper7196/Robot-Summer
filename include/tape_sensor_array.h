#pragma once

#include "pins.h"
#include <Adafruit_MCP3008.h>
#include <Arduino.h>
#include <SPI.h>

class TapeSensorArray {
public:
  static constexpr uint8_t CHANNEL_COUNT = 8;
  static constexpr uint16_t MAX_READING = 1023;
  static constexpr uint16_t DEFAULT_THRESHOLD = 512;

  enum class TapePolarity : uint8_t {
    DarkTapeIsLow,
    LightTapeIsHigh,
  };

  struct SpiDiagnostic {
    bool initialized = false;
    bool validChannel = false;
    uint16_t reading = 0;
  };

  explicit TapeSensorArray(SPIClass &spi = SPI,
                           uint8_t chipSelectPin = pins::TAPE_CS_PIN);

  bool begin(uint8_t sckPin = pins::TAPE_SCK_PIN,
             uint8_t misoPin = pins::TAPE_MISO_PIN,
             uint8_t mosiPin = pins::TAPE_MOSI_PIN);

  uint16_t readChannel(uint8_t channel);
  void readAll(uint16_t readings[CHANNEL_COUNT]);
  bool readDiagnostic(uint8_t channel, SpiDiagnostic *diagnostic);
  void printSpiDiagnostic(Stream &stream);

  void setThreshold(uint16_t threshold);
  void setThreshold(uint8_t channel, uint16_t threshold);
  uint16_t threshold(uint8_t channel) const;

  void setTapePolarity(TapePolarity polarity);
  bool seesTape(uint8_t channel);
  uint8_t tapeMask();

private:
  Adafruit_MCP3008 adc_;
  uint8_t chipSelectPin_;
  bool initialized_;
  uint16_t thresholds_[CHANNEL_COUNT];
  TapePolarity tapePolarity_;

  bool validChannel(uint8_t channel) const;
};
