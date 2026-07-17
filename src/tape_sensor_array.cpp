#include "tape_sensor_array.h"

TapeSensorArray::TapeSensorArray(SPIClass &spi, uint8_t chipSelectPin)
    : adc_(), chipSelectPin_(chipSelectPin), initialized_(false), thresholds_{},
      tapePolarity_(TapePolarity::DarkTapeIsLow) {
  (void)spi;
  setThreshold(DEFAULT_THRESHOLD);
}

bool TapeSensorArray::begin(uint8_t sckPin, uint8_t misoPin,
                            uint8_t mosiPin) {
  initialized_ = adc_.begin(sckPin, mosiPin, misoPin, chipSelectPin_);
  return initialized_;
}

uint16_t TapeSensorArray::readChannel(uint8_t channel) {
  SpiDiagnostic diagnostic;
  if (!readDiagnostic(channel, &diagnostic)) {
    return 0;
  }

  return diagnostic.reading;
}

bool TapeSensorArray::readDiagnostic(uint8_t channel,
                                     SpiDiagnostic *diagnostic) {
  if (diagnostic == nullptr) {
    return false;
  }

  diagnostic->initialized = initialized_;
  diagnostic->validChannel = validChannel(channel);
  diagnostic->reading = 0;

  if (!diagnostic->initialized || !diagnostic->validChannel) {
    return false;
  }

  diagnostic->reading = static_cast<uint16_t>(adc_.readADC(channel));
  return true;
}

void TapeSensorArray::readAll(uint16_t readings[CHANNEL_COUNT]) {
  if (readings == nullptr) {
    return;
  }

  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    readings[channel] = readChannel(channel);
  }
}

void TapeSensorArray::printSpiDiagnostic(Stream &stream) {
  bool allZero = true;
  bool allMax = true;

  stream.println("MCP3008 Adafruit library diagnostic");
  stream.printf("begin=%s cs=%u sck=%u miso=%u mosi=%u\n",
                initialized_ ? "ok" : "failed", chipSelectPin_,
                pins::TAPE_SCK_PIN, pins::TAPE_MISO_PIN, pins::TAPE_MOSI_PIN);
  stream.println("Tie CH0 to 3V3/VREF for this test; ch0 should read near 1023.");

  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    SpiDiagnostic diagnostic;
    if (!readDiagnostic(channel, &diagnostic)) {
      stream.printf("ch%u unavailable initialized=%u valid=%u\n", channel,
                    diagnostic.initialized, diagnostic.validChannel);
      continue;
    }

    allZero = allZero && diagnostic.reading == 0;
    allMax = allMax && diagnostic.reading == MAX_READING;
    stream.printf("ch%u adc=%u\n", channel, diagnostic.reading);
  }

  if (allZero) {
    stream.println("All readings are 0: check MCP3008 VDD/VREF/AGND/DGND, "
                   "MISO/DOUT, CS, SCK, MOSI/DIN, and whether inputs are 0V.");
  } else if (allMax) {
    stream.println("All readings are 1023: MISO may be stuck high or all inputs "
                   "may be tied to VREF.");
  } else {
    stream.println("SPI is returning varying ADC data.");
  }
}

void TapeSensorArray::setThreshold(uint16_t threshold) {
  if (threshold > MAX_READING) {
    threshold = MAX_READING;
  }

  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    thresholds_[channel] = threshold;
  }
}

void TapeSensorArray::setThreshold(uint8_t channel, uint16_t threshold) {
  if (!validChannel(channel)) {
    return;
  }

  thresholds_[channel] = threshold > MAX_READING ? MAX_READING : threshold;
}

uint16_t TapeSensorArray::threshold(uint8_t channel) const {
  if (!validChannel(channel)) {
    return 0;
  }

  return thresholds_[channel];
}

void TapeSensorArray::setTapePolarity(TapePolarity polarity) {
  tapePolarity_ = polarity;
}

bool TapeSensorArray::seesTape(uint8_t channel) {
  if (!validChannel(channel)) {
    return false;
  }

  const uint16_t reading = readChannel(channel);
  if (tapePolarity_ == TapePolarity::DarkTapeIsLow) {
    return reading <= thresholds_[channel];
  }

  return reading >= thresholds_[channel];
}

uint8_t TapeSensorArray::tapeMask() {
  uint8_t mask = 0;

  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    if (seesTape(channel)) {
      mask |= static_cast<uint8_t>(1U << channel);
    }
  }

  return mask;
}

bool TapeSensorArray::validChannel(uint8_t channel) const {
  return channel < CHANNEL_COUNT;
}
