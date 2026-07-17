#pragma once

#include <Arduino.h>

class AngleServo {
public:
  struct Config {
    float minAngleDeg;
    float maxAngleDeg;
    uint16_t minPulseUs;
    uint16_t maxPulseUs;
    uint16_t frequencyHz;
    uint8_t resolutionBits;

    Config(float minAngleDeg = 0.0f, float maxAngleDeg = 180.0f,
           uint16_t minPulseUs = 500, uint16_t maxPulseUs = 2500,
           uint16_t frequencyHz = 50, uint8_t resolutionBits = 14)
        : minAngleDeg(minAngleDeg), maxAngleDeg(maxAngleDeg),
          minPulseUs(minPulseUs), maxPulseUs(maxPulseUs),
          frequencyHz(frequencyHz), resolutionBits(resolutionBits) {}
  };

  explicit AngleServo(uint8_t pin, uint8_t ledcChannel = 0,
                      const Config &config = Config());

  bool begin(float initialAngleDeg = 90.0f);
  bool setAngle(float angleDeg);
  void disable();

  float commandedAngle() const;
  bool initialized() const;

private:
  static float clamp(float value, float minValue, float maxValue);

  uint8_t pin_;
  uint8_t ledcChannel_;
  Config config_;
  float commandedAngleDeg_;
  bool initialized_;
};
