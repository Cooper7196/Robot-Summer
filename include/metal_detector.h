#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// Measures the frequency of a logic-level signal with the ESP32 PCNT
// peripheral. begin() starts an instance-owned periodic task; pulse counting
// happens in hardware and no application polling is required.
class MetalDetector {
public:
  static constexpr uint8_t MAX_DEVIATION_AVERAGE_SAMPLES = 16;

  struct Config {
    // A shorter window responds faster; a longer window gives finer frequency
    // resolution. Keep the expected pulse count below MAX_PULSES_PER_SAMPLE.
    // At 10 ms the 16-bit PCNT range covers approximately 3.27 MHz. One
    // hundred calibration samples still gives a one-second startup baseline.
    uint32_t sampleWindowMs = 100;
    uint16_t calibrationSamples = 10;
    float baselineTimeConstantSeconds = 20000.0f;
    // Metal is detected when the absolute difference between the measured
    // frequency and baseline is at least this many hertz.
    float anomalyThresholdHz = 100.0f;
    // Require this many consecutive usable deviations and compare their
    // rolling mean against the anomaly threshold. More samples reject brief
    // spikes, at the cost of sampleWindowMs of additional response time each.
    uint8_t deviationAverageSamples = 3;
    float clearThresholdFraction = 0.70f;

    // Pulses shorter than this many APB clock cycles are rejected. Zero
    // disables the PCNT glitch filter; the legacy ESP32 driver permits 1-1023.
    uint16_t glitchFilterCycles = 8;

    uint32_t taskStackSize = 2048;
    UBaseType_t taskPriority = 1;
  };

  struct Reading {
    float frequencyHz = 0.0f;
    float baselineHz = 0.0f;
    float deviationHz = 0.0f;
    float averagedDeviationHz = 0.0f;
    float relativeDeviation = 0.0f;
    uint8_t averagedSampleCount = 0;
    uint32_t sampleTimeUs = 0;
    int16_t pulseCount = 0;
    bool baselineReady = false;
    bool anomaly = false;
    bool counterSaturated = false;
  };

  static constexpr int16_t MAX_PULSES_PER_SAMPLE = 32767;

  MetalDetector(uint8_t inputPin, pcnt_unit_t unit);
  MetalDetector(uint8_t inputPin, pcnt_unit_t unit, const Config &config);

  bool begin();

  // Thread-safe access to the most recent sample produced by this detector's
  // background task.
  Reading reading() const;
  bool getReading(Reading *reading) const;
  float frequencyHz() const;
  float baselineHz() const;
  float deviationHz() const;
  bool anomalyDetected() const;
  bool baselineReady() const;
  bool initialized() const;

  // Starts a fresh baseline calibration without stopping the PCNT counter.
  void resetBaseline();

private:
  uint8_t inputPin_;
  pcnt_unit_t unit_;
  Config config_;
  Reading reading_;
  int64_t sampleStartUs_;
  uint16_t calibrationCount_;
  float deviationSamples_[MAX_DEVIATION_AVERAGE_SAMPLES];
  float deviationSumHz_;
  uint8_t deviationSampleCount_;
  uint8_t deviationSampleIndex_;
  bool initialized_;
  mutable SemaphoreHandle_t stateMutex_;
  TaskHandle_t taskHandle_;

  static void taskEntry(void *context);
  void run();
  bool update();
};
