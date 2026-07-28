#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// Continuously parses newline-delimited camera results on a background task.
//
// Accepted lines:
//   RESULT,<label>,<confidence>
//   RESULT,none
//
// Parsed samples are retained in a thread-safe ring buffer so callers can
// inspect several recent results without ever blocking on UART input.
class CameraDetector {
public:
  static constexpr uint8_t MAX_HISTORY_SAMPLES = 24;
  static constexpr size_t MAX_LABEL_LENGTH = 15;

  struct Config {
    uint32_t baudRate = 115200;
    uint32_t taskStackSize = 3072;
    UBaseType_t taskPriority = 1;
  };

  struct Sample {
    char label[MAX_LABEL_LENGTH + 1] = {};
    float confidence = 0.0f;
    uint32_t timestampMs = 0;
    bool detected = false;
  };

  struct Consensus {
    char label[MAX_LABEL_LENGTH + 1] = {};
    float averageConfidence = 0.0f;
    float maximumConfidence = 0.0f;
    uint8_t samplesConsidered = 0;
    uint8_t detections = 0;
    bool detected = false;
  };

  explicit CameraDetector(HardwareSerial &serial);
  CameraDetector(HardwareSerial &serial, const Config &config);

  bool begin(uint8_t rxPin, uint8_t txPin);
  bool initialized() const;

  // Examines at most latestSampleCount newest samples. A detection is
  // confirmed when requiredDetections samples meet minConfidence. Samples
  // older than maxAgeMs are ignored; pass 0 to disable the age limit.
  bool getConsensus(Consensus *consensus, uint8_t latestSampleCount = 5,
                    uint8_t requiredDetections = 3,
                    float minConfidence = 0.50f,
                    uint32_t maxAgeMs = 1000) const;
  bool detectionConfirmed(uint8_t latestSampleCount = 5,
                          uint8_t requiredDetections = 3,
                          float minConfidence = 0.50f,
                          uint32_t maxAgeMs = 1000) const;

  bool getLatestSample(Sample *sample) const;
  uint8_t sampleCount() const;

  // Starts a fresh logical observation window while UART parsing continues.
  // Call this when the robot reaches a location where detections matter.
  void resetSamples();

private:
  static constexpr size_t MAX_LINE_LENGTH = 63;

  HardwareSerial *serial_;
  Config config_;
  Sample history_[MAX_HISTORY_SAMPLES];
  uint8_t historyStart_;
  uint8_t historyCount_;
  char lineBuffer_[MAX_LINE_LENGTH + 1];
  size_t lineLength_;
  bool droppingLongLine_;
  bool initialized_;
  mutable SemaphoreHandle_t stateMutex_;
  TaskHandle_t taskHandle_;

  static void taskEntry(void *context);
  void run();
  void consume(char character);
  bool parseLine(Sample *sample);
  void storeSample(const Sample &sample);
};
