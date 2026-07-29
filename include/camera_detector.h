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
// Positive results are counted so callers can reset an observation window and
// query its count without ever blocking on UART input.
class CameraDetector {
public:
  static constexpr size_t MAX_LABEL_LENGTH = 15;

  struct Config {
    uint32_t baudRate = 115200;
    uint32_t taskStackSize = 3072;
    UBaseType_t taskPriority = 1;
  };

  explicit CameraDetector(HardwareSerial &serial);
  CameraDetector(HardwareSerial &serial, const Config &config);

  bool begin(uint8_t rxPin, uint8_t txPin);
  bool initialized() const;

  // Every valid RESULT,<label>,<confidence> line increments this count.
  // RESULT,none is parsed but does not increment it.
  uint32_t positiveCount() const;

  // Starts a fresh logical observation window while UART parsing continues.
  void resetPositiveCount();

private:
  static constexpr size_t MAX_LINE_LENGTH = 63;

  HardwareSerial *serial_;
  Config config_;
  uint32_t positiveCount_;
  char lineBuffer_[MAX_LINE_LENGTH + 1];
  size_t lineLength_;
  bool droppingLongLine_;
  bool initialized_;
  mutable SemaphoreHandle_t stateMutex_;
  TaskHandle_t taskHandle_;

  static void taskEntry(void *context);
  void run();
  void consume(char character);
  bool parseLine(bool *positive);
};
