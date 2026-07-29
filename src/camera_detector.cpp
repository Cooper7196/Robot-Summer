#include "camera_detector.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

CameraDetector::CameraDetector(HardwareSerial &serial)
    : CameraDetector(serial, Config()) {}

CameraDetector::CameraDetector(HardwareSerial &serial, const Config &config)
    : serial_(&serial), config_(config), positiveCount_(0), lineBuffer_{},
      lineLength_(0), droppingLongLine_(false), initialized_(false),
      stateMutex_(nullptr), taskHandle_(nullptr) {}

bool CameraDetector::begin(uint8_t rxPin, uint8_t txPin) {
  if (initialized_ || serial_ == nullptr || config_.baudRate == 0 ||
      config_.taskStackSize == 0) {
    return false;
  }

  stateMutex_ = xSemaphoreCreateMutex();
  if (stateMutex_ == nullptr) {
    return false;
  }

  serial_->begin(config_.baudRate, SERIAL_8N1, rxPin, txPin);
  initialized_ = true;
  if (xTaskCreate(taskEntry, "camera-rx", config_.taskStackSize, this,
                  config_.taskPriority, &taskHandle_) != pdPASS) {
    initialized_ = false;
    serial_->end();
    vSemaphoreDelete(stateMutex_);
    stateMutex_ = nullptr;
    return false;
  }

  return true;
}

bool CameraDetector::initialized() const { return initialized_; }

void CameraDetector::taskEntry(void *context) {
  static_cast<CameraDetector *>(context)->run();
}

void CameraDetector::run() {
  for (;;) {
    while (serial_->available() > 0) {
      const int value = serial_->read();
      if (value >= 0) {
        consume(static_cast<char>(value));
      }
    }
    vTaskDelay(1);
  }
}

void CameraDetector::consume(char character) {
  if (character == '\n') {
    if (!droppingLongLine_ && lineLength_ > 0) {
      lineBuffer_[lineLength_] = '\0';
      bool positive = false;
      if (parseLine(&positive) && positive) {
        xSemaphoreTake(stateMutex_, portMAX_DELAY);
        if (positiveCount_ != UINT32_MAX) {
          ++positiveCount_;
        }
        xSemaphoreGive(stateMutex_);
      }
    }
    lineLength_ = 0;
    droppingLongLine_ = false;
    return;
  }

  if (character == '\r' || droppingLongLine_) {
    return;
  }

  if (lineLength_ >= MAX_LINE_LENGTH) {
    lineLength_ = 0;
    droppingLongLine_ = true;
    return;
  }

  lineBuffer_[lineLength_++] = character;
}

bool CameraDetector::parseLine(bool *positive) {
  if (positive == nullptr) {
    return false;
  }
  *positive = false;

  constexpr char prefix[] = "RESULT,";
  if (strncmp(lineBuffer_, prefix, sizeof(prefix) - 1) != 0) {
    return false;
  }

  char *payload = lineBuffer_ + sizeof(prefix) - 1;
  if (strcmp(payload, "none") == 0) {
    return true;
  }

  char *separator = strchr(payload, ',');
  if (separator == nullptr || separator == payload ||
      strchr(separator + 1, ',') != nullptr) {
    return false;
  }

  const size_t labelLength = static_cast<size_t>(separator - payload);
  if (labelLength > MAX_LABEL_LENGTH) {
    return false;
  }

  *separator = '\0';
  char *parseEnd = nullptr;
  const float confidence = strtof(separator + 1, &parseEnd);
  if (parseEnd == separator + 1 || *parseEnd != '\0' || !isfinite(confidence) ||
      confidence < 0.0f || confidence > 1.0f) {
    return false;
  }

  *positive = true;
  return true;
}

uint32_t CameraDetector::positiveCount() const {
  if (stateMutex_ == nullptr) {
    return 0;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const uint32_t count = positiveCount_;
  xSemaphoreGive(stateMutex_);
  return count;
}

void CameraDetector::resetPositiveCount() {
  if (stateMutex_ == nullptr) {
    return;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  positiveCount_ = 0;
  xSemaphoreGive(stateMutex_);
}
