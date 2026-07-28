#include "camera_detector.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

CameraDetector::CameraDetector(HardwareSerial &serial)
    : CameraDetector(serial, Config()) {}

CameraDetector::CameraDetector(HardwareSerial &serial, const Config &config)
    : serial_(&serial), config_(config), history_{}, historyStart_(0),
      historyCount_(0), lineBuffer_{}, lineLength_(0),
      droppingLongLine_(false), initialized_(false), stateMutex_(nullptr),
      taskHandle_(nullptr) {}

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
      Sample sample;
      if (parseLine(&sample)) {
        storeSample(sample);
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

bool CameraDetector::parseLine(Sample *sample) {
  if (sample == nullptr) {
    return false;
  }

  constexpr char prefix[] = "RESULT,";
  if (strncmp(lineBuffer_, prefix, sizeof(prefix) - 1) != 0) {
    return false;
  }

  char *payload = lineBuffer_ + sizeof(prefix) - 1;
  if (strcmp(payload, "none") == 0) {
    *sample = Sample();
    sample->timestampMs = millis();
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
  if (parseEnd == separator + 1 || *parseEnd != '\0' ||
      !isfinite(confidence) || confidence < 0.0f || confidence > 1.0f) {
    return false;
  }

  *sample = Sample();
  memcpy(sample->label, payload, labelLength);
  sample->label[labelLength] = '\0';
  sample->confidence = confidence;
  sample->timestampMs = millis();
  sample->detected = true;
  return true;
}

void CameraDetector::storeSample(const Sample &sample) {
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  uint8_t writeIndex = 0;
  if (historyCount_ < MAX_HISTORY_SAMPLES) {
    writeIndex =
        static_cast<uint8_t>((historyStart_ + historyCount_) %
                             MAX_HISTORY_SAMPLES);
    ++historyCount_;
  } else {
    writeIndex = historyStart_;
    historyStart_ =
        static_cast<uint8_t>((historyStart_ + 1) % MAX_HISTORY_SAMPLES);
  }
  history_[writeIndex] = sample;
  xSemaphoreGive(stateMutex_);
}

bool CameraDetector::getConsensus(Consensus *consensus,
                                  uint8_t latestSampleCount,
                                  uint8_t requiredDetections,
                                  float minConfidence,
                                  uint32_t maxAgeMs) const {
  if (consensus == nullptr || stateMutex_ == nullptr ||
      latestSampleCount == 0 || requiredDetections == 0 ||
      requiredDetections > latestSampleCount || minConfidence < 0.0f ||
      minConfidence > 1.0f) {
    return false;
  }

  *consensus = Consensus();
  const uint32_t nowMs = millis();

  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const uint8_t count =
      historyCount_ < latestSampleCount ? historyCount_ : latestSampleCount;
  for (uint8_t offset = 0; offset < count; ++offset) {
    const uint8_t index = static_cast<uint8_t>(
        (historyStart_ + historyCount_ - 1 - offset) % MAX_HISTORY_SAMPLES);
    const Sample &sample = history_[index];
    if (maxAgeMs != 0 &&
        static_cast<uint32_t>(nowMs - sample.timestampMs) > maxAgeMs) {
      continue;
    }

    ++consensus->samplesConsidered;
    if (!sample.detected || sample.confidence < minConfidence) {
      continue;
    }

    ++consensus->detections;
    consensus->averageConfidence += sample.confidence;
    if (sample.confidence > consensus->maximumConfidence) {
      consensus->maximumConfidence = sample.confidence;
      memcpy(consensus->label, sample.label, sizeof(consensus->label));
    }
  }
  xSemaphoreGive(stateMutex_);

  if (consensus->detections > 0) {
    consensus->averageConfidence /= consensus->detections;
  }
  consensus->detected = consensus->detections >= requiredDetections;
  return true;
}

bool CameraDetector::detectionConfirmed(uint8_t latestSampleCount,
                                        uint8_t requiredDetections,
                                        float minConfidence,
                                        uint32_t maxAgeMs) const {
  Consensus consensus;
  return getConsensus(&consensus, latestSampleCount, requiredDetections,
                      minConfidence, maxAgeMs) &&
         consensus.detected;
}

bool CameraDetector::getLatestSample(Sample *sample) const {
  if (sample == nullptr || stateMutex_ == nullptr) {
    return false;
  }

  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  if (historyCount_ == 0) {
    xSemaphoreGive(stateMutex_);
    return false;
  }
  const uint8_t index = static_cast<uint8_t>(
      (historyStart_ + historyCount_ - 1) % MAX_HISTORY_SAMPLES);
  *sample = history_[index];
  xSemaphoreGive(stateMutex_);
  return true;
}

uint8_t CameraDetector::sampleCount() const {
  if (stateMutex_ == nullptr) {
    return 0;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const uint8_t count = historyCount_;
  xSemaphoreGive(stateMutex_);
  return count;
}

void CameraDetector::resetSamples() {
  if (stateMutex_ == nullptr) {
    return;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  historyStart_ = 0;
  historyCount_ = 0;
  xSemaphoreGive(stateMutex_);
}
