#include "metal_detector.h"

#include <esp_timer.h>
#include <math.h>

MetalDetector::MetalDetector(uint8_t inputPin, pcnt_unit_t unit)
    : MetalDetector(inputPin, unit, Config()) {}

MetalDetector::MetalDetector(uint8_t inputPin, pcnt_unit_t unit,
                             const Config &config)
    : inputPin_(inputPin), unit_(unit), config_(config), reading_(),
      sampleStartUs_(0), calibrationCount_(0), deviationSamples_{},
      deviationSumHz_(0.0f), deviationSampleCount_(0),
      deviationSampleIndex_(0), initialized_(false), stateMutex_(nullptr),
      taskHandle_(nullptr) {}

bool MetalDetector::begin() {
  if (initialized_ || unit_ < PCNT_UNIT_0 || unit_ >= PCNT_UNIT_MAX ||
      config_.sampleWindowMs == 0 || config_.calibrationSamples == 0 ||
      config_.baselineTimeConstantSeconds <= 0.0f ||
      config_.anomalyThresholdHz < 0.0f ||
      config_.deviationAverageSamples == 0 ||
      config_.deviationAverageSamples > MAX_DEVIATION_AVERAGE_SAMPLES ||
      config_.clearThresholdFraction < 0.0f ||
      config_.clearThresholdFraction > 1.0f ||
      config_.glitchFilterCycles > 1023 || config_.taskStackSize == 0) {
    return false;
  }

  pinMode(inputPin_, INPUT);

  pcnt_config_t pcntConfig = {};
  pcntConfig.pulse_gpio_num = inputPin_;
  pcntConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcntConfig.lctrl_mode = PCNT_MODE_KEEP;
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
  pcntConfig.pos_mode = PCNT_COUNT_INC;
  pcntConfig.neg_mode = PCNT_COUNT_DIS;
  pcntConfig.counter_h_lim = MAX_PULSES_PER_SAMPLE;
  pcntConfig.counter_l_lim = 0;
  pcntConfig.unit = unit_;
  pcntConfig.channel = PCNT_CHANNEL_0;

  if (pcnt_unit_config(&pcntConfig) != ESP_OK) {
    return false;
  }

  esp_err_t filterResult = ESP_OK;
  if (config_.glitchFilterCycles == 0) {
    filterResult = pcnt_filter_disable(unit_);
  } else {
    filterResult = pcnt_set_filter_value(unit_, config_.glitchFilterCycles);
    if (filterResult == ESP_OK) {
      filterResult = pcnt_filter_enable(unit_);
    }
  }

  if (filterResult != ESP_OK || pcnt_counter_pause(unit_) != ESP_OK ||
      pcnt_counter_clear(unit_) != ESP_OK ||
      pcnt_counter_resume(unit_) != ESP_OK) {
    return false;
  }

  stateMutex_ = xSemaphoreCreateMutex();
  if (stateMutex_ == nullptr) {
    return false;
  }

  resetBaseline();
  sampleStartUs_ = esp_timer_get_time();
  initialized_ = true;

  char taskName[configMAX_TASK_NAME_LEN];
  snprintf(taskName, sizeof(taskName), "metal%u",
           static_cast<unsigned>(unit_));
  if (xTaskCreate(taskEntry, taskName, config_.taskStackSize, this,
                  config_.taskPriority, &taskHandle_) != pdPASS) {
    initialized_ = false;
    vSemaphoreDelete(stateMutex_);
    stateMutex_ = nullptr;
    return false;
  }

  return true;
}

void MetalDetector::taskEntry(void *context) {
  static_cast<MetalDetector *>(context)->run();
}

void MetalDetector::run() {
  TickType_t period = pdMS_TO_TICKS(config_.sampleWindowMs);
  if (period == 0) {
    period = 1;
  }

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    update();
  }
}

bool MetalDetector::update() {
  if (!initialized_) {
    return false;
  }

  if (pcnt_counter_pause(unit_) != ESP_OK) {
    return false;
  }

  const int64_t sampleEndUs = esp_timer_get_time();
  int16_t pulses = 0;
  const esp_err_t readResult = pcnt_get_counter_value(unit_, &pulses);
  const esp_err_t clearResult = pcnt_counter_clear(unit_);
  const esp_err_t resumeResult = pcnt_counter_resume(unit_);
  const int64_t nextSampleStartUs = esp_timer_get_time();

  if (readResult != ESP_OK || clearResult != ESP_OK || resumeResult != ESP_OK) {
    return false;
  }

  const int64_t elapsedUs = sampleEndUs - sampleStartUs_;
  sampleStartUs_ = nextSampleStartUs;
  if (elapsedUs <= 0) {
    return false;
  }

  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  reading_.sampleTimeUs = static_cast<uint32_t>(elapsedUs);
  reading_.pulseCount = pulses;
  // The legacy PCNT event-status bit can remain latched without an ISR and
  // therefore cannot reliably identify a new overflow here. With the default
  // 10 ms window, frequencies below about 3.27 MHz remain within this counter.
  reading_.counterSaturated = pulses >= MAX_PULSES_PER_SAMPLE;
  reading_.frequencyHz =
      (static_cast<float>(pulses) * 1000000.0f) /
      static_cast<float>(elapsedUs);

  // A saturated counter is not a usable frequency sample. Preserve the last
  // baseline and flag the condition so the sample window can be shortened.
  if (reading_.counterSaturated) {
    reading_.deviationHz = reading_.frequencyHz - reading_.baselineHz;
    reading_.relativeDeviation =
        reading_.baselineHz > 0.0f
            ? reading_.deviationHz / reading_.baselineHz
            : 0.0f;
    reading_.anomaly = true;
    deviationSumHz_ = 0.0f;
    deviationSampleCount_ = 0;
    deviationSampleIndex_ = 0;
    reading_.averagedDeviationHz = 0.0f;
    reading_.averagedSampleCount = 0;
    xSemaphoreGive(stateMutex_);
    return true;
  }

  if (calibrationCount_ < config_.calibrationSamples) {
    ++calibrationCount_;
    // A running mean gives the initial baseline a clean, deterministic
    // calibration period before anomaly detection is enabled.
    reading_.baselineHz +=
        (reading_.frequencyHz - reading_.baselineHz) /
        static_cast<float>(calibrationCount_);
    reading_.baselineReady = calibrationCount_ >= config_.calibrationSamples;
    reading_.anomaly = false;
  } else {
    const float sampleDeviation = reading_.frequencyHz - reading_.baselineHz;
    if (deviationSampleCount_ < config_.deviationAverageSamples) {
      deviationSamples_[deviationSampleIndex_] = sampleDeviation;
      deviationSumHz_ += sampleDeviation;
      ++deviationSampleCount_;
    } else {
      deviationSumHz_ -= deviationSamples_[deviationSampleIndex_];
      deviationSamples_[deviationSampleIndex_] = sampleDeviation;
      deviationSumHz_ += sampleDeviation;
    }
    deviationSampleIndex_ = static_cast<uint8_t>(
        (deviationSampleIndex_ + 1) % config_.deviationAverageSamples);
    reading_.averagedSampleCount = deviationSampleCount_;
    reading_.averagedDeviationHz =
        deviationSumHz_ / static_cast<float>(deviationSampleCount_);

    const float threshold = config_.anomalyThresholdHz;
    const float activeThreshold =
        reading_.anomaly ? threshold * config_.clearThresholdFraction
                         : threshold;
    // Do not report metal until a complete window is available. Consistent
    // positive or negative shifts remain; isolated spikes are averaged down.
    reading_.anomaly =
        deviationSampleCount_ >= config_.deviationAverageSamples &&
        fabsf(reading_.averagedDeviationHz) >= activeThreshold;

    // Freeze the baseline while anomalous, otherwise a stationary metal
    // target would gradually become the new normal.
    if (!reading_.anomaly) {
      const float elapsedSeconds = static_cast<float>(elapsedUs) / 1000000.0f;
      const float alpha =
          1.0f - expf(-elapsedSeconds / config_.baselineTimeConstantSeconds);
      reading_.baselineHz +=
          alpha * (reading_.frequencyHz - reading_.baselineHz);
    }
  }

  reading_.deviationHz = reading_.frequencyHz - reading_.baselineHz;
  reading_.relativeDeviation =
      reading_.baselineHz > 0.0f ? reading_.deviationHz / reading_.baselineHz
                                : 0.0f;
  xSemaphoreGive(stateMutex_);
  return true;
}

MetalDetector::Reading MetalDetector::reading() const {
  Reading snapshot;
  getReading(&snapshot);
  return snapshot;
}

bool MetalDetector::getReading(Reading *reading) const {
  if (reading == nullptr || stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  *reading = reading_;
  xSemaphoreGive(stateMutex_);
  return true;
}

float MetalDetector::frequencyHz() const { return reading().frequencyHz; }

float MetalDetector::baselineHz() const { return reading().baselineHz; }

float MetalDetector::deviationHz() const { return reading().deviationHz; }

bool MetalDetector::anomalyDetected() const { return reading().anomaly; }

bool MetalDetector::baselineReady() const { return reading().baselineReady; }

bool MetalDetector::initialized() const { return initialized_; }

void MetalDetector::resetBaseline() {
  if (stateMutex_ != nullptr) {
    xSemaphoreTake(stateMutex_, portMAX_DELAY);
  }
  calibrationCount_ = 0;
  deviationSumHz_ = 0.0f;
  deviationSampleCount_ = 0;
  deviationSampleIndex_ = 0;
  for (uint8_t i = 0; i < MAX_DEVIATION_AVERAGE_SAMPLES; ++i) {
    deviationSamples_[i] = 0.0f;
  }
  reading_ = Reading();
  if (stateMutex_ != nullptr) {
    xSemaphoreGive(stateMutex_);
  }
}
