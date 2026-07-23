#include "robot_tasks.h"

namespace {
constexpr TickType_t kTaskPeriod = pdMS_TO_TICKS(4);
constexpr uint32_t kOtosReadTimeoutMs = 50;
constexpr uint32_t kPoseLogPeriodMs = 100;
} // namespace

DriveTask::DriveTask(MecanumDrive &drive, OtosSensor &otos)
    : drive_(&drive), otos_(&otos), pwmMutex_(nullptr), stateMutex_(nullptr),
      commandQueue_(nullptr), taskHandle_(nullptr), currentPose_(),
      poseValid_(false), busy_(false), atTarget_(false),
      calibrationInProgress_(false), calibrationSucceeded_(false) {}

bool DriveTask::begin(SemaphoreHandle_t pwmMutex, uint32_t stackSize,
                      UBaseType_t priority) {
  if (taskHandle_ != nullptr || pwmMutex == nullptr) {
    return false;
  }

  pwmMutex_ = pwmMutex;
  stateMutex_ = xSemaphoreCreateMutex();
  commandQueue_ = xQueueCreate(8, sizeof(Command));
  if (stateMutex_ == nullptr || commandQueue_ == nullptr) {
    return false;
  }

  return xTaskCreate(taskEntry, "drive", stackSize, this, priority,
                     &taskHandle_) == pdPASS;
}

bool DriveTask::setTargetPose(const OtosSensor::Pose &targetPose,
                              float maxPower, bool intermediaryPosition,
                              float maxHeadingPower, float minHeadingPower,
                              float headingToleranceDeg) {
  if (commandQueue_ == nullptr) {
    return false;
  }

  Command command{CommandType::TargetPose, targetPose, maxPower,
                  intermediaryPosition, maxHeadingPower, minHeadingPower,
                  headingToleranceDeg, 0.0f, 0.0f, 0};
  const bool queued = xQueueSendToBack(commandQueue_, &command, 0) == pdPASS;
  if (queued) {
    updateStatus(true, false);
  }
  return queued;
}

bool DriveTask::setMotionTolerance(float positionToleranceCm,
                                   float headingToleranceDeg) {
  if (commandQueue_ == nullptr || positionToleranceCm <= 0.0f ||
      headingToleranceDeg <= 0.0f) {
    return false;
  }

  Command command{CommandType::SetMotionTolerance, OtosSensor::Pose(), 0.0f,
                  false, -1.0f, 0.0f, -1.0f, positionToleranceCm,
                  headingToleranceDeg, 0};
  return xQueueSendToBack(commandQueue_, &command, 0) == pdPASS;
}

bool DriveTask::setOtosPose(const OtosSensor::Pose &currentPose) {
  if (commandQueue_ == nullptr) {
    return false;
  }
  Command command{CommandType::SetCurrentPose, currentPose, 0.0f, false, -1.0f,
                  0.0f, -1.0f, 0.0f, 0.0f, 0};
  return xQueueSendToBack(commandQueue_, &command, 0) == pdPASS;
}

bool DriveTask::setCurrentPose(const OtosSensor::Pose &currentPose) {
  return setOtosPose(currentPose);
}

bool DriveTask::getCurrentPose(OtosSensor::Pose *currentPose) const {
  if (currentPose == nullptr || stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool valid = poseValid_;
  if (valid) {
    *currentPose = currentPose_;
  }
  xSemaphoreGive(stateMutex_);
  return valid;
}

bool DriveTask::calibrateImuBlocking(uint32_t durationMs) {
  if (commandQueue_ == nullptr || stateMutex_ == nullptr || durationMs == 0 ||
      isBusy()) {
    return false;
  }

  uint32_t sampleCount = static_cast<uint32_t>(
      (static_cast<uint64_t>(durationMs) * 480U + 500U) / 1000U);
  if (sampleCount == 0) {
    sampleCount = 1;
  }

  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  if (calibrationInProgress_) {
    xSemaphoreGive(stateMutex_);
    return false;
  }
  calibrationInProgress_ = true;
  calibrationSucceeded_ = false;
  xSemaphoreGive(stateMutex_);

  Command command{CommandType::CalibrateImu, OtosSensor::Pose(), 0.0f, false,
                  -1.0f, 0.0f, -1.0f, 0.0f, 0.0f, sampleCount};
  if (xQueueSendToBack(commandQueue_, &command, 0) != pdPASS) {
    xSemaphoreTake(stateMutex_, portMAX_DELAY);
    calibrationInProgress_ = false;
    xSemaphoreGive(stateMutex_);
    return false;
  }

  for (;;) {
    xSemaphoreTake(stateMutex_, portMAX_DELAY);
    const bool inProgress = calibrationInProgress_;
    const bool succeeded = calibrationSucceeded_;
    xSemaphoreGive(stateMutex_);
    if (!inProgress) {
      return succeeded;
    }
    vTaskDelay(1);
  }
}

bool DriveTask::isBusy() const {
  if (stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool busy = busy_;
  xSemaphoreGive(stateMutex_);
  return busy;
}

bool DriveTask::atTarget() const {
  if (stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool atTarget = atTarget_;
  xSemaphoreGive(stateMutex_);
  return atTarget;
}

bool DriveTask::waitUntilMotionFinished(uint32_t timeoutMs) const {
  const uint32_t startMs = millis();
  while (isBusy()) {
    if (millis() - startMs >= timeoutMs) {
      return false;
    }
    vTaskDelay(1);
  }
  return true;
}

void DriveTask::cancel() {
  if (commandQueue_ == nullptr) {
    return;
  }
  Command command{CommandType::Cancel, OtosSensor::Pose(), 0.0f, false, -1.0f,
                  0.0f, -1.0f, 0.0f, 0.0f, 0};
  xQueueReset(commandQueue_);
  xQueueSendToBack(commandQueue_, &command, 0);
  updateStatus(false, false);
}

void DriveTask::taskEntry(void *context) {
  static_cast<DriveTask *>(context)->run();
}

void DriveTask::run() {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastPoseLogMs = 0;
  for (;;) {
    // Poll the physical enable switch independently of motion commands. The
    // PWM expander retains its last duty cycle while the drive is otherwise
    // idle, including after an intermediary waypoint.
    xSemaphoreTake(pwmMutex_, portMAX_DELAY);
    drive_->stopIfDisabled();
    xSemaphoreGive(pwmMutex_);

    Command command;
    if (xQueueReceive(commandQueue_, &command, 0) == pdTRUE) {
      if (command.type == CommandType::TargetPose) {
        drive_->setTargetPose(command.pose, command.maxPower,
                              command.intermediaryPosition,
                              command.maxHeadingPower,
                              command.minHeadingPower,
                              command.targetHeadingToleranceDeg);
        updateStatus(true, false);
      } else if (command.type == CommandType::SetCurrentPose) {
        const bool set = otos_->setPose(command.pose);
        if (set) {
          updateSnapshot(command.pose, true);
          drive_->resetPosePid();
        }
      } else if (command.type == CommandType::SetMotionTolerance) {
        drive_->setMotionTolerance(command.positionToleranceCm,
                                   command.headingToleranceDeg);
      } else if (command.type == CommandType::CalibrateImu) {
        xSemaphoreTake(pwmMutex_, portMAX_DELAY);
        drive_->cancelPoseTarget();
        drive_->stop();
        drive_->resetPosePid();
        xSemaphoreGive(pwmMutex_);
        updateStatus(false, false);

        const bool started =
            otos_->startGyroCalibration(command.gyroCalibrationSamples);
        if (started) {
          const uint64_t durationMs =
              (static_cast<uint64_t>(command.gyroCalibrationSamples) * 1000U +
               479U) /
              480U;
          vTaskDelay(pdMS_TO_TICKS(durationMs));
        }

        xSemaphoreTake(stateMutex_, portMAX_DELAY);
        calibrationSucceeded_ = started;
        calibrationInProgress_ = false;
        xSemaphoreGive(stateMutex_);
      } else {
        xSemaphoreTake(pwmMutex_, portMAX_DELAY);
        drive_->cancelPoseTarget();
        xSemaphoreGive(pwmMutex_);
        updateStatus(false, false);
      }
    }

    OtosSensor::Pose pose;
    const bool poseValid = otos_->getPose(&pose, kOtosReadTimeoutMs);
    updateSnapshot(pose, poseValid);

    const uint32_t nowMs = millis();
    if (nowMs - lastPoseLogMs >= kPoseLogPeriodMs) {
      lastPoseLogMs = nowMs;
      if (poseValid) {
        // Serial.printf("Drive x=%.2f cm y=%.2f cm heading=%.2f deg\n",
        // pose.xCm,
        //               pose.yCm, pose.headingDeg);
      } else {
        Serial.println("Drive pose unavailable");
      }
    }

    if (!poseValid) {
      xSemaphoreTake(pwmMutex_, portMAX_DELAY);
      drive_->stop();
      drive_->resetPosePid();
      xSemaphoreGive(pwmMutex_);
    } else if (drive_->hasPoseTarget()) {
      xSemaphoreTake(pwmMutex_, portMAX_DELAY);
      const bool reachedTarget = drive_->updatePoseTarget(pose);
      xSemaphoreGive(pwmMutex_);
      if (reachedTarget) {
        updateStatus(false, true);
      }
    }

    vTaskDelayUntil(&lastWake, kTaskPeriod);
  }
}

void DriveTask::updateSnapshot(const OtosSensor::Pose &pose, bool valid) {
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  if (valid) {
    currentPose_ = pose;
  }
  poseValid_ = valid;
  xSemaphoreGive(stateMutex_);
}

void DriveTask::updateStatus(bool busy, bool atTarget) {
  if (stateMutex_ == nullptr) {
    return;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  busy_ = busy;
  atTarget_ = atTarget;
  xSemaphoreGive(stateMutex_);
}

ArmTask::ArmTask(Arm &arm)
    : arm_(&arm), pwmMutex_(nullptr), stateMutex_(nullptr),
      commandQueue_(nullptr), taskHandle_(nullptr), currentAngles_(),
      telemetry_(), anglesValid_(false), telemetryValid_(false), busy_(false),
      atTarget_(false) {}

bool ArmTask::begin(SemaphoreHandle_t pwmMutex, uint32_t stackSize,
                    UBaseType_t priority) {
  if (taskHandle_ != nullptr || pwmMutex == nullptr) {
    return false;
  }

  pwmMutex_ = pwmMutex;
  stateMutex_ = xSemaphoreCreateMutex();
  commandQueue_ = xQueueCreate(8, sizeof(Command));
  if (stateMutex_ == nullptr || commandQueue_ == nullptr) {
    return false;
  }

  return xTaskCreate(taskEntry, "arm", stackSize, this, priority,
                     &taskHandle_) == pdPASS;
}

bool ArmTask::setTargetAngles(const Arm::JointAngles &targetAngles) {
  if (commandQueue_ == nullptr) {
    return false;
  }
  Command command{CommandType::TargetAngles, targetAngles, Arm::Position(),
                  false};
  const bool queued = xQueueSendToBack(commandQueue_, &command, 0) == pdPASS;
  if (queued) {
    updateStatus(true, false);
  }
  return queued;
}

bool ArmTask::setTargetPosition(const Arm::Position &targetPosition,
                                bool elbowUp) {
  if (commandQueue_ == nullptr) {
    return false;
  }
  Command command{CommandType::TargetPosition, Arm::JointAngles(),
                  targetPosition, elbowUp};
  const bool queued = xQueueSendToBack(commandQueue_, &command, 0) == pdPASS;
  if (queued) {
    updateStatus(true, false);
  }
  return queued;
}

bool ArmTask::getCurrentAngles(Arm::JointAngles *currentAngles) const {
  if (currentAngles == nullptr || stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool valid = anglesValid_;
  if (valid) {
    *currentAngles = currentAngles_;
  }
  xSemaphoreGive(stateMutex_);
  return valid;
}

bool ArmTask::getTelemetry(Arm::Telemetry *telemetry) const {
  if (telemetry == nullptr || stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool valid = telemetryValid_;
  if (valid) {
    *telemetry = telemetry_;
  }
  xSemaphoreGive(stateMutex_);
  return valid;
}

bool ArmTask::isBusy() const {
  if (stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool busy = busy_;
  xSemaphoreGive(stateMutex_);
  return busy;
}

bool ArmTask::atTarget() const {
  if (stateMutex_ == nullptr) {
    return false;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool atTarget = atTarget_;
  xSemaphoreGive(stateMutex_);
  return atTarget;
}

bool ArmTask::waitUntilSettled(uint32_t timeoutMs, uint32_t settleTimeMs,
                               float maxVelocityDegPerSec) const {
  if (stateMutex_ == nullptr || timeoutMs == 0 ||
      maxVelocityDegPerSec < 0.0f) {
    return false;
  }

  const uint32_t startMs = millis();
  uint32_t settledSinceMs = 0;
  while (millis() - startMs < timeoutMs) {
    if (!isBusy()) {
      return false;
    }

    Arm::Telemetry telemetry;
    if (!getTelemetry(&telemetry) || telemetry.faulted) {
      settledSinceMs = 0;
    } else {
      const bool velocitySettled =
          fabsf(telemetry.shoulder.measuredVelocityDegPerSec) <=
              maxVelocityDegPerSec &&
          fabsf(telemetry.elbow.measuredVelocityDegPerSec) <=
              maxVelocityDegPerSec;
      if (atTarget() && velocitySettled) {
        const uint32_t nowMs = millis();
        if (settledSinceMs == 0) {
          settledSinceMs = nowMs;
        }
        if (nowMs - settledSinceMs >= settleTimeMs) {
          return true;
        }
      } else {
        settledSinceMs = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return false;
}

void ArmTask::cancel() {
  if (commandQueue_ == nullptr) {
    return;
  }
  Command command{CommandType::Cancel, Arm::JointAngles(), Arm::Position(),
                  false};
  xQueueReset(commandQueue_);
  xQueueSendToBack(commandQueue_, &command, 0);
  updateStatus(false, false);
}

void ArmTask::taskEntry(void *context) {
  static_cast<ArmTask *>(context)->run();
}

void ArmTask::run() {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastTelemetryMs = 0;
  uint32_t lastControllerSampleUs = 0;
  for (;;) {
    Command command;
    if (xQueueReceive(commandQueue_, &command, 0) == pdTRUE) {
      bool accepted = true;
      if (command.type == CommandType::TargetAngles) {
        xSemaphoreTake(pwmMutex_, portMAX_DELAY);
        accepted = arm_->setTargetAngles(command.angles);
        xSemaphoreGive(pwmMutex_);
      } else if (command.type == CommandType::TargetPosition) {
        xSemaphoreTake(pwmMutex_, portMAX_DELAY);
        accepted = arm_->setTargetPosition(command.position, command.elbowUp);
        xSemaphoreGive(pwmMutex_);
      } else {
        xSemaphoreTake(pwmMutex_, portMAX_DELAY);
        arm_->stop();
        xSemaphoreGive(pwmMutex_);
        accepted = false;
      }
      updateStatus(accepted, false);
    }

    const bool controllerActive = isBusy();
    if (controllerActive) {
      xSemaphoreTake(pwmMutex_, portMAX_DELAY);
      const bool reachedTarget = arm_->update();
      xSemaphoreGive(pwmMutex_);
      // Keep the controller active after reaching the target so it resumes
      // driving if an external load moves either joint out of tolerance.
      updateStatus(true, reachedTarget);
    }

    Arm::JointAngles angles;
    Arm::Telemetry latestTelemetry;
    const bool controllerTelemetryValid =
        controllerActive && arm_->getTelemetry(&latestTelemetry) &&
        !latestTelemetry.faulted && latestTelemetry.timestampUs != 0 &&
        latestTelemetry.timestampUs != lastControllerSampleUs;
    if (controllerTelemetryValid) {
      lastControllerSampleUs = latestTelemetry.timestampUs;
      angles.shoulderDeg = latestTelemetry.shoulder.measuredPositionDeg;
      angles.elbowDeg = latestTelemetry.elbow.measuredPositionDeg;
      updateSnapshot(angles, true);
    } else {
      updateSnapshot(angles, arm_->readAngles(&angles));
    }
    const uint32_t nowMs = millis();
    if (nowMs - lastTelemetryMs >= 20) {
      lastTelemetryMs = nowMs;
      Arm::Telemetry telemetry;
      const bool telemetryValid = arm_->getTelemetry(&telemetry);
      xSemaphoreTake(stateMutex_, portMAX_DELAY);
      if (telemetryValid) {
        telemetry_ = telemetry;
      }
      telemetryValid_ = telemetryValid;
      xSemaphoreGive(stateMutex_);
    }
    vTaskDelayUntil(&lastWake, kTaskPeriod);
  }
}

void ArmTask::updateSnapshot(const Arm::JointAngles &angles, bool valid) {
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  if (valid) {
    currentAngles_ = angles;
  }
  anglesValid_ = valid;
  xSemaphoreGive(stateMutex_);
}

void ArmTask::updateStatus(bool busy, bool atTarget) {
  if (stateMutex_ == nullptr) {
    return;
  }
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  busy_ = busy;
  atTarget_ = atTarget;
  xSemaphoreGive(stateMutex_);
}
