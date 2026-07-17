#pragma once

#include "arm.h"
#include "mecanum_drive.h"
#include "otos_sensor.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

class DriveTask {
public:
  DriveTask(MecanumDrive &drive, OtosSensor &otos);

  bool begin(SemaphoreHandle_t pwmMutex, uint32_t stackSize = 4096,
             UBaseType_t priority = 1);
  bool setTargetPose(const OtosSensor::Pose &targetPose,
                     float maxPower = 1.0f,
                     bool intermediaryPosition = false);
  bool setOtosPose(const OtosSensor::Pose &currentPose);
  // Backward-compatible alias for setOtosPose().
  bool setCurrentPose(const OtosSensor::Pose &currentPose);
  bool getCurrentPose(OtosSensor::Pose *currentPose) const;
  bool isBusy() const;
  bool atTarget() const;
  // Blocks until the current motion finishes or timeoutMs elapses.
  bool waitUntilMotionFinished(uint32_t timeoutMs) const;
  void cancel();

private:
  enum class CommandType : uint8_t { TargetPose, SetCurrentPose, Cancel };

  struct Command {
    CommandType type;
    OtosSensor::Pose pose;
    float maxPower;
    bool intermediaryPosition;
  };

  static void taskEntry(void *context);
  void run();
  void updateSnapshot(const OtosSensor::Pose &pose, bool valid);
  void updateStatus(bool busy, bool atTarget);

  MecanumDrive *drive_;
  OtosSensor *otos_;
  SemaphoreHandle_t pwmMutex_;
  SemaphoreHandle_t stateMutex_;
  QueueHandle_t commandQueue_;
  TaskHandle_t taskHandle_;
  OtosSensor::Pose currentPose_;
  bool poseValid_;
  bool busy_;
  bool atTarget_;
};

class ArmTask {
public:
  explicit ArmTask(Arm &arm);

  bool begin(SemaphoreHandle_t pwmMutex, uint32_t stackSize = 4096,
             UBaseType_t priority = 1);
  bool setTargetAngles(const Arm::JointAngles &targetAngles);
  bool setTargetPosition(const Arm::Position &targetPosition,
                         bool elbowUp = false);
  bool getCurrentAngles(Arm::JointAngles *currentAngles) const;
  bool getTelemetry(Arm::Telemetry *telemetry) const;
  bool isBusy() const;
  bool atTarget() const;
  // Waits until both joints remain inside their position tolerances and below
  // maxVelocityDegPerSec for settleTimeMs. Returns false on timeout or fault.
  bool waitUntilSettled(uint32_t timeoutMs = 5000,
                        uint32_t settleTimeMs = 250,
                        float maxVelocityDegPerSec = 2.0f) const;
  void cancel();

private:
  enum class CommandType : uint8_t { TargetAngles, TargetPosition, Cancel };

  struct Command {
    CommandType type;
    Arm::JointAngles angles;
    Arm::Position position;
    bool elbowUp;
  };

  static void taskEntry(void *context);
  void run();
  void updateSnapshot(const Arm::JointAngles &angles, bool valid);
  void updateStatus(bool busy, bool atTarget);

  Arm *arm_;
  SemaphoreHandle_t pwmMutex_;
  SemaphoreHandle_t stateMutex_;
  QueueHandle_t commandQueue_;
  TaskHandle_t taskHandle_;
  Arm::JointAngles currentAngles_;
  Arm::Telemetry telemetry_;
  bool anglesValid_;
  bool telemetryValid_;
  bool busy_;
  bool atTarget_;
};
