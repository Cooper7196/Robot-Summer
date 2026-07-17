#pragma once

#include "motor.h"
#include "otos_sensor.h"
#include <Arduino.h>

struct motorConstants {
  static constexpr float frontLeftkS = 10;
  static constexpr float frontRightkS = 7;
  static constexpr float backLeftkS = 7;
  static constexpr float backRightkS = 7;
};

class MecanumDrive {
public:
  static constexpr int8_t NO_ENABLE_PIN = -1;
  static constexpr float POSE_POSITION_TOLERANCE_CM = 1.0f;
  static constexpr float POSE_HEADING_TOLERANCE_DEG = 1.5f;
  static constexpr uint32_t POSE_PID_PERIOD_MS = 10;
  static constexpr float POSE_EXIT_MAX_TRANSLATION_VELOCITY_CM_PER_SEC = 1.5f;
  static constexpr float POSE_EXIT_MAX_HEADING_VELOCITY_DEG_PER_SEC = 3.0f;
  static constexpr float POSE_X_KP = 0.035f;
  static constexpr float POSE_X_KI = 0.0f;
  static constexpr float POSE_X_KD = 0.0025f;
  static constexpr float POSE_Y_KP = POSE_X_KP;
  static constexpr float POSE_Y_KI = POSE_X_KI;
  static constexpr float POSE_Y_KD = POSE_X_KD;
  static constexpr float POSE_PATH_LOOKAHEAD_CM = 10.0f;
  static constexpr float POSE_HEADING_KP = 0.025f;
  static constexpr float POSE_HEADING_KI = 0.0f;
  static constexpr float POSE_HEADING_KD = 0.0011f;
  static constexpr float POSE_INTERMEDIARY_MIN_TRANSLATION_POWER = 0.20f;
  static constexpr float POSE_INTERMEDIARY_MIN_HEADING_POWER = 0.12f;

  MecanumDrive(Motor &frontLeft, Motor &frontRight, Motor &backLeft,
               Motor &backRight, int8_t enablePin = NO_ENABLE_PIN);

  void begin();
  void drive(float xVelocity, float yVelocity, float omega);
  void setTargetPose(const OtosSensor::Pose &targetPose,
                     float maxPower = 1.0f,
                     bool intermediaryPosition = false);
  bool updatePoseTarget(OtosSensor &otosSensor);
  bool updatePoseTarget(const OtosSensor::Pose &currentPose);
  bool hasPoseTarget() const;
  bool atPoseTarget() const;
  void cancelPoseTarget();
  bool driveToPose(OtosSensor &otosSensor, const OtosSensor::Pose &targetPose,
                   const float maxPower = 1.0f,
                   bool intermediaryPosition = false);
  void resetPosePid();
  void setMaxOutputPercent(float maxOutputPercent);
  void stop();
  void setWheelSpeeds(float frontLeft, float frontRight, float backLeft,
                      float backRight);

private:
  static float clamp(float value, float minValue, float maxValue);
  static float signum(float value);
  static float wrapDegrees(float degrees);
  bool enabled() const;
  bool updateDriveToPose(const OtosSensor::Pose &currentPose,
                         const OtosSensor::Pose &targetPose,
                         const float maxPower,
                         bool intermediaryPosition);
  // void setWheelSpeeds(float frontLeft, float frontRight, float backLeft,
  //                     float backRight);

  Motor *frontLeft_;
  Motor *frontRight_;
  Motor *backLeft_;
  Motor *backRight_;
  float maxOutputPercent_;
  int8_t enablePin_;

  uint32_t lastPosePidMs_;
  float xIntegral_;
  float yIntegral_;
  float headingIntegral_;
  float lastXError_;
  float lastYError_;
  float lastHeadingError_;
  OtosSensor::Pose lastPose_;
  bool hasLastPose_;
  OtosSensor::Pose pathStartPose_;
  bool hasPathStartPose_;
  OtosSensor::Pose targetPose_;
  float targetMaxPower_;
  bool targetIsIntermediary_;
  bool hasPoseTarget_;
  bool atPoseTarget_;
};
