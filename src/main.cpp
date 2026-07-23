#include "angle_servo.h"
#include "arm.h"
#include "as5600_encoder.h"
#include "mecanum_drive.h"
#include "metal_detector.h"
#include "motor.h"
#include "otos_sensor.h"
#include "pins.h"
#include "pwm_expander.h"
#include "robot_tasks.h"
#include "tape_sensor_array.h"
#include <Arduino.h>

constexpr uint32_t kSerialBaud = 115200;
const OtosSensor::Pose kTargetPose(0.0f, 100.0f, 0.0f);
constexpr float kTapeAdcReferenceVolts = 3.3f;
constexpr float kTapeThresholdVolts = 1.1f;
constexpr uint8_t kTapeCalibrationSensorCount = 3;
constexpr uint8_t kTapeCalibrationSensorChannels[kTapeCalibrationSensorCount] =
    {2, 4, 6};
// Installed sensors are two array positions apart. Channel 6 is on robot -X
// (left) and channel 2 is on robot +X (right).
constexpr float kTapeCalibrationSensorLocalXCm[kTapeCalibrationSensorCount] = {
    2.4f, 0.0f, -2.4f};
constexpr float kTapeCalibrationSensorLocalYCm = 11.33f;
constexpr uint8_t kMiddleTapeSensorChannel = 4;
constexpr float kTapeCalibrationMaxTravelCm = 30.0f;
constexpr uint32_t kTapeCalibrationTimeoutMs = 10000;
constexpr float kTapeCalibrationMaxHeadingPower = 0.20f;
constexpr float kTapeCalibrationMinHeadingPower = 0.00f;
constexpr float kTapeCalibrationHeadingToleranceDeg = 0.5f;
constexpr float kTapeCalibrationMinPeakRise = 50.0f;
constexpr uint8_t kTapeCalibrationRiseConfirmSamples = 5;
constexpr float kTapeCalibrationMinTravelPastPeakCm = 1.0f;
constexpr uint8_t kTapeCalibrationPeakConfirmSamples = 10;
constexpr float kTapeCalibrationProfileSpacingCm = 0.05f;
constexpr uint16_t kTapeCalibrationMaxProfileSamples = 640;
constexpr float kMetalAnomalyThresholdHz = 90.0f;
constexpr uint8_t kMetalDeviationAverageSamples = 5;
// Adjust this signed value until the elbow just begins moving. Use a
// negative value to measure friction in the opposite direction.
constexpr bool kElbowConstantPidTestEnabled = false;
constexpr float kElbowConstantPidTestPercent = -9.0f;

// Replace these with each AS5600's raw reading at the position that should be
// reported as 0 degrees.
constexpr float kShoulderEncoderDegAtZero = 10.2f;
constexpr float kElbowEncoderDegAtZero = (286.8f);

// Mechanical joint limits in calibrated joint coordinates. Targets outside
// these ranges are rejected rather than clamped.
constexpr float kShoulderMinAngleDeg = -180.0f;
constexpr float kShoulderMaxAngleDeg = 180.0f;
constexpr float kElbowMinAngleDeg = -180.0f;
constexpr float kElbowMaxAngleDeg = 180.0f;
constexpr float clawOpenAngle = 40.0f;
constexpr float clawHabitatOpenAngle = 115.0f;
constexpr float clawClosedAngle = 115.0f;
constexpr float clawFullyClosedAngle = 142.0f;

uint16_t adcCountFromVolts(float volts) {
  if (volts <= 0.0f) {
    return 0;
  }

  if (volts >= kTapeAdcReferenceVolts) {
    return TapeSensorArray::MAX_READING;
  }

  return static_cast<uint16_t>(
      ((volts / kTapeAdcReferenceVolts) * TapeSensorArray::MAX_READING) + 0.5f);
}

float voltsFromAdcCount(uint16_t count) {
  return (static_cast<float>(count) / TapeSensorArray::MAX_READING) *
         kTapeAdcReferenceVolts;
}

MetalDetector::Config makeMetalDetectorConfig() {
  MetalDetector::Config config;
  config.anomalyThresholdHz = kMetalAnomalyThresholdHz;
  config.deviationAverageSamples = kMetalDeviationAverageSamples;
  return config;
}

PwmExpander pwmExpander;
AngleServo servo1(pins::SERVO1_PWM_PIN);
OtosSensor otosSensor(Serial1);
TapeSensorArray tapeSensors;
MetalDetector::Config metalDetectorConfig = makeMetalDetectorConfig();
MetalDetector metalDetectorLeft(pins::MD_RIGHT_PIN, PCNT_UNIT_0,
                                metalDetectorConfig);
MetalDetector metalDetectorRight(pins::MD_LEFT_PIN, PCNT_UNIT_1,
                                 metalDetectorConfig);

bool confirmedMetalDetected(const MetalDetector &detector) {
  MetalDetector::Reading reading;
  return detector.getReading(&reading) && reading.baselineReady &&
         !reading.counterSaturated &&
         reading.averagedSampleCount >= kMetalDeviationAverageSamples &&
         reading.anomaly;
}

As5600Encoder elbowEncoder(pins::ENCODER_MUX_CHANNEL0_PIN);
As5600Encoder shoulderEncoder(pins::ENCODER_MUX_CHANNEL1_PIN);
bool elbowEncoderReady = false;
bool shoulderEncoderReady = false;

Arm::Config makeArmConfig() {
  Arm::Config config;
  config.motorDisablePin = pins::EXTRA2_PIN;
  config.maxCartesianSpeedCmPerSec = 30.0f;
  // Signed PWM percentages required to counter gravity when the corresponding
  // link is horizontal. Reverse a sign if compensation assists gravity.
  config.gravityA1Percent = 0.0f;
  config.gravityA12Percent = 0.0f;
  config.gravityA2Percent = 24.5f;

  config.shoulder.encoderReferenceDeg = kShoulderEncoderDegAtZero;
  config.shoulder.jointReferenceDeg = 0.0f;
  config.shoulder.direction = 1.0f;
  // Gains are PWM percent per degree (P) and per degree/second (D).
  config.shoulder.kP = 1.2f;
  config.shoulder.kI = 0.2f;
  config.shoulder.kD = 0.15f;
  config.shoulder.constantPidTestEnabled = false;

  config.shoulder.positiveStaticFrictionPercent = 12.0f;
  config.shoulder.negativeStaticFrictionPercent = -9.0f;
  config.shoulder.minAngleDeg = kShoulderMinAngleDeg;
  config.shoulder.maxAngleDeg = kShoulderMaxAngleDeg;
  config.shoulder.positionToleranceDeg = 1.0f;
  config.shoulder.integralLimitDegSec = 30.0f;
  config.shoulder.maxPwmPercent = 100.0f;
  config.shoulder.maxOutputSlewPercentPerSec = 400.0f;
  config.shoulder.constantPidTestEnabled = kElbowConstantPidTestEnabled;
  config.shoulder.constantPidOutputPercent = kElbowConstantPidTestPercent;

  config.elbow.encoderReferenceDeg = kElbowEncoderDegAtZero;
  config.elbow.jointReferenceDeg = 0.0f;
  config.elbow.direction = -1.0f;
  config.elbow.kP = 0.9f;
  config.elbow.kI = 0.1f;
  config.elbow.kD = 0.2f;

  // config.elbow.kP = 0.0f;
  // config.elbow.kI = 0.0f;
  // config.elbow.kD = 0.0f;
  config.elbow.constantPidTestEnabled = false;
  config.elbow.constantPidOutputPercent = kElbowConstantPidTestPercent;

  config.elbow.positiveStaticFrictionPercent = 4.0f;
  config.elbow.negativeStaticFrictionPercent = 20.0f;
  config.elbow.minAngleDeg = kElbowMinAngleDeg;
  config.elbow.maxAngleDeg = kElbowMaxAngleDeg;
  config.elbow.positionToleranceDeg = 1.0f;
  config.elbow.integralLimitDegSec = 30.0f;
  config.elbow.maxPwmPercent = 100.0f;
  config.elbow.maxOutputSlewPercentPerSec = 400.0f;

  return config;
}

// 1 is front left
// 2 is front right
// 3 is back left
// 4 is back right
// 5 is shoulder
// 6 is elbow
Motor hbridge1(pwmExpander, pins::HBRIDGE5_PWM2_PIN, pins::HBRIDGE5_PWM1_PIN);
Motor hbridge2(pwmExpander, pins::HBRIDGE6_PWM1_PIN, pins::HBRIDGE6_PWM2_PIN);
Motor hbridge3(pwmExpander, pins::HBRIDGE1_PWM1_PIN, pins::HBRIDGE1_PWM2_PIN);
Motor hbridge4(pwmExpander, pins::HBRIDGE2_PWM2_PIN, pins::HBRIDGE2_PWM1_PIN);
Motor hbridge5(pwmExpander, pins::HBRIDGE3_PWM1_PIN, pins::HBRIDGE3_PWM2_PIN);
Motor hbridge6(pwmExpander, pins::HBRIDGE4_PWM2_PIN, pins::HBRIDGE4_PWM1_PIN);

MecanumDrive driveBase(hbridge1, hbridge2, hbridge3, hbridge4,
                       pins::EXTRA1_PIN);
Arm::Config armConfig = makeArmConfig();
Arm robotArm(hbridge5, hbridge6, shoulderEncoder, elbowEncoder, armConfig);
DriveTask driveTask(driveBase, otosSensor);
ArmTask armTask(robotArm);
SemaphoreHandle_t pwmMutex = nullptr;
bool driveTaskReady = false;
bool armTaskReady = false;
bool gravityCompensationReady = false;
bool metalDetectorLeftReady = false;
bool metalDetectorRightReady = false;

bool rockHeld = false;

constexpr uint32_t kGrabRockTaskStackSize = 3072;

void logArmPositionAndRobotPose() {
  Arm::JointAngles armAngles;
  Arm::Position armPosition;
  OtosSensor::Pose robotPose;
  const bool armPositionValid =
      armTaskReady && armTask.getCurrentAngles(&armAngles);
  const bool robotPoseValid =
      driveTaskReady && driveTask.getCurrentPose(&robotPose);

  if (!armPositionValid || !robotPoseValid) {
    Serial.printf("Position unavailable: arm=%s robot=%s\n",
                  armPositionValid ? "ready" : "unavailable",
                  robotPoseValid ? "ready" : "unavailable");
    return;
  }

  armPosition = robotArm.positionFromAngles(armAngles);

  // Output is intentionally limited to 10 Hz by the callers so serial traffic
  // does not interfere with the 4 ms arm control loop.
  Serial.printf("Arm shoulder=%.2f deg elbow=%.2f deg | "
                "x=%.2f cm y=%.2f cm | "
                "Robot x=%.2f cm y=%.2f cm heading=%.2f deg\n",
                armAngles.shoulderDeg, armAngles.elbowDeg, armPosition.xCm,
                armPosition.yCm, robotPose.xCm, robotPose.yCm,
                robotPose.headingDeg);
}

void delayWithArmLogging(uint32_t durationMs) {
  const uint32_t startMs = millis();
  uint32_t lastLogMs = startMs - 100;
  while (millis() - startMs < durationMs) {
    const uint32_t nowMs = millis();
    if (nowMs - lastLogMs >= 100) {
      lastLogMs = nowMs;
      logArmPositionAndRobotPose();
    }
    delay(1);
  }
}

bool waitForDrivePose(const OtosSensor::Pose &expectedPose,
                      uint32_t timeoutMs = 500) {
  const uint32_t startMs = millis();
  while (millis() - startMs < timeoutMs) {
    OtosSensor::Pose currentPose;
    if (driveTask.getCurrentPose(&currentPose)) {
      float headingError = currentPose.headingDeg - expectedPose.headingDeg;
      while (headingError > 180.0f) {
        headingError -= 360.0f;
      }
      while (headingError < -180.0f) {
        headingError += 360.0f;
      }

      if (fabsf(currentPose.xCm - expectedPose.xCm) <= 0.15f &&
          fabsf(currentPose.yCm - expectedPose.yCm) <= 0.15f &&
          fabsf(headingError) <= 0.5f) {
        return true;
      }
    }
    delay(1);
  }
  return false;
}

enum class TapeCalibrationAxis : uint8_t { X, Y };

bool calibrateWithMiddleTapeSensor(TapeCalibrationAxis axis,
                                   float knownTapeCoordinateCm,
                                   float searchDirection, float searchPower) {
  if (searchDirection == 0.0f) {
    Serial.println("Tape calibration failed: search direction is zero");
    return false;
  }
  if (searchPower <= 0.0f || searchPower > 1.0f) {
    Serial.println("Tape calibration failed: search power must be (0, 1]");
    return false;
  }
  searchDirection = searchDirection < 0.0f ? -1.0f : 1.0f;

  OtosSensor::Pose pose;
  if (!driveTask.getCurrentPose(&pose)) {
    Serial.println("Tape calibration failed: pose unavailable");
    return false;
  }
  if (driveTask.isBusy()) {
    Serial.println("Tape calibration failed: drive is already moving");
    return false;
  }
  const OtosSensor::Pose startPose = pose;
  const float startCrossTrackCm =
      axis == TapeCalibrationAxis::X ? startPose.yCm : startPose.xCm;

  uint16_t readings[TapeSensorArray::CHANNEL_COUNT];
  float sampleReadings[kTapeCalibrationSensorCount][5] = {};
  float samplePoseCoordinateCm[5] = {};
  OtosSensor::Pose samplePoses[5] = {};
  uint8_t sampleCount = 0;
  float centeredReading = 0.0f;
  float baselineReading = 0.0f;
  float peakReading = 0.0f;
  float peakPoseCoordinateCm = 0.0f;
  static float profileReadings[kTapeCalibrationSensorCount]
                              [kTapeCalibrationMaxProfileSamples];
  static float profileCoordinatesCm[kTapeCalibrationMaxProfileSamples];
  uint16_t profileCount = 0;
  uint16_t peakProfileIndex = 0;
  bool baselineValid = false;
  bool significantRiseDetected = false;
  bool peakPoseValid = false;
  bool peakConfirmed = false;
  bool profileOverflow = false;
  float maxCrossTrackDriftCm = 0.0f;
  float maxHeadingDriftDeg = 0.0f;
  uint8_t riseCount = 0;
  uint8_t peakDropCount = 0;

  // Drive in the requested world-axis direction beyond the expected tape
  // position so a local maximum can be distinguished from noise.
  OtosSensor::Pose searchTarget = pose;
  if (axis == TapeCalibrationAxis::X) {
    searchTarget.xCm += searchDirection * kTapeCalibrationMaxTravelCm;
  } else {
    searchTarget.yCm += searchDirection * kTapeCalibrationMaxTravelCm;
  }
  if (!driveTask.setTargetPose(searchTarget, searchPower, false,
                               kTapeCalibrationMaxHeadingPower,
                               kTapeCalibrationMinHeadingPower,
                               kTapeCalibrationHeadingToleranceDeg)) {
    Serial.println("Tape calibration failed: search command not queued");
    return false;
  }

  uint32_t searchStartMs = millis();
  while (millis() - searchStartMs < kTapeCalibrationTimeoutMs) {
    tapeSensors.readAll(readings);
    OtosSensor::Pose samplePose;
    if (!driveTask.getCurrentPose(&samplePose)) {
      delay(1);
      continue;
    }

    if (sampleCount < 5) {
      for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
        sampleReadings[sensor][sampleCount] =
            readings[kTapeCalibrationSensorChannels[sensor]];
      }
      samplePoseCoordinateCm[sampleCount] =
          axis == TapeCalibrationAxis::X ? samplePose.xCm : samplePose.yCm;
      samplePoses[sampleCount] = samplePose;
      ++sampleCount;
    } else {
      for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount;
             ++sensor) {
          sampleReadings[sensor][i] = sampleReadings[sensor][i + 1];
        }
        samplePoseCoordinateCm[i] = samplePoseCoordinateCm[i + 1];
        samplePoses[i] = samplePoses[i + 1];
      }
      for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
        sampleReadings[sensor][4] =
            readings[kTapeCalibrationSensorChannels[sensor]];
      }
      samplePoseCoordinateCm[4] =
          axis == TapeCalibrationAxis::X ? samplePose.xCm : samplePose.yCm;
      samplePoses[4] = samplePose;
    }

    if (sampleCount < 5) {
      delay(1);
      continue;
    }

    // A centered median rejects short ADC spikes without shifting the detected
    // peak in the direction of travel. Its position is the middle sample.
    float centeredReadings[kTapeCalibrationSensorCount];
    for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
      float sortedReadings[5];
      for (uint8_t i = 0; i < 5; ++i) {
        sortedReadings[i] = sampleReadings[sensor][i];
      }
      for (uint8_t i = 1; i < 5; ++i) {
        const float value = sortedReadings[i];
        uint8_t position = i;
        while (position > 0 && sortedReadings[position - 1] > value) {
          sortedReadings[position] = sortedReadings[position - 1];
          --position;
        }
        sortedReadings[position] = value;
      }
      centeredReadings[sensor] = sortedReadings[2];
    }
    centeredReading = centeredReadings[1];

    const OtosSensor::Pose &centeredPose = samplePoses[2];
    const float centeredCoordinateCm = samplePoseCoordinateCm[2];
    const float centeredCrossTrackCm =
        axis == TapeCalibrationAxis::X ? centeredPose.yCm : centeredPose.xCm;
    float headingDriftDeg = centeredPose.headingDeg - startPose.headingDeg;
    while (headingDriftDeg > 180.0f) {
      headingDriftDeg -= 360.0f;
    }
    while (headingDriftDeg < -180.0f) {
      headingDriftDeg += 360.0f;
    }
    maxCrossTrackDriftCm = fmaxf(
        maxCrossTrackDriftCm, fabsf(centeredCrossTrackCm - startCrossTrackCm));
    maxHeadingDriftDeg = fmaxf(maxHeadingDriftDeg, fabsf(headingDriftDeg));

    // Process samples at fixed spatial intervals so detection does not depend
    // on loop rate, OTOS update rate, or small changes in crossing speed.
    if (profileCount > 0 &&
        (centeredCoordinateCm - profileCoordinatesCm[profileCount - 1]) *
                searchDirection <
            kTapeCalibrationProfileSpacingCm) {
      if (!driveTask.isBusy()) {
        break;
      }
      delay(1);
      continue;
    }
    if (profileCount >= kTapeCalibrationMaxProfileSamples) {
      profileOverflow = true;
      break;
    }
    for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
      profileReadings[sensor][profileCount] = centeredReadings[sensor];
    }
    profileCoordinatesCm[profileCount] = centeredCoordinateCm;
    const uint16_t currentProfileIndex = profileCount++;

    if (!significantRiseDetected) {
      if (!baselineValid) {
        baselineReading = centeredReading;
        baselineValid = true;
      } else {
        // Until the tape is encountered, retain the lowest stable background
        // value. The five-sample median prevents a single low ADC spike from
        // artificially lowering this reference.
        baselineReading = fminf(baselineReading, centeredReading);
      }

      if (centeredReading - baselineReading >= kTapeCalibrationMinPeakRise) {
        if (riseCount < kTapeCalibrationRiseConfirmSamples) {
          ++riseCount;
        }
      } else {
        riseCount = 0;
      }

      if (riseCount >= kTapeCalibrationRiseConfirmSamples) {
        significantRiseDetected = true;
        peakReading = centeredReading;
        peakPoseCoordinateCm = centeredCoordinateCm;
        peakProfileIndex = currentProfileIndex;
        peakPoseValid = true;
        peakDropCount = 0;
        Serial.printf("Tape rise detected: baseline=%.1f reading=%.1f\n",
                      baselineReading, centeredReading);
      }
    } else {
      if (centeredReading > peakReading) {
        peakReading = centeredReading;
        peakPoseCoordinateCm = centeredCoordinateCm;
        peakProfileIndex = currentProfileIndex;
        peakPoseValid = true;
        peakDropCount = 0;
      }
    }

    if (peakPoseValid) {
      const float travelPastPeakCm =
          (centeredCoordinateCm - peakPoseCoordinateCm) * searchDirection;
      const bool sufficientlyPastPeak =
          travelPastPeakCm >= kTapeCalibrationMinTravelPastPeakCm;
      const float currentHalfHeight =
          baselineReading + (0.5f * (peakReading - baselineReading));
      const bool readingBelowHalfHeight = centeredReading <= currentHalfHeight;

      if (sufficientlyPastPeak && readingBelowHalfHeight) {
        if (peakDropCount < kTapeCalibrationPeakConfirmSamples) {
          ++peakDropCount;
        }
      } else {
        peakDropCount = 0;
      }

      if (peakDropCount >= kTapeCalibrationPeakConfirmSamples) {
        peakConfirmed = true;
        break;
      }
    }

    if (!driveTask.isBusy()) {
      break;
    }
    delay(1);
  }

  driveTask.cancel();
  delay(50);
  if (profileOverflow) {
    Serial.println("Tape calibration warning: profile buffer full");
  }
  if (profileCount < 3) {
    Serial.println("Tape calibration failed: insufficient profile samples");
    return false;
  }

  const float headingRad = startPose.headingDeg * DEG_TO_RAD;
  float measuredTapeCenterSumCm = 0.0f;
  uint8_t validSensorCount = 0;

  for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
    float sensorBaseline = profileReadings[sensor][0];
    float sensorPeak = profileReadings[sensor][0];
    uint16_t sensorPeakIndex = 0;
    for (uint16_t i = 1; i < profileCount; ++i) {
      sensorBaseline = fminf(sensorBaseline, profileReadings[sensor][i]);
      if (profileReadings[sensor][i] > sensorPeak) {
        sensorPeak = profileReadings[sensor][i];
        sensorPeakIndex = i;
      }
    }

    if (sensorPeak - sensorBaseline < kTapeCalibrationMinPeakRise) {
      Serial.printf("Tape ch%u ignored: rise %.1f is too small\n",
                    kTapeCalibrationSensorChannels[sensor],
                    sensorPeak - sensorBaseline);
      continue;
    }

    const float edgeReading =
        sensorBaseline + (0.5f * (sensorPeak - sensorBaseline));
    float risingEdgeCoordinateCm = 0.0f;
    float fallingEdgeCoordinateCm = 0.0f;
    bool risingEdgeValid = false;
    bool fallingEdgeValid = false;

    // Keep the last upward half-height crossing before the peak.
    for (uint16_t i = 1; i <= sensorPeakIndex; ++i) {
      const float previousReading = profileReadings[sensor][i - 1];
      const float currentReading = profileReadings[sensor][i];
      if (previousReading < edgeReading && currentReading >= edgeReading) {
        const float fraction = (edgeReading - previousReading) /
                               (currentReading - previousReading);
        risingEdgeCoordinateCm =
            profileCoordinatesCm[i - 1] +
            fraction * (profileCoordinatesCm[i] - profileCoordinatesCm[i - 1]);
        risingEdgeValid = true;
      }
    }

    // Use the first downward half-height crossing after the peak.
    for (uint16_t i = sensorPeakIndex + 1; i < profileCount; ++i) {
      const float previousReading = profileReadings[sensor][i - 1];
      const float currentReading = profileReadings[sensor][i];
      if (previousReading >= edgeReading && currentReading < edgeReading) {
        const float fraction = (edgeReading - previousReading) /
                               (currentReading - previousReading);
        fallingEdgeCoordinateCm =
            profileCoordinatesCm[i - 1] +
            fraction * (profileCoordinatesCm[i] - profileCoordinatesCm[i - 1]);
        fallingEdgeValid = true;
        break;
      }
    }

    if (!risingEdgeValid || !fallingEdgeValid) {
      Serial.printf("Tape ch%u ignored: half-height edges unavailable\n",
                    kTapeCalibrationSensorChannels[sensor]);
      continue;
    }

    const float sensorMeasuredCenterCm =
        0.5f * (risingEdgeCoordinateCm + fallingEdgeCoordinateCm);
    // Rotate each sensor's lateral X offset and the tape array's common
    // forward Y offset into the world axis being calibrated.
    const float sensorLocalXCm = kTapeCalibrationSensorLocalXCm[sensor];
    const float sensorWorldOffsetXCm =
        (cosf(headingRad) * sensorLocalXCm) -
        (sinf(headingRad) * kTapeCalibrationSensorLocalYCm);
    const float sensorWorldOffsetYCm =
        (sinf(headingRad) * sensorLocalXCm) +
        (cosf(headingRad) * kTapeCalibrationSensorLocalYCm);
    const float sensorAxisOffsetCm = axis == TapeCalibrationAxis::X
                                         ? sensorWorldOffsetXCm
                                         : sensorWorldOffsetYCm;
    const float robotCenterEquivalentCm =
        sensorMeasuredCenterCm + sensorAxisOffsetCm;
    measuredTapeCenterSumCm += robotCenterEquivalentCm;
    ++validSensorCount;

    Serial.printf("Tape ch%u: base=%.1f peak=%.1f edges=(%.2f,%.2f) "
                  "offset=%.2f center=%.2f\n",
                  kTapeCalibrationSensorChannels[sensor], sensorBaseline,
                  sensorPeak, risingEdgeCoordinateCm, fallingEdgeCoordinateCm,
                  sensorAxisOffsetCm, robotCenterEquivalentCm);
  }

  if (validSensorCount == 0) {
    Serial.println("Tape calibration failed: no sensor produced both edges");
    return false;
  }
  const float measuredTapeCenterCm = measuredTapeCenterSumCm / validSensorCount;

  // The robot is now slightly beyond the tape because a peak can only be
  // confirmed after the reading falls. Shift the current pose by the error at
  // the measured edge midpoint instead of physically returning to the tape.
  if (!driveTask.getCurrentPose(&pose)) {
    Serial.println("Tape calibration failed: pose unavailable after search");
    return false;
  }
  if (axis == TapeCalibrationAxis::X) {
    pose.xCm += knownTapeCoordinateCm - measuredTapeCenterCm;
  } else {
    pose.yCm += knownTapeCoordinateCm - measuredTapeCenterCm;
  }
  if (!driveTask.setOtosPose(pose)) {
    Serial.println("Tape calibration failed: pose update not queued");
    return false;
  }

  const char axisName = axis == TapeCalibrationAxis::X ? 'X' : 'Y';
  const float correctedCoordinateCm =
      axis == TapeCalibrationAxis::X ? pose.xCm : pose.yCm;
  Serial.printf("Tape %c calibrated from %u sensor(s): center=%.2f "
                "current %c=%.2f cm drift=(%.2f cm,%.2f deg)\n",
                axisName, validSensorCount, measuredTapeCenterCm, axisName,
                correctedCoordinateCm, maxCrossTrackDriftCm,
                maxHeadingDriftDeg);
  return true;
}

bool calibrateXWithMiddleTapeSensor(float knownTapeXCm, float searchDirection,
                                    float searchPower) {
  return calibrateWithMiddleTapeSensor(TapeCalibrationAxis::X, knownTapeXCm,
                                       searchDirection, searchPower);
}

bool calibrateYWithMiddleTapeSensor(float knownTapeYCm, float searchDirection,
                                    float searchPower) {
  return calibrateWithMiddleTapeSensor(TapeCalibrationAxis::Y, knownTapeYCm,
                                       searchDirection, searchPower);
}

void stowGrabbedRock() {
  armTask.waitUntilSettled(1000);
  armTask.setTargetPosition({10.0f, 7.0f}, true);
  armTask.waitUntilSettled(1500);
  armTask.setTargetPosition({8.480f, 1.093f}, true);
  armTask.waitUntilSettled(1500);
  servo1.setAngle(clawOpenAngle);
  armTask.setTargetPosition({10.80f, 13.0f}, true);
  armTask.waitUntilSettled(1000);
  armTask.setTargetPosition({24, 3}, true);
}

void grabRock() {
  armTask.setTargetPosition({28.0f, -4.5f}, true);
  armTask.waitUntilSettled(1000);
  servo1.setAngle(clawClosedAngle);
  delay(500);
  armTask.setTargetPosition({28.0f, 16.0f}, true);
  delay(500);
  // The rock is secured before the asynchronous stowing motion begins. This
  // also prevents the path logic from starting another grab while it runs.
  rockHeld = true;

  const BaseType_t taskCreated = xTaskCreate(
      [](void *) {
        stowGrabbedRock();
        vTaskDelete(nullptr);
      },
      "stow-rock", kGrabRockTaskStackSize, nullptr, 1, nullptr);

  if (taskCreated != pdPASS) {
    Serial.println("Could not start rock stowing task; running synchronously");
    stowGrabbedRock();
  }
}

void runPath() {
  // delay(2000);
  // Print the three installed tape sensors continuously for diagnostics.

  // delay(1000);
  // servo1.setAngle(clawOpenAngle);
  // delay(5000);
  // servo1.setAngle(clawClosedAngle);
  // while (true) {
  //   continue;
  // }

  while (false) {
    uint16_t tapeReadings[TapeSensorArray::CHANNEL_COUNT];
    tapeSensors.readAll(tapeReadings);
    Serial.printf("Tape raw: right(ch2)=%u center(ch4)=%u left(ch6)=%u\n",
                  tapeReadings[2], tapeReadings[4], tapeReadings[6]);
    delay(50);
  }
  // calibrateXWithMiddleTapeSensor(0.0f, -1.0f, 0.1f);
  driveTask.setTargetPose({-4.5f, 50.0f, 0.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);

  driveTask.setTargetPose({-4.5f, 95.0f, 0.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  if (!rockHeld) {
    driveTask.calibrateImuBlocking(750);
    if (confirmedMetalDetected(metalDetectorRight)) {
      servo1.setAngle(clawOpenAngle);
      driveTask.setTargetPose({-7, 68.5, -43.0f}, 1.0f, true);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({2.5, 77, -40.0f}, 0.25f);
      driveTask.waitUntilMotionFinished(10000);
      grabRock();
      delay(500);
      driveTask.setTargetPose({0, 80, 0.0f}, 1.0f);
      driveTask.waitUntilMotionFinished(10000);
    }
  }

  driveTask.setTargetPose({-1.0f, 116.0f, 0.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({10.5f, 132.0f, 0.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({9.0f, 158.0f, 15.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-7.5f, 168.0f, 61.5f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);

  if (!rockHeld) {
    driveTask.calibrateImuBlocking(750);
    if (confirmedMetalDetected(metalDetectorLeft)) {
      servo1.setAngle(clawOpenAngle);
      driveTask.setTargetPose({-17, 180, 61.0f}, 1.0f);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({-17, 184, -180.0f}, 1.0f);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({-17, 172, -180.0f}, 0.25f);
      delay(500);
      grabRock();
    }
  }

  if (!rockHeld) {
    driveTask.setTargetPose({-3.5f, 175.0f, 61.5f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    delay(650);
    if (confirmedMetalDetected(metalDetectorRight)) {
      servo1.setAngle(clawOpenAngle);
      driveTask.setTargetPose({-30, 185, 61.0f}, 1.0f);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({-28, 193, -90.0f}, 1.0f);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({-19, 193, -90.0f}, 0.25f);
      driveTask.waitUntilMotionFinished(10000);
      delay(500);
      grabRock();
    }
  }

  driveTask.setTargetPose({-28.5f, 187.0f, 90.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-49.5f, 182.0f, 145.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);

  if (!rockHeld) {
    driveTask.calibrateImuBlocking(750);
    if (confirmedMetalDetected(metalDetectorRight)) {
      servo1.setAngle(clawOpenAngle);
      driveTask.setTargetPose({-30, 189, 121.0f}, 1.0f, true);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({-34, 194, 88.0f}, 1.0f, true);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({-43, 194, 88.0f}, 0.25f);
      driveTask.waitUntilMotionFinished(10000);
      grabRock();
    }
  }

  driveTask.setTargetPose({-55.0f, 184.0f, 180.0f}, 0.8f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-55.0f, 150.0f, 180.0f}, 0.8f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-55.0f, 27.0f, 180.0f}, 0.8f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-65.0f, 9.0f, 134.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  if (!rockHeld) {
    driveTask.calibrateImuBlocking(750);
    if (confirmedMetalDetected(metalDetectorLeft)) {
      servo1.setAngle(clawOpenAngle);
      delay(250);
      driveTask.setTargetPose({-74.5f, 22.0f, 147.0f}, 1.0f);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({-74.5f, 20.0f, -140.0f}, 1.0f);
      driveTask.waitUntilMotionFinished(10000);
      driveTask.setTargetPose({-67.0f, 11.0f, -140.0f}, 0.25f);
      driveTask.waitUntilMotionFinished(10000);
      delay(250);
      grabRock();
    }
  }

  if (rockHeld) {
    driveTask.setTargetPose({-72.0f, 13.0f, 134.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
  } else {
    driveTask.setTargetPose({-67, 21.5, 78}, 1.0);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-77.8, 19.2, 78}, 0.25);
    driveTask.waitUntilMotionFinished(10000);
    delay(250);
    grabRock();
  }

  driveTask.setTargetPose({-100.0f, 5.0f, 90.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-130.0f, 5.0f, 90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-130.0f, 5.0f, -90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-130.0f, 0.0f, -90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  if (!calibrateYWithMiddleTapeSensor(4.0f, 1.0f, 0.1f)) {
    Serial.println("Continuing with the existing OTOS Y position");
  }
  driveTask.setTargetPose({-144.0f, 94.0f, -90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-127.0f, 94.0f, -90.0f}, 0.1f);
  driveTask.waitUntilMotionFinished(3000);
  servo1.setAngle(clawOpenAngle);
  delay(250);
  armTask.setTargetPosition({26.0, 4.0}, true);
  armTask.waitUntilSettled(500);
  servo1.setAngle(clawFullyClosedAngle);
  delay(500);

  // OtosSensor::Pose currPose;
  // if (!driveTask.getCurrentPose(&currPose)) {
  //   Serial.println("Current pose unavailable");
  // }
  // driveTask.setOtosPose({currPose.xCm, currPose.yCm, -90.0f});

  armTask.setTargetPosition({24, 18}, true);
  armTask.waitUntilSettled(1000);
  driveTask.setTargetPose({-140.0f, 95.0f, -90.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-140.0f, 140.0f, -90.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-112.0f, 140.0f, -90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(500);
  servo1.setAngle(clawOpenAngle);
  driveTask.setTargetPose({-112.0f, 140.0f, -100.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-112.0f, 140.0f, -80.0f}, 0.3f);
  delay(500);
  driveTask.setMotionTolerance(0.5f, 1.5f);
  driveTask.setTargetPose({-127.0f, 158.5f, 0.0f}, 0.5f);
  delay(250);
  armTask.setTargetPosition({26.5f, -5.0f}, true);
  servo1.setAngle(clawFullyClosedAngle);
  driveTask.waitUntilMotionFinished(10000);
  delay(500);
  OtosSensor::Pose actualPickupPose;
  if (driveTask.getCurrentPose(&actualPickupPose)) {
    const OtosSensor::Pose pickupTarget{-127.0f, 158.5f, 0.0f};
    const OtosSensor::Pose pickupReference{-125.0f, 160.0f, 0.0f};

    const OtosSensor::Pose correctedPickupReference{
        actualPickupPose.xCm - -127.0f + pickupReference.xCm,
        actualPickupPose.yCm - 158.5f + pickupReference.yCm,
        pickupReference.headingDeg + actualPickupPose.headingDeg};
    if (!driveTask.setOtosPose(correctedPickupReference)) {
      Serial.println("Failed to queue corrected pickup pose");
      while (true) {
        Serial.println("Failed to queue corrected pickup pose");
        delay(100);
      }
    }
  } else {
    while (true) {
      Serial.println("Pickup pose unavailable; pose reference not changed");
      delay(100);
    }
  }
  driveTask.setMotionTolerance(0.5f, 1.0f);
  //  driveTask.setOtosPose({-125, 160.5, 0});
  delay(250);
  driveTask.setTargetPose({-137.5f, 160.5f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  // Calibrate X when the centered middle tape sensor is over the known
  // line.
  if (!calibrateXWithMiddleTapeSensor(-151.7f, -1.0f, 0.1f)) {
    Serial.println("Continuing with the existing OTOS X position");
  }
  OtosSensor::Pose currentPose;
  if (!driveTask.getCurrentPose(&currentPose)) {
    Serial.println("Current pose unavailable");
  }
  armTask.setTargetPosition({26.5f, -5.0f}, true);
  driveTask.setTargetPose({currentPose.xCm, 162.0f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(1000);

  // armTask.setTargetPosition({28.0f, -8.0f}, true);
  // servo1.setAngle(clawFullyClosedAngle);
  // delay(500);
  // driveTask.setTargetPose({-125.0f, 170.0f, 0.0f}, 0.2f);
  // driveTask.waitUntilMotionFinished(10000);
  // armTask.setTargetPosition({28.0f, 0.0f}, true);
  // delay(1000);
  // servo1.setAngle(clawHabitatOpenAngle);
  // armTask.setTargetPosition({28.0f, 10.0f}, true);
  // armTask.waitUntilSettled(1000);
  // driveTask.setTargetPose({-125.0f, 138.5f, 0.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-125.0f, 138.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-155.5f, 138.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // armTask.setTargetPosition({28.0f, -8.0f}, true);
  // armTask.waitUntilSettled(1500);
  // servo1.setAngle(clawFullyClosedAngle);
  // delay(500);
  // driveTask.setTargetPose({-155.5f, 138.5f, 85.0f}, 0.4f);
  // delay(500);
  // driveTask.setTargetPose({-155.5f, 138.5f, 95.0f}, 0.4f);
  // delay(500);
  // driveTask.setTargetPose({-147.5f, 138.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-147.5f, 148.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-156.5f, 148.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-156.5f, 126.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-143.5f, 126.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-143.5f, 138.5f, 0.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-143.5f, 163.0f, 0.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);

  driveTask.setTargetPose({-180.5f, 161.5f, 0.0f}, 0.3f);
  servo1.setAngle(clawFullyClosedAngle);
  armTask.setTargetPosition({26.0f, -8.0f}, true);
  driveTask.waitUntilMotionFinished(2250);

  delay(500);
  driveTask.setTargetPose({-180.5f, 170.0f, 0.0f}, 0.1f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(1000);
  servo1.setAngle(clawHabitatOpenAngle);
  armTask.setTargetPosition({28.0f, 10.0f}, true);
  armTask.waitUntilSettled(1000);
  driveTask.setTargetPose({-185.0f, 175.0f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-185.0f, 175.0f, 45.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-155.5f, 138.5f, 45.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-155.5f, 137.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1500);
  servo1.setAngle(clawFullyClosedAngle);
  delay(500);
  driveTask.setTargetPose({-155.5f, 138.5f, 85.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-155.5f, 138.5f, 95.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-147.5f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -5.0f}, true);
  driveTask.setTargetPose({-147.5f, 148.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-158.0f, 148.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-157.0f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-143.5f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-143.5f, 138.5f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);

  driveTask.setTargetPose({-142.5f, 162.0f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(500);
  delay(500);
  driveTask.setTargetPose({-142.5f, 170.0f, 0.0f}, 0.1f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(1000);
  servo1.setAngle(clawHabitatOpenAngle);
  armTask.setTargetPosition({28.0f, 10.0f}, true);
  armTask.waitUntilSettled(1000);
  driveTask.setTargetPose({-143.0f, 138.5f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-143.0f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);

  driveTask.setTargetPose({-155.5f, 137.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1500);
  servo1.setAngle(clawFullyClosedAngle);
  delay(500);
  driveTask.setTargetPose({-155.5f, 138.5f, 85.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-155.5f, 138.5f, 95.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-147.5f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -5.0f}, true);
  driveTask.setTargetPose({-147.5f, 148.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-158.0f, 148.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-157.0f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-143.5f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-161.5f, 161.0f, 0.0f}, 0.3);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(500);
  delay(500);
  driveTask.setTargetPose({-161.5f, 170.0f, 0.0f}, 0.1f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(1000);
  servo1.setAngle(clawHabitatOpenAngle);
  armTask.setTargetPosition({28.0f, 10.0f}, true);
  armTask.waitUntilSettled(1000);
  driveTask.setTargetPose({-161.8f, 150.0f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-143.5f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-155.5f, 137.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1500);
  servo1.setAngle(clawFullyClosedAngle);
  delay(500);
  driveTask.setTargetPose({-155.5f, 138.5f, 85.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-155.5f, 138.5f, 95.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-140.0f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -5.0f}, true);
  driveTask.setTargetPose({-147.5f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-147.5f, 148.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-158.0f, 148.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-157.0f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-143.5f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);

  driveTask.setTargetPose({-124.9f, 162.0f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1000);
  driveTask.setTargetPose({-124.9f, 170.0f, 0.0f}, 0.1f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(1000);
  servo1.setAngle(clawHabitatOpenAngle);
  armTask.setTargetPosition({28.0f, 10.0f}, true);
  armTask.waitUntilSettled(1000);
  driveTask.setTargetPose({-125.0f, 138.5f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-125.0f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-155.5f, 137.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1500);
  servo1.setAngle(clawFullyClosedAngle);
  delay(500);
  driveTask.setTargetPose({-155.5f, 138.5f, 85.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-155.5f, 138.5f, 95.0f}, 0.3f);
  delay(500);
  driveTask.setTargetPose({-147.5f, 138.5f, 90.0f}, 0.3f);

  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-147.5f, 148.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-156.5f, 148.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-156.5f, 126.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-143.5f, 126.5f, 90.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-143.5f, 138.5f, 0.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);
  // driveTask.setTargetPose({-143.5f, 163.0f, 0.0f}, 0.4f);
  // driveTask.waitUntilMotionFinished(10000);

  // driveTask.setTargetPose({-143.5f,
  // delay(500);
  // driveTask.setTargetPose({-126.5f, 166.0f, 0.0f}, 1.0f);
  // driveTask.waitUntilMotionFinished(10000);
  /*
    servo1.setAngle(15);
    delay(3000);
    armTask.setTargetPosition({27.972f, 12.175f}, true);
    delayWithArmLogging(2000);
    armTask.setTargetPosition({23.0f, 1.5f}, true);
    delayWithArmLogging(2000);
    servo1.setAngle(115);
    delayWithArmLogging(1000);
    driveTask.setTargetPose({-138.0f, 97.0f, -90.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    servo1.setAngle(15);
    */
}

void setup() {
  Serial.begin(kSerialBaud);
  const bool servo1Ready = servo1.begin(115.0f);
  driveBase.begin();
  const bool tapeReady = tapeSensors.begin();
  metalDetectorLeftReady = metalDetectorLeft.begin();
  metalDetectorRightReady = metalDetectorRight.begin();
  const uint16_t tapeThreshold = adcCountFromVolts(kTapeThresholdVolts);
  tapeSensors.setThreshold(tapeThreshold);
  otosSensor.begin(OtosSensor::DEFAULT_BAUD_RATE, pins::OTOS_RX_PIN,
                   pins::OTOS_TX_PIN, pins::OTOS_DIR_PIN);
  delay(4000);
  pinMode(pins::EXTRA2_PIN, INPUT_PULLDOWN);
  Serial.println("Starting PWM expander");
  //
  const bool pwmReady = pwmExpander.begin(pins::PWM_EXPANDER_SDA_PIN,
                                          pins::PWM_EXPANDER_SCL_PIN, 50.0f);
  elbowEncoderReady = elbowEncoder.begin();
  shoulderEncoderReady = shoulderEncoder.begin();
  const bool encoderMuxReady = elbowEncoder.muxIsConnected();
  const bool otosReady = otosSensor.ping();
  Serial.println("PCA9685 " + String(pwmReady ? "ready" : "not found"));
  Serial.println("Servo 1 " +
                 String(servo1Ready ? "ready at 90 degrees" : "not started"));
  Serial.println("TCA9548 encoder mux " +
                 String(encoderMuxReady ? "ready" : "not found"));
  Serial.println("Shoulder AS5600 encoder " +
                 String(shoulderEncoderReady ? "ready" : "not found"));
  if (shoulderEncoderReady) {
    Serial.println(
        "Shoulder AS5600 magnet " +
        String(shoulderEncoder.magnetDetected() ? "detected" : "missing"));
  }
  Serial.println("Elbow AS5600 encoder " +
                 String(elbowEncoderReady ? "ready" : "not found"));
  if (elbowEncoderReady) {
    Serial.println("Elbow AS5600 magnet " + String(elbowEncoder.magnetDetected()
                                                       ? "detected"
                                                       : "missing"));
  }
  Serial.println("OTOS sensor " + String(otosReady ? "ready" : "not found"));
  Serial.println("MCP3008 " + String(tapeReady ? "ready" : "not found"));
  Serial.println("Metal detector left " +
                 String(metalDetectorLeftReady ? "ready" : "not started"));
  Serial.println("Metal detector right " +
                 String(metalDetectorRightReady ? "ready" : "not started"));
  Serial.printf("Tape threshold: %.2f V adc=%u\n", kTapeThresholdVolts,
                tapeThreshold);
  const bool initialPoseSet =
      otosReady && otosSensor.setPose({0.0f, 0.0f, 0.0f});
  while (digitalRead(pins::EXTRA1_PIN) == LOW) {
    Serial.println("Waiting for motor disable");
    delay(100);
  }

  Arm::JointAngles currentArmAngles;
  const bool armAnglesReady = shoulderEncoderReady && elbowEncoderReady &&
                              robotArm.readAngles(&currentArmAngles);

  pwmMutex = xSemaphoreCreateMutex();
  driveTaskReady =
      pwmReady && otosReady && initialPoseSet && driveTask.begin(pwmMutex);
  armTaskReady = pwmReady && armAnglesReady && armTask.begin(pwmMutex);
  Serial.println("Drive task " +
                 String(driveTaskReady ? "ready" : "not started"));
  Serial.println("Arm task " + String(armTaskReady ? "ready" : "not started"));
  if (armTaskReady) {
    const bool gravityCompensationConfigured =
        armConfig.gravityA1Percent != 0.0f ||
        armConfig.gravityA12Percent != 0.0f ||
        armConfig.gravityA2Percent != 0.0f;
    // gravityCompensationReady =
    //     gravityCompensationConfigured && armTask.setTargetAngles({90, 90});
    // Serial.println("Gravity compensation " +
    //                String(gravityCompensationReady ? "ready" : "not ready"));

    // Joint-angle commands return immediately and execute in the arm task:
    // while (true) {
    //   armTask.setTargetAngles({55.0f, -70.0f});
    //   delay(2000);
    //   armTask.setTargetAngles({90.0f, -130.0f});
    //   delay(2000);
    // }
    // Once zero offsets, motor directions, and link lengths are calibrated,
    // command an XY target in centimeters from the shoulder joint like this:
    // armTask.setTargetPosition({20.0f, 15.0f}, true);
  }

  // Reset the OTOS coordinate system through its owning task:
  driveTask.calibrateImuBlocking(2000);
  driveTask.setOtosPose({0.0f, 0.0f, 0.0f});
  // hbridge1.setSpeedPercent(20.0f);
  // hbridge2.setSpeedPercent(20.0f);
  // hbridge3.setSpeedPercent(20.0f);
  // hbridge4.setSpeedPercent(20.0f);
  // hbridge6.setSpeedPercent(20.0f);
  // hbridge4.setSpeedPercent(20.0f);

  // while (true) {
  //   Arm::JointAngles armAngles;
  //   if (armTaskReady && armTask.getCurrentAngles(&armAngles)) {
  //     Serial.printf("Arm shoulder=%.1f deg | elbow=%.1f deg | target=%s\n",
  //                   armAngles.shoulderDeg, armAngles.elbowDeg,
  //                   armTask.atTarget() ? "yes" : "no");
  //   } else {
  //     Serial.println("Arm position unavailable");
  //   }
  //   delay(100);
  // }
  // Fully disconnect the PWM peripheral from the servo signal pin. Call
  // servo1.enable() to resume output at the last commanded angle.
  servo1.enable();
  // servo1.setAngle(170);
  // delay(1000);
  // servo1.disable();
  // armTask.setTargetAngles({90.0f, -90.0f});
  servo1.setAngle(clawOpenAngle);
  armTask.setTargetPosition({28.0f, -4.5f}, true);
  // armTask.waitUntilSettled(1500);
  // grabRock();
  // servo1.setAngle(130);
  // delay(500);
  // armTask.setTargetPosition({28.0f, 16.0f}, true);
  // armTask.waitUntilSettled(1500);
  // armTask.setTargetPosition({10.0f, 7.0f}, true);
  // armTask.waitUntilSettled(1500);
  // armTask.setTargetPosition({8.480f, 1.093f}, true);
  // armTask.waitUntilSettled(1500);
  // servo1.setAngle(clawOpenAngles);
  // armTask.setTargetPosition({10.80f, 13.0f}, true);

  runPath();
  // delay(2000);
  // Serial.printf("calibrating X with middle tape sensor\n");
  // calibrateXWithMiddleTapeSensor(0.0f, -1.0f, 0.05f);
  // Serial.printf("calibrated\n");

  // driveTask.setTargetPose({0.0f, 0.0f, 0.0f}, 0.1f);
  // armTask.setTargetPosition({25.0f, 11.0f}, true);
  // armTask.waitUntilSettled(3000);
  // armTask.setTargetPosition({15.0f, 11.0f}, true);
  // armTask.waitUntilSettled(3000);
  // armTask.setTargetPosition({8.480f, 1.093f}, true);
  // armTask.waitUntilSettled(3000);
  // // servo1.setAngle(clawOpenAngle);
  // armTask.setTargetPosition({10.80f, 13.0f}, true);

  while (false) {
    MetalDetector::Reading detectorLeftReading;
    MetalDetector::Reading detectorRightReading;
    const bool detectorLeftValid =
        metalDetectorLeftReady &&
        metalDetectorLeft.getReading(&detectorLeftReading);
    const bool detectorRightValid =
        metalDetectorRightReady &&
        metalDetectorRight.getReading(&detectorRightReading);

    const char *detectorLeftState = !detectorLeftValid ? "OFFLINE"
                                    : detectorLeftReading.counterSaturated
                                        ? "OVERFLOW"
                                    : !detectorLeftReading.baselineReady ? "CAL"
                                    : detectorLeftReading.anomaly ? "METAL"
                                                                  : "OK";
    const char *detectorRightState =
        !detectorRightValid                     ? "OFFLINE"
        : detectorRightReading.counterSaturated ? "OVERFLOW"
        : !detectorRightReading.baselineReady   ? "CAL"
        : detectorRightReading.anomaly          ? "METAL"
                                                : "OK";

    Serial.printf(
        "MD LEFT %.1f Hz | base %.1f | delta %.1f | %s  ||  "
        "MD RIGHT %.1f Hz | base %.1f | delta %.1f | %s\n",
        detectorLeftReading.frequencyHz, detectorLeftReading.baselineHz,
        detectorLeftReading.deviationHz, detectorLeftState,
        detectorRightReading.frequencyHz, detectorRightReading.baselineHz,
        detectorRightReading.deviationHz, detectorRightState);
    delay(100);
  }
}

void loop() {
  static uint32_t lastArmLogMs = 0;
  const uint32_t nowMs = millis();
  if (armTaskReady && nowMs - lastArmLogMs >= 100) {
    lastArmLogMs = nowMs;
    logArmPositionAndRobotPose();
  }

  // Metal Detector Test
  // static uint32_t lastMetalDetectorPrintMs = 0;
  // if (millis() - lastMetalDetectorPrintMs >= 100) {
  //   lastMetalDetectorPrintMs = millis();

  //   MetalDetector::Reading detectorLeftReading;
  //   MetalDetector::Reading detectorRightReading;
  //   const bool detectorLeftValid =
  //       metalDetectorLeftReady &&
  //       metalDetectorLeft.getReading(&detectorLeftReading);
  //   const bool detectorRightValid =
  //       metalDetectorRightReady &&
  //       metalDetectorRight.getReading(&detectorRightReading);

  //   char detectorLeftFrequency[16];
  //   char detectorLeftBaseline[16];
  //   char detectorRightFrequency[16];
  //   char detectorRightBaseline[16];

  //   if (detectorLeftValid) {
  //     snprintf(detectorLeftFrequency, sizeof(detectorLeftFrequency), "%.1f",
  //              detectorLeftReading.frequencyHz);
  //     snprintf(detectorLeftBaseline, sizeof(detectorLeftBaseline), "%.1f",
  //              detectorLeftReading.baselineHz);
  //   } else {
  //     snprintf(detectorLeftFrequency, sizeof(detectorLeftFrequency), "--");
  //     snprintf(detectorLeftBaseline, sizeof(detectorLeftBaseline), "--");
  //   }

  //   if (detectorRightValid) {
  //     snprintf(detectorRightFrequency, sizeof(detectorRightFrequency),
  //     "%.1f",
  //              detectorRightReading.frequencyHz);
  //     snprintf(detectorRightBaseline, sizeof(detectorRightBaseline), "%.1f",
  //              detectorRightReading.baselineHz);
  //   } else {
  //     snprintf(detectorRightFrequency, sizeof(detectorRightFrequency), "--");
  //     snprintf(detectorRightBaseline, sizeof(detectorRightBaseline), "--");
  //   }

  //   const char *detectorLeftState = !detectorLeftValid ? "OFFLINE"
  //                                   : detectorLeftReading.counterSaturated
  //                                    ? "OVERFLOW"
  //                                   : !detectorLeftReading.baselineReady ?
  //                                   "CAL" : detectorLeftReading.anomaly ?
  //                                "METAL"
  //                                                                  : "OK";
  //   const char *detectorRightState = !detectorRightValid ? "OFFLINE"
  //                                    : detectorRightReading.counterSaturated
  //                                    ? "OVERFLOW"
  //                                    : !detectorRightReading.baselineReady ?
  //                                    "CAL" : detectorRightReading.anomaly ?
  //                                "METAL"
  //                                                                  : "OK";

  //   // Carriage return updates one fixed-width dashboard line instead of
  //   // continuously scrolling the serial monitor.
  //   Serial.printf("\rMD LEFT %9s Hz | base %9s | %-8s  ||  "
  //                 "MD RIGHT %9s Hz | base %9s | %-8s    ",
  //                 detectorLeftFrequency, detectorLeftBaseline,
  //                 detectorLeftState, detectorRightFrequency,
  //                 detectorRightBaseline, detectorRightState);
  // }

  // if (armTaskReady) {
  //   static uint32_t lastArmPrintMs = 0;
  //   if (millis() - lastArmPrintMs >= 250) {
  //     lastArmPrintMs = millis();

  //     Arm::JointAngles armAngles;
  //     if (armTask.getCurrentAngles(&armAngles)) {
  //       Serial.printf("Arm shoulder=%.1f elbow=%.1f target=%s\n",
  //                     armAngles.shoulderDeg, armAngles.elbowDeg,
  //                     armTask.atTarget() ? "yes" : "no");
  //     } else {
  //       Serial.println("Arm encoder read failed");
  //     }
  //   }
  // }
  // delay(100);

  // hbridge6.setSpeedPercent(-20.0f);

  /*
  enum class TapeCenterState : uint8_t {
    Searching,
    Reversing,
    Centered,
  };

  static float filteredch4 = 0.0f;
  static float bestch4 = 0.0f;
  static TapeCenterState tapeCenterState = TapeCenterState::Searching;

  constexpr float kTapeSearchSpeed = 0.02f;
  constexpr float kTapeReverseSpeed = -0.02f;
  constexpr float kMinTapeReading = 100.0f;
  constexpr float kPeakDropToReverse = 15.0f;
  constexpr float kPeakCloseEnough = 5.0f;

  uint16_t tapeReadings[TapeSensorArray::CHANNEL_COUNT];
  tapeSensors.readAll(tapeReadings);
  filteredch4 = 0.9f * filteredch4 + 0.1f *
  static_cast<float>(tapeReadings[4]);

  Serial.print("Tape:");
  // Serial.printf(" ch%u=%u", 2, tapeReadings[2]);
  // Serial.printf(" ch%u=%u", 4, tapeReadings[4]);
  Serial.printf(" ch%u=%.1f %.2fV", 4, filteredch4,
                voltsFromAdcCount(static_cast<uint16_t>(filteredch4)));
  Serial.printf(" threshold=%u %.2fV", tapeSensors.threshold(4),
                voltsFromAdcCount(tapeSensors.threshold(4)));
  Serial.printf(" best=%.1f", bestch4);
  Serial.println();

  float speed = 0.0f;
  switch (tapeCenterState) {
  case TapeCenterState::Searching:
    speed = kTapeSearchSpeed;

    if (filteredch4 > kMinTapeReading) {
      if (filteredch4 > bestch4) {
        bestch4 = filteredch4;
      }

      if (bestch4 - filteredch4 > kPeakDropToReverse) {
        tapeCenterState = TapeCenterState::Reversing;
        Serial.println("Tape sensor 4 peak passed");
      }
    }
    break;

  case TapeCenterState::Reversing:
    if (bestch4 - filteredch4 > kPeakCloseEnough) {
      speed = kTapeReverseSpeed;
    } else {
      tapeCenterState = TapeCenterState::Centered;
      Serial.println("Tape sensor 4 centered");
    }
    break;

  case TapeCenterState::Centered:
    speed = 0.0f;
    break;
  }

  driveBase.drive(speed, 0, 0);
  delay(10);
*/
  // OtosSensor::Pose currentPose;
  // if (!otosSensor.getPose(&currentPose)) {
  //   Serial.println("Failed to get pose");
  //   delay(100);
  // } else {
  //   printf("x=%.1f y=%.1f heading=%.1f\n", currentPose.xCm,
  //   currentPose.yCm,
  //          currentPose.headingDeg);
  // }
  // delay(100);

  // The main task can read the latest OTOS snapshot without accessing the
  // sensor directly:
  // OtosSensor::Pose pose;
  // if (driveTaskReady && driveTask.getCurrentPose(&pose)) {
  //   Serial.printf("Drive x=%.1f y=%.1f heading=%.1f\n", pose.xCm, pose.yCm,
  //                 pose.headingDeg);
  // }

  delay(10);
}
