#include "angle_servo.h"
#include "arm.h"
#include "as5600_encoder.h"
#include "camera_detector.h"
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
// LEDC channels 0 and 1 share a timer. Servo 1 uses channel 0 at 50 Hz, so
// use channel 2 to keep the LED's 5 kHz PWM on a separate timer.
constexpr uint8_t kLedPwmChannel = 2;
constexpr uint16_t kLedPwmFrequencyHz = 5000;
constexpr uint8_t kLedPwmResolutionBits = 8;
constexpr float kInitialLedBrightnessPercent = 10.0f;
constexpr uint32_t kLedBlinkIntervalMs = 100;
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
constexpr float kTapeCalibrationMaxHeadingPower = 0.15f;
constexpr float kTapeCalibrationMinHeadingPower = 0.00f;
constexpr float kTapeCalibrationHeadingToleranceDeg = 1.0f;
constexpr float kTapeCalibrationMinPeakRise = 100.0f;
// Average several MCP3008 conversions at each calibration position before the
// spatial profile and temporal median filters see the reading.
constexpr uint8_t kTapeCalibrationOversampleCount = 4;
// Use a low percentile of the background profile rather than its absolute
// minimum. This remains representative of the floor while ignoring isolated
// low readings that survive the short temporal median filter.
constexpr uint8_t kTapeCalibrationBaselinePercentile = 25;
// The rolling five-reading median already rejects isolated ADC spikes.
// Requiring five additional spatial profile samples made narrow tape easy to
// miss unless the robot crossed it unusually slowly.
constexpr uint8_t kTapeCalibrationRiseConfirmSamples = 2;
// The outer sensors can cross an axis-aligned tape up to 2.4 cm after the
// middle sensor. Continue beyond the middle peak so all three profiles include
// their falling half-height edge.
constexpr float kTapeCalibrationMinTravelPastPeakCm = 3.5f;
constexpr uint8_t kTapeCalibrationPeakConfirmSamples = 1;
constexpr float kTapeCalibrationProfileSpacingCm = 0.05f;
constexpr uint16_t kTapeCalibrationMaxProfileSamples = 640;
constexpr uint32_t kCalibrationSettleDelayMs = 250;
constexpr float kMetalAnomalyThresholdHz = 110.0f;
constexpr float kMetalThresholdHighBatteryVoltage = 16.8f;
constexpr float kMetalThresholdLowBatteryVoltage = 14.7f;
constexpr float kMetalLowBatteryAnomalyThresholdHz = 60.0f;
constexpr uint8_t kMetalDeviationAverageSamples = 5;
// Calibrated against a 16.20 V multimeter reading (ADC initially reported
// 15.77 V with the nominal 6.5:1 divider ratio).
constexpr float kBatteryVoltageDividerRatio = 6.69f;
constexpr float kNormalizedMotorVoltage = 15.4f;
enum class ArmTuningMode { Disabled, StaticFriction, Velocity, Position };

// StaticFriction applies the signed manual PWM below on top of gravity.
// Velocity applies the fixed signed target speed below directly to the inner
// velocity loop. Position alternates a bounded angle step through the cascaded
// controller. The other joint holds its startup angle and autonomous operation
// is bypassed.
constexpr ArmTuningMode kArmTuningMode = ArmTuningMode::Disabled;
constexpr bool kTuneShoulder = false;
constexpr bool kHoldOtherJointDuringTuning = true;
constexpr float kArmStaticFrictionManualPwmPercent = 20.0f;
constexpr float kArmManualTargetVelocityDegPerSec = 10.0f;
constexpr float kArmVelocityTuningMaxTravelDeg = 45.0f;
constexpr float kArmPositionTuningStepDeg = 30.0f;
constexpr float kArmPositionTuningMaxVelocityDegPerSec = 50.0f;
constexpr uint32_t kArmPositionTuningInitialHoldMs = 2000;
constexpr uint32_t kArmPositionTuningStepHoldMs = 4000;
// Exponential velocity filter coefficient: lower is smoother but adds lag.
// At the 4 ms arm update period, 0.10 is approximately a 4 Hz low-pass filter.
constexpr float kArmVelocityFilterAlpha = 0.10f;
constexpr bool kArmStaticFrictionTuningEnabled =
    kArmTuningMode == ArmTuningMode::StaticFriction;
constexpr bool kArmVelocityTuningEnabled =
    kArmTuningMode == ArmTuningMode::Velocity;
constexpr bool kArmPositionTuningEnabled =
    kArmTuningMode == ArmTuningMode::Position;
constexpr bool kArmTuningEnabled = kArmTuningMode != ArmTuningMode::Disabled;
constexpr uint32_t kArmVelocityTuningLogPeriodMs = 100;

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
constexpr float clawClosedAngle = 105.0f;
constexpr float clawFullyClosedAngle = 135.0f;

constexpr float kIdleLedBrightnessPercent = 50.0f;

int teletubbyCount = 0;
bool rockHeld = false;

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

float tapeReadingPercentile(
    const uint16_t histogram[TapeSensorArray::MAX_READING + 1],
    uint16_t sampleCount, uint8_t percentile) {
  if (sampleCount == 0) {
    return 0.0f;
  }

  const uint32_t targetRank =
      (static_cast<uint32_t>(sampleCount - 1) * percentile) / 100U;
  uint32_t cumulativeCount = 0;
  for (uint16_t reading = 0; reading <= TapeSensorArray::MAX_READING;
       ++reading) {
    cumulativeCount += histogram[reading];
    if (cumulativeCount > targetRank) {
      return static_cast<float>(reading);
    }
  }
  return static_cast<float>(TapeSensorArray::MAX_READING);
}

float batteryVoltageForMetalDetector();

MetalDetector::Config makeMetalDetectorConfig() {
  MetalDetector::Config config;
  config.anomalyThresholdHz = kMetalAnomalyThresholdHz;
  config.highBatteryVoltage = kMetalThresholdHighBatteryVoltage;
  config.lowBatteryVoltage = kMetalThresholdLowBatteryVoltage;
  config.lowBatteryAnomalyThresholdHz = kMetalLowBatteryAnomalyThresholdHz;
  config.batteryVoltageProvider = batteryVoltageForMetalDetector;
  config.deviationAverageSamples = kMetalDeviationAverageSamples;
  return config;
}

PwmExpander pwmExpander;

float batteryVoltageForMetalDetector() { return pwmExpander.supplyVoltage(); }

AngleServo servo1(pins::SERVO1_PWM_PIN);
OtosSensor otosSensor(Serial1);
CameraDetector cameraDetector(Serial2);
TapeSensorArray tapeSensors;
MetalDetector::Config metalDetectorConfig = makeMetalDetectorConfig();
MetalDetector metalDetectorLeft(pins::MD_RIGHT_PIN, PCNT_UNIT_0,
                                metalDetectorConfig);
MetalDetector metalDetectorRight(pins::MD_LEFT_PIN, PCNT_UNIT_1,
                                 metalDetectorConfig);

bool confirmedMetalDetected(const MetalDetector &detector) {
  MetalDetector::Reading reading;
  bool isMetal = detector.getReading(&reading) && reading.baselineReady &&
                 !reading.counterSaturated &&
                 reading.averagedSampleCount >= kMetalDeviationAverageSamples &&
                 reading.anomaly;
  rockHeld = isMetal;
  return isMetal;
}

As5600Encoder elbowEncoder(pins::ENCODER_MUX_CHANNEL0_PIN);
As5600Encoder shoulderEncoder(pins::ENCODER_MUX_CHANNEL1_PIN);
bool elbowEncoderReady = false;
bool shoulderEncoderReady = false;

Arm::Config makeArmConfig() {
  Arm::Config config;
  config.motorDisablePin = pins::EXTRA2_PIN;
  config.gravityHoldEnabled = !kArmTuningEnabled;
  config.maxCartesianSpeedCmPerSec = 30.0f;
  config.pidReenableDriftDeg = 1.0f;
  // Signed PWM percentages required to counter gravity when the corresponding
  // link is horizontal. Reverse a sign if compensation assists gravity.
  config.gravityA1Percent = 0.0f;
  config.gravityA12Percent = 0.0f;
  config.gravityA2Percent = 0.0f;

  config.shoulder.encoderReferenceDeg = kShoulderEncoderDegAtZero;
  config.shoulder.jointReferenceDeg = 0.0f;
  config.shoulder.direction = 1.0f;
  // The outer position PID outputs deg/s. The inner velocity PID converts its
  // deg/s error to PWM percent. These are safe starting values and should be
  // tuned on the assembled arm, velocity loop first.
  config.shoulder.positionKp = 4.0f;
  config.shoulder.positionKi = 0.0f;
  config.shoulder.positionKd = 0.3f;
  config.shoulder.velocityKp = 0.2f;
  config.shoulder.velocityKi = 0.0f;
  config.shoulder.velocityKd = 0.03f;
  config.shoulder.velocityAlpha = kArmVelocityFilterAlpha;
  config.shoulder.kVPercentPerDegPerSec = 0.15f;
  config.shoulder.maxVelocityDegPerSec = 100.0f;
  config.shoulder.constantPidTestEnabled = false;
  config.shoulder.positiveStaticFrictionPercent = 15.0f;
  config.shoulder.negativeStaticFrictionPercent = 15.0f;
  config.shoulder.minAngleDeg = kShoulderMinAngleDeg;
  config.shoulder.maxAngleDeg = kShoulderMaxAngleDeg;
  config.shoulder.positionToleranceDeg = 1.0f;
  config.shoulder.positionIntegralLimitDegSec = 30.0f;
  config.shoulder.velocityIntegralLimitDeg = 30.0f;
  config.shoulder.maxPwmPercent = 50.0f;
  config.shoulder.maxOutputSlewPercentPerSec = 4000.0f;
  config.shoulder.constantPidOutputPercent = 0.0f;

  config.elbow.encoderReferenceDeg = kElbowEncoderDegAtZero;
  config.elbow.jointReferenceDeg = 0.0f;
  config.elbow.direction = -1.0f;
  config.elbow.positionKp = 5.0f;
  config.elbow.positionKi = 0.0f;
  config.elbow.positionKd = 0.0f;
  config.elbow.velocityKp = 0.15f;
  config.elbow.velocityKi = 0.0f;
  config.elbow.velocityKd = 0.01f;
  config.elbow.velocityAlpha = kArmVelocityFilterAlpha;
  config.elbow.kVPercentPerDegPerSec = 0.12f;
  config.elbow.maxVelocityDegPerSec = 50.0f;
  config.elbow.constantPidTestEnabled = false;
  config.elbow.constantPidOutputPercent = 0.0f;

  config.elbow.positiveStaticFrictionPercent = 6.0f;
  config.elbow.negativeStaticFrictionPercent = 3.0f;
  config.elbow.minAngleDeg = kElbowMinAngleDeg;
  config.elbow.maxAngleDeg = kElbowMaxAngleDeg;
  config.elbow.positionToleranceDeg = 1.0f;
  config.elbow.positionIntegralLimitDegSec = 30.0f;
  config.elbow.velocityIntegralLimitDeg = 30.0f;
  config.elbow.maxPwmPercent = 50.0f;
  config.elbow.maxOutputSlewPercentPerSec = 4000.0f;

  if (kArmStaticFrictionTuningEnabled) {
    Arm::JointConfig &selectedJoint =
        kTuneShoulder ? config.shoulder : config.elbow;
    selectedJoint.constantPidTestEnabled = true;
    selectedJoint.constantPidOutputPercent = kArmStaticFrictionManualPwmPercent;
  } else if (kArmVelocityTuningEnabled) {
    Arm::JointConfig &selectedJoint =
        kTuneShoulder ? config.shoulder : config.elbow;
    selectedJoint.manualVelocityTestEnabled = true;
    selectedJoint.manualTargetVelocityDegPerSec =
        kArmManualTargetVelocityDegPerSec;
  } else if (kArmPositionTuningEnabled) {
    Arm::JointConfig &selectedJoint =
        kTuneShoulder ? config.shoulder : config.elbow;
    selectedJoint.maxVelocityDegPerSec = kArmPositionTuningMaxVelocityDegPerSec;
  }

  // Optionally force the unselected motor to zero. By default it instead holds
  // its measured startup angle using its normal cascaded controller.
  if (kArmTuningEnabled && !kHoldOtherJointDuringTuning) {
    if (kTuneShoulder) {
      config.elbow.maxPwmPercent = 0.0f;
    } else {
      config.shoulder.maxPwmPercent = 0.0f;
    }
  }

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
Motor hbridge5(pwmExpander, pins::HBRIDGE4_PWM1_PIN, pins::HBRIDGE4_PWM2_PIN);
Motor hbridge6(pwmExpander, pins::HBRIDGE3_PWM2_PIN, pins::HBRIDGE3_PWM1_PIN);

MecanumDrive driveBase(hbridge1, hbridge2, hbridge3, hbridge4,
                       pins::EXTRA1_PIN);
Arm::Config armConfig = makeArmConfig();
Arm robotArm(hbridge5, hbridge6, shoulderEncoder, elbowEncoder, armConfig);
DriveTask driveTask(driveBase, otosSensor);
ArmTask armTask(robotArm);
SemaphoreHandle_t pwmMutex = nullptr;
bool driveTaskReady = false;
bool armTaskReady = false;
bool armStaticFrictionTuningReady = false;
uint32_t lastArmStaticFrictionLogMs = 0;
bool armVelocityTuningReady = false;
uint32_t lastArmVelocityTuningLogMs = 0;
float armVelocityTuningStartAngleDeg = 0.0f;
bool armPositionTuningReady = false;
bool armPositionTuningAtStep = false;
uint32_t nextArmPositionTuningCommandMs = 0;
uint32_t lastArmPositionTuningLogMs = 0;
Arm::JointAngles armPositionTuningBaseAngles;
Arm::JointAngles armPositionTuningStepAngles;
bool gravityCompensationReady = false;
bool metalDetectorLeftReady = false;
bool metalDetectorRightReady = false;
bool cameraDetectorReady = false;
bool ledPwmReady = false;

constexpr uint32_t kGrabRockTaskStackSize = 3072;

bool setLedBrightness(float percent) {
  if (!ledPwmReady || !isfinite(percent)) {
    return false;
  }

  percent = constrain(percent, 0.0f, 100.0f);
  const uint32_t maxDuty = (1UL << kLedPwmResolutionBits) - 1UL;
  const uint32_t duty =
      static_cast<uint32_t>((percent / 100.0f) * maxDuty + 0.5f);
  ledcWrite(kLedPwmChannel, duty);
  return true;
}

bool beginLedPwm() {
  if (ledcSetup(kLedPwmChannel, kLedPwmFrequencyHz, kLedPwmResolutionBits) <=
      0.0) {
    return false;
  }

  ledcAttachPin(pins::SERVO2_PWM_PIN, kLedPwmChannel);
  ledPwmReady = true;
  return setLedBrightness(kIdleLedBrightnessPercent);
}

bool blinkLeds(uint32_t durationMs) {
  if (!ledPwmReady) {
    return false;
  }
  // setLedBrightness(0.0f);
  // delay(durationMs);
  // return setLedBrightness(kIdleLedBrightnessPercent);

  const uint32_t startMs = millis();
  bool ledsOn = true;

  while (millis() - startMs < durationMs) {
    if (!setLedBrightness(ledsOn ? 100.0f : 0.0f)) {
      return false;
    }

    const uint32_t elapsedMs = millis() - startMs;
    const uint32_t remainingMs =
        elapsedMs < durationMs ? durationMs - elapsedMs : 0;
    delay(min(kLedBlinkIntervalMs, remainingMs));
    ledsOn = !ledsOn;
  }

  return setLedBrightness(kIdleLedBrightnessPercent);
}

bool beginArmStaticFrictionTuning(const Arm::JointAngles &startingAngles) {
  if (!armTask.setTargetAngles(startingAngles)) {
    Serial.println("ARM_FRICTION setup failed: hold command rejected");
    return false;
  }

  lastArmStaticFrictionLogMs = 0;
  Serial.printf("ARM_FRICTION joint=%s manual=%+.1f%% plus gravity\n",
                kTuneShoulder ? "shoulder" : "elbow",
                kArmStaticFrictionManualPwmPercent);
  Serial.printf("ARM_FRICTION %s holds its startup angle\n",
                kTuneShoulder ? "elbow" : "shoulder");
  Serial.println(
      "ARM_FRICTION log: angle deg, velocity deg/s, outputs PWM percent");
  return true;
}

void updateArmStaticFrictionTuning(uint32_t nowMs) {
  if (!armStaticFrictionTuningReady ||
      nowMs - lastArmStaticFrictionLogMs < kArmVelocityTuningLogPeriodMs) {
    return;
  }
  lastArmStaticFrictionLogMs = nowMs;

  Arm::Telemetry telemetry;
  if (!armTask.getTelemetry(&telemetry)) {
    return;
  }
  const Arm::JointTelemetry &joint =
      kTuneShoulder ? telemetry.shoulder : telemetry.elbow;
  Serial.printf("%s angle %7.2f  vel %+7.2f  manual %+6.1f  gravity %+6.1f  "
                "pwm %+6.1f%s%s\n",
                kTuneShoulder ? "SH" : "EL", joint.measuredPositionDeg,
                joint.measuredVelocityDegPerSec,
                kArmStaticFrictionManualPwmPercent, joint.gravityPercent,
                joint.finalPwmPercent, joint.saturated ? " SAT" : "",
                joint.slewLimited ? " SLEW" : "");
}

bool beginArmVelocityTuning(const Arm::JointAngles &startingAngles) {
  armVelocityTuningStartAngleDeg =
      kTuneShoulder ? startingAngles.shoulderDeg : startingAngles.elbowDeg;
  if (!armTask.setTargetAngles(startingAngles)) {
    Serial.println("ARM_VELOCITY setup failed: start command rejected");
    return false;
  }

  lastArmVelocityTuningLogMs = 0;
  Serial.printf(
      "ARM_VELOCITY joint=%s target=%+.1f deg/s max_travel=%.1f deg\n",
      kTuneShoulder ? "shoulder" : "elbow", kArmManualTargetVelocityDegPerSec,
      kArmVelocityTuningMaxTravelDeg);
  Serial.printf("ARM_VELOCITY %s holds its startup angle\n",
                kTuneShoulder ? "elbow" : "shoulder");
  Serial.println(
      "ARM_VELOCITY log: velocity is measured/target; outputs are PWM percent");
  return true;
}

void updateArmVelocityTuning(uint32_t nowMs) {
  if (!armVelocityTuningReady) {
    return;
  }

  if (nowMs - lastArmVelocityTuningLogMs < kArmVelocityTuningLogPeriodMs) {
    return;
  }
  lastArmVelocityTuningLogMs = nowMs;

  Arm::Telemetry telemetry;
  if (!armTask.getTelemetry(&telemetry)) {
    return;
  }
  const Arm::JointTelemetry &joint =
      kTuneShoulder ? telemetry.shoulder : telemetry.elbow;
  const Arm::JointConfig &jointConfig =
      kTuneShoulder ? armConfig.shoulder : armConfig.elbow;
  const float travel =
      fabsf(joint.measuredPositionDeg - armVelocityTuningStartAngleDeg);
  const bool nearDirectionalLimit =
      (kArmManualTargetVelocityDegPerSec > 0.0f &&
       joint.measuredPositionDeg >= jointConfig.maxAngleDeg - 5.0f) ||
      (kArmManualTargetVelocityDegPerSec < 0.0f &&
       joint.measuredPositionDeg <= jointConfig.minAngleDeg + 5.0f);
  if (travel >= fabsf(kArmVelocityTuningMaxTravelDeg) || nearDirectionalLimit) {
    armTask.cancel();
    armVelocityTuningReady = false;
    Serial.printf("ARM_VELOCITY stopped after %.1f deg of travel\n", travel);
    return;
  }

  const float totalFeedforward = joint.velocityFeedforwardPercent +
                                 joint.frictionPercent + joint.gravityPercent;
  Serial.printf("%s angle %6.1f  vel %+6.1f/%+6.1f  err %+6.1f  "
                "P/I/D %+5.1f/%+5.1f/%+5.1f  ff %+5.1f  pwm %+5.1f%s%s\n",
                kTuneShoulder ? "SH" : "EL", joint.measuredPositionDeg,
                joint.measuredVelocityDegPerSec, joint.targetVelocityDegPerSec,
                joint.velocityErrorDegPerSec, joint.velocityPOutputPercent,
                joint.velocityIOutputPercent, joint.velocityDOutputPercent,
                totalFeedforward, joint.finalPwmPercent,
                joint.saturated ? " SAT" : "",
                joint.slewLimited ? " SLEW" : "");
}

bool beginArmPositionTuning(const Arm::JointAngles &startingAngles) {
  armPositionTuningBaseAngles = startingAngles;
  armPositionTuningStepAngles = startingAngles;

  const Arm::JointConfig &jointConfig =
      kTuneShoulder ? armConfig.shoulder : armConfig.elbow;
  const float startingAngle =
      kTuneShoulder ? startingAngles.shoulderDeg : startingAngles.elbowDeg;
  float step = fabsf(kArmPositionTuningStepDeg);
  if (startingAngle + step > jointConfig.maxAngleDeg - 5.0f) {
    step = -step;
  }
  if (kTuneShoulder) {
    armPositionTuningStepAngles.shoulderDeg += step;
  } else {
    armPositionTuningStepAngles.elbowDeg += step;
  }

  if (!armTask.setTargetAngles(armPositionTuningBaseAngles)) {
    Serial.println("ARM_POSITION setup failed: hold command rejected");
    return false;
  }

  armPositionTuningAtStep = false;
  nextArmPositionTuningCommandMs = millis() + kArmPositionTuningInitialHoldMs;
  lastArmPositionTuningLogMs = 0;
  Serial.printf(
      "ARM_POSITION joint=%s base=%.1f step=%.1f deg max_vel=%.1f deg/s\n",
      kTuneShoulder ? "shoulder" : "elbow", startingAngle, startingAngle + step,
      kArmPositionTuningMaxVelocityDegPerSec);
  Serial.printf("ARM_POSITION %s holds its startup angle\n",
                kTuneShoulder ? "elbow" : "shoulder");
  Serial.println("ARM_POSITION log: position and velocity are measured/target");
  return true;
}

void updateArmPositionTuning(uint32_t nowMs) {
  if (!armPositionTuningReady) {
    return;
  }

  if (static_cast<int32_t>(nowMs - nextArmPositionTuningCommandMs) >= 0) {
    armPositionTuningAtStep = !armPositionTuningAtStep;
    const Arm::JointAngles &target = armPositionTuningAtStep
                                         ? armPositionTuningStepAngles
                                         : armPositionTuningBaseAngles;
    if (!armTask.setTargetAngles(target)) {
      Serial.println("ARM_POSITION command rejected; stopping test");
      armTask.cancel();
      armPositionTuningReady = false;
      return;
    }
    nextArmPositionTuningCommandMs = nowMs + kArmPositionTuningStepHoldMs;
  }

  if (nowMs - lastArmPositionTuningLogMs < kArmVelocityTuningLogPeriodMs) {
    return;
  }
  lastArmPositionTuningLogMs = nowMs;

  Arm::Telemetry telemetry;
  if (!armTask.getTelemetry(&telemetry)) {
    return;
  }
  const Arm::JointTelemetry &joint =
      kTuneShoulder ? telemetry.shoulder : telemetry.elbow;
  Serial.printf("%s pos %6.1f/%6.1f err %+5.1f  pos P/I/D "
                "%+5.1f/%+5.1f/%+5.1f dps  vel %+5.1f/%+5.1f  pwm %+5.1f%s%s\n",
                kTuneShoulder ? "SH" : "EL", joint.measuredPositionDeg,
                joint.commandedPositionDeg, joint.positionErrorDeg,
                joint.positionPOutputDegPerSec, joint.positionIOutputDegPerSec,
                joint.positionDOutputDegPerSec, joint.measuredVelocityDegPerSec,
                joint.targetVelocityDegPerSec, joint.finalPwmPercent,
                joint.saturated ? " SAT" : "",
                joint.slewLimited ? " SLEW" : "");
}

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

void readOversampledTapeCalibrationSensors(
    float readings[kTapeCalibrationSensorCount]) {
  uint32_t sums[kTapeCalibrationSensorCount] = {};
  for (uint8_t sample = 0; sample < kTapeCalibrationOversampleCount; ++sample) {
    for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
      sums[sensor] +=
          tapeSensors.readChannel(kTapeCalibrationSensorChannels[sensor]);
    }
  }

  for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
    readings[sensor] =
        static_cast<float>(sums[sensor]) / kTapeCalibrationOversampleCount;
  }
}

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

  delay(kCalibrationSettleDelayMs);
  if (!driveTask.getCurrentPose(&pose) || driveTask.isBusy()) {
    Serial.println(
        "Tape calibration failed: drive changed during settle delay");
    return false;
  }
  const OtosSensor::Pose startPose = pose;
  const float startCrossTrackCm =
      axis == TapeCalibrationAxis::X ? startPose.yCm : startPose.xCm;

  float readings[kTapeCalibrationSensorCount];
  float sampleReadings[kTapeCalibrationSensorCount][5] = {};
  float samplePoseCoordinateCm[5] = {};
  OtosSensor::Pose samplePoses[5] = {};
  uint8_t sampleCount = 0;
  float centeredReading = 0.0f;
  float baselineReading = 0.0f;
  float peakReading = 0.0f;
  float peakMiddleSensorCoordinateCm = 0.0f;
  static float profileReadings[kTapeCalibrationSensorCount]
                              [kTapeCalibrationMaxProfileSamples];
  static float profileCoordinatesCm[kTapeCalibrationSensorCount]
                                   [kTapeCalibrationMaxProfileSamples];
  static uint16_t baselineHistogram[TapeSensorArray::MAX_READING + 1];
  memset(baselineHistogram, 0, sizeof(baselineHistogram));
  uint16_t baselineSampleCount = 0;
  uint16_t profileCount = 0;
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
    readOversampledTapeCalibrationSensors(readings);
    OtosSensor::Pose samplePose;
    if (!driveTask.getCurrentPose(&samplePose)) {
      delay(1);
      continue;
    }

    if (sampleCount < 5) {
      for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
        sampleReadings[sensor][sampleCount] = readings[sensor];
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
        sampleReadings[sensor][4] = readings[sensor];
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
    const float sampleHeadingRad = centeredPose.headingDeg * DEG_TO_RAD;
    const float cosSampleHeading = cosf(sampleHeadingRad);
    const float sinSampleHeading = sinf(sampleHeadingRad);
    float sensorCoordinatesCm[kTapeCalibrationSensorCount];
    for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
      const float sensorLocalXCm = kTapeCalibrationSensorLocalXCm[sensor];
      const float sensorWorldOffsetXCm =
          (cosSampleHeading * sensorLocalXCm) -
          (sinSampleHeading * kTapeCalibrationSensorLocalYCm);
      const float sensorWorldOffsetYCm =
          (sinSampleHeading * sensorLocalXCm) +
          (cosSampleHeading * kTapeCalibrationSensorLocalYCm);
      const float sensorAxisOffsetCm = axis == TapeCalibrationAxis::X
                                           ? sensorWorldOffsetXCm
                                           : sensorWorldOffsetYCm;
      sensorCoordinatesCm[sensor] = centeredCoordinateCm + sensorAxisOffsetCm;
    }
    const float middleSensorCoordinateCm = sensorCoordinatesCm[1];
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
        (middleSensorCoordinateCm - profileCoordinatesCm[1][profileCount - 1]) *
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
      // Store the sensor's physical world-axis coordinate at this sample.
      // Using the measured heading here accounts for small heading changes
      // during the scan instead of rotating every sample by the start heading.
      profileCoordinatesCm[sensor][profileCount] = sensorCoordinatesCm[sensor];
    }
    ++profileCount;

    if (!significantRiseDetected) {
      const uint16_t readingIndex = static_cast<uint16_t>(
          fminf(centeredReading, TapeSensorArray::MAX_READING) + 0.5f);
      ++baselineHistogram[readingIndex];
      ++baselineSampleCount;
      // This spatial lower quartile is calculated from readings that have
      // already passed through the five-sample temporal median. A few samples
      // from the beginning of the tape therefore cannot pull the background
      // upward, and an isolated low value cannot pull it downward.
      baselineReading =
          tapeReadingPercentile(baselineHistogram, baselineSampleCount,
                                kTapeCalibrationBaselinePercentile);

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
        peakMiddleSensorCoordinateCm = middleSensorCoordinateCm;
        peakPoseValid = true;
        peakDropCount = 0;
        Serial.printf("Tape rise detected: baseline=%.1f reading=%.1f\n",
                      baselineReading, centeredReading);
      }
    } else {
      if (centeredReading > peakReading) {
        peakReading = centeredReading;
        peakMiddleSensorCoordinateCm = middleSensorCoordinateCm;
        peakPoseValid = true;
        peakDropCount = 0;
      }
    }

    if (peakPoseValid) {
      const float travelPastPeakCm =
          (middleSensorCoordinateCm - peakMiddleSensorCoordinateCm) *
          searchDirection;
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
  if (!peakConfirmed) {
    Serial.println("Tape calibration failed: peak not confirmed");
    return false;
  }
  if (profileOverflow) {
    Serial.println("Tape calibration warning: profile buffer full");
  }
  if (profileCount < 3) {
    Serial.println("Tape calibration failed: insufficient profile samples");
    return false;
  }

  float measuredTapeCentersCm[kTapeCalibrationSensorCount] = {};
  uint8_t validSensorCount = 0;

  for (uint8_t sensor = 0; sensor < kTapeCalibrationSensorCount; ++sensor) {
    memset(baselineHistogram, 0, sizeof(baselineHistogram));
    for (uint16_t i = 0; i < profileCount; ++i) {
      const uint16_t readingIndex = static_cast<uint16_t>(
          fminf(profileReadings[sensor][i], TapeSensorArray::MAX_READING) +
          0.5f);
      ++baselineHistogram[readingIndex];
    }
    const float sensorBaseline = tapeReadingPercentile(
        baselineHistogram, profileCount, kTapeCalibrationBaselinePercentile);
    float sensorPeak = profileReadings[sensor][0];
    uint16_t sensorPeakIndex = 0;
    for (uint16_t i = 1; i < profileCount; ++i) {
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
            profileCoordinatesCm[sensor][i - 1] +
            fraction * (profileCoordinatesCm[sensor][i] -
                        profileCoordinatesCm[sensor][i - 1]);
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
            profileCoordinatesCm[sensor][i - 1] +
            fraction * (profileCoordinatesCm[sensor][i] -
                        profileCoordinatesCm[sensor][i - 1]);
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
    measuredTapeCentersCm[validSensorCount++] = sensorMeasuredCenterCm;

    Serial.printf("Tape ch%u: base=%.1f peak=%.1f world_edges=(%.2f,%.2f) "
                  "center=%.2f\n",
                  kTapeCalibrationSensorChannels[sensor], sensorBaseline,
                  sensorPeak, risingEdgeCoordinateCm, fallingEdgeCoordinateCm,
                  sensorMeasuredCenterCm);
  }

  if (validSensorCount != kTapeCalibrationSensorCount) {
    Serial.printf(
        "Tape calibration failed: only %u of %u sensors produced both edges\n",
        validSensorCount, kTapeCalibrationSensorCount);
    return false;
  }

  // Use the median of the three independently corrected sensor estimates so
  // one mislocated edge or noisy sensor cannot pull the calibrated pose away
  // from the other two.
  if (measuredTapeCentersCm[0] > measuredTapeCentersCm[1]) {
    const float value = measuredTapeCentersCm[0];
    measuredTapeCentersCm[0] = measuredTapeCentersCm[1];
    measuredTapeCentersCm[1] = value;
  }
  if (measuredTapeCentersCm[1] > measuredTapeCentersCm[2]) {
    const float value = measuredTapeCentersCm[1];
    measuredTapeCentersCm[1] = measuredTapeCentersCm[2];
    measuredTapeCentersCm[2] = value;
  }
  if (measuredTapeCentersCm[0] > measuredTapeCentersCm[1]) {
    const float value = measuredTapeCentersCm[0];
    measuredTapeCentersCm[0] = measuredTapeCentersCm[1];
    measuredTapeCentersCm[1] = value;
  }
  const float measuredTapeCenterCm = measuredTapeCentersCm[1];
  Serial.printf("Tape center median: %.2f cm (%.2f, %.2f, %.2f)\n",
                measuredTapeCenterCm, measuredTapeCentersCm[0],
                measuredTapeCentersCm[1], measuredTapeCentersCm[2]);

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

bool checkForTeletubby() {
  Serial2.write('C');
  if (teletubbyCount >= 2) {
    return false;
  }

  cameraDetector.resetPositiveCount();
  delay(750);
  if (cameraDetector.positiveCount() >= 2) {
    blinkLeds(600);
    Serial2.write('B');
    teletubbyCount++;
    return true;
  }
  return false;
}

void stowGrabbedRock() {
  armTask.waitUntilSettled(750);
  armTask.setTargetPosition({11.0f, 17.0f}, true);
  armTask.waitUntilSettled(750);
  armTask.setTargetPosition({8.5f, 2.5f}, true);
  armTask.waitUntilSettled(750);
  servo1.setAngle(clawOpenAngle);
  delay(500);
  armTask.setTargetPosition({10.80f, 13.0f}, true);
  armTask.waitUntilSettled(750);
  armTask.setTargetPosition({24, 3}, true);
}

void grabRock() {
  OtosSensor::Pose currentPose;
  if (driveTask.getCurrentPose(&currentPose)) {
    constexpr float kGrabApproachDistanceCm = 2.0f;
    const float headingRad = currentPose.headingDeg * DEG_TO_RAD;
    const OtosSensor::Pose approachPose{
        currentPose.xCm - (kGrabApproachDistanceCm * sinf(headingRad)),
        currentPose.yCm + (kGrabApproachDistanceCm * cosf(headingRad)),
        currentPose.headingDeg};
    if (driveTask.setTargetPose(approachPose, 0.25f)) {
      driveTask.waitUntilMotionFinished(3000);
    } else {
      Serial.println("Could not queue grab approach");
    }
  } else {
    Serial.println("Could not get pose for grab approach");
  }

  armTask.setTargetPosition({29.5f, -3.5f}, true);
  armTask.waitUntilSettled(1000);
  servo1.setAngle(clawClosedAngle);
  delay(500);
  armTask.setTargetPosition({29.5f, 17.0f}, true);
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
  constexpr bool firstField = false;

  bool lastRockMetal = false;
  bool teletubbyFoundAtRock = false;

  // Drive to first rock
  driveTask.setTargetPose({-4.5f, 50.0f, 0.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);

  driveTask.setTargetPose({-4.5f, 95.0f, 0.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);

  // Scan first rock for metal
  if (!rockHeld) {
    driveTask.calibrateImuBlocking(500);
    lastRockMetal = confirmedMetalDetected(metalDetectorRight);
  }
  // Move to first scanning position for rock 1
  driveTask.setTargetPose({-8.0f, 95.0f, 0.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-8.0f, 109.0f, 80.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  teletubbyFoundAtRock = checkForTeletubby();

  // Pickup rock 1 if it is metal
  if (lastRockMetal) {
    servo1.setAngle(clawOpenAngle);
    driveTask.setTargetPose({-8.0f, 95.0f, -90.0f}, 1.0f);
    armTask.setTargetPosition({29.5f, 5.0f}, true);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-7.0f, 95.0f, -90.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({29.5f, -2.5f}, true);
    armTask.waitUntilSettled(1000);
    driveTask.setTargetPose({-3.0, 95.0f, -90.0f}, 0.25f);
    driveTask.waitUntilMotionFinished(10000);
    grabRock();
    delay(250);
  }
  lastRockMetal = false;

  // Drive to second rock
  driveTask.setTargetPose({-2.0f, 116.0f, 0.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({10.5f, 144.0f, 0.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({8.5f, 166.0f, 15.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-3.5f, 175.0f, 61.5f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);

  bool leftRockMetal = false;
  bool rightRockMetal = false;
  // Scan second rock for metal
  if (!rockHeld) {
    driveTask.calibrateImuBlocking(500);
    rightRockMetal = confirmedMetalDetected(metalDetectorRight);
  }

  // scan third rock for metal
  if (!rockHeld) {
    driveTask.setTargetPose({-4.0f, 169.5f, 61.5f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.calibrateImuBlocking(500);
    leftRockMetal = confirmedMetalDetected(metalDetectorLeft);
  }

  // Move to scanning position for rock 3
  driveTask.setTargetPose({-30, 185, 90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-30, 195, 100.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  checkForTeletubby();

  // Move to scanning position for rock 2
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-23, 187, 20.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  teletubbyFoundAtRock = checkForTeletubby();
  // Move to second scanning position for rock 2
  driveTask.setTargetPose({-30, 185, 20.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  if (!teletubbyFoundAtRock) {
    checkForTeletubby();
  }

  // Grab left rock if it is metal
  if (leftRockMetal) {
    armTask.setTargetPosition({29.5f, 5.0f}, true);
    servo1.setAngle(clawOpenAngle);
    driveTask.setTargetPose({-15, 184, 61.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({29.5f, -2.5f}, true);
    driveTask.setTargetPose({-15, 184, -180.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-15, 172, -180.0f}, 0.25f);
    delay(500);
    grabRock();
  }
  // Grab right rock if it is metal
  if (rightRockMetal) {
    armTask.setTargetPosition({29.5f, 5.0f}, true);
    servo1.setAngle(clawOpenAngle);
    driveTask.setTargetPose({-30, 185, 0.0f}, 1.0f);
    armTask.setTargetPosition({29.5f, -3.5f}, true);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-28, 194, -90.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-18, 194, -90.0f}, 0.25f);
    driveTask.waitUntilMotionFinished(10000);
    delay(500);
    grabRock();
  }
  // Move to scanning position for rock 4
  driveTask.setTargetPose({-28.5f, 187.0f, 90.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  if (!rockHeld) {
    driveTask.setTargetPose({-49.5f, 183.0f, 145.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);

    driveTask.calibrateImuBlocking(500);
    lastRockMetal = confirmedMetalDetected(metalDetectorRight);
    driveTask.setTargetPose({-43.0f, 177.0f, 145.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
  }

  // Move to first scanning position for rock 4
  driveTask.setTargetPose({-38.0f, 185.0f, -125.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  teletubbyFoundAtRock = checkForTeletubby();
  // Move to second scanning position for rock 4
  driveTask.setTargetPose({-45.0f, 181.0f, -125.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  if (!teletubbyFoundAtRock) {
    checkForTeletubby();
  }

  // Grab rock 4 if it is metal
  if (lastRockMetal) {
    armTask.setTargetPosition({29.5f, 5.0f}, true);
    servo1.setAngle(clawOpenAngle);
    driveTask.setTargetPose({-30, 197.5, 90.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({29.5f, -3.5f}, true);
    delay(500);
    driveTask.setTargetPose({-43, 197.5, 90.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    grabRock();
  }
  lastRockMetal = false;

  // Move to scanning position for rock 5 (Up Ramp)
  driveTask.setTargetPose({-55.0f, 184.0f, 180.0f}, 0.8f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-55.0f, 150.0f, 180.0f}, 0.8f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-55.0f, 27.0f, 180.0f}, 0.8f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-65.0f, 9.0f, 134.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.calibrateImuBlocking(750);

  // Scan rock 5 for metal
  if (!rockHeld) {
    lastRockMetal = confirmedMetalDetected(metalDetectorLeft);
  }

  // Move to first scanning position for rock 5
  driveTask.setTargetPose({-72.0f, 13.0f, 134.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-77.0f, 11.5f, 45.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  teletubbyFoundAtRock = checkForTeletubby();
  // Move to second scanning position for rock 5
  driveTask.setTargetPose({-69.0f, 18.0f, 45.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  if (!teletubbyFoundAtRock) {
    checkForTeletubby();
  }

  // Grab rock 5 if it is metal
  if (lastRockMetal) {
    servo1.setAngle(clawOpenAngle);
    armTask.setTargetPosition({29.5f, 5.0f}, true);
    driveTask.setTargetPose({-69.0f, 18.0f, -135.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({29.5f, -3.5f}, true);
    driveTask.setTargetPose({-74.5f, 20.0f, -135.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-65.5f, 11.0f, -135.0f}, 0.25f);
    driveTask.waitUntilMotionFinished(10000);
    delay(250);
    grabRock();
  }
  // Flash at rock 6 if less than 2 teletubbies have been found
  if (teletubbyCount < 2) {
    driveTask.setTargetPose({-72.5f, 11.0f, -125.0f}, 0.25f);
    driveTask.waitUntilMotionFinished(10000);
    blinkLeds(600);
    Serial2.write('B');
  }

  // Move to first tape calibration position
  driveTask.setTargetPose({-85.0f, 5.0f, -90.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);

  driveTask.setTargetPose({-100.0f, 5.0f, -90.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-130.0f, 5.0f, -90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-130.0f, 0.0f, -90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);

  // Run tape calibration
  float tapePosition = (firstField ? 4.0f : 3.0f);
  while (!calibrateYWithMiddleTapeSensor(tapePosition, 1.0f, 0.09f)) {
    driveTask.setTargetPose({-130.0f, 0.0f, -90.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
  }

  // Grab rock 6 if no other metal rocks
  if (!rockHeld) {
    armTask.setTargetPosition({29.5f, 5.0f}, true);
    servo1.setAngle(clawOpenAngle);
    driveTask.setTargetPose({-123.0f, 3.0f, -45.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({29.5f, -3.5f}, true);
    delay(500);
    driveTask.setTargetPose({-116.0f, 7.5f, -45.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    grabRock();
    driveTask.setTargetPose({-144.0f, 25.0f, -90.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
  }

  // Move to habitat position
  driveTask.setTargetPose({-144.0f, 94.0f, 0.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);

  driveTask.setMotionTolerance(0.5f, 1.5f);
  driveTask.setTargetPose({-127.0f, 158.5f, 0.0f}, 0.5f);
  delay(250);
  armTask.setTargetPosition({26.5f, -5.0f}, true);
  servo1.setAngle(clawFullyClosedAngle);
  driveTask.waitUntilMotionFinished(10000);
  delay(500);

  // Reset position to habitat relative position
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

  // Reduce tolerance
  driveTask.setMotionTolerance(0.5f, 1.0f);
  delay(250);

  // Move to habitat tape calibration position
  driveTask.setTargetPose({-137.0f, 160.5f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);

  // Run habitat tape calibration
  tapePosition = (firstField ? -151.7f : -149.5f);
  constexpr float habitatX = firstField ? -155.75f : -155.4f;
  constexpr float habitatY = firstField ? 137.5f : 139.0f;

  while (!calibrateXWithMiddleTapeSensor(tapePosition, -1.0f, 0.08f)) {
    driveTask.setTargetPose({-137.0f, 160.5f, 0.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(10000);
  }
  OtosSensor::Pose currentPose;
  if (!driveTask.getCurrentPose(&currentPose)) {
    Serial.println("Current pose unavailable");
  }

  if (firstField) {
    armTask.setTargetPosition({26.5f, -5.0f}, true);
    driveTask.setTargetPose({currentPose.xCm, 162.0f, 0.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(1000);
    delay(500);
    // Move to first habitat position
    driveTask.setTargetPose({-180.5f, 162.0f, 0.0f}, 0.3f);
    servo1.setAngle(clawFullyClosedAngle);
    armTask.setTargetPosition({28.0f, -8.0f}, true);
    driveTask.waitUntilMotionFinished(2250);
    delay(500);

    // Move into first habitat
    driveTask.setTargetPose({-180.5f, 170.0f, 0.0f}, 0.1f);
    driveTask.waitUntilMotionFinished(10000);
  } else {
    armTask.setTargetPosition({26.5f, -5.0f}, true);
    driveTask.setTargetPose({currentPose.xCm, 159.0f, 0.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(1000);
    delay(500);
    // Move to first habitat position
    driveTask.setTargetPose({-178.5f, 159.0f, 0.0f}, 0.3f);
    servo1.setAngle(clawFullyClosedAngle);
    armTask.setTargetPosition({28.0f, -8.0f}, true);
    driveTask.waitUntilMotionFinished(2250);
    delay(500);

    // Move into first habitat
    driveTask.setTargetPose({-178.5f, 170.0f, 0.0f}, 0.1f);
    driveTask.waitUntilMotionFinished(10000);
  }
  // Lift arm and open claw
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(500);
  servo1.setAngle(clawHabitatOpenAngle);
  armTask.setTargetPosition({28.0f, 10.0f}, true);
  armTask.waitUntilSettled(1000);

  // Move to habitat placement position
  driveTask.setTargetPose({-185.0f, 175.0f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-185.0f, 175.0f, 45.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  if (!firstField) {
    driveTask.setTargetPose({habitatX + 3.0f, 141.0f, 90.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({habitatX + 1.0f, habitatY - 1.5f, 90.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(10000);
  } else {
    driveTask.setTargetPose({habitatX, habitatY, 90.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(10000);
  }
  // Place first habitat
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1000);
  servo1.setAngle(clawFullyClosedAngle);
  delay(300);
  // Robot wiggle
  driveTask.setTargetPose({habitatX, 138.5f, 85.0f}, 0.3f);
  delay(300);
  driveTask.setTargetPose({habitatX, 138.5f, 95.0f}, 0.3f);
  delay(300);

  // Backout and rotate habitats
  driveTask.setTargetPose({habitatX + 8.0f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -5.0f}, true);
  driveTask.setTargetPose({habitatX + 8.0f, 148.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX - 2.5f, 148.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX - 1.5f, 126.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX + 12.0f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX + 12.0f, 138.5f, 0.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);

  // Allign to second habitat
  driveTask.setTargetPose({-142.5f, 162.0f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(500);
  delay(500);
  // Move into second habitat
  driveTask.setTargetPose({-142.5f, 170.0f, 0.0f}, 0.1f);
  driveTask.waitUntilMotionFinished(10000);
  // Lift arm and open claw
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(500);
  servo1.setAngle(clawHabitatOpenAngle);
  armTask.setTargetPosition({28.0f, 10.0f}, true);
  armTask.waitUntilSettled(1000);

  // Move to habitat placement position
  driveTask.setTargetPose({-143.0f, 138.5f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-143.0f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX, habitatY, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  // Place second habitat
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1000);
  servo1.setAngle(clawFullyClosedAngle);
  delay(300);
  // Robot wiggle
  driveTask.setTargetPose({habitatX, 138.5f, 85.0f}, 0.3f);
  delay(300);
  driveTask.setTargetPose({habitatX, 138.5f, 95.0f}, 0.3f);
  delay(300);
  // backout and rotate habitats
  driveTask.setTargetPose({habitatX + 8.0f, 138.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -5.0f}, true);
  driveTask.setTargetPose({habitatX + 8.0f, 148.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX - 2.5f, 148.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX - 1.5f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX + 12.0f, 126.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);

  // Allign to third habitat
  if (firstField) {
    driveTask.setTargetPose({-161.5f, 161.0f, 0.0f}, 0.3);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({28.0f, -8.0f}, true);
    armTask.waitUntilSettled(500);
    delay(500);
    // Move into third habitat
    driveTask.setTargetPose({-161.5f, 170.0f, 0.0f}, 0.1f);
    driveTask.waitUntilMotionFinished(10000);
  } else {
    driveTask.setTargetPose({-162.5f, 161.0f, 0.0f}, 0.3);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({28.0f, -8.0f}, true);
    armTask.waitUntilSettled(500);
    delay(500);
    // Move into third habitat
    driveTask.setTargetPose({-162.5f, 170.0f, 0.0f}, 0.1f);
    driveTask.waitUntilMotionFinished(10000);
  }
  // Lift arm and open claw
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(500);
  servo1.setAngle(clawHabitatOpenAngle);
  armTask.setTargetPosition({28.0f, 10.0f}, true);
  armTask.waitUntilSettled(1000);
  // Move to habitat placement position
  driveTask.setTargetPose({-161.8f, 150.0f, 0.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-143.5f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX, habitatY, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  // Place third habitat
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1000);
  servo1.setAngle(clawFullyClosedAngle);
  delay(300);
  // Robot wiggle
  driveTask.setTargetPose({habitatX, 138.5f, 85.0f}, 0.3f);
  delay(300);
  driveTask.setTargetPose({habitatX, 138.5f, 95.0f}, 0.3f);
  delay(300);
  // backout and rotate habitats
  driveTask.setTargetPose({habitatX + 8.5f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({28.0f, -5.0f}, true);
  driveTask.setTargetPose({habitatX + 8.0f, 138.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX + 8.0f, 148.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX - 2.5f, 148.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX - 1.5f, 126.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({habitatX + 12.0f, 126.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);

  // Allign to fourth habitat
  if (firstField) {
    driveTask.setTargetPose({-124.9f, 162.0f, 0.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({28.0f, -8.0f}, true);
    armTask.waitUntilSettled(1000);
    // Move into fourth habitat
    driveTask.setTargetPose({-124.9f, 170.0f, 0.0f}, 0.1f);
    driveTask.waitUntilMotionFinished(10000);
  } else {
    driveTask.setTargetPose({-125.9f, 162.0f, 0.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetPosition({28.0f, -8.0f}, true);
    armTask.waitUntilSettled(1000);
    // Move into fourth habitat
    driveTask.setTargetPose({-125.9f, 170.0f, 0.0f}, 0.1f);
    driveTask.waitUntilMotionFinished(10000);
  }
  // Lift arm and open claw
  armTask.setTargetPosition({28.0f, 0.0f}, true);
  delay(500);
  servo1.setAngle(clawHabitatOpenAngle);
  armTask.setTargetPosition({28.0f, 10.0f}, true);
  armTask.waitUntilSettled(1000);
  // Move to habitat placement position
  driveTask.setTargetPose({-130.0f, 138.5f, 90.0f}, 0.3f);
  driveTask.waitUntilMotionFinished(10000);
  if (firstField) {
    driveTask.setTargetPose({habitatX, habitatY, 90.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(10000);
  } else {
    driveTask.setTargetPose({habitatX - 2.0f, habitatY, 90.0f}, 0.3f);
    driveTask.waitUntilMotionFinished(10000);
  }
  // Place fourth habitat
  armTask.setTargetPosition({28.0f, -8.0f}, true);
  armTask.waitUntilSettled(1000);
  servo1.setAngle(clawFullyClosedAngle);
  delay(300);
  // Robot wiggle
  driveTask.setTargetPose({habitatX, 138.5f, 85.0f}, 0.3f);
  delay(300);
  driveTask.setTargetPose({habitatX, 138.5f, 95.0f}, 0.3f);
  delay(400);
  // backout
  driveTask.setTargetPose({habitatX + 12.5f, 138.5f, 90.0f}, 0.3f, true);
  driveTask.waitUntilMotionFinished(10000);
  armTask.setTargetPosition({25.5, 6.0}, true);
  servo1.setAngle(clawOpenAngle);

  // Drive to solar panel pickup position
  armTask.setTargetPosition({27.0, 5.5}, true);
  driveTask.setTargetPose({-144.0f, 96.0f, -90.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-129.0f, 96.0f, -90.0f}, 0.1f);
  driveTask.waitUntilMotionFinished(3000);
  if (firstField) {
    armTask.setTargetPosition({27.5, 6.0}, true);
    armTask.waitUntilSettled(500);
  } else {
    armTask.setTargetPosition({25.5, 6.0}, true);
    armTask.waitUntilSettled(500);
  }

  // Grab solar panel
  servo1.setAngle(clawFullyClosedAngle);
  delay(750);
  driveTask.setTargetPose({-160.0f, 94.0f, -90.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  servo1.setAngle(clawOpenAngle);
  driveTask.setTargetPose({-160.0f, 94.0f, -100.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  driveTask.setTargetPose({-160.0f, 94.0f, -80.0f}, 1.0f, true);
  driveTask.waitUntilMotionFinished(10000);
  delay(500);
  armTask.cancel();
  driveTask.cancel();

  /**/
}

void setup() {
  Serial.begin(kSerialBaud);
  cameraDetectorReady =
      cameraDetector.begin(pins::CAMERA_RX_PIN, pins::CAMERA_TX_PIN);
  const bool servo1Ready = servo1.begin(115.0f);
  const bool ledPwmStarted = beginLedPwm();
  driveBase.begin();
  const bool tapeReady = tapeSensors.begin();
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
  pwmExpander.enableVoltageNormalization(pins::BATTERY_VOLTAGE_PIN,
                                         kBatteryVoltageDividerRatio,
                                         kNormalizedMotorVoltage);
  metalDetectorLeftReady = metalDetectorLeft.begin();
  metalDetectorRightReady = metalDetectorRight.begin();
  elbowEncoderReady = elbowEncoder.begin();
  shoulderEncoderReady = shoulderEncoder.begin();
  const bool encoderMuxReady = elbowEncoder.muxIsConnected();
  const bool otosReady = otosSensor.ping();
  Serial.println("PCA9685 " + String(pwmReady ? "ready" : "not found"));
  Serial.printf("Battery: %.2f V (PWM normalized to %.1f V)\n",
                pwmExpander.supplyVoltage(), kNormalizedMotorVoltage);
  Serial.println("Servo 1 " +
                 String(servo1Ready ? "ready at 90 degrees" : "not started"));
  Serial.println("Servo 2 LED PWM " +
                 String(ledPwmStarted ? "ready at 10%" : "not started"));
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
  Serial.println("Camera detector " +
                 String(cameraDetectorReady ? "ready" : "not started"));
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

  cameraDetector.resetPositiveCount();
  // while (true) {
  //   if (cameraDetector.positiveCount() >= 3) {
  //     Serial.println("Camera detector positive count reached threshold");
  //     blinkLeds(600);
  //     cameraDetector.resetPositiveCount();
  //   }
  //   delay(10);
  // }
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
    //                String(gravityCompensationReady ? "ready" : "not
    //                ready"));

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

  if (kArmTuningEnabled) {
    if (armTaskReady) {
      if (kArmStaticFrictionTuningEnabled) {
        armStaticFrictionTuningReady =
            beginArmStaticFrictionTuning(currentArmAngles);
      } else if (kArmVelocityTuningEnabled) {
        armVelocityTuningReady = beginArmVelocityTuning(currentArmAngles);
      } else if (kArmPositionTuningEnabled) {
        armPositionTuningReady = beginArmPositionTuning(currentArmAngles);
      }
    } else {
      Serial.println("ARM_TUNE unavailable: arm task did not start");
    }
    // Do not calibrate the drive, issue Cartesian arm commands, or run the
    // autonomous path while the tuning harness owns the arm.
    return;
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
  armTask.setTargetPosition({20, 6}, true);
  blinkLeds(500);
  // delay(3000);
  // grabRock();
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

  // armTask.setTargetPosition({28.0f, -8.0f}, true);
  // delay(5000);
  // servo1.setAngle(137.0f);
  // delay(750);
  // armTask.setTargetPosition({28.0f, 3.0f}, true);
  // servo1.setAngle(130.0f);
  // delay(2000);
  // servo1.setAngle(100.0f);
  // delay(250);
  // servo1.setAngle(142.0f);
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
  if (kArmTuningEnabled) {
    if (kArmStaticFrictionTuningEnabled) {
      updateArmStaticFrictionTuning(nowMs);
    } else if (kArmVelocityTuningEnabled) {
      updateArmVelocityTuning(nowMs);
    } else if (kArmPositionTuningEnabled) {
      updateArmPositionTuning(nowMs);
    }
    delay(1);
    return;
  }
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
  //     snprintf(detectorLeftFrequency, sizeof(detectorLeftFrequency),
  //     "%.1f",
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
  //     snprintf(detectorRightBaseline, sizeof(detectorRightBaseline),
  //     "%.1f",
  //              detectorRightReading.baselineHz);
  //   } else {
  //     snprintf(detectorRightFrequency, sizeof(detectorRightFrequency),
  //     "--"); snprintf(detectorRightBaseline, sizeof(detectorRightBaseline),
  //     "--");
  //   }

  //   const char *detectorLeftState = !detectorLeftValid ? "OFFLINE"
  //                                   : detectorLeftReading.counterSaturated
  //                                    ? "OVERFLOW"
  //                                   : !detectorLeftReading.baselineReady ?
  //                                   "CAL" : detectorLeftReading.anomaly ?
  //                                "METAL"
  //                                                                  : "OK";
  //   const char *detectorRightState = !detectorRightValid ? "OFFLINE"
  //                                    :
  //                                    detectorRightReading.counterSaturated
  //                                    ? "OVERFLOW"
  //                                    : !detectorRightReading.baselineReady
  //                                    ? "CAL" : detectorRightReading.anomaly
  //                                    ?
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
