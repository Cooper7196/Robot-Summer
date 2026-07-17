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
constexpr float kMetalAnomalyThresholdHz = 30.0f;
// Adjust this signed value until the elbow just begins moving. Use a
// negative value to measure friction in the opposite direction.
constexpr bool kElbowConstantPidTestEnabled = false;
constexpr float kElbowConstantPidTestPercent = -13.0f;

// Replace these with each AS5600's raw reading at the position that should be
// reported as 0 degrees.
constexpr float kShoulderEncoderDegAtZero = 10.2f;
constexpr float kElbowEncoderDegAtZero = 105.8f;

// Mechanical joint limits in calibrated joint coordinates. Targets outside
// these ranges are rejected rather than clamped.
constexpr float kShoulderMinAngleDeg = -180.0f;
constexpr float kShoulderMaxAngleDeg = 180.0f;
constexpr float kElbowMinAngleDeg = -180.0f;
constexpr float kElbowMaxAngleDeg = 180.0f;

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
  return config;
}

PwmExpander pwmExpander;
AngleServo servo1(pins::SERVO1_PWM_PIN);
OtosSensor otosSensor(Serial1);
TapeSensorArray tapeSensors;
MetalDetector::Config metalDetectorConfig = makeMetalDetectorConfig();
// metalDetector1 = right
// metalDetector2 = left
MetalDetector metalDetector1(pins::MD1_PIN, PCNT_UNIT_0, metalDetectorConfig);
MetalDetector metalDetector2(pins::MD2_PIN, PCNT_UNIT_1, metalDetectorConfig);
As5600Encoder elbowEncoder(pins::ENCODER_MUX_CHANNEL0_PIN);
As5600Encoder shoulderEncoder(pins::ENCODER_MUX_CHANNEL1_PIN);
bool elbowEncoderReady = false;
bool shoulderEncoderReady = false;

Arm::Config makeArmConfig() {
  Arm::Config config;
  config.motorDisablePin = pins::EXTRA2_PIN;
  // Signed PWM percentages required to counter gravity when the corresponding
  // link is horizontal. Reverse a sign if compensation assists gravity.
  config.gravityA1Percent = 2.0f;
  config.gravityA12Percent = 0.5f;
  config.gravityA2Percent = 15.0f;

  config.shoulder.encoderReferenceDeg = kShoulderEncoderDegAtZero;
  config.shoulder.jointReferenceDeg = 0.0f;
  config.shoulder.direction = 1.0f;
  // Gains are PWM percent per degree (P) and per degree/second (D).
  config.shoulder.kP = 0.8f;
  config.shoulder.kI = 0.0f;
  config.shoulder.kD = 0.1f;
  config.shoulder.constantPidTestEnabled = false;

  config.shoulder.maxVelocityDegPerSec = 60000.0f;
  config.shoulder.maxAccelerationDegPerSec2 = 120000.0f;
  config.shoulder.positiveFrictionPercent = 22.0f;
  config.shoulder.negativeFrictionPercent = 10.0f;
  config.shoulder.minAngleDeg = kShoulderMinAngleDeg;
  config.shoulder.maxAngleDeg = kShoulderMaxAngleDeg;
  config.shoulder.positionToleranceDeg = 1.0f;
  // Apply full breakaway compensation everywhere outside the tolerance.
  config.shoulder.fullFrictionErrorDeg = config.shoulder.positionToleranceDeg;
  config.shoulder.maxPwmPercent = 100.0f;

  config.elbow.encoderReferenceDeg = kElbowEncoderDegAtZero;
  config.elbow.jointReferenceDeg = 0.0f;
  config.elbow.direction = -1.0f;
  config.elbow.kP = 0.5f;
  config.elbow.kI = 0.1f;
  config.elbow.kD = 0.1f;
  config.elbow.constantPidTestEnabled = kElbowConstantPidTestEnabled;
  config.elbow.constantPidOutputPercent = kElbowConstantPidTestPercent;

  // config.elbow.kP = 0.0f;
  // config.elbow.kI = 0.0f;
  // config.elbow.kD = 0.0f;

  config.elbow.maxVelocityDegPerSec = 60000.0f;
  config.elbow.maxAccelerationDegPerSec2 = 120000.0f;
  config.elbow.positiveFrictionPercent =
      kElbowConstantPidTestEnabled ? 0.0f : 13.0f;
  config.elbow.negativeFrictionPercent =
      kElbowConstantPidTestEnabled ? 0.0f : 13.0f;
  config.elbow.minAngleDeg = kElbowMinAngleDeg;
  config.elbow.maxAngleDeg = kElbowMaxAngleDeg;
  config.elbow.positionToleranceDeg = 1.0f;
  config.elbow.fullFrictionErrorDeg = config.elbow.positionToleranceDeg;
  config.elbow.maxPwmPercent = 100.0f;

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
bool metalDetector1Ready = false;
bool metalDetector2Ready = false;

void logArmTelemetry() {
  Arm::JointAngles armAngles;
  Arm::Telemetry telemetry;
  if (!armTaskReady || !armTask.getCurrentAngles(&armAngles) ||
      !armTask.getTelemetry(&telemetry)) {
    Serial.println("Arm telemetry unavailable");
    return;
  }

  Serial.printf(
      "Arm shoulder=%.1f deg pid=%.1f friction=%.1f gravity=%.1f pwm=%.1f | "
      "elbow=%.1f deg pid=%.1f friction=%.1f gravity=%.1f pwm=%.1f | "
      "target=%s fault=%s enable=%s\n",
      armAngles.shoulderDeg, telemetry.shoulder.pidOutputPercent,
      telemetry.shoulder.frictionPercent, telemetry.shoulder.gravityPercent,
      telemetry.shoulder.finalPwmPercent, armAngles.elbowDeg,
      telemetry.elbow.pidOutputPercent, telemetry.elbow.frictionPercent,
      telemetry.elbow.gravityPercent, telemetry.elbow.finalPwmPercent,
      armTask.atTarget() ? "yes" : "no", telemetry.faulted ? "yes" : "no",
      digitalRead(pins::EXTRA2_PIN) == HIGH ? "yes" : "no");
}

void delayWithArmLogging(uint32_t durationMs) {
  const uint32_t startMs = millis();
  uint32_t lastLogMs = startMs - 100;
  while (millis() - startMs < durationMs) {
    const uint32_t nowMs = millis();
    if (nowMs - lastLogMs >= 100) {
      lastLogMs = nowMs;
      logArmTelemetry();
    }
    delay(1);
  }
}

void runPath() {
  servo1.setAngle(15);
  driveTask.setTargetPose({-4.5f, 95.0f, 0.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(10000);
  delay(250);
  if (metalDetector1.anomalyDetected() || metalDetector2.anomalyDetected()) {
    servo1.setAngle(15);
    driveTask.setTargetPose({-6, 68, -43.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({2.5, 77, -40.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    armTask.setTargetAngles({47.0f, -100.0f});
    delay(500);
    servo1.setAngle(90);
    delay(500);
    armTask.setTargetAngles({71.0f, -94.0f});
    armTask.waitUntilSettled(1500);
    armTask.setTargetAngles({134.0f, -153.0f});
    armTask.waitUntilSettled(1500);
    armTask.setTargetAngles({124.0f, -168.0f});
    armTask.waitUntilSettled(1500);
    servo1.setAngle(15);
  }
  while (true) {
    // Log drive pose and arm pose
    OtosSensor::Pose currentPose;
    if (!driveTask.getCurrentPose(&currentPose)) {
      Serial.println("Drive pose unavailable");
    } else {
      // get arm angles
      Arm::JointAngles armAngles;
      if (!armTask.getCurrentAngles(&armAngles)) {
        Serial.println("Arm angles unavailable");
      } else {
        Serial.printf("Drive x=%.2f cm y=%.2f cm heading=%.2f deg, shoudler "
                      "elbox= %.2f deg, elbow angle=%.2f deg\n ",
                      currentPose.xCm, currentPose.yCm, currentPose.headingDeg,
                      armAngles.shoulderDeg, armAngles.elbowDeg);
      }
    }
  }

  /*  driveTask.setTargetPose({11.0f, 155.0f, 0.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-4.5f, 170.0f, 61.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    delay(250);
    if (metalDetector1.anomalyDetected() || metalDetector2.anomalyDetected())
    { servo1.setAngle(15); delay(250); servo1.setAngle(90);
    }

    driveTask.setTargetPose({-28.5f, 187.0f, 90.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-47.0f, 184.0f, 145.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    delay(250);
    if (metalDetector1.anomalyDetected() || metalDetector2.anomalyDetected())
    { servo1.setAngle(15); delay(250); servo1.setAngle(90);
    }

    driveTask.setTargetPose({-55.0f, 184.0f, 180.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-55.0f, 150.0f, 180.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-55.0f, 27.0f, 180.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-65.0f, 9.0f, 134.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    delay(250);
    if (metalDetector1.anomalyDetected() || metalDetector2.anomalyDetected())
    { servo1.setAngle(15); delay(250); servo1.setAngle(90);
    }

    driveTask.setTargetPose({-72.0f, 13.0f, 134.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-100.0f, 4.0f, 90.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-130.0f, 4.0f, 90.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-144.0f, 97.0f, -90.0f}, 1.0f);
    driveTask.waitUntilMotionFinished(10000);
    driveTask.setTargetPose({-128.0f, 97.0f, -90.0f}, 1.0f);

    servo1.setAngle(15);
    delay(3000);
    armTask.setTargetAngles({75.0f, -90.0f});
    delayWithArmLogging(2000);
    armTask.setTargetAngles({72.0f, -116.0f});
    delayWithArmLogging(2000);
    servo1.setAngle(115);
    delayWithArmLogging(1000);
    driveTask.setTargetPose({-155.0f, 97.0f, -90.0f}, 1.0f, true);
    driveTask.waitUntilMotionFinished(10000);
    servo1.setAngle(15);
    */
}

void setup() {
  Serial.begin(kSerialBaud);
  const bool servo1Ready = servo1.begin(115.0f);
  driveBase.begin();
  const bool tapeReady = tapeSensors.begin();
  metalDetector1Ready = metalDetector1.begin();
  metalDetector2Ready = metalDetector2.begin();
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
  Serial.println("Metal detector 1 " +
                 String(metalDetector1Ready ? "ready" : "not started"));
  Serial.println("Metal detector 2 " +
                 String(metalDetector2Ready ? "ready" : "not started"));
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
    gravityCompensationReady =
        gravityCompensationConfigured && armTask.setTargetAngles({90, 90});
    Serial.println("Gravity compensation " +
                   String(gravityCompensationReady ? "ready" : "not ready"));

    // Joint-angle commands return immediately and execute in the arm task:
    // while (true) {
    //   armTask.setTargetAngles({55.0f, -70.0f});
    //   delay(2000);
    //   armTask.setTargetAngles({90.0f, -130.0f});
    //   delay(2000);
    // }
    // Once zero offsets, motor directions, and link lengths are calibrated,
    // command an XY target in centimeters from the shoulder joint like this:
    // armTask.setTargetPosition({20.0f, 15.0f});
  }

  // Reset the OTOS coordinate system through its owning task:
  driveTask.setOtosPose({0.0f, 0.0f, 0.0f});
  driveTask.setTargetPose({0.0f, 0.0f, 0.0f}, 1.0f);
  driveTask.waitUntilMotionFinished(1000);
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
  // servo1.setAngle(115);
  armTask.setTargetAngles({47.0f, -100.0f});
  runPath();

  // while (true) {
  //   MetalDetector::Reading detector1Reading;
  //   MetalDetector::Reading detector2Reading;
  //   const bool detector1Valid =
  //       metalDetector1Ready &&
  //       metalDetector1.getReading(&detector1Reading);
  //   const bool detector2Valid =
  //       metalDetector2Ready &&
  //       metalDetector2.getReading(&detector2Reading);

  //   const char *detector1State = !detector1Valid ? "OFFLINE"
  //                                : detector1Reading.counterSaturated
  //                                    ? "OVERFLOW"
  //                                : !detector1Reading.baselineReady ? "CAL"
  //                                : detector1Reading.anomaly        ?
  //                                "METAL"
  //                                                                  : "OK";
  //   const char *detector2State = !detector2Valid ? "OFFLINE"
  //                                : detector2Reading.counterSaturated
  //                                    ? "OVERFLOW"
  //                                : !detector2Reading.baselineReady ? "CAL"
  //                                : detector2Reading.anomaly        ?
  //                                "METAL"
  //                                                                  : "OK";

  //   Serial.printf("MD1 %.1f Hz | base %.1f | delta %.1f | %s  ||  "
  //                 "MD2 %.1f Hz | base %.1f | delta %.1f | %s\n",
  //                 detector1Reading.frequencyHz,
  //                 detector1Reading.baselineHz,
  //                 detector1Reading.deviationHz, detector1State,
  //                 detector2Reading.frequencyHz,
  //                 detector2Reading.baselineHz,
  //                 detector2Reading.deviationHz, detector2State);
  //   delay(100);
  // }
}

void loop() {
  static uint32_t lastArmLogMs = 0;
  const uint32_t nowMs = millis();
  if (armTaskReady && nowMs - lastArmLogMs >= 100) {
    lastArmLogMs = nowMs;
    logArmTelemetry();
  }

  // Metal Detector Test
  // static uint32_t lastMetalDetectorPrintMs = 0;
  // if (millis() - lastMetalDetectorPrintMs >= 100) {
  //   lastMetalDetectorPrintMs = millis();

  //   MetalDetector::Reading detector1Reading;
  //   MetalDetector::Reading detector2Reading;
  //   const bool detector1Valid =
  //       metalDetector1Ready &&
  //       metalDetector1.getReading(&detector1Reading);
  //   const bool detector2Valid =
  //       metalDetector2Ready &&
  //       metalDetector2.getReading(&detector2Reading);

  //   char detector1Frequency[16];
  //   char detector1Baseline[16];
  //   char detector2Frequency[16];
  //   char detector2Baseline[16];

  //   if (detector1Valid) {
  //     snprintf(detector1Frequency, sizeof(detector1Frequency), "%.1f",
  //              detector1Reading.frequencyHz);
  //     snprintf(detector1Baseline, sizeof(detector1Baseline), "%.1f",
  //              detector1Reading.baselineHz);
  //   } else {
  //     snprintf(detector1Frequency, sizeof(detector1Frequency), "--");
  //     snprintf(detector1Baseline, sizeof(detector1Baseline), "--");
  //   }

  //   if (detector2Valid) {
  //     snprintf(detector2Frequency, sizeof(detector2Frequency), "%.1f",
  //              detector2Reading.frequencyHz);
  //     snprintf(detector2Baseline, sizeof(detector2Baseline), "%.1f",
  //              detector2Reading.baselineHz);
  //   } else {
  //     snprintf(detector2Frequency, sizeof(detector2Frequency), "--");
  //     snprintf(detector2Baseline, sizeof(detector2Baseline), "--");
  //   }

  //   const char *detector1State = !detector1Valid ? "OFFLINE"
  //                                : detector1Reading.counterSaturated
  //                                    ? "OVERFLOW"
  //                                : !detector1Reading.baselineReady ? "CAL"
  //                                : detector1Reading.anomaly        ?
  //                                "METAL"
  //                                                                  : "OK";
  //   const char *detector2State = !detector2Valid ? "OFFLINE"
  //                                : detector2Reading.counterSaturated
  //                                    ? "OVERFLOW"
  //                                : !detector2Reading.baselineReady ? "CAL"
  //                                : detector2Reading.anomaly        ?
  //                                "METAL"
  //                                                                  : "OK";

  //   // Carriage return updates one fixed-width dashboard line instead of
  //   // continuously scrolling the serial monitor.
  //   Serial.printf("\rMD1 %9s Hz | base %9s | %-8s  ||  "
  //                 "MD2 %9s Hz | base %9s | %-8s    ",
  //                 detector1Frequency, detector1Baseline, detector1State,
  //                 detector2Frequency, detector2Baseline, detector2State);
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
