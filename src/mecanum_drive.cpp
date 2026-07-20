#include "mecanum_drive.h"
#include <math.h>

MecanumDrive::MecanumDrive(Motor &frontLeft, Motor &frontRight, Motor &backLeft,
                           Motor &backRight, int8_t enablePin)
    : frontLeft_(&frontLeft), frontRight_(&frontRight), backLeft_(&backLeft),
      backRight_(&backRight), maxOutputPercent_(100.0f), enablePin_(enablePin),
      lastPosePidMs_(0), xIntegral_(0.0f), yIntegral_(0.0f),
      headingIntegral_(0.0f), lastXError_(0.0f), lastYError_(0.0f),
      lastHeadingError_(0.0f), lastPose_(), hasLastPose_(false),
      pathStartPose_(), hasPathStartPose_(false), targetPose_(),
      targetMaxPower_(1.0f), targetIsIntermediary_(false),
      hasPoseTarget_(false), atPoseTarget_(false), outputsStopped_(true),
      lastDisabledStopMs_(0) {}

void MecanumDrive::begin() {
  if (enablePin_ != NO_ENABLE_PIN) {
    pinMode(enablePin_, INPUT_PULLUP);
  }
}

void MecanumDrive::drive(float xVelocity, float yVelocity, float omega) {
  if (!enabled()) {
    stop();
    resetPosePid();
    return;
  }

  xVelocity = clamp(xVelocity, -1.0f, 1.0f);
  yVelocity = clamp(yVelocity, -1.0f, 1.0f);
  omega = clamp(omega, -1.0f, 1.0f);

  const float frontLeft = yVelocity + xVelocity + omega;
  const float frontRight = yVelocity - xVelocity - omega;
  const float backLeft = yVelocity - xVelocity + omega;
  const float backRight = yVelocity + xVelocity - omega;

  setWheelSpeeds(frontLeft, frontRight, backLeft, backRight);
}

void MecanumDrive::setTargetPose(const OtosSensor::Pose &targetPose,
                                 float maxPower,
                                 bool intermediaryPosition) {
  targetPose_ = targetPose;
  targetMaxPower_ = clamp(maxPower, 0.0f, 1.0f);
  targetIsIntermediary_ = intermediaryPosition;
  hasPoseTarget_ = true;
  atPoseTarget_ = false;
  resetPosePid();
}

bool MecanumDrive::updatePoseTarget(OtosSensor &otosSensor) {
  if (!hasPoseTarget_) {
    return atPoseTarget_;
  }

  if (!enabled()) {
    stop();
    resetPosePid();
    return false;
  }

  OtosSensor::Pose currentPose;
  if (!otosSensor.getPose(&currentPose)) {
    stop();
    resetPosePid();
    return false;
  }

  return updatePoseTarget(currentPose);
}

bool MecanumDrive::updatePoseTarget(const OtosSensor::Pose &currentPose) {
  if (!hasPoseTarget_) {
    return atPoseTarget_;
  }

  if (!enabled()) {
    stop();
    resetPosePid();
    return false;
  }

  if (updateDriveToPose(currentPose, targetPose_, targetMaxPower_,
                        targetIsIntermediary_)) {
    hasPoseTarget_ = false;
    atPoseTarget_ = true;
  }
  return atPoseTarget_;
}

bool MecanumDrive::hasPoseTarget() const { return hasPoseTarget_; }

bool MecanumDrive::atPoseTarget() const { return atPoseTarget_; }

void MecanumDrive::cancelPoseTarget() {
  hasPoseTarget_ = false;
  atPoseTarget_ = false;
  stop();
  resetPosePid();
}

bool MecanumDrive::driveToPose(OtosSensor &otosSensor,
                               const OtosSensor::Pose &targetPose,
                               const float maxPower,
                               bool intermediaryPosition) {
  resetPosePid();

  while (true) {
    if (!enabled()) {
      stop();
      resetPosePid();
      continue;
    }

    OtosSensor::Pose currentPose;
    if (!otosSensor.getPose(&currentPose)) {
      stop();
      resetPosePid();
      return false;
    }

    const float xError = targetPose.xCm - currentPose.xCm;
    const float yError = targetPose.yCm - currentPose.yCm;
    const float headingError =
        wrapDegrees(targetPose.headingDeg - currentPose.headingDeg);

    Serial.printf(" x=%.1f y=%.1f heading=%.1f xErr=%.1f yErr=%.1f hErr=%.1f\n",
                  currentPose.xCm, currentPose.yCm, currentPose.headingDeg,
                  xError, yError, headingError);

    if (updateDriveToPose(currentPose, targetPose, maxPower,
                          intermediaryPosition)) {
      return true;
    }

    delay(POSE_PID_PERIOD_MS);
  }
}

bool MecanumDrive::updateDriveToPose(const OtosSensor::Pose &currentPose,
                                     const OtosSensor::Pose &targetPose,
                                     const float maxPower,
                                     bool intermediaryPosition) {
  const uint32_t nowMs = millis();
  const float dtSec =
      lastPosePidMs_ != 0 ? (nowMs - lastPosePidMs_) / 1000.0f : 0.0f;
  lastPosePidMs_ = nowMs;

  const float xError = targetPose.xCm - currentPose.xCm;
  const float yError = targetPose.yCm - currentPose.yCm;
  const float headingError =
      wrapDegrees(targetPose.headingDeg - currentPose.headingDeg);

  if (!hasPathStartPose_) {
    pathStartPose_ = currentPose;
    hasPathStartPose_ = true;
  }

  float translationVelocity = 0.0f;
  float headingVelocity = 0.0f;
  float xVelocity = 0.0f;
  float yVelocity = 0.0f;
  const bool hasVelocityEstimate = hasLastPose_ && dtSec > 0.0f;
  if (hasVelocityEstimate) {
    xVelocity = (currentPose.xCm - lastPose_.xCm) / dtSec;
    yVelocity = (currentPose.yCm - lastPose_.yCm) / dtSec;
    translationVelocity = hypotf(xVelocity, yVelocity);
    headingVelocity =
        wrapDegrees(currentPose.headingDeg - lastPose_.headingDeg) / dtSec;
  }
  lastPose_ = currentPose;
  hasLastPose_ = true;

  const bool insideTranslation =
      fabsf(xError) < POSE_POSITION_TOLERANCE_CM &&
      fabsf(yError) < POSE_POSITION_TOLERANCE_CM;
  const bool insideHeading =
      fabsf(headingError) < POSE_HEADING_TOLERANCE_DEG;
  const bool insidePosition = insideTranslation && insideHeading;
  const bool underVelocity =
      hasVelocityEstimate &&
      translationVelocity < POSE_EXIT_MAX_TRANSLATION_VELOCITY_CM_PER_SEC &&
      fabsf(headingVelocity) < POSE_EXIT_MAX_HEADING_VELOCITY_DEG_PER_SEC;

  if (insidePosition && (intermediaryPosition || underVelocity)) {
    // Intermediary positions are waypoints, so finish as soon as the pose is
    // inside tolerance and leave the current command active for a smooth
    // transition to the next target. Final positions still settle and stop.
    if (!intermediaryPosition) {
      stop();
      resetPosePid();
    }
    return true;
  }

  xIntegral_ += xError * dtSec;
  yIntegral_ += yError * dtSec;
  headingIntegral_ += headingError * dtSec;

  const float headingDerivative =
      dtSec > 0.0f ? (headingError - lastHeadingError_) / dtSec : 0.0f;

  lastXError_ = xError;
  lastYError_ = yError;
  lastHeadingError_ = headingError;

  const float pathX = targetPose.xCm - pathStartPose_.xCm;
  const float pathY = targetPose.yCm - pathStartPose_.yCm;
  const float pathLength = hypotf(pathX, pathY);
  float waypointX = targetPose.xCm;
  float waypointY = targetPose.yCm;

  if (pathLength > 0.001f) {
    const float pathUnitX = pathX / pathLength;
    const float pathUnitY = pathY / pathLength;
    const float currentFromStartX = currentPose.xCm - pathStartPose_.xCm;
    const float currentFromStartY = currentPose.yCm - pathStartPose_.yCm;
    const float projectedDistance =
        (currentFromStartX * pathUnitX) + (currentFromStartY * pathUnitY);
    const float lookaheadDistance =
        clamp(projectedDistance + POSE_PATH_LOOKAHEAD_CM, 0.0f, pathLength);
    waypointX = pathStartPose_.xCm + (pathUnitX * lookaheadDistance);
    waypointY = pathStartPose_.yCm + (pathUnitY * lookaheadDistance);
  }

  const float waypointXError = waypointX - currentPose.xCm;
  const float waypointYError = waypointY - currentPose.yCm;
  const float waypointDistance = hypotf(waypointXError, waypointYError);
  const float targetDistance = hypotf(xError, yError);
  const float translationPower =
      clamp(POSE_X_KP * targetDistance, 0.0f, maxPower);

  float xCommand = 0.0f;
  float yCommand = 0.0f;
  if (waypointDistance > 0.001f) {
    xCommand = translationPower * waypointXError / waypointDistance;
    yCommand = translationPower * waypointYError / waypointDistance;
  }

  if (hasVelocityEstimate) {
    xCommand -= POSE_X_KD * xVelocity;
    yCommand -= POSE_Y_KD * yVelocity;
  }

  float translationCommandMagnitude = hypotf(xCommand, yCommand);
  if (intermediaryPosition && !insideTranslation &&
      waypointDistance > 0.001f) {
    const float minimumTranslationPower =
        fminf(POSE_INTERMEDIARY_MIN_TRANSLATION_POWER, maxPower);
    if (translationCommandMagnitude < minimumTranslationPower) {
      // Do not let proportional and derivative braking make the robot crawl
      // into a waypoint. Continue toward the lookahead point at a useful
      // minimum effort until translation is within tolerance.
      xCommand =
          minimumTranslationPower * waypointXError / waypointDistance;
      yCommand =
          minimumTranslationPower * waypointYError / waypointDistance;
      translationCommandMagnitude = minimumTranslationPower;
    }
  }

  if (translationCommandMagnitude > maxPower &&
      translationCommandMagnitude > 0.0f) {
    const float translationScale = maxPower / translationCommandMagnitude;
    xCommand *= translationScale;
    yCommand *= translationScale;
  }

  float omegaCommand = (POSE_HEADING_KP * headingError) +
                       (POSE_HEADING_KI * headingIntegral_) +
                       (POSE_HEADING_KD * headingDerivative);

  omegaCommand = clamp(omegaCommand, -maxPower, maxPower);
  if (intermediaryPosition && !insideHeading) {
    const float minimumHeadingPower =
        fminf(POSE_INTERMEDIARY_MIN_HEADING_POWER, maxPower);
    if (fabsf(omegaCommand) < minimumHeadingPower) {
      omegaCommand = minimumHeadingPower * signum(headingError);
    }
  }

  const float headingRad = currentPose.headingDeg * DEG_TO_RAD;
  const float cosHeading = cosf(headingRad);
  const float sinHeading = sinf(headingRad);
  const float robotXCommand = (xCommand * cosHeading) + (yCommand * sinHeading);
  const float robotYCommand =
      (-xCommand * sinHeading) + (yCommand * cosHeading);

  static uint32_t lastPoseCommandLogMs = 0;
  if (nowMs - lastPoseCommandLogMs >= 100) {
    lastPoseCommandLogMs = nowMs;
    const float appliedOmegaCommand = -omegaCommand;
    const float frontLeftCommand =
        robotYCommand + robotXCommand + appliedOmegaCommand;
    const float frontRightCommand =
        robotYCommand - robotXCommand - appliedOmegaCommand;
    const float backLeftCommand =
        robotYCommand - robotXCommand + appliedOmegaCommand;
    const float backRightCommand =
        robotYCommand + robotXCommand - appliedOmegaCommand;
    // Serial.printf("Drive PID err=(%.2f,%.2f,%.2f) "
    //               "robotCmd=(%.2f,%.2f,%.2f) mix=(%.2f,%.2f,%.2f,%.2f)\n",
    //               xError, yError, headingError, robotXCommand, robotYCommand,
    //               appliedOmegaCommand, frontLeftCommand, frontRightCommand,
    //               backLeftCommand, backRightCommand);
  }

  drive(robotXCommand, robotYCommand, -omegaCommand);
  return false;
}

void MecanumDrive::resetPosePid() {
  lastPosePidMs_ = 0;
  xIntegral_ = 0.0f;
  yIntegral_ = 0.0f;
  headingIntegral_ = 0.0f;
  lastXError_ = 0.0f;
  lastYError_ = 0.0f;
  lastHeadingError_ = 0.0f;
  lastPose_ = OtosSensor::Pose();
  hasLastPose_ = false;
  pathStartPose_ = OtosSensor::Pose();
  hasPathStartPose_ = false;
}

void MecanumDrive::setMaxOutputPercent(float maxOutputPercent) {
  maxOutputPercent_ = clamp(maxOutputPercent, 0.0f, 100.0f);
}

bool MecanumDrive::stopIfDisabled() {
  if (enabled()) {
    lastDisabledStopMs_ = 0;
    return false;
  }

  const uint32_t nowMs = millis();
  if (!outputsStopped_ || lastDisabledStopMs_ == 0 ||
      nowMs - lastDisabledStopMs_ >= DISABLED_STOP_REASSERT_PERIOD_MS) {
    // Force a periodic rewrite so a transient I2C problem cannot leave stale
    // PWM latched, without flooding the shared bus for the entire off period.
    outputsStopped_ = false;
    stop();
    lastDisabledStopMs_ = nowMs == 0 ? 1 : nowMs;
  }
  resetPosePid();
  return true;
}

void MecanumDrive::stop() {
  if (outputsStopped_) {
    return;
  }

  frontLeft_->stop();
  frontRight_->stop();
  backLeft_->stop();
  backRight_->stop();
  outputsStopped_ = true;
}

float MecanumDrive::clamp(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }

  if (value > maxValue) {
    return maxValue;
  }

  return value;
}

float MecanumDrive::signum(float value) {
  return (value > 0.0f) - (value < 0.0f);
}

bool MecanumDrive::enabled() const {
  return enablePin_ == NO_ENABLE_PIN || digitalRead(enablePin_) == LOW;
}

float MecanumDrive::wrapDegrees(float degrees) {
  while (degrees > 180.0f) {
    degrees -= 360.0f;
  }

  while (degrees < -180.0f) {
    degrees += 360.0f;
  }

  return degrees;
}

void MecanumDrive::setWheelSpeeds(float frontLeft, float frontRight,
                                  float backLeft, float backRight) {
  if (!enabled()) {
    stop();
    resetPosePid();
    return;
  }

  float maxMagnitude = fabsf(frontLeft);
  maxMagnitude = fmaxf(maxMagnitude, fabsf(frontRight));
  maxMagnitude = fmaxf(maxMagnitude, fabsf(backLeft));
  maxMagnitude = fmaxf(maxMagnitude, fabsf(backRight));

  if (maxMagnitude < 1.0f) {
    maxMagnitude = 1.0f;
  }

  float maxStaticFrictionPercent =
      motorConstants::frontLeftkS * fabsf(signum(frontLeft));
  maxStaticFrictionPercent =
      fmaxf(maxStaticFrictionPercent,
            motorConstants::frontRightkS * fabsf(signum(frontRight)));
  maxStaticFrictionPercent =
      fmaxf(maxStaticFrictionPercent,
            motorConstants::backLeftkS * fabsf(signum(backLeft)));
  maxStaticFrictionPercent =
      fmaxf(maxStaticFrictionPercent,
            motorConstants::backRightkS * fabsf(signum(backRight)));
  maxStaticFrictionPercent =
      clamp(maxStaticFrictionPercent, 0.0f, maxOutputPercent_);

  const float scale =
      (maxOutputPercent_ - maxStaticFrictionPercent) / maxMagnitude;

  float frontLeftPercent = frontLeft * scale;
  float frontRightPercent = frontRight * scale;
  float backLeftPercent = backLeft * scale;
  float backRightPercent = backRight * scale;

  frontLeftPercent += motorConstants::frontLeftkS * signum(frontLeftPercent);
  frontRightPercent += motorConstants::frontRightkS * signum(frontRightPercent);
  backLeftPercent += motorConstants::backLeftkS * signum(backLeftPercent);
  backRightPercent += motorConstants::backRightkS * signum(backRightPercent);

  frontLeftPercent =
      clamp(frontLeftPercent, -maxOutputPercent_, maxOutputPercent_);
  frontRightPercent =
      clamp(frontRightPercent, -maxOutputPercent_, maxOutputPercent_);
  backLeftPercent =
      clamp(backLeftPercent, -maxOutputPercent_, maxOutputPercent_);
  backRightPercent =
      clamp(backRightPercent, -maxOutputPercent_, maxOutputPercent_);

  outputsStopped_ = false;
  frontLeft_->setSpeedPercent(frontLeftPercent);
  frontRight_->setSpeedPercent(frontRightPercent);
  backLeft_->setSpeedPercent(backLeftPercent);
  backRight_->setSpeedPercent(backRightPercent);
}
