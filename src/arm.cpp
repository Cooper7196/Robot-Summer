#include "arm.h"
#include <math.h>

namespace {
constexpr float kDefaultToleranceDeg = 1.0f;
constexpr float kDefaultIntegralZoneDeg = 7.5f;
} // namespace

Arm::JointConfig::JointConfig()
    : encoderReferenceDeg(0.0f), jointReferenceDeg(0.0f), direction(1.0f),
      minAngleDeg(-180.0f), maxAngleDeg(180.0f), kP(5.0f), kI(0.0f), kD(0.0f),
      constantPidTestEnabled(false), constantPidOutputPercent(0.0f),
      velocityAlpha(0.15f), positiveStaticFrictionPercent(0.0f),
      negativeStaticFrictionPercent(0.0f),
      positionToleranceDeg(kDefaultToleranceDeg),
      integralZoneDeg(kDefaultIntegralZoneDeg), integralLimitDegSec(5.0f),
      integralDecay(0.95f), maxPwmPercent(60.0f),
      maxOutputSlewPercentPerSec(200.0f) {}

Arm::Config::Config()
    : shoulderLinkCm(19.0f), elbowLinkCm(23.8675f),
      maxCartesianSpeedCmPerSec(10.0f), updatePeriodUs(4000),
      maxLoopDelayUs(50000), latchFaults(false), motorDisablePin(-1),
      gravityHoldSettleTimeMs(250), gravityHoldMaxVelocityDegPerSec(2.0f),
      pidReenableDriftDeg(2.5f),
      gravityA1Percent(0.0f), gravityA12Percent(0.0f), gravityA2Percent(0.0f),
      shoulder(), elbow() {}

Arm::JointState::JointState()
    : positionDeg(0.0f), previousPositionDeg(0.0f),
      previousWrappedPositionDeg(0.0f), previousCommandedPositionDeg(0.0f),
      velocityDegPerSec(0.0f), commandedVelocityDegPerSec(0.0f),
      integralDegSec(0.0f), outputPercent(0.0f), encoderInitialized(false),
      controllerInitialized(false), commandInitialized(false),
      outputSaturated(false) {}

Arm::Arm(Motor &shoulderMotor, Motor &elbowMotor,
         As5600Encoder &shoulderEncoder, As5600Encoder &elbowEncoder,
         const Config &config)
    : shoulderMotor_(&shoulderMotor), elbowMotor_(&elbowMotor),
      shoulderEncoder_(&shoulderEncoder), elbowEncoder_(&elbowEncoder),
      config_(config), commandedAngles_(), targetPosition_(),
      commandedPosition_(), shoulderState_(), elbowState_(), telemetry_(),
      lastUpdateUs_(0),
      hasTarget_(false), cartesianTarget_(false),
      cartesianPathInitialized_(false), cartesianPathComplete_(false),
      elbowUp_(false), atTarget_(false), gravityHoldActive_(false),
      gravityHoldSettledSinceUs_(0), gravityHoldAngles_(), faulted_(false) {}

bool Arm::setTargetAngles(const JointAngles &targetAngles) {
  JointAngles fitted;
  if (!fitAngleToRange(targetAngles.shoulderDeg, config_.shoulder.minAngleDeg,
                       config_.shoulder.maxAngleDeg, &fitted.shoulderDeg) ||
      !fitAngleToRange(targetAngles.elbowDeg, config_.elbow.minAngleDeg,
                       config_.elbow.maxAngleDeg, &fitted.elbowDeg)) {
    stop();
    hasTarget_ = false;
    atTarget_ = false;
    return false;
  }

  commandedAngles_ = fitted;
  hasTarget_ = true;
  cartesianTarget_ = false;
  cartesianPathInitialized_ = false;
  cartesianPathComplete_ = false;
  atTarget_ = false;
  resetPid();
  return true;
}

bool Arm::setTargetPosition(const Position &targetPosition, bool elbowUp) {
  JointAngles angles;
  if (!solveInverseKinematics(targetPosition, config_.shoulderLinkCm,
                              config_.elbowLinkCm, elbowUp, &angles) ||
      !fitAngleToRange(angles.shoulderDeg, config_.shoulder.minAngleDeg,
                       config_.shoulder.maxAngleDeg, &angles.shoulderDeg) ||
      !fitAngleToRange(angles.elbowDeg, config_.elbow.minAngleDeg,
                       config_.elbow.maxAngleDeg, &angles.elbowDeg) ||
      config_.maxCartesianSpeedCmPerSec <= 0.0f) {
    stop();
    hasTarget_ = false;
    atTarget_ = false;
    return false;
  }

  // Keep the XY goal separate from the joint command. The first update starts
  // the path at the measured pose, so every target is absolute and does not
  // depend on the preceding command having completed perfectly.
  targetPosition_ = targetPosition;
  commandedAngles_ = angles;
  elbowUp_ = elbowUp;
  hasTarget_ = true;
  cartesianTarget_ = true;
  cartesianPathInitialized_ = false;
  cartesianPathComplete_ = false;
  atTarget_ = false;
  resetPid();
  return true;
}

bool Arm::update() {
  if (faulted_) {
    stop();
    if (config_.latchFaults) {
      return false;
    }
    faulted_ = false;
  }
  if (!motorsEnabled()) {
    stop();
    return false;
  }
  if (!hasTarget_) {
    stop();
    return false;
  }
  if (config_.updatePeriodUs == 0) {
    enterFault();
    return false;
  }

  const uint32_t nowUs = micros();
  uint32_t elapsedUs = config_.updatePeriodUs;
  if (lastUpdateUs_ != 0) {
    elapsedUs = nowUs - lastUpdateUs_;
    if (elapsedUs < config_.updatePeriodUs) {
      return atTarget_;
    }
    if (config_.maxLoopDelayUs > 0 && elapsedUs > config_.maxLoopDelayUs) {
      enterFault();
      return false;
    }
  }
  lastUpdateUs_ = nowUs;
  const float dtSec = static_cast<float>(elapsedUs) * 1.0e-6f;

  float shoulderPositionDeg = 0.0f;
  float elbowPositionDeg = 0.0f;
  if (!readJointPosition(shoulderEncoder_, config_.shoulder, &shoulderState_,
                         &shoulderPositionDeg) ||
      !readJointPosition(elbowEncoder_, config_.elbow, &elbowState_,
                         &elbowPositionDeg)) {
    enterFault();
    return false;
  }

  if (cartesianTarget_) {
    if (!cartesianPathInitialized_) {
      commandedPosition_ = solveForwardKinematics(
          JointAngles(shoulderPositionDeg, elbowPositionDeg),
          config_.shoulderLinkCm, config_.elbowLinkCm);
      cartesianPathInitialized_ = true;
    }

    const float dx = targetPosition_.xCm - commandedPosition_.xCm;
    const float dy = targetPosition_.yCm - commandedPosition_.yCm;
    const float remainingCm = hypotf(dx, dy);
    const float maxStepCm = config_.maxCartesianSpeedCmPerSec * dtSec;
    if (remainingCm <= maxStepCm) {
      commandedPosition_ = targetPosition_;
      cartesianPathComplete_ = true;
    } else if (remainingCm > 0.0f) {
      const float scale = maxStepCm / remainingCm;
      commandedPosition_.xCm += dx * scale;
      commandedPosition_.yCm += dy * scale;
      cartesianPathComplete_ = false;
    }

    JointAngles nextAngles;
    if (!solveInverseKinematics(commandedPosition_, config_.shoulderLinkCm,
                                config_.elbowLinkCm, elbowUp_, &nextAngles) ||
        !fitAngleToRange(nextAngles.shoulderDeg,
                         config_.shoulder.minAngleDeg,
                         config_.shoulder.maxAngleDeg,
                         &nextAngles.shoulderDeg) ||
        !fitAngleToRange(nextAngles.elbowDeg, config_.elbow.minAngleDeg,
                         config_.elbow.maxAngleDeg, &nextAngles.elbowDeg)) {
      enterFault();
      return false;
    }
    commandedAngles_ = nextAngles;
  }

  const float shoulderPositionRad = shoulderPositionDeg * DEG_TO_RAD;
  const float forearmAbsoluteRad =
      (shoulderPositionDeg + elbowPositionDeg) * DEG_TO_RAD;
  const float shoulderGravity =
      config_.gravityA1Percent * cosf(shoulderPositionRad) +
      config_.gravityA12Percent * cosf(forearmAbsoluteRad);
  const float elbowGravity =
      config_.gravityA2Percent * cosf(forearmAbsoluteRad);

  if (gravityHoldActive_ &&
      (fabsf(shoulderPositionDeg - gravityHoldAngles_.shoulderDeg) >
           fabsf(config_.pidReenableDriftDeg) ||
       fabsf(elbowPositionDeg - gravityHoldAngles_.elbowDeg) >
           fabsf(config_.pidReenableDriftDeg))) {
    gravityHoldActive_ = false;
    gravityHoldSettledSinceUs_ = 0;
  }

  const bool feedbackEnabled = !gravityHoldActive_;
  updateJoint(shoulderMotor_, config_.shoulder, &shoulderState_,
              commandedAngles_.shoulderDeg, shoulderPositionDeg, dtSec,
              shoulderGravity, feedbackEnabled, &telemetry_.shoulder);
  updateJoint(elbowMotor_, config_.elbow, &elbowState_,
              commandedAngles_.elbowDeg, elbowPositionDeg, dtSec, elbowGravity,
              feedbackEnabled, &telemetry_.elbow);

  const bool shoulderAtTarget =
      fabsf(telemetry_.shoulder.commandedPositionDeg - shoulderPositionDeg) <=
          config_.shoulder.positionToleranceDeg;
  const bool elbowAtTarget =
      fabsf(telemetry_.elbow.commandedPositionDeg - elbowPositionDeg) <=
          config_.elbow.positionToleranceDeg;
  atTarget_ = shoulderAtTarget && elbowAtTarget &&
              (!cartesianTarget_ || cartesianPathComplete_);
  const bool velocitySettled =
      fabsf(telemetry_.shoulder.measuredVelocityDegPerSec) <=
          fabsf(config_.gravityHoldMaxVelocityDegPerSec) &&
      fabsf(telemetry_.elbow.measuredVelocityDegPerSec) <=
          fabsf(config_.gravityHoldMaxVelocityDegPerSec);
  if (!gravityHoldActive_) {
    if (atTarget_ && velocitySettled) {
      if (gravityHoldSettledSinceUs_ == 0) {
        gravityHoldSettledSinceUs_ = nowUs;
      }
      const uint32_t requiredSettleUs =
          config_.gravityHoldSettleTimeMs * 1000UL;
      if (requiredSettleUs == 0 ||
          nowUs - gravityHoldSettledSinceUs_ >= requiredSettleUs) {
        gravityHoldActive_ = true;
        gravityHoldAngles_ =
            JointAngles(shoulderPositionDeg, elbowPositionDeg);
      }
    } else {
      gravityHoldSettledSinceUs_ = 0;
    }
  }
  telemetry_.gravityHoldActive = gravityHoldActive_;
  telemetry_.faulted = false;
  telemetry_.timestampUs = nowUs;
  return atTarget_;
}

bool Arm::moveToAngles(const JointAngles &targetAngles, uint32_t timeoutMs) {
  if (!setTargetAngles(targetAngles)) {
    return false;
  }
  const uint32_t startMs = millis();
  while (millis() - startMs < timeoutMs) {
    if (update()) {
      return true;
    }
    delay(max(1UL, config_.updatePeriodUs / 1000UL));
  }
  stop();
  return false;
}

bool Arm::moveToPosition(const Position &targetPosition, bool elbowUp,
                         uint32_t timeoutMs) {
  if (!setTargetPosition(targetPosition, elbowUp)) {
    return false;
  }
  const uint32_t startMs = millis();
  while (millis() - startMs < timeoutMs) {
    if (update()) {
      return true;
    }
    delay(max(1UL, config_.updatePeriodUs / 1000UL));
  }
  stop();
  return false;
}

bool Arm::readAngles(JointAngles *angles) {
  if (angles == nullptr) {
    return false;
  }
  float shoulderDeg = 0.0f;
  float elbowDeg = 0.0f;
  if (!readJointPosition(shoulderEncoder_, config_.shoulder, &shoulderState_,
                         &shoulderDeg) ||
      !readJointPosition(elbowEncoder_, config_.elbow, &elbowState_,
                         &elbowDeg)) {
    return false;
  }
  angles->shoulderDeg = shoulderDeg;
  angles->elbowDeg = elbowDeg;
  return true;
}

Arm::Position Arm::positionFromAngles(const JointAngles &angles) const {
  return solveForwardKinematics(angles, config_.shoulderLinkCm,
                                config_.elbowLinkCm);
}

bool Arm::getTelemetry(Telemetry *telemetry) const {
  if (telemetry == nullptr) {
    return false;
  }
  *telemetry = telemetry_;
  return telemetry_.timestampUs != 0;
}

bool Arm::atTarget() const { return atTarget_; }
bool Arm::faulted() const { return faulted_; }

void Arm::resetPid() {
  shoulderState_.velocityDegPerSec = 0.0f;
  shoulderState_.commandedVelocityDegPerSec = 0.0f;
  shoulderState_.integralDegSec = 0.0f;
  shoulderState_.outputPercent = 0.0f;
  shoulderState_.controllerInitialized = false;
  shoulderState_.commandInitialized = false;
  shoulderState_.outputSaturated = false;
  elbowState_.velocityDegPerSec = 0.0f;
  elbowState_.commandedVelocityDegPerSec = 0.0f;
  elbowState_.integralDegSec = 0.0f;
  elbowState_.outputPercent = 0.0f;
  elbowState_.controllerInitialized = false;
  elbowState_.commandInitialized = false;
  elbowState_.outputSaturated = false;
  gravityHoldActive_ = false;
  gravityHoldSettledSinceUs_ = 0;
  telemetry_.gravityHoldActive = false;
  lastUpdateUs_ = 0;
}

void Arm::clearFault() {
  faulted_ = false;
  telemetry_.faulted = false;
  resetPid();
}

void Arm::stop() {
  shoulderMotor_->stop();
  elbowMotor_->stop();
  shoulderState_.outputPercent = 0.0f;
  elbowState_.outputPercent = 0.0f;
}

void Arm::setConfig(const Config &config) {
  config_ = config;
  commandedAngles_.shoulderDeg =
      clamp(commandedAngles_.shoulderDeg, config_.shoulder.minAngleDeg,
            config_.shoulder.maxAngleDeg);
  commandedAngles_.elbowDeg =
      clamp(commandedAngles_.elbowDeg, config_.elbow.minAngleDeg,
            config_.elbow.maxAngleDeg);
  shoulderState_ = JointState();
  elbowState_ = JointState();
  resetPid();
}

const Arm::Config &Arm::config() const { return config_; }

float Arm::clamp(float value, float minValue, float maxValue) {
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

float Arm::wrapDegrees(float degrees) {
  while (degrees > 180.0f)
    degrees -= 360.0f;
  while (degrees < -180.0f)
    degrees += 360.0f;
  return degrees;
}

int Arm::directionSign(float value) { return (value > 0.0f) - (value < 0.0f); }

bool Arm::fitAngleToRange(float angleDeg, float minAngleDeg, float maxAngleDeg,
                          float *fittedAngleDeg) {
  if (fittedAngleDeg == nullptr || !isfinite(angleDeg) ||
      !isfinite(minAngleDeg) || !isfinite(maxAngleDeg) ||
      minAngleDeg > maxAngleDeg) {
    return false;
  }
  const float revolutions = ceilf((minAngleDeg - angleDeg) / 360.0f);
  const float candidate = angleDeg + revolutions * 360.0f;
  if (candidate < minAngleDeg || candidate > maxAngleDeg) {
    return false;
  }
  *fittedAngleDeg = candidate;
  return true;
}

bool Arm::solveInverseKinematics(const Position &targetPosition,
                                 float shoulderLinkCm, float elbowLinkCm,
                                 bool elbowUp, JointAngles *angles) {
  if (angles == nullptr || shoulderLinkCm <= 0.0f || elbowLinkCm <= 0.0f) {
    return false;
  }
  const float x = targetPosition.xCm;
  const float y = targetPosition.yCm;
  const float distanceSquared = x * x + y * y;
  const float distance = sqrtf(distanceSquared);
  if (distance > shoulderLinkCm + elbowLinkCm ||
      distance < fabsf(shoulderLinkCm - elbowLinkCm)) {
    return false;
  }
  float cosElbow = (distanceSquared - shoulderLinkCm * shoulderLinkCm -
                    elbowLinkCm * elbowLinkCm) /
                   (2.0f * shoulderLinkCm * elbowLinkCm);
  cosElbow = clamp(cosElbow, -1.0f, 1.0f);
  const float elbowRad = elbowUp ? -acosf(cosElbow) : acosf(cosElbow);
  const float shoulderRad =
      atan2f(y, x) - atan2f(elbowLinkCm * sinf(elbowRad),
                            shoulderLinkCm + elbowLinkCm * cosf(elbowRad));
  angles->shoulderDeg = wrapDegrees(shoulderRad * RAD_TO_DEG);
  angles->elbowDeg = elbowRad * RAD_TO_DEG;
  return true;
}

Arm::Position Arm::solveForwardKinematics(const JointAngles &angles,
                                          float shoulderLinkCm,
                                          float elbowLinkCm) {
  const float shoulderRad = angles.shoulderDeg * DEG_TO_RAD;
  const float forearmRad =
      (angles.shoulderDeg + angles.elbowDeg) * DEG_TO_RAD;
  return Position(shoulderLinkCm * cosf(shoulderRad) +
                      elbowLinkCm * cosf(forearmRad),
                  shoulderLinkCm * sinf(shoulderRad) +
                      elbowLinkCm * sinf(forearmRad));
}

bool Arm::readJointPosition(As5600Encoder *encoder, const JointConfig &config,
                            JointState *state, float *positionDeg) {
  if (encoder == nullptr || state == nullptr || positionDeg == nullptr) {
    return false;
  }
  const float rawDeg = encoder->readAngleDegrees();
  if (!isfinite(rawDeg)) {
    return false;
  }
  const float wrappedPosition = wrapDegrees(
      config.jointReferenceDeg +
      wrapDegrees(rawDeg - config.encoderReferenceDeg) * config.direction);
  if (!state->encoderInitialized) {
    state->positionDeg = wrappedPosition;
    state->previousWrappedPositionDeg = wrappedPosition;
    state->encoderInitialized = true;
  } else {
    state->positionDeg +=
        wrapDegrees(wrappedPosition - state->previousWrappedPositionDeg);
    state->previousWrappedPositionDeg = wrappedPosition;
  }
  *positionDeg = state->positionDeg;
  return isfinite(*positionDeg);
}

bool Arm::motorsEnabled() const {
  return config_.motorDisablePin < 0 ||
         digitalRead(config_.motorDisablePin) == HIGH;
}

void Arm::enterFault() {
  stop();
  faulted_ = true;
  atTarget_ = false;
  telemetry_.faulted = true;
  telemetry_.timestampUs = micros();
  shoulderState_.controllerInitialized = false;
  shoulderState_.commandInitialized = false;
  shoulderState_.outputSaturated = false;
  elbowState_.controllerInitialized = false;
  elbowState_.commandInitialized = false;
  elbowState_.outputSaturated = false;
  gravityHoldActive_ = false;
  gravityHoldSettledSinceUs_ = 0;
  telemetry_.gravityHoldActive = false;
  lastUpdateUs_ = 0;
}

float Arm::updateJoint(Motor *motor, const JointConfig &config,
                       JointState *state, float commandedPositionDeg,
                       float positionDeg, float dtSec, float gravityPercent,
                       bool feedbackEnabled, JointTelemetry *telemetry) {
  if (!state->controllerInitialized) {
    state->previousPositionDeg = positionDeg;
    state->velocityDegPerSec = 0.0f;
    state->controllerInitialized = true;
  }

  if (!state->commandInitialized) {
    state->previousCommandedPositionDeg = commandedPositionDeg;
    state->commandedVelocityDegPerSec = 0.0f;
    state->commandInitialized = true;
  } else {
    state->commandedVelocityDegPerSec =
        (commandedPositionDeg - state->previousCommandedPositionDeg) / dtSec;
    state->previousCommandedPositionDeg = commandedPositionDeg;
  }

  const float rawVelocity = (positionDeg - state->previousPositionDeg) / dtSec;
  const float alpha = clamp(config.velocityAlpha, 0.0f, 1.0f);
  state->velocityDegPerSec += alpha * (rawVelocity - state->velocityDegPerSec);
  state->previousPositionDeg = positionDeg;

  const float error = commandedPositionDeg - positionDeg;
  const bool withinPositionTolerance =
      fabsf(error) <= fabsf(config.positionToleranceDeg);
  float pOutput = config.kP * error;
  // Derivative on measurement avoids a derivative kick when the command changes.
  float dOutput = -config.kD * state->velocityDegPerSec;

  if (!feedbackEnabled || config.constantPidTestEnabled) {
    state->integralDegSec = 0.0f;
  } else if (!withinPositionTolerance) {
    if (fabsf(error) < fabsf(config.integralZoneDeg) &&
        !state->outputSaturated) {
      state->integralDegSec += error * dtSec;
    } else {
      state->integralDegSec *= clamp(config.integralDecay, 0.0f, 1.0f);
    }
  }
  state->integralDegSec =
      clamp(state->integralDegSec, -fabsf(config.integralLimitDegSec),
            fabsf(config.integralLimitDegSec));
  float integralOutput = config.kI * state->integralDegSec;
  float pidOutput = pOutput + dOutput + integralOutput;
  if (!feedbackEnabled) {
    pOutput = 0.0f;
    dOutput = 0.0f;
    integralOutput = 0.0f;
    pidOutput = 0.0f;
  } else if (config.constantPidTestEnabled) {
    pidOutput =
        clamp(config.constantPidOutputPercent, -fabsf(config.maxPwmPercent),
              fabsf(config.maxPwmPercent));
    pOutput = pidOutput;
    dOutput = 0.0f;
    integralOutput = 0.0f;
  } else if (withinPositionTolerance) {
    // Hold the accumulated integral inside the acceptable position window and
    // use only that existing I contribution plus gravity compensation.
    pOutput = 0.0f;
    dOutput = 0.0f;
    pidOutput = integralOutput;
  }

  // While an interpolated command is moving, use its direction for static
  // friction feedforward. This avoids waiting for the position error to grow
  // past the tolerance and then breaking away in visible steps.
  const bool commandMoving =
      fabsf(state->commandedVelocityDegPerSec) > 0.01f;
  const int motionDirection =
      commandMoving ? directionSign(state->commandedVelocityDegPerSec)
                    : directionSign(error);
  float friction = 0.0f;
  if (feedbackEnabled && !config.constantPidTestEnabled &&
      !withinPositionTolerance) {
    if (motionDirection > 0) {
      friction = fabsf(config.positiveStaticFrictionPercent);
    } else if (motionDirection < 0) {
      friction = -fabsf(config.negativeStaticFrictionPercent);
    }
  }

  const float feedbackWithFriction = pidOutput + friction;
  const float unclamped = feedbackWithFriction + gravityPercent;
  const float clampedOutput =
      clamp(unclamped, -fabsf(config.maxPwmPercent),
            fabsf(config.maxPwmPercent));
  float output = clampedOutput;
  bool slewLimited = false;
  if (config.maxOutputSlewPercentPerSec > 0.0f) {
    const float maxOutputChange =
        config.maxOutputSlewPercentPerSec * dtSec;
    const int requestedDirection = directionSign(clampedOutput);
    const int previousDirection = directionSign(state->outputPercent);
    const bool reversing = requestedDirection != 0 &&
                           previousDirection != 0 &&
                           requestedDirection != previousDirection;
    const bool increasingMagnitude =
        fabsf(clampedOutput) > fabsf(state->outputPercent);

    // Reductions and reversals must happen immediately so the limiter never
    // keeps driving against the PID's requested braking direction. Only ramp
    // up when continuing (or starting) in the same direction.
    if (!reversing && increasingMagnitude) {
      const float limitedMagnitude =
          min(fabsf(clampedOutput),
              fabsf(state->outputPercent) + maxOutputChange);
      output = requestedDirection * limitedMagnitude;
      slewLimited = output != clampedOutput;
    }
  }
  if ((positionDeg <= config.minAngleDeg && output < 0.0f) ||
      (positionDeg >= config.maxAngleDeg && output > 0.0f)) {
    output = 0.0f;
  }
  state->outputPercent = output;
  state->outputSaturated = clampedOutput != unclamped || slewLimited;
  motor->setSpeedPercent(output);

  telemetry->commandedPositionDeg = commandedPositionDeg;
  telemetry->measuredPositionDeg = positionDeg;
  telemetry->measuredVelocityDegPerSec = state->velocityDegPerSec;
  telemetry->positionErrorDeg = error;
  telemetry->pOutputPercent = pOutput;
  telemetry->dOutputPercent = dOutput;
  telemetry->frictionPercent = friction;
  telemetry->gravityPercent = gravityPercent;
  telemetry->integralOutputPercent = integralOutput;
  telemetry->pidOutputPercent = pidOutput;
  telemetry->unclampedOutputPercent = unclamped;
  telemetry->finalPwmPercent = output;
  telemetry->saturated = clampedOutput != unclamped;
  telemetry->slewLimited = slewLimited;
  return output;
}
