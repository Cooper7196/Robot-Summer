#pragma once

#include "as5600_encoder.h"
#include "motor.h"
#include <Arduino.h>

class Arm {
public:
  struct JointAngles {
    float shoulderDeg;
    float elbowDeg;

    JointAngles(float shoulderDeg = 0.0f, float elbowDeg = 0.0f)
        : shoulderDeg(shoulderDeg), elbowDeg(elbowDeg) {}
  };

  struct Position {
    float xCm;
    float yCm;

    Position(float xCm = 0.0f, float yCm = 0.0f) : xCm(xCm), yCm(yCm) {}
  };

  // Controller gains use degrees and seconds. Outputs are signed PWM percent.
  struct JointConfig {
    float encoderReferenceDeg;
    float jointReferenceDeg;
    float direction;
    float minAngleDeg;
    float maxAngleDeg;
    float kP;
    float kI;
    float kD;
    bool constantPidTestEnabled;
    float constantPidOutputPercent;
    float velocityAlpha;
    float positiveStaticFrictionPercent;
    float negativeStaticFrictionPercent;
    float positionToleranceDeg;
    float integralZoneDeg;
    float integralLimitDegSec;
    float integralDecay;
    float maxPwmPercent;
    float maxOutputSlewPercentPerSec;

    JointConfig();
  };

  struct Config {
    float shoulderLinkCm;
    float elbowLinkCm;
    // Maximum speed of the commanded end effector along an XY path.
    float maxCartesianSpeedCmPerSec;
    uint32_t updatePeriodUs;
    uint32_t maxLoopDelayUs;
    bool latchFaults;
    int8_t motorDisablePin;
    // Signed PWM percentages: A1*cos(q1) + A12*cos(q1+q2), A2*cos(q1+q2).
    float gravityA1Percent;
    float gravityA12Percent;
    float gravityA2Percent;
    JointConfig shoulder;
    JointConfig elbow;

    Config();
  };

  struct JointTelemetry {
    float commandedPositionDeg;
    float measuredPositionDeg;
    float measuredVelocityDegPerSec;
    float positionErrorDeg;
    float pOutputPercent;
    float dOutputPercent;
    float frictionPercent;
    float gravityPercent;
    float integralOutputPercent;
    float pidOutputPercent;
    float unclampedOutputPercent;
    float finalPwmPercent;
    bool saturated;
    bool slewLimited;
  };

  struct Telemetry {
    JointTelemetry shoulder;
    JointTelemetry elbow;
    bool faulted;
    uint32_t timestampUs;
  };

  Arm(Motor &shoulderMotor, Motor &elbowMotor, As5600Encoder &shoulderEncoder,
      As5600Encoder &elbowEncoder, const Config &config = Config());

  bool setTargetAngles(const JointAngles &targetAngles);
  // Targets are absolute coordinates from the shoulder pivot. Position
  // commands are interpolated in XY space, producing a straight-line path.
  bool setTargetPosition(const Position &targetPosition, bool elbowUp = false);
  bool update();
  bool moveToAngles(const JointAngles &targetAngles, uint32_t timeoutMs = 5000);
  bool moveToPosition(const Position &targetPosition, bool elbowUp = false,
                      uint32_t timeoutMs = 5000);

  bool readAngles(JointAngles *angles);
  // Converts measured or commanded joint angles to an end-effector position
  // in centimeters relative to the shoulder pivot.
  Position positionFromAngles(const JointAngles &angles) const;
  bool getTelemetry(Telemetry *telemetry) const;
  bool atTarget() const;
  bool faulted() const;
  void resetPid();
  void clearFault();
  void stop();
  void setConfig(const Config &config);
  const Config &config() const;

private:
  struct JointState {
    float positionDeg;
    float previousPositionDeg;
    float previousWrappedPositionDeg;
    float previousCommandedPositionDeg;
    float velocityDegPerSec;
    float commandedVelocityDegPerSec;
    float integralDegSec;
    float outputPercent;
    bool encoderInitialized;
    bool controllerInitialized;
    bool commandInitialized;
    bool outputSaturated;

    JointState();
  };

  static float clamp(float value, float minValue, float maxValue);
  static float wrapDegrees(float degrees);
  static int directionSign(float value);
  static bool fitAngleToRange(float angleDeg, float minAngleDeg,
                              float maxAngleDeg, float *fittedAngleDeg);
  static bool solveInverseKinematics(const Position &targetPosition,
                                     float shoulderLinkCm, float elbowLinkCm,
                                     bool elbowUp, JointAngles *angles);
  static Position solveForwardKinematics(const JointAngles &angles,
                                         float shoulderLinkCm,
                                         float elbowLinkCm);

  bool readJointPosition(As5600Encoder *encoder, const JointConfig &config,
                         JointState *state, float *positionDeg);
  bool motorsEnabled() const;
  void enterFault();
  float updateJoint(Motor *motor, const JointConfig &config, JointState *state,
                    float commandedPositionDeg, float positionDeg, float dtSec,
                    float gravityPercent, JointTelemetry *telemetry);

  Motor *shoulderMotor_;
  Motor *elbowMotor_;
  As5600Encoder *shoulderEncoder_;
  As5600Encoder *elbowEncoder_;
  Config config_;
  JointAngles commandedAngles_;
  Position targetPosition_;
  Position commandedPosition_;
  JointState shoulderState_;
  JointState elbowState_;
  Telemetry telemetry_;
  uint32_t lastUpdateUs_;
  bool hasTarget_;
  bool cartesianTarget_;
  bool cartesianPathInitialized_;
  bool cartesianPathComplete_;
  bool elbowUp_;
  bool atTarget_;
  bool faulted_;
};
