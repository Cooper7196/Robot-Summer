#pragma once

#include <Arduino.h>
#include <Stream.h>

class OtosSensor {
public:
  enum class Command : uint8_t {
    None = 0x00,
    Ping = 0x01,
    GetPosition = 0x03,
    CalibrateGyro = 0x04,
    SetPose = 0x05,
    Error = 0x7F,
    Pong = 0x81,
    PositionReply = 0x83,
    CalibrateGyroReply = 0x84,
    SetPoseReply = 0x85,
  };

  enum class ErrorCode : uint8_t {
    None = 0,
    BadCobs = 1,
    BadStart = 2,
    BadLength = 3,
    BadCrc = 4,
    UnknownCommand = 5,
    BusyOrOverflow = 6,
    Timeout = 7,
  };

  struct Pose {
    Pose() = default;
    Pose(float xCm, float yCm, float headingDeg)
        : xCm(xCm), yCm(yCm), headingDeg(headingDeg) {}

    float xCm = 0.0f;
    float yCm = 0.0f;
    float headingDeg = 0.0f;
  };

  static constexpr uint8_t MAX_PAYLOAD_SIZE = 240;
  static constexpr uint32_t DEFAULT_BAUD_RATE = 115200;
  static constexpr uint32_t DEFAULT_TIMEOUT_MS = 250;

  explicit OtosSensor(Stream &stream);
  explicit OtosSensor(HardwareSerial &serial);

  void begin(int8_t directionPin = -1, bool transmitLevel = HIGH);
  void begin(uint32_t baudRate, int8_t rxPin, int8_t txPin,
             int8_t directionPin = -1, bool transmitLevel = HIGH);

  bool ping(uint32_t timeoutMs = DEFAULT_TIMEOUT_MS);
  bool getPose(Pose *pose, uint32_t timeoutMs = DEFAULT_TIMEOUT_MS);
  // Starts sample collection and waits only for the start acknowledgment.
  // Keep the sensor stationary for approximately
  // gyroCalibrationDurationSeconds(sampleCount) after this returns true.
  bool startGyroCalibration(uint32_t sampleCount,
                            uint32_t timeoutMs = DEFAULT_TIMEOUT_MS);
  static float gyroCalibrationDurationSeconds(uint32_t sampleCount);
  bool setPose(const Pose &pose, uint32_t timeoutMs = DEFAULT_TIMEOUT_MS);

  ErrorCode remoteError() const;
  ErrorCode localError() const;
  size_t lastWaitBytesRead() const;
  size_t lastWaitDelimiterCount() const;
  size_t lastWaitPartialLength() const;
  uint8_t lastWaitLastCommand() const;

private:
  static constexpr uint8_t START_BYTE = 0xA5;
  static constexpr uint8_t DELIMITER = 0x00;
  static constexpr uint8_t PACKET_MAX_SIZE = 255;
  static constexpr uint8_t HEADER_SIZE = 3;
  static constexpr uint8_t CRC_SIZE = 2;
  static constexpr uint8_t MIN_FRAME_SIZE = HEADER_SIZE + CRC_SIZE;
  static constexpr float SENSOR_OFFSET_X_CM = -11.2f;
  static constexpr float SENSOR_OFFSET_Y_CM = 5.6f;
  static constexpr float DEGREES_TO_RADIANS = 0.0174532925f;

  Stream *stream_;
  HardwareSerial *serial_ = nullptr;
  int8_t directionPin_ = -1;
  bool transmitLevel_ = HIGH;

  uint8_t rxEncoded_[PACKET_MAX_SIZE] = {};
  size_t rxEncodedLength_ = 0;
  bool dropUntilDelimiter_ = false;

  bool hasPacket_ = false;
  Command lastCommand_ = Command::None;
  uint8_t payload_[MAX_PAYLOAD_SIZE] = {};
  uint8_t payloadLength_ = 0;
  ErrorCode remoteError_ = ErrorCode::None;
  ErrorCode localError_ = ErrorCode::None;
  uint32_t lastPacketMs_ = 0;
  size_t waitBytesRead_ = 0;
  size_t waitDelimiterCount_ = 0;
  uint8_t waitRecent_[8] = {};
  size_t waitRecentCount_ = 0;
  size_t waitRecentIndex_ = 0;

  bool update();
  bool sendCommand(Command command, const uint8_t *payload = nullptr,
                   uint8_t payloadLength = 0);
  bool waitForPacket(Command command, uint32_t timeoutMs);
  void clearPacket();
  bool readLastPose(Pose *pose) const;
  bool processFrame(const uint8_t *encoded, size_t encodedLength);
  void setLocalError(ErrorCode error);
  void setTransmitEnabled(bool enabled);
  void resetWaitDiagnostics();
  void recordWaitByte(uint8_t byte);

  static Pose sensorPoseToRobotPose(const Pose &sensorPose);
  static Pose robotPoseToSensorPose(const Pose &robotPose);
  static int32_t readInt32Le(const uint8_t *data);
  static int32_t floatToInt32Rounded(float value);
  static void writeInt32Le(uint8_t *data, int32_t value);
  static uint16_t crc16Ccitt(const uint8_t *data, size_t length);
  static bool cobsEncode(const uint8_t *in, size_t inLength, uint8_t *out,
                         size_t *outLength);
  static bool cobsDecode(const uint8_t *in, size_t inLength, uint8_t *out,
                         size_t *outLength);
};
