#include "otos_sensor.h"

#include <math.h>

OtosSensor::OtosSensor(Stream &stream) : stream_(&stream) {}

OtosSensor::OtosSensor(HardwareSerial &serial)
    : stream_(&serial), serial_(&serial) {}

void OtosSensor::begin(int8_t directionPin, bool transmitLevel) {
  directionPin_ = directionPin;
  transmitLevel_ = transmitLevel;

  if (directionPin_ >= 0) {
    pinMode(directionPin_, OUTPUT);
    setTransmitEnabled(false);
  }

  rxEncodedLength_ = 0;
  dropUntilDelimiter_ = false;
  clearPacket();
  localError_ = ErrorCode::None;
  remoteError_ = ErrorCode::None;
  lastPacketMs_ = 0;
}

void OtosSensor::begin(uint32_t baudRate, int8_t rxPin, int8_t txPin,
                           int8_t directionPin, bool transmitLevel) {
  if (serial_ != nullptr) {
    serial_->begin(baudRate, SERIAL_8N1, rxPin, txPin);
  }

  begin(directionPin, transmitLevel);
}

bool OtosSensor::update() {
  bool receivedPacket = false;

  while (stream_->available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(stream_->read());
    recordWaitByte(byte);

    if (byte == DELIMITER) {
      ++waitDelimiterCount_;
      dropUntilDelimiter_ = false;
      if (rxEncodedLength_ > 0) {
        receivedPacket =
            processFrame(rxEncoded_, rxEncodedLength_) || receivedPacket;
        rxEncodedLength_ = 0;
      }
      continue;
    }

    if (dropUntilDelimiter_) {
      continue;
    }

    if (rxEncodedLength_ >= sizeof(rxEncoded_)) {
      rxEncodedLength_ = 0;
      dropUntilDelimiter_ = true;
      setLocalError(ErrorCode::BusyOrOverflow);
      continue;
    }

    rxEncoded_[rxEncodedLength_++] = byte;
  }

  return receivedPacket;
}

bool OtosSensor::ping(uint32_t timeoutMs) {
  return sendCommand(Command::Ping) && waitForPacket(Command::Pong, timeoutMs);
}

bool OtosSensor::getPose(Pose *pose, uint32_t timeoutMs) {
  if (!sendCommand(Command::GetPosition) ||
      !waitForPacket(Command::PositionReply, timeoutMs)) {
    return false;
  }

  return readLastPose(pose);
}

bool OtosSensor::startGyroCalibration(uint32_t sampleCount,
                                      uint32_t timeoutMs) {
  if (sampleCount == 0) {
    setLocalError(ErrorCode::BusyOrOverflow);
    return false;
  }

  const uint8_t payload[4] = {
      static_cast<uint8_t>(sampleCount),
      static_cast<uint8_t>(sampleCount >> 8),
      static_cast<uint8_t>(sampleCount >> 16),
      static_cast<uint8_t>(sampleCount >> 24),
  };

  if (!sendCommand(Command::CalibrateGyro, payload, sizeof(payload)) ||
      !waitForPacket(Command::CalibrateGyroReply, timeoutMs)) {
    return false;
  }

  if (payloadLength_ != 0) {
    setLocalError(ErrorCode::BadLength);
    return false;
  }

  return true;
}

float OtosSensor::gyroCalibrationDurationSeconds(uint32_t sampleCount) {
  return static_cast<float>(sampleCount) / 480.0f;
}

bool OtosSensor::setPose(const Pose &pose, uint32_t timeoutMs) {
  const Pose sensorPose = robotPoseToSensorPose(pose);
  uint8_t payload[12] = {};
  writeInt32Le(&payload[0], floatToInt32Rounded(sensorPose.xCm * 10.0f));
  writeInt32Le(&payload[4], floatToInt32Rounded(sensorPose.yCm * 10.0f));
  writeInt32Le(&payload[8],
               floatToInt32Rounded(sensorPose.headingDeg * 1000.0f));

  return sendCommand(Command::SetPose, payload, sizeof(payload)) &&
         waitForPacket(Command::SetPoseReply, timeoutMs);
}

bool OtosSensor::sendCommand(Command command, const uint8_t *payload,
                                 uint8_t payloadLength) {
  localError_ = ErrorCode::None;

  if ((payloadLength > 0 && payload == nullptr) ||
      payloadLength > MAX_PAYLOAD_SIZE) {
    setLocalError(ErrorCode::BadLength);
    return false;
  }

  uint8_t decoded[PACKET_MAX_SIZE] = {};
  uint8_t encoded[PACKET_MAX_SIZE] = {};
  const size_t decodedLength = HEADER_SIZE + payloadLength + CRC_SIZE;
  size_t encodedLength = 0;

  if (decodedLength > sizeof(decoded)) {
    setLocalError(ErrorCode::BadLength);
    return false;
  }

  decoded[0] = START_BYTE;
  decoded[1] = payloadLength + 1;
  decoded[2] = static_cast<uint8_t>(command);

  for (uint8_t i = 0; i < payloadLength; ++i) {
    decoded[HEADER_SIZE + i] = payload[i];
  }

  const uint16_t crc = crc16Ccitt(decoded, decodedLength - CRC_SIZE);
  decoded[decodedLength - 2] = static_cast<uint8_t>(crc);
  decoded[decodedLength - 1] = static_cast<uint8_t>(crc >> 8);

  if (!cobsEncode(decoded, decodedLength, encoded, &encodedLength)) {
    setLocalError(ErrorCode::BadCobs);
    return false;
  }

  setTransmitEnabled(true);
  const size_t written = stream_->write(encoded, encodedLength);
  stream_->write(DELIMITER);
  stream_->flush();
  setTransmitEnabled(false);

  return written == encodedLength;
}

bool OtosSensor::waitForPacket(Command command, uint32_t timeoutMs) {
  const uint32_t startMs = millis();
  resetWaitDiagnostics();

  while (static_cast<uint32_t>(millis() - startMs) <= timeoutMs) {
    if (update()) {
      if (lastCommand_ == command) {
        return true;
      }

      if (lastCommand_ == Command::Error) {
        return false;
      }
    }
    delay(1);
  }

  setLocalError(ErrorCode::Timeout);
  return false;
}

void OtosSensor::clearPacket() {
  hasPacket_ = false;
  lastCommand_ = Command::None;
  payloadLength_ = 0;
}

bool OtosSensor::readLastPose(Pose *pose) const {
  if (pose == nullptr || lastCommand_ != Command::PositionReply ||
      payloadLength_ != 12) {
    return false;
  }

  const Pose sensorPose(
      static_cast<float>(readInt32Le(&payload_[0])) / 10.0f,
      static_cast<float>(readInt32Le(&payload_[4])) / 10.0f,
      static_cast<float>(readInt32Le(&payload_[8])) / 1000.0f);
  *pose = sensorPoseToRobotPose(sensorPose);
  return true;
}

OtosSensor::ErrorCode OtosSensor::remoteError() const {
  return remoteError_;
}

OtosSensor::ErrorCode OtosSensor::localError() const {
  return localError_;
}

size_t OtosSensor::lastWaitBytesRead() const { return waitBytesRead_; }

size_t OtosSensor::lastWaitDelimiterCount() const {
  return waitDelimiterCount_;
}

size_t OtosSensor::lastWaitPartialLength() const {
  return rxEncodedLength_;
}

uint8_t OtosSensor::lastWaitLastCommand() const {
  return static_cast<uint8_t>(lastCommand_);
}

bool OtosSensor::processFrame(const uint8_t *encoded,
                                  size_t encodedLength) {
  uint8_t decoded[PACKET_MAX_SIZE] = {};
  size_t decodedLength = 0;

  if (!cobsDecode(encoded, encodedLength, decoded, &decodedLength)) {
    setLocalError(ErrorCode::BadCobs);
    return false;
  }

  if (decodedLength < MIN_FRAME_SIZE) {
    setLocalError(ErrorCode::BadLength);
    return false;
  }

  if (decoded[0] != START_BYTE) {
    setLocalError(ErrorCode::BadStart);
    return false;
  }

  const uint8_t bodyLength = decoded[1];
  if (bodyLength == 0 ||
      decodedLength !=
          static_cast<size_t>(HEADER_SIZE + bodyLength - 1 + CRC_SIZE)) {
    setLocalError(ErrorCode::BadLength);
    return false;
  }

  const uint16_t receivedCrc =
      static_cast<uint16_t>(decoded[decodedLength - 2]) |
      (static_cast<uint16_t>(decoded[decodedLength - 1]) << 8);
  const uint16_t calculatedCrc = crc16Ccitt(decoded, decodedLength - CRC_SIZE);
  if (receivedCrc != calculatedCrc) {
    setLocalError(ErrorCode::BadCrc);
    return false;
  }

  const uint8_t receivedPayloadLength = bodyLength - 1;
  if (receivedPayloadLength > MAX_PAYLOAD_SIZE) {
    setLocalError(ErrorCode::BadLength);
    return false;
  }

  lastCommand_ = static_cast<Command>(decoded[2]);
  payloadLength_ = receivedPayloadLength;
  for (uint8_t i = 0; i < payloadLength_; ++i) {
    payload_[i] = decoded[HEADER_SIZE + i];
  }

  if (lastCommand_ == Command::Error && payloadLength_ >= 1) {
    remoteError_ = static_cast<ErrorCode>(payload_[0]);
  }

  hasPacket_ = true;
  localError_ = ErrorCode::None;
  lastPacketMs_ = millis();
  return true;
}

void OtosSensor::setLocalError(ErrorCode error) { localError_ = error; }

void OtosSensor::setTransmitEnabled(bool enabled) {
  if (directionPin_ < 0) {
    return;
  }

  digitalWrite(directionPin_, enabled ? transmitLevel_ : !transmitLevel_);
}

void OtosSensor::resetWaitDiagnostics() {
  waitBytesRead_ = 0;
  waitDelimiterCount_ = 0;
  waitRecentCount_ = 0;
  waitRecentIndex_ = 0;
}

void OtosSensor::recordWaitByte(uint8_t byte) {
  ++waitBytesRead_;
  waitRecent_[waitRecentIndex_] = byte;
  waitRecentIndex_ = (waitRecentIndex_ + 1) % sizeof(waitRecent_);
  if (waitRecentCount_ < sizeof(waitRecent_)) {
    ++waitRecentCount_;
  }
}

OtosSensor::Pose OtosSensor::sensorPoseToRobotPose(const Pose &sensorPose) {
  const float headingRad = sensorPose.headingDeg * DEGREES_TO_RADIANS;
  const float cosHeading = cosf(headingRad);
  const float sinHeading = sinf(headingRad);
  const float offsetX =
      (cosHeading * SENSOR_OFFSET_X_CM) - (sinHeading * SENSOR_OFFSET_Y_CM);
  const float offsetY =
      (sinHeading * SENSOR_OFFSET_X_CM) + (cosHeading * SENSOR_OFFSET_Y_CM);

  return Pose(sensorPose.xCm - offsetX, sensorPose.yCm - offsetY,
              sensorPose.headingDeg);
}

OtosSensor::Pose OtosSensor::robotPoseToSensorPose(const Pose &robotPose) {
  const float headingRad = robotPose.headingDeg * DEGREES_TO_RADIANS;
  const float cosHeading = cosf(headingRad);
  const float sinHeading = sinf(headingRad);
  const float offsetX =
      (cosHeading * SENSOR_OFFSET_X_CM) - (sinHeading * SENSOR_OFFSET_Y_CM);
  const float offsetY =
      (sinHeading * SENSOR_OFFSET_X_CM) + (cosHeading * SENSOR_OFFSET_Y_CM);

  return Pose(robotPose.xCm + offsetX, robotPose.yCm + offsetY,
              robotPose.headingDeg);
}

int32_t OtosSensor::readInt32Le(const uint8_t *data) {
  const uint32_t raw = static_cast<uint32_t>(data[0]) |
                       (static_cast<uint32_t>(data[1]) << 8) |
                       (static_cast<uint32_t>(data[2]) << 16) |
                       (static_cast<uint32_t>(data[3]) << 24);
  return static_cast<int32_t>(raw);
}

int32_t OtosSensor::floatToInt32Rounded(float value) {
  if (value >= 0.0f) {
    return static_cast<int32_t>(value + 0.5f);
  }

  return static_cast<int32_t>(value - 0.5f);
}

void OtosSensor::writeInt32Le(uint8_t *data, int32_t value) {
  const uint32_t raw = static_cast<uint32_t>(value);
  data[0] = static_cast<uint8_t>(raw);
  data[1] = static_cast<uint8_t>(raw >> 8);
  data[2] = static_cast<uint8_t>(raw >> 16);
  data[3] = static_cast<uint8_t>(raw >> 24);
}

uint16_t OtosSensor::crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  while (length-- > 0) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }

  return crc;
}

bool OtosSensor::cobsEncode(const uint8_t *in, size_t inLength,
                                uint8_t *out, size_t *outLength) {
  size_t read = 0;
  size_t write = 1;
  size_t codePosition = 0;
  uint8_t code = 1;

  while (read < inLength) {
    if (in[read] == 0) {
      out[codePosition] = code;
      codePosition = write++;
      code = 1;
    } else {
      out[write++] = in[read];
      ++code;
      if (code == 0xFF) {
        out[codePosition] = code;
        codePosition = write++;
        code = 1;
      }
    }
    ++read;
  }

  out[codePosition] = code;
  *outLength = write;
  return true;
}

bool OtosSensor::cobsDecode(const uint8_t *in, size_t inLength,
                                uint8_t *out, size_t *outLength) {
  size_t read = 0;
  size_t write = 0;

  while (read < inLength) {
    const uint8_t code = in[read++];
    if (code == 0) {
      return false;
    }

    for (uint8_t i = 1; i < code; ++i) {
      if (read >= inLength) {
        return false;
      }
      out[write++] = in[read++];
    }

    if (code < 0xFF && read < inLength) {
      out[write++] = 0;
    }
  }

  *outLength = write;
  return true;
}
