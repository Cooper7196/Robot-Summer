#pragma once

#include <Arduino.h>

namespace pins {

// ESP32-S3 GPIO assignments from the robot controller schematic.
constexpr uint8_t OTOS_TX_PIN = 1;
constexpr uint8_t OTOS_RX_PIN = 2;
constexpr uint8_t OTOS_DIR_PIN = 10;

constexpr uint8_t TAPE_MOSI_PIN = 4;
constexpr uint8_t TAPE_MISO_PIN = 5;
constexpr uint8_t TAPE_SCK_PIN = 6;
constexpr uint8_t TAPE_CS_PIN = 7;

constexpr uint8_t ENCODER_I2C_SDA_PIN = 8;
constexpr uint8_t ENCODER_I2C_SCL_PIN = 9;

constexpr uint8_t SERVO1_PWM_PIN = 11;
constexpr uint8_t SERVO2_PWM_PIN = 12;

constexpr uint8_t MD_LEFT_PIN = 13;
constexpr uint8_t MD_RIGHT_PIN = 14;

constexpr uint8_t PWM_EXPANDER_SCL_PIN = 15;
constexpr uint8_t PWM_EXPANDER_SDA_PIN = 16;

constexpr uint8_t EXTRA1_PIN = 41;
constexpr uint8_t EXTRA2_PIN = 42;

constexpr uint8_t CAMERA_TX_PIN = 47;
constexpr uint8_t CAMERA_RX_PIN = 48;

// External PCA9685 PWM expander channels.
constexpr uint8_t HBRIDGE1_PWM1_PIN = 0;
constexpr uint8_t HBRIDGE1_PWM2_PIN = 1;
constexpr uint8_t HBRIDGE2_PWM1_PIN = 2;
constexpr uint8_t HBRIDGE2_PWM2_PIN = 3;
constexpr uint8_t HBRIDGE3_PWM1_PIN = 4;
constexpr uint8_t HBRIDGE3_PWM2_PIN = 5;
constexpr uint8_t HBRIDGE4_PWM1_PIN = 6;
constexpr uint8_t HBRIDGE4_PWM2_PIN = 7;
constexpr uint8_t HBRIDGE5_PWM1_PIN = 8;
constexpr uint8_t HBRIDGE5_PWM2_PIN = 9;
constexpr uint8_t HBRIDGE6_PWM1_PIN = 10;
constexpr uint8_t HBRIDGE6_PWM2_PIN = 11;

constexpr uint8_t UNUSED_PWM12_PIN = 12;
constexpr uint8_t UNUSED_PWM13_PIN = 13;
constexpr uint8_t UNUSED_PWM14_PIN = 14;
constexpr uint8_t UNUSED_PWM15_PIN = 15;

// External TCA9548A I2C mux channels.
constexpr uint8_t ENCODER_MUX_CHANNEL0_PIN = 0;
constexpr uint8_t ENCODER_MUX_CHANNEL1_PIN = 1;
constexpr uint8_t ENCODER_MUX_CHANNEL2_PIN = 2;
constexpr uint8_t ENCODER_MUX_CHANNEL3_PIN = 3;
constexpr uint8_t ENCODER_MUX_CHANNEL4_PIN = 4;
constexpr uint8_t ENCODER_MUX_CHANNEL5_PIN = 5;
constexpr uint8_t ENCODER_MUX_CHANNEL6_PIN = 6;
constexpr uint8_t ENCODER_MUX_CHANNEL7_PIN = 7;

} // namespace pins
