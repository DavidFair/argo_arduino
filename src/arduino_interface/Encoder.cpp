#include <math.h>
#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "Length.hpp"
#include "arduino_interface.hpp"

#include "Encoder.hpp"

using namespace Globals;
using namespace Libs;

namespace {

constexpr Length WHEEL_DIAMETER = 0.58_m;
constexpr int ENC_COUNTS_PER_MOTOR_ROT = 25;
constexpr int MOTOR_ROT_PER_WHEEL_ROT = 20;

constexpr Length WHEEL_RADIUS = WHEEL_DIAMETER * M_PI;
constexpr int ENC_COUNTS_PER_WHEEL_ROT =
    ENC_COUNTS_PER_MOTOR_ROT * MOTOR_ROT_PER_WHEEL_ROT;

constexpr Length DISTANCE_PER_ENC_COUNT =
    WHEEL_RADIUS / ENC_COUNTS_PER_WHEEL_ROT;
} // End of anonymous namespace

namespace Hardware {
Encoder::Encoder(ArduinoInterface &hardware)
    : m_hardware(hardware), m_lastReadTime(m_hardware.millis()),
      m_lastEncValues() {}

Encoder::Encoder(Encoder &other)
    : m_hardware(other.m_hardware), m_lastReadTime(other.m_lastReadTime),
      m_lastEncValues(other.m_lastEncValues) {}

Encoder &Encoder::operator=(Encoder &other) {
  if (this != &other) {
    m_hardware = other.m_hardware;
    m_lastReadTime = other.m_lastReadTime;
    m_lastEncValues = other.m_lastEncValues;
  }
  return *this;
}

// Allow move constructors
Encoder::Encoder(Encoder &&other)
    : m_hardware(other.m_hardware), m_lastReadTime(other.m_lastReadTime),
      m_lastEncValues(other.m_lastEncValues) {}

Encoder &Encoder::operator=(Encoder &&other) {
  if (this != &other) {
    m_hardware = other.m_hardware;
    m_lastReadTime = other.m_lastReadTime;
    m_lastEncValues = other.m_lastEncValues;
  }
  return *this;
}

WheelSpeeds Encoder::calculateSpeed() {
  auto currentEncoderValues = read();
  auto currentTime = m_hardware.millis();
  long timeDifference = currentTime - m_lastReadTime;

  auto leftPulses =
      currentEncoderValues.leftEncoderVal - m_lastEncValues.leftEncoderVal;
  auto rightPulses =
      currentEncoderValues.rightEncoderVal - m_lastEncValues.rightEncoderVal;

  Speed leftSpeed(DISTANCE_PER_ENC_COUNT * leftPulses, timeDifference);
  Speed rightSpeed(DISTANCE_PER_ENC_COUNT * rightPulses, timeDifference);

  // Update our last values
  m_lastReadTime = currentTime;
  m_lastEncValues = currentEncoderValues;

  return WheelSpeeds{leftSpeed, rightSpeed};
}

EncoderPulses Encoder::read() const {
  EncoderPulses encoderData;
  encoderData.leftEncoderVal = InterruptData::g_pinEncoderData.leftEncoderCount;

  encoderData.rightEncoderVal =
      InterruptData::g_pinEncoderData.rightEncoderCount;

  return encoderData;
}

void Encoder::reset() const {
  write(EncoderPositions::LEFT_ENCODER, 0);
  write(EncoderPositions::RIGHT_ENCODER, 0);
}

void Encoder::write(EncoderPositions targetEncoder, int32_t val) const {
  if (targetEncoder == EncoderPositions::LEFT_ENCODER) {
    InterruptData::g_pinEncoderData.leftEncoderCount = val;
  } else if (targetEncoder == EncoderPositions::RIGHT_ENCODER) {
    InterruptData::g_pinEncoderData.rightEncoderCount = val;
  }
}

} // namespace Hardware