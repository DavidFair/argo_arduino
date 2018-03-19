#include <math.h>
#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "Distance.hpp"
#include "arduino_interface.hpp"

#include "Encoder.hpp"

using namespace Globals;
using namespace Libs;

namespace {

constexpr int ENC_COUNTS_PER_MOTOR_ROT = 25;
constexpr int MOTOR_ROT_PER_WHEEL_ROT = 20;

constexpr Distance WHEEL_RADIUS = 1.91_m;
constexpr int ENC_COUNTS_PER_WHEEL_ROT =
    ENC_COUNTS_PER_MOTOR_ROT * MOTOR_ROT_PER_WHEEL_ROT;

constexpr Distance DISTANCE_PER_ENC_COUNT =
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
  Time timeDifference(currentTime - m_lastReadTime);

  if (timeDifference.millis() < m_minTimeBetweenSpeedCalc) {
    return m_previousCalcSpeeds;
  }

  auto leftPulses =
      currentEncoderValues.leftEncoderVal - m_lastEncValues.leftEncoderVal;
  auto rightPulses =
      currentEncoderValues.rightEncoderVal - m_lastEncValues.rightEncoderVal;

  Distance leftDist(DISTANCE_PER_ENC_COUNT * leftPulses);
  Distance rightDist(DISTANCE_PER_ENC_COUNT * rightPulses);

  Speed leftSpeed(leftDist, timeDifference);
  Speed rightSpeed(rightDist, timeDifference);

  if (leftSpeed.getUnitDistance().millimeters() != 0) {
    Serial.println("distDiff was");
    Serial.println(leftDist.millimeters());
    Serial.println("Pulses was");
    Serial.println(leftPulses);
    Serial.println("Dist per rot:");
    Serial.println(DISTANCE_PER_ENC_COUNT.native());
  }

  // Update our last values
  m_lastReadTime = currentTime;
  m_lastEncValues = currentEncoderValues;

  WheelSpeeds newSpeeds{leftSpeed, rightSpeed};
  m_previousCalcSpeeds = newSpeeds;

  return newSpeeds;
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