#include <math.h>
#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "Length.hpp"
#include "arduino_interface.hpp"

#include "Encoder.hpp"

using namespace Globals;
using namespace Libs;

namespace {

constexpr Length calcRadius(Length diameter) {
  int32_t radius = diameter.getNanoMeters() * M_PI;
  return Length(radius);
}

constexpr Length calcDistancePerEncTick(Length wheelRadius,
                                        int countsPerWheelRotation) {
  int32_t distancePerTick =
      wheelRadius.getNanoMeters() / countsPerWheelRotation;
  return Length(distancePerTick);
}

constexpr Length WHEEL_DIAMETER = 0.58_m;
constexpr int ENC_COUNTS_PER_MOTOR_ROT = 25;
constexpr int MOTOR_ROT_PER_WHEEL_ROT = 20;

constexpr Length WHEEL_RADIUS = calcRadius(WHEEL_DIAMETER);
constexpr int ENC_COUNTS_PER_WHEEL_ROT =
    ENC_COUNTS_PER_MOTOR_ROT * MOTOR_ROT_PER_WHEEL_ROT;

constexpr Length DISTANCE_PER_ENC_COUNT =
    calcDistancePerEncTick(WHEEL_RADIUS, ENC_COUNTS_PER_WHEEL_ROT);
} // End of anonymous namespace

namespace Hardware {
Encoder::Encoder(ArduinoInterface &hardware) : m_hardware(hardware) {}

Encoder::Encoder(Encoder &other)
    : m_hardware(other.m_hardware), m_lastReadTime(other.m_lastReadTime) {}

Encoder &Encoder::operator=(Encoder &other) {
  if (this != &other) {
    m_hardware = other.m_hardware;
    m_lastReadTime = other.m_lastReadTime;
  }
  return *this;
}

// Allow move constructors
Encoder::Encoder(Encoder &&other)
    : m_hardware(other.m_hardware), m_lastReadTime(other.m_lastReadTime) {}

Encoder &Encoder::operator=(Encoder &&other) {
  if (this != &other) {
    m_hardware = other.m_hardware;
    m_lastReadTime = other.m_lastReadTime;
  }
  return *this;
}

EncoderPulses Encoder::read() const {
  EncoderPulses encoderData;
  encoderData.leftEncoderVal = InterruptData::g_pinEncoderData.leftEncoderCount;

  encoderData.rightEncoderVal =
      InterruptData::g_pinEncoderData.rightEncoderCount;

  return encoderData;
}

void Encoder::reset() const {
  write(ArgoEncoderPositions::LEFT_ENCODER, 0);
  write(ArgoEncoderPositions::RIGHT_ENCODER, 0);
}

void Encoder::write(ArgoEncoderPositions targetEncoder, int32_t val) const {
  if (targetEncoder == ArgoEncoderPositions::LEFT_ENCODER) {
    InterruptData::g_pinEncoderData.leftEncoderCount = val;
  } else if (targetEncoder == ArgoEncoderPositions::RIGHT_ENCODER) {
    InterruptData::g_pinEncoderData.rightEncoderCount = val;
  }
}

} // namespace Hardware