#include <math.h>
#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "ArduinoInterface.hpp"
#include "Distance.hpp"
#include "Encoder.hpp"

using namespace Globals;
using namespace Libs;

namespace {
/// The wheel circumference of the Argo
constexpr Distance WHEEL_CIRC = 1.91_m;
/// The number of encoder pulses per full whell rotation
constexpr int ENC_COUNTS_PER_WHEEL_ROT = 490;

/// The distance travelled per encoder pulse in a given direction
constexpr Distance DISTANCE_PER_ENC_COUNT =
    WHEEL_CIRC / ENC_COUNTS_PER_WHEEL_ROT;
} // End of anonymous namespace

namespace Hardware {
/**
 * Creates a new encoder object whose last read time is set to the current
 * time and whose last encoder values are 0
 */
Encoder::Encoder(ArduinoInterface &hardware)
    : m_hardware(hardware), m_lastReadTime(m_hardware.millis()),
      m_lastEncValues() {}

/**
 * Copy constructor for the encoder
 */
Encoder::Encoder(Encoder &other)
    : m_hardware(other.m_hardware), m_lastReadTime(other.m_lastReadTime),
      m_lastEncValues(other.m_lastEncValues) {}

/**
 * Copy operator for the encoder
 */
Encoder &Encoder::operator=(Encoder &other) {
  if (this != &other) {
    m_hardware = other.m_hardware;
    m_lastReadTime = other.m_lastReadTime;
    m_lastEncValues = other.m_lastEncValues;
  }
  return *this;
}

// Allow move constructors
/**
 * Move constructor for the encoder
 */
Encoder::Encoder(Encoder &&other)
    : m_hardware(other.m_hardware), m_lastReadTime(other.m_lastReadTime),
      m_lastEncValues(other.m_lastEncValues) {}

/**
 * Move operator for the encoder
 */
Encoder &Encoder::operator=(Encoder &&other) {
  if (this != &other) {
    m_hardware = other.m_hardware;
    m_lastReadTime = other.m_lastReadTime;
    m_lastEncValues = other.m_lastEncValues;
  }
  return *this;
}

/**
 * Returns the vehicles current speed. This is calculated if
 * a period has elapsed since last calculation (see m_minTimeBetweenSpeedCalc)
 * or the last calculated value is reused if not.
 *
 * @return An object containing the wheel speeds in millis per second
 */
WheelSpeeds Encoder::calculateSpeed() {
  auto currentTime = m_hardware.millis();
  Time timeDifference(currentTime - m_lastReadTime);

  if (timeDifference.millis() < m_minTimeBetweenSpeedCalc) {
    return m_previousCalcSpeeds;
  }
  auto currentEncoderValues = read();

  auto leftPulses =
      currentEncoderValues.leftEncoderVal - m_lastEncValues.leftEncoderVal;
  auto rightPulses =
      currentEncoderValues.rightEncoderVal - m_lastEncValues.rightEncoderVal;

  Distance leftDist(DISTANCE_PER_ENC_COUNT * leftPulses);
  Distance rightDist(DISTANCE_PER_ENC_COUNT * rightPulses);

  Speed leftSpeed(Libs::move(leftDist), timeDifference);
  Speed rightSpeed(Libs::move(rightDist), timeDifference);

  // Update our last values
  m_lastReadTime = currentTime;
  m_lastEncValues = currentEncoderValues;

  WheelSpeeds newSpeeds{Libs::move(leftSpeed), Libs::move(rightSpeed)};
  m_previousCalcSpeeds = newSpeeds;

  return newSpeeds;
}

/**
 * Reads the raw number of encoder pulses from the global variable
 *
 * @return The raw number of encoder pulses for the left and right wheel
 */
EncoderPulses Encoder::read() const {
  EncoderPulses encoderData;
  encoderData.leftEncoderVal = InterruptData::g_pinEncoderData.leftEncoderCount;

  encoderData.rightEncoderVal =
      InterruptData::g_pinEncoderData.rightEncoderCount;

  return encoderData;
}

/**
 * Resets the current encoder counts of both sides to zero
 */
void Encoder::reset() const {
  write(EncoderPositions::LEFT_ENCODER, 0);
  write(EncoderPositions::RIGHT_ENCODER, 0);
}

/**
 * Write the given val to the targetted encoder
 *
 * @param targetEncoder The encoder to write the value to
 * @param val The value to write to the encoder
 */
void Encoder::write(EncoderPositions targetEncoder, int32_t val) const {
  if (targetEncoder == EncoderPositions::LEFT_ENCODER) {
    InterruptData::g_pinEncoderData.leftEncoderCount = val;
  } else if (targetEncoder == EncoderPositions::RIGHT_ENCODER) {
    InterruptData::g_pinEncoderData.rightEncoderCount = val;
  }
}

} // namespace Hardware