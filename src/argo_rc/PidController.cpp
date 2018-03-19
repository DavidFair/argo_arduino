#include <stdint.h>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "PidController.hpp"
#include "argo_rc_lib.hpp"

using namespace Libs;
using namespace Hardware;

namespace {
constexpr int MAX_PWM_VAL = 255;
}

namespace ArgoRcLib {

PidController::PidController(Hardware::ArduinoInterface &hardware)
    : m_hardwareInterface(hardware),
      m_previousTime(m_hardwareInterface.millis()),
      m_previousError(), m_totalIntegral{0, 0} {}

PidController::PidController(PidController &&other)
    : m_hardwareInterface(other.m_hardwareInterface),
      m_previousTime(other.m_previousTime), m_previousError(),
      m_totalIntegral() {
  for (int i = 0; i < EncoderPositions::_NUM_OF_ENCODERS; i++) {
    m_previousError[i] = other.m_previousError[i];
    m_totalIntegral[i] = other.m_totalIntegral[i];
  }
}

PidController &PidController::operator=(PidController &&other) {
  m_hardwareInterface = other.m_hardwareInterface;
  m_previousTime = other.m_previousTime;
  for (int i = 0; i < EncoderPositions::_NUM_OF_ENCODERS; i++) {
    m_previousError[i] = other.m_previousError[i];
    m_totalIntegral[i] = other.m_totalIntegral[i];
  }
  return *this;
}

PwmTargets
PidController::calculatePwmTargets(const Hardware::WheelSpeeds &currentSpeeds,
                                   const Hardware::WheelSpeeds &targetSpeeds) {
  if (m_hardwareInterface.millis() - m_previousTime <
      PID_CONSTANTS::timeBetween) {
    return m_previousTargets;
  }
  PwmTargets target;
  target.leftPwm =
      calculatePwmValue(currentSpeeds.leftWheel, targetSpeeds.leftWheel,
                        EncoderPositions::LEFT_ENCODER);
  target.rightPwm =
      calculatePwmValue(currentSpeeds.rightWheel, targetSpeeds.rightWheel,
                        EncoderPositions::RIGHT_ENCODER);

  m_previousTargets = target;
  m_previousTime = m_hardwareInterface.millis();
  return target;
}

int16_t PidController::calcProportional(const Distance &errorPerSec) {
  auto errorSpeed = errorPerSec.millimeters();

  int16_t targetSpeed = errorSpeed * PID_CONSTANTS::prop;
  return targetSpeed;
}

int16_t PidController::calcIntegral(const Distance &errorPerSec,
                                    EncoderPositions position) {
  auto errorSpeed = errorPerSec.millimeters();

  int16_t integralVal = errorSpeed * PID_CONSTANTS::integral;
  auto newIntegralVal = m_totalIntegral[position] + integralVal;

  // Constrain to a maximum of 255 to prevent "lag" whilst the integral drains
  if (newIntegralVal > MAX_PWM_VAL) {
    newIntegralVal = MAX_PWM_VAL;
  } else if (newIntegralVal < -MAX_PWM_VAL) {
    newIntegralVal = -MAX_PWM_VAL;
  }

  m_totalIntegral[position] = newIntegralVal;
  return newIntegralVal;
}

int16_t PidController::calcDeriv(const Distance &errorPerSec,
                                 EncoderPositions position) {
  const auto errorDifference = errorPerSec - m_previousError[position];
  m_previousError[position] = errorPerSec;

  const auto errorDelta = errorDifference.millimeters();
  int16_t derivVal = errorDelta * PID_CONSTANTS::deriv;
  // As we want to resist kicking the motors up power when the set
  // point changes this is a negative contribution
  derivVal = -derivVal;
  return derivVal;
}

void PidController::resetPid() {}

// -------- Private Methods ------
int16_t PidController::calculatePwmValue(const Libs::Speed &currentSpeed,
                                         const Libs::Speed &targetSpeed,
                                         EncoderPositions position) {
  const auto speedError = targetSpeed - currentSpeed;
  const Distance unitError = speedError.getUnitDistance();

  const auto basePower = targetSpeed.getUnitDistance().millimeters() *
                         PID_CONSTANTS::powerConstant;

  auto p = calcProportional(unitError);
  auto i = calcIntegral(unitError, position);
  auto d = calcDeriv(unitError, position);

  auto sum = basePower + (p + i + d);

  if (sum > MAX_PWM_VAL) {
    sum = MAX_PWM_VAL;
  } else if (sum < -MAX_PWM_VAL) {
    sum = -MAX_PWM_VAL;
  }

  return sum;
}

} // namespace ArgoRcLib