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
    : m_hardware(hardware), m_previousTime(m_hardware.millis()),
      m_previousError(), m_totalIntegral{0, 0} {}

PwmTargets
PidController::calculatePwmTargets(const Hardware::WheelSpeeds &currentSpeeds,
                                   const Hardware::WheelSpeeds &targetSpeeds) {
  PwmTargets target;
  target.leftPwm =
      calculatePwmValue(currentSpeeds.leftWheel, targetSpeeds.leftWheel,
                        EncoderPositions::LEFT_ENCODER);
  target.rightPwm =
      calculatePwmValue(currentSpeeds.rightWheel, targetSpeeds.rightWheel,
                        EncoderPositions::RIGHT_ENCODER);
  return target;
}

int16_t PidController::calcProportional(const Distance &errorPerSec) {
  auto errorSpeed = errorPerSec.millimeters();

  int16_t targetSpeed = isLowerThanBoundary(errorSpeed)
                            ? errorSpeed * PID_CONSTANTS::propLower
                            : errorSpeed * PID_CONSTANTS::propUpper;
  return targetSpeed;
}

int16_t PidController::calcIntegral(const Distance &errorPerSec,
                                    EncoderPositions position) {
  auto errorSpeed = errorPerSec.millimeters();

  int16_t integralVal = isLowerThanBoundary(errorSpeed)
                            ? errorSpeed * PID_CONSTANTS::integralLower
                            : errorSpeed * PID_CONSTANTS::integralUpper;
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
  int16_t derivVal = isLowerThanBoundary(errorDelta)
                         ? errorDelta * PID_CONSTANTS::derivLower
                         : errorDelta * PID_CONSTANTS::derivUpper;
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

  auto p = calcProportional(unitError);
  auto i = calcIntegral(unitError, position);
  auto d = calcDeriv(unitError, position);

  return p + i + d;
}

bool PidController::isLowerThanBoundary(const int32_t &val) {
  return val < PID_CONSTANTS::boundarySpeed;
}

} // namespace ArgoRcLib