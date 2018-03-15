#include <stdint.h>

#include "Distance.hpp"
#include "PidController.hpp"
#include "argo_rc_lib.hpp"

using namespace Libs;

namespace {
constexpr int MAX_PWM_VAL = 255;
}

namespace ArgoRcLib {

PidController::PidController(Hardware::ArduinoInterface &hardware)
    : m_hardware(hardware), m_previousError(),
      m_previousTime(m_hardware.millis()), m_totalIntegral(0) {}

PwmTargets
PidController::calculatePwmTargets(const Hardware::WheelSpeeds &targetSpeeds) {
  (void)targetSpeeds;
  return PwmTargets();
}

int16_t PidController::calcProportional(const Distance &errorPerSec) {
  auto errorSpeed = errorPerSec.getMilliMeters();

  int16_t targetSpeed = isLowerThanBoundary(errorSpeed)
                            ? errorSpeed * PID_CONSTANTS::propLower
                            : errorSpeed * PID_CONSTANTS::propUpper;
  return targetSpeed;
}

int16_t PidController::calcIntegral(const Distance &errorPerSec) {
  auto errorSpeed = errorPerSec.getMilliMeters();

  int16_t integralVal = isLowerThanBoundary(errorSpeed)
                            ? errorSpeed * PID_CONSTANTS::integralLower
                            : errorSpeed * PID_CONSTANTS::integralUpper;
  m_totalIntegral += integralVal;

  // Constrain to a maximum of 255 to prevent "lag" whilst the integral drains
  if (m_totalIntegral > MAX_PWM_VAL) {
    m_totalIntegral = MAX_PWM_VAL;
  } else if (m_totalIntegral < -MAX_PWM_VAL) {
    m_totalIntegral = -MAX_PWM_VAL;
  }

  return m_totalIntegral;
}

int16_t PidController::calcDifferential(const Libs::Speed &speedError,
                                        const Libs::Time &timeDifference) {
  (void)speedError;
  (void)timeDifference;
  return 0;
}

void PidController::resetPid() {}

// -------- Private Methods ------
bool PidController::isLowerThanBoundary(const int32_t &val) {
  return val < PID_CONSTANTS::boundarySpeed;
}

} // namespace ArgoRcLib