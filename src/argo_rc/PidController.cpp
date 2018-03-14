#include <stdint.h>

#include "PidController.hpp"
#include "argo_rc_lib.hpp"

namespace ArgoRcLib {

PidController::PidController(Hardware::ArduinoInterface &hardware)
    : m_hardware(hardware), m_previousError(),
      m_previousTime(m_hardware.millis()), m_totalIntegral(0) {}

PwmTargets
PidController::calculatePwmTargets(const Hardware::WheelSpeeds &targetSpeeds) {
  (void)targetSpeeds;
  return PwmTargets();
}

int16_t PidController::calcProportional(const Libs::Speed &speedError,
                                        const Libs::Time &timeDifference) {
  (void)speedError;
  (void)timeDifference;
  return 0;
}

int16_t PidController::calcIntegral(const Libs::Speed &speedError,
                                    const Libs::Time &timeDifference) {
  (void)speedError;
  (void)timeDifference;
  return 0;
}

int16_t PidController::calcDifferential(const Libs::Speed &speedError,
                                        const Libs::Time &timeDifference) {
  (void)speedError;
  (void)timeDifference;
  return 0;
}

void PidController::resetPid() {}

} // namespace ArgoRcLib