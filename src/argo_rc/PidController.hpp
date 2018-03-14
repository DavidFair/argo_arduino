#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <stdint.h>

#include "Encoder.hpp"
#include "Speed.hpp"
#include "Time.hpp"
#include "arduino_interface.hpp"
#include "argo_rc_lib.hpp"

namespace ArgoRcLib {

// Forward declarations to solve cyclic deps
struct PwmTargets;

struct PidConstants {
  float proportional{1};
  float integral{1};
  float derivative{1};
};

class PidController {
public:
  PidController(Hardware::ArduinoInterface &hardware);

  PwmTargets calculatePwmTargets(const Hardware::WheelSpeeds &targetSpeeds);

  int16_t calcProportional(const Libs::Speed &speedError,
                           const Libs::Time &timeDifference);

  int16_t calcIntegral(const Libs::Speed &speedError,
                       const Libs::Time &timeDifference);

  int16_t calcDifferential(const Libs::Speed &speedError,
                           const Libs::Time &timeDifference);

  void resetPid();

private:
  static const PidConstants m_pidConstants;

  Hardware::ArduinoInterface &m_hardware;

  Libs::Speed m_previousError;
  unsigned long m_previousTime;

  int32_t m_totalIntegral{0};
};

} // namespace ArgoRcLib

#endif // PID_CONTROLLER_HPP_