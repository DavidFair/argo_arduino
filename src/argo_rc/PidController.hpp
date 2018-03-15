#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <stdint.h>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "arduino_interface.hpp"
#include "argo_rc_lib.hpp"

namespace ArgoRcLib {

// Forward declarations to solve cyclic deps
struct PwmTargets;

struct PidConstants {
  //  Boundary speed in mm/s where we switch to less aggressive constants
  float boundarySpeed{700};

  float propLower{0.040};
  float propUpper{0.085};

  float integralUpper{0.1};
  float derivative{0.1};
};

class PidController {
public:
  PidController(Hardware::ArduinoInterface &hardware);

  PwmTargets calculatePwmTargets(const Hardware::WheelSpeeds &targetSpeeds);

  int16_t calcProportional(const Libs::Distance &errorPerSec);

  int16_t calcIntegral(const Libs::Speed &speedError,
                       const Libs::Time &timeDifference);

  int16_t calcDifferential(const Libs::Speed &speedError,
                           const Libs::Time &timeDifference);

  void resetPid();

private:
  Hardware::ArduinoInterface &m_hardware;

  Libs::Speed m_previousError;
  unsigned long m_previousTime;

  int32_t m_totalIntegral{0};
};

} // namespace ArgoRcLib

#endif // PID_CONTROLLER_HPP_