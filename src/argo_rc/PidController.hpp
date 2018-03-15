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

struct PID_CONSTANTS {
  //  Boundary speed in mm/s where we switch to less aggressive constants
  constexpr static float boundarySpeed{700};

  // Controls how quickly the vehicle accelerates
  constexpr static float propLower{0.040};
  constexpr static float propUpper{0.085};

  // Helps the vehicle reach target speed -
  // if too high the vehicle speed oscillates
  constexpr static float integralLower{0.01};
  constexpr static float integralUpper{0.03};

  // Controls how quickly the acceleration ramps up preventing large spikes
  // Aka derivative on measurement
  constexpr static float derivLower{0.01};
  constexpr static float derivUpper{0.03};
};

class PidController {
public:
  PidController(Hardware::ArduinoInterface &hardware);

  PwmTargets calculatePwmTargets(const Hardware::WheelSpeeds &targetSpeeds);

  int16_t calcProportional(const Libs::Distance &errorPerSec);

  int16_t calcIntegral(const Libs::Distance &errorPerSec);

  int16_t calcDeriv(const Libs::Distance &errorPerSec);

  void resetPid();

private:
  static bool isLowerThanBoundary(const int32_t &val);

  Hardware::ArduinoInterface &m_hardware;

  Libs::Distance m_previousError;
  unsigned long m_previousTime;

  int16_t m_totalIntegral{0};
};

} // namespace ArgoRcLib

#endif // PID_CONTROLLER_HPP_