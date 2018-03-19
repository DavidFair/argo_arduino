#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <stdint.h>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "arduino_interface.hpp"

namespace ArgoRcLib {

struct PID_CONSTANTS {
  // Every n milliseconds check PWM value
  constexpr static int timeBetween{500};

  // PWM constant
  constexpr static float powerConstant{1e-2};

  // Controls how quickly the vehicle accelerates
  constexpr static float prop{0};

  // Helps the vehicle reach target speed -
  // if too high the vehicle speed oscillates
  constexpr static float integral{0};

  // Controls how quickly the acceleration ramps up preventing large spikes
  // Aka derivative on measurement
  constexpr static float deriv{0};
};

struct PwmTargets {
  PwmTargets() : PwmTargets(0, 0) {}

  PwmTargets(int leftPwmTarget, int rightPwmTarget)
      : leftPwm(leftPwmTarget), rightPwm(rightPwmTarget) {}

  int leftPwm{0};
  int rightPwm{0};
};

class PidController {
public:
  PidController(Hardware::ArduinoInterface &hardware);

  PidController(PidController &) = delete;
  PidController &operator=(PidController &) = delete;

  PidController(PidController &&other);
  PidController &operator=(PidController &&other);

  PwmTargets calculatePwmTargets(const Hardware::WheelSpeeds &currentSpeeds,
                                 const Hardware::WheelSpeeds &targetSpeeds);

  int16_t calcProportional(const Libs::Distance &errorPerSec);

  int16_t calcIntegral(const Libs::Distance &errorPerSec,
                       Hardware::EncoderPositions position);

  int16_t calcDeriv(const Libs::Distance &errorPerSec,
                    Hardware::EncoderPositions position);

  void resetPid();

private:
  int16_t calculatePwmValue(const Libs::Speed &currentSpeed,
                            const Libs::Speed &targetSpeed,
                            Hardware::EncoderPositions position);

  Hardware::ArduinoInterface &m_hardwareInterface;

  unsigned long m_previousTime;
  PwmTargets m_previousTargets;

  Libs::Distance m_previousError[Hardware::EncoderPositions::_NUM_OF_ENCODERS];
  int16_t m_totalIntegral[Hardware::EncoderPositions::_NUM_OF_ENCODERS];
};

} // namespace ArgoRcLib

#endif // PID_CONTROLLER_HPP_