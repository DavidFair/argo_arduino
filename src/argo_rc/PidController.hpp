#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <stdint.h>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "arduino_interface.hpp"

namespace ArgoRcLib {

struct PID_CONSTANTS {
  // Every n milliseconds check PWM value
  constexpr static float timeBetween{50};

  // Controls how quickly the vehicle accelerates
  constexpr static float prop{0.2};

  // Helps the vehicle reach target speed -
  // if too high the vehicle speed oscillates
  constexpr static float integral{0.01};

  // Controls how quickly the acceleration ramps up preventing large spikes
  // Aka derivative on measurement
  constexpr static float deriv{0.01};
};

struct PwmTargets {
  PwmTargets() : PwmTargets(0, 0) {}

  PwmTargets(float leftPwmTarget, float rightPwmTarget)
      : leftPwm(leftPwmTarget), rightPwm(rightPwmTarget) {}

  PwmTargets operator+(const PwmTargets &other) {
    return PwmTargets(leftPwm + other.leftPwm, rightPwm + other.rightPwm);
  }

  float leftPwm{0};
  float rightPwm{0};
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

  float calcProportional(const Libs::Distance &errorPerSec);

  float calcIntegral(const Libs::Distance &errorPerSec,
                     Hardware::EncoderPositions position,
                     const Libs::Time &timeDiff);

  float calcDeriv(const Libs::Distance &errorPerSec,
                  Hardware::EncoderPositions position,
                  const Libs::Time &timeDiff);

  void resetPid();

private:
  float calculatePwmValue(const Libs::Speed &currentSpeed,
                          const Libs::Speed &targetSpeed,
                          Hardware::EncoderPositions position);

  Hardware::ArduinoInterface &m_hardwareInterface;

  Libs::Time m_previousTime;
  Libs::Time m_timeDiff;

  PwmTargets m_previousTargets;

  Libs::Distance m_previousError[Hardware::EncoderPositions::_NUM_OF_ENCODERS];
  float m_totalIntegral[Hardware::EncoderPositions::_NUM_OF_ENCODERS];
};

} // namespace ArgoRcLib

#endif // PID_CONTROLLER_HPP_