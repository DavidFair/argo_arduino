#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <stdint.h>

#include "ArduinoInterface.hpp"
#include "Distance.hpp"
#include "Encoder.hpp"


namespace ArgoRcLib {

struct PID_CONSTANTS {
  /// Every n milliseconds calculates a new PWM value
  constexpr static float timeBetween{50};

  /// The proportional component
  constexpr static float prop{0.2};

  /// The integral component
  constexpr static float integral{0.01};

  // Controls how quickly the acceleration ramps up preventing large spikes
  /// Derivative on measurement component
  constexpr static float deriv{0.01};
};

/// Struct describing PWM targets for the left and right wheel
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

/// Class that implements PID controllers for both wheels
class PidController {
public:
  /// Constructs a PID controller for the given hardware
  PidController(Hardware::ArduinoInterface &hardware);

  // Copy constructors
  PidController(PidController &) = delete;
  PidController &operator=(PidController &) = delete;

  // Move constructors
  PidController(PidController &&other);
  PidController &operator=(PidController &&other);

  /// Calculates new PWM targets based on the current and target speeds
  PwmTargets calculatePwmTargets(const Hardware::WheelSpeeds &currentSpeeds,
                                 const Hardware::WheelSpeeds &targetSpeeds);

  /// Calculates the proportional component based on difference in speed
  float calcProportional(const Libs::Distance &errorPerSec);

  /// Calculates the integral component based on the difference in speed
  float calcIntegral(const Libs::Distance &errorPerSec,
                     Hardware::EncoderPositions position,
                     const Libs::Time &timeDiff);

  /// Calculates the derivative component based on the difference in speed
  float calcDeriv(const Libs::Distance &errorPerSec,
                  Hardware::EncoderPositions position,
                  const Libs::Time &timeDiff);

  /// Resets the PID controller to its initial state of 0
  void resetPid();

private:
  /// Calculates a single PWM value based on the current and target speeds
  float calculatePwmValue(const Libs::Speed &currentSpeed,
                          const Libs::Speed &targetSpeed,
                          Hardware::EncoderPositions position);

  /// Reference to the Arduino hardware
  Hardware::ArduinoInterface &m_hardwareInterface;

  /// Time when the last PID values were calculated
  Libs::Time m_previousTime;
  /// Time difference between now and the last time PWM values were calculated
  Libs::Time m_timeDiff;

  /// The previously calculated PWM targets
  PwmTargets m_previousTargets;

  /// Holds the previous delta in speed for each wheel
  Libs::Distance m_previousError[Hardware::EncoderPositions::_NUM_OF_ENCODERS];
  /// Holds the sum of the integral for each wheel
  float m_totalIntegral[Hardware::EncoderPositions::_NUM_OF_ENCODERS];
};

} // namespace ArgoRcLib

#endif // PID_CONTROLLER_HPP_