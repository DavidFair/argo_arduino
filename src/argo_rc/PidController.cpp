#include <stdint.h>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "PidController.hpp"
#include "argo_rc_lib.hpp"

using namespace Libs;
using namespace Hardware;

namespace {
/// The maximum value that can be sent over PWM
constexpr int MAX_PWM_VAL = 255;
/// The minimum distance per second before stopping instead
const Distance STOP_SPEED_DIST(0, 100); // 100mm
/// The minimum speed before stopping instead
const Speed STOP_SPEED{STOP_SPEED_DIST, 1_s}; // < 0.1m/s is stopped

/**
 * Constrains a calculated PWM value
 * to within the range of -MAX_PWM_VAL to MAX_PWM_VAL
 *
 * @param val The value to constrain
 * @return A constrained value between the specified range
 */
float constrainPwmValue(float val) {
  if (val > MAX_PWM_VAL) {
    val = MAX_PWM_VAL;
  } else if (val < -MAX_PWM_VAL) {
    val = -MAX_PWM_VAL;
  }
  return val;
}
} // End of anonymous namespace

namespace ArgoRcLib {

/**
 * Creates a new PWM controller for the given hardware
 *
 * @param hardware The Arduino hardware to use
 */
PidController::PidController(Hardware::ArduinoInterface &hardware)
    : m_hardwareInterface(hardware),
      m_previousTime(m_hardwareInterface.millis()),
      m_previousError(), m_totalIntegral{0, 0} {}

/// Move constructor
PidController::PidController(PidController &&other)
    : m_hardwareInterface(other.m_hardwareInterface),
      m_previousTime(other.m_previousTime), m_previousError(),
      m_totalIntegral() {
  for (int i = 0; i < EncoderPositions::_NUM_OF_ENCODERS; i++) {
    m_previousError[i] = other.m_previousError[i];
    m_totalIntegral[i] = other.m_totalIntegral[i];
  }
}

/// Move operator
PidController &PidController::operator=(PidController &&other) {
  m_hardwareInterface = other.m_hardwareInterface;
  m_previousTime = other.m_previousTime;
  for (int i = 0; i < EncoderPositions::_NUM_OF_ENCODERS; i++) {
    m_previousError[i] = other.m_previousError[i];
    m_totalIntegral[i] = other.m_totalIntegral[i];
  }
  return *this;
}

/**
 * Calculates the current PWM targets based off the difference in
 * current speed and target speeds.
 *
 * If the target speed is below a threshold (see STOP_SPEED) the
 * vehicle instead stops and resets the PWM controllers.
 *
 * If an elapsed time has passed (see PID_CONSTANTS::timeBetween)
 * the PWM values are recalculated relative to previous calculations.
 *
 * If the elapsed time has not passed the previous PWM values are returned
 * instead
 *
 * @param currentSpeeds The vehicles current speeds
 * @param targetSpeeds The vehicles target speeds
 *
 * @return An object containing the left and right PWM values
 */
PwmTargets
PidController::calculatePwmTargets(const Hardware::WheelSpeeds &currentSpeeds,
                                   const Hardware::WheelSpeeds &targetSpeeds) {
  const bool stopLeft = targetSpeeds.leftWheel >= -STOP_SPEED &&
                        targetSpeeds.leftWheel <= STOP_SPEED;
  const bool stopRight = targetSpeeds.rightWheel >= -STOP_SPEED &&
                         targetSpeeds.rightWheel <= STOP_SPEED;

  // If we are between -STOP_SPEED and STOP_SPEED interval
  if (stopLeft && stopRight) {
    // Don't use a PID controller to stop - reset and exit.
    resetPid(); // This sets m_previousTargets to 0
    return m_previousTargets;
  }

  const auto timeDiff = m_hardwareInterface.millis() - m_previousTime.millis();
  if (timeDiff < PID_CONSTANTS::timeBetween) {
    return m_previousTargets;
  }

  m_timeDiff = timeDiff;

  PwmTargets target;
  target.leftPwm =
      calculatePwmValue(currentSpeeds.leftWheel, targetSpeeds.leftWheel,
                        EncoderPositions::LEFT_ENCODER);
  target.rightPwm =
      calculatePwmValue(currentSpeeds.rightWheel, targetSpeeds.rightWheel,
                        EncoderPositions::RIGHT_ENCODER);
  auto newTarget = m_previousTargets + target;

  // Constrain to max
  newTarget.leftPwm = constrainPwmValue(newTarget.leftPwm);
  newTarget.rightPwm = constrainPwmValue(newTarget.rightPwm);

  m_previousTargets = newTarget;
  m_previousTime = m_hardwareInterface.millis();
  return newTarget;
}

/**
 * Calculates the proportional component of the PWM controller
 * based on the normalised (to a unit second) error in velocity
 *
 * @param errorPerSec The normalised distance per second delta error
 * @return The proportional contribution to the PWM value
 */
float PidController::calcProportional(const Distance &errorPerSec) {
  return PID_CONSTANTS::prop * errorPerSec.meters();
}

/**
 * Calculates the integral component of the PWM controller based
 * on the normalised (to a unit second) error in velocity and
 * time since it was last called for a given set of wheels
 * as indicated by position.
 *
 * The value is summed into the complete integration (see m_totalIntegral)
 * but constrained to within the maximum PWM values (see constrainPwmInput())
 *
 * @param errorPerSec The normalised distance per second delta error
 * @param position The wheels to calculate the integral component for
 * @param timeDiff The time elapsed since the last call to this method
 *
 * @return The integral contribution to the PWM value
 */
float PidController::calcIntegral(const Distance &errorPerSec,
                                  EncoderPositions position,
                                  const Libs::Time &timeDiff) {
  auto errorSpeed = errorPerSec.meters();

  float integralVal = errorSpeed * PID_CONSTANTS::integral * timeDiff.seconds();
  float newIntegralVal = m_totalIntegral[position] + integralVal;

  // Constrain to a maximum of 255 to prevent "lag" whilst the integral drains
  newIntegralVal = constrainPwmValue(newIntegralVal);

  m_totalIntegral[position] = newIntegralVal;

  return newIntegralVal;
}

/**
 * Calculates the derivative component of the PWM controller based
 * on the normalised (to a unit second) error in velocity and
 * time since it was last called for a given set of wheels
 * as indicated by position.
 *
 * The value is based on the difference between the last
 * velocity delta to the current velocity delta.
 *
 * @param errorPerSec The normalised distance per second delta error
 * @param position The wheels to calculate the derivative component for
 * @param timeDiff The time elapsed since the last call to this method
 *
 * @return The derivative contribution to the PWM value
 */
float PidController::calcDeriv(const Distance &errorPerSec,
                               EncoderPositions position,
                               const Libs::Time &timeDiff) {
  const auto errorDifference = errorPerSec - m_previousError[position];
  m_previousError[position] = errorPerSec;

  const auto errorDelta = errorDifference.meters();
  float derivVal = (errorDelta * PID_CONSTANTS::deriv) / timeDiff.seconds();

  // As we want to resist kicking the motors up power when the set
  // point changes this is a negative contribution
  derivVal = -derivVal;
  return derivVal;
}

/**
 * Resets the PWM internal variables to their initial state and
 * sets the previous targets to 0 (see m_previousTargets)
 */
void PidController::resetPid() {
  PwmTargets stopTarget{0, 0};
  m_previousTargets = stopTarget;
  m_previousTime = m_hardwareInterface.millis();

  for (int i = 0; i < EncoderPositions::_NUM_OF_ENCODERS; i++) {
    m_previousError[i] = Distance{};
    m_totalIntegral[i] = 0;
  }
}

// -------- Private Methods ------

/**
 * Calculates an individual PWM values for based on the delta
 * in speed for a given wheel position
 *
 * @param currentSpeed The current speed of the given wheel
 * @param targetSpeed The target speed of the given wheel
 * @param position The wheel position to calculate for
 *
 * @return The new target PWM value for the given wheel
 */
float PidController::calculatePwmValue(const Libs::Speed &currentSpeed,
                                       const Libs::Speed &targetSpeed,
                                       EncoderPositions position) {
  const auto speedError = targetSpeed - currentSpeed;
  const Distance unitError = speedError.getUnitDistance();

  auto p = calcProportional(unitError);
  auto i = calcIntegral(unitError, position, m_timeDiff);
  auto d = calcDeriv(unitError, position, m_timeDiff);

  float sum = constrainPwmValue(p + i + d);

  return sum;
}

} // namespace ArgoRcLib