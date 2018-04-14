#include <stdint.h>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "PidController.hpp"
#include "argo_rc_lib.hpp"

using namespace Libs;
using namespace Hardware;

namespace {
constexpr int MAX_PWM_VAL = 255;
constexpr Distance STOP_SPEED_DIST(0, 100); // 100 mm
constexpr Speed STOP_SPEED{STOP_SPEED_DIST,
                           1_s}; // Below 0.1 m/s we consider stopped

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

PidController::PidController(Hardware::ArduinoInterface &hardware)
    : m_hardwareInterface(hardware),
      m_previousTime(m_hardwareInterface.millis()),
      m_previousError(), m_totalIntegral{0, 0} {}

PidController::PidController(PidController &&other)
    : m_hardwareInterface(other.m_hardwareInterface),
      m_previousTime(other.m_previousTime), m_previousError(),
      m_totalIntegral() {
  for (int i = 0; i < EncoderPositions::_NUM_OF_ENCODERS; i++) {
    m_previousError[i] = other.m_previousError[i];
    m_totalIntegral[i] = other.m_totalIntegral[i];
  }
}

PidController &PidController::operator=(PidController &&other) {
  m_hardwareInterface = other.m_hardwareInterface;
  m_previousTime = other.m_previousTime;
  for (int i = 0; i < EncoderPositions::_NUM_OF_ENCODERS; i++) {
    m_previousError[i] = other.m_previousError[i];
    m_totalIntegral[i] = other.m_totalIntegral[i];
  }
  return *this;
}

PwmTargets
PidController::calculatePwmTargets(const Hardware::WheelSpeeds &currentSpeeds,
                                   const Hardware::WheelSpeeds &targetSpeeds) {
  const bool stopLeft = targetSpeeds.leftWheel >= -STOP_SPEED &&
                        targetSpeeds.leftWheel <= STOP_SPEED;
  const bool stopRight = targetSpeeds.rightWheel >= -STOP_SPEED &&
                         targetSpeeds.rightWheel <= STOP_SPEED;

  // If we are between -STOP_SPEED and STOP_SPEED interval
  if (stopLeft && stopRight) {
    // Don't use a PID controller to stop - set the target to 0
    resetPid();
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

float PidController::calcProportional(const Distance &errorPerSec) {
  return PID_CONSTANTS::prop * errorPerSec.meters();
}

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