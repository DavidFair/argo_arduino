
#include "math.h"
#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "ArduinoInterface.hpp"
#include "Encoder.hpp"
#include "Speed.hpp"
#include "Time.hpp"
#include "arduino_enums.hpp"
#include "arduino_lib_wrapper.hpp"
#include "argo_rc_lib.hpp"

namespace {
/// The delay before the vehicles telemtry is sent
const unsigned long OUTPUT_DELAY = 200;
/// The delay before another pint is sent
const unsigned long PING_DELAY = 100;

/// The minimum PWM value before engaging the vehicles drive
const int MIN_PWM_VAL = 5;
/// The maximum value a motor PWM output can hold
const int PWM_MAXIMUM_OUTPUT = 255;

/// The central PWM value received from the RC receiver
const int PWM_RC_CENTER_VAL = 1520;
/// The range of PWM values which the RC receiver can send
const int PWM_RC_RANGE = 500;

/// The maximum linear speed under RC control
const int MAX_RC_LINEAR_VEL = 10000; // in millimeters/s or 10 meters/s
/// The maximum rotational velocity under RC control
// One rotation per second in milli units
const int MAX_ROTATIONAL_VEL = M_PI * 2 * 100;

/// The length between the wheels on the Argo
const double LEN_BETWEEN_WHEELS = 1.473; // Meters

/**
 * Constrains a given input to be between the min and max value
 * which is returned. If mustBePos is set the absolute value is
 * returned instead
 *
 * @param initialValue The value to constrain
 * @param minValue The minimum value allowed (inclusive)
 * @param maxValue The maximum value allowed (inclusive)
 * @param mustBePos (Default: false) If true the absolute value is returned
 *
 * @return The constrained value
 */
int constrainInput(int initialValue, int minValue, int maxValue,
                   bool mustBePos = false) {
  if (initialValue < minValue) {
    initialValue = minValue;
  } else if (initialValue > maxValue) {
    initialValue = maxValue;
  }

  if (mustBePos)
    initialValue = abs(initialValue);

  return initialValue;
}

} // namespace

using namespace ArduinoEnums;
using namespace Globals;
using namespace Hardware;
using namespace Libs;

namespace ArgoRcLib {
/**
 * Constructs a new ArgoRc object with the hardware interface
 * provided. If usePingTimeout if enabled the device will
 * set the speed to 0 if a ping is not received in time (see
 * SerialComms.isPingGood())
 *
 * @param hardwareInterface Reference to the Arduino Hardware
 * @param usePingTimeout Indicates whether pings timeout should be used
 */
ArgoRc::ArgoRc(Hardware::ArduinoInterface &hardwareInterface,
               bool usePingTimeout)
    : m_usePingTimeout(usePingTimeout),
      m_pingTimer(PING_DELAY, hardwareInterface.millis()),
      m_serialOutputTimer(OUTPUT_DELAY, hardwareInterface.millis()),
      m_hardwareInterface(hardwareInterface), m_encoders(m_hardwareInterface),
      m_commsObject(m_hardwareInterface), m_pidController(m_hardwareInterface) {
}

/**
 * Sets the Arduino into a safe state, starts the serial communications
 * and sets up various pin modes
 */
void ArgoRc::setup() {
  m_hardwareInterface.serialBegin(115200);

  setupDigitalPins();
  footswitch_off();
  direction_relays_off();

  m_hardwareInterface.analogWrite(pinMapping::STEERING_PWM_OUTPUT, 0);
  m_hardwareInterface.analogWrite(pinMapping::BRAKING_PWM_OUTPUT, 128);
}

/**
 * Turns the direction relays of the Argo off
 */
void ArgoRc::direction_relays_off() {
  ArgoData::g_currentVehicleDirection.leftWheelDirection = Direction::STOP;
  ArgoData::g_currentVehicleDirection.rightWheelDirection = Direction::STOP;
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                   digitalIO::E_HIGH);
}

/**
 * Sets the relays on the Argo ready to move forward left
 */
void ArgoRc::forward_left() {
  footswitch_on();
  ArgoData::g_currentVehicleDirection.leftWheelDirection = Direction::FORWARD;
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                   digitalIO::E_LOW);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                   digitalIO::E_HIGH);
}

/**
 * Sets the relays on the Argo ready to move forward right
 */
void ArgoRc::forward_right() {
  footswitch_on();
  ArgoData::g_currentVehicleDirection.rightWheelDirection = Direction::FORWARD;
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                   digitalIO::E_LOW);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                   digitalIO::E_HIGH);
}

/**
 * Sets the relays on the Argo ready to move reverse left
 */
void ArgoRc::reverse_left() {
  footswitch_on();
  ArgoData::g_currentVehicleDirection.leftWheelDirection = Direction::REVERSE;
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                   digitalIO::E_LOW);
}

/**
 * Sets the relays on the Argo ready to move reverse right
 */
void ArgoRc::reverse_right() {
  footswitch_on();
  ArgoData::g_currentVehicleDirection.rightWheelDirection = Direction::REVERSE;
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                   digitalIO::E_LOW);
}

/**
 * Switches the drive footswitch on the Argo on
 */
void ArgoRc::footswitch_on() {
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                   digitalIO::E_LOW);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                   digitalIO::E_LOW);
}

/**
 * Switches the drive footswitch on the Argo off
 */
void ArgoRc::footswitch_off() {
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                   digitalIO::E_HIGH);
}

/**
 * Main program loop, dispatches serial communications when appropriate,
 * checks various safety functions. Gets and updates the target speeds
 * and writes out the target PWM values to the motor controls.
 */
void ArgoRc::loop() {
  auto currentTime = m_hardwareInterface.millis();

  m_commsObject.parseIncomingBuffer();

  auto currentSpeed = m_encoders.calculateSpeed();

  // If channel is more than threshold we use RC
  const bool usingRcInput =
      InterruptData::g_pinData[2].lastGoodWidth >= PWM_RC_CENTER_VAL;

  bool pingTimedOut = false;
  PwmTargets targetPwmVals;

  if (usingRcInput) {
    auto targetSpeed = mapRcInput();
    m_pidController.calculatePwmTargets(currentSpeed, targetSpeed);
  } else {
    // Ros control
    pingTimedOut = m_usePingTimeout && !m_commsObject.isPingGood();
    // If ping has timed out reset speed to 0 else get target speed
    auto targetSpeed = pingTimedOut ? Hardware::WheelSpeeds()
                                    : m_commsObject.getTargetSpeeds();
    targetPwmVals =
        m_pidController.calculatePwmTargets(currentSpeed, targetSpeed);
  }

  applyPwmOutput(targetPwmVals);

  if (m_pingTimer.hasTimerFired(currentTime)) {
    if (pingTimedOut) {
      m_commsObject.addWarning("Ping has timed out");
    }

    m_commsObject.addPing();

    m_pingTimer.reset(currentTime);
  }

  // Check if were ready to send our buffer details over serial yet
  // if we still have comms
  if (!pingTimedOut && m_serialOutputTimer.hasTimerFired(currentTime)) {
    m_serialOutputTimer.reset(currentTime);
    m_commsObject.addEncoderRotation(m_encoders.read());
    m_commsObject.addVehicleSpeed(currentSpeed);
    m_commsObject.addPwmValues(targetPwmVals);
  }

  m_commsObject.sendCurrentBuffer();
}

// ---------- Private Methods --------------

void ArgoRc::applyPwmOutput(const PwmTargets &pwmValues) {
  int leftPwmValue = pwmValues.leftPwm;
  int rightPwmValue = pwmValues.rightPwm;

  if ((leftPwmValue > -MIN_PWM_VAL && leftPwmValue < MIN_PWM_VAL) &&
      (rightPwmValue > -MIN_PWM_VAL && rightPwmValue < MIN_PWM_VAL)) {
    footswitch_off();
    direction_relays_off();
  }

  if (leftPwmValue >= MIN_PWM_VAL) {
    forward_left();
  } else if (leftPwmValue <= -MIN_PWM_VAL) {
    reverse_left();
  }

  if (rightPwmValue >= MIN_PWM_VAL) {
    forward_right();
  } else if (rightPwmValue <= -MIN_PWM_VAL) {
    reverse_right();
  }

  m_hardwareInterface.analogWrite(pinMapping::LEFT_PWM_OUTPUT,
                                  abs(leftPwmValue));
  m_hardwareInterface.analogWrite(pinMapping::RIGHT_PWM_OUTPUT,
                                  abs(rightPwmValue));
}

/**
 * Sets the PWM values for the left and right wheels based on
 * the target speed and steering values. This is used when the
 * vehicle is under remote control
 *
 * @param velocity The target velocity in millis / s of the vehicle
 * @param angular The target anglar momentum (rad * 100) / s of the vehicle
 *
 * @return An object representing the left and right speed targets
 */
Hardware::WheelSpeeds ArgoRc::calculateVelocities(int velocity,
                                                  int angularMomentum) {
  // Divide difference by two to add component to each wheel
  int velocityDifference = (angularMomentum * LEN_BETWEEN_WHEELS) / 2;

  Distance leftWheel(0, velocity + velocityDifference);
  Distance rightWheel(0, velocity - velocityDifference);

  Speed leftSpeed(leftWheel, 1_s);
  Speed rightSpeed(rightWheel, 1_s);

  Hardware::WheelSpeeds newSpeeds(leftSpeed, rightSpeed);

  return newSpeeds;
}

/**
 * Reads the PWM input from the remote control based on the last
 * interrupt data received and maps it to a target speed
 *
 * @return An object representing the left and right wheel speeds
 */
Hardware::WheelSpeeds ArgoRc::mapRcInput() {
  // Get PWM high edge times
  int rcPwmThrottleRaw = InterruptData::g_pinData[0].lastGoodWidth;
  int rcPwmSteeringRaw = InterruptData::g_pinData[1].lastGoodWidth;

  if (rcPwmThrottleRaw == 0 && rcPwmSteeringRaw == 0) {
    // Pin probably not connected or no data. Bail setting the PWM to 0
    return WheelSpeeds(Speed(), Speed());
  }

  constexpr int minValue = PWM_RC_CENTER_VAL - PWM_RC_RANGE;
  constexpr int maxValue = PWM_RC_CENTER_VAL + PWM_RC_RANGE;

  // Clamp to expected min and max values before mapping
  rcPwmThrottleRaw = constrainInput(rcPwmThrottleRaw, minValue, maxValue);
  rcPwmSteeringRaw = constrainInput(rcPwmSteeringRaw, minValue, maxValue);

  // Map onto min and max linear speeds
  int targetVel = map(rcPwmThrottleRaw, minValue, maxValue, -MAX_RC_LINEAR_VEL,
                      MAX_RC_LINEAR_VEL);
  // Map onto min and max angular momentums
  int targetAngMomentum = map(rcPwmSteeringRaw, minValue, maxValue,
                              -MAX_ROTATIONAL_VEL, MAX_ROTATIONAL_VEL);

  return calculateVelocities(targetVel, targetAngMomentum);
}

/**
 * Sets all the digital pins to their appropriate modes
 * such as input or output mode
 */
void ArgoRc::setupDigitalPins() {
  m_hardwareInterface.setPinMode(pinMapping::LEFT_FORWARD_RELAY,
                                 digitalIO::E_OUTPUT);
  m_hardwareInterface.setPinMode(pinMapping::LEFT_REVERSE_RELAY,
                                 digitalIO::E_OUTPUT);

  m_hardwareInterface.setPinMode(pinMapping::RIGHT_FORWARD_RELAY,
                                 digitalIO::E_OUTPUT);
  m_hardwareInterface.setPinMode(pinMapping::RIGHT_REVERSE_RELAY,
                                 digitalIO::E_OUTPUT);

  m_hardwareInterface.setPinMode(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                 digitalIO::E_OUTPUT);
  m_hardwareInterface.setPinMode(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                 digitalIO::E_OUTPUT);

  m_hardwareInterface.setPinMode(pinMapping::LEFT_ENCODER,
                                 digitalIO::E_INPUT_PULLUP);

  m_hardwareInterface.setPinMode(pinMapping::RIGHT_ENCODER,
                                 digitalIO::E_INPUT_PULLUP);

  m_hardwareInterface.setPinMode(pinMapping::RC_DEADMAN, digitalIO::E_INPUT);
}

} // namespace ArgoRcLib