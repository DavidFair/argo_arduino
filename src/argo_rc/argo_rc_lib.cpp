
#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "ArduinoInterface.hpp"
#include "arduino_enums.hpp"
#include "arduino_lib_wrapper.hpp"
#include "argo_rc_lib.hpp"

namespace {
/// The time between the deadman switch first toggling and it being tested again
const unsigned long DEADMAN_TIMEOUT_DELAY = 500;
/// The delay before the vehicles telemtry is sent
const unsigned long OUTPUT_DELAY = 200;
/// The delay before another pint is sent
const unsigned long PING_DELAY = 100;

/// The minimum PWM value before engaging the vehicles drive
const int MIN_PWM_VAL = 20;

/// THe maximum value a PWM output can hold
const int PWM_MAXIMUM_OUTPUT = 255;

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
  if (!checkDeadmanSwitch()) {
    // This is for unit testing - enterDeadmanSafetyMode on hardware gets
    // stuck in an infinite loop
    return;
  }

  auto currentTime = m_hardwareInterface.millis();

  m_commsObject.parseIncomingBuffer();

  auto currentSpeed = m_encoders.calculateSpeed();

  // Check if were ready to send our buffer details over serial yet
  if (m_serialOutputTimer.hasTimerFired(currentTime)) {
    m_serialOutputTimer.reset(currentTime);
    m_commsObject.addEncoderRotation(m_encoders.read());
    m_commsObject.addVehicleSpeed(currentSpeed);
  }

  // Deadman switch is high at this point
  // ---- RC control ----
  auto targetPwmVals = readPwmInput();

  // // ------- ROS control ------
  // auto targetSpeed = m_commsObject.getTargetSpeeds();
  // if (m_usePingTimeout && !m_commsObject->isPingGood()) {
  //   m_commsObject.addWarning("Ping timeout. Resetting speed to 0");
  //   targetSpeed.leftWheel = 0;
  //   targetSpeed.rightWheel = 0;
  // }
  // auto targetPwmVals =
  //     m_pidController.calculatePwmTargets(currentSpeed, targetSpeed);
  // // --------------

  int leftPwmValue = targetPwmVals.leftPwm;
  int rightPwmValue = targetPwmVals.rightPwm;

  if ((leftPwmValue > -MIN_PWM_VAL && leftPwmValue < MIN_PWM_VAL) &&
      (rightPwmValue > -MIN_PWM_VAL && rightPwmValue < MIN_PWM_VAL)) {
    footswitch_off();
    direction_relays_off();
  }

  if (leftPwmValue >= MIN_PWM_VAL)
    forward_left();
  if (rightPwmValue >= MIN_PWM_VAL)
    forward_right();
  if (leftPwmValue <= -MIN_PWM_VAL)
    reverse_left();
  if (rightPwmValue <= -MIN_PWM_VAL)
    reverse_right();

  m_hardwareInterface.analogWrite(pinMapping::LEFT_PWM_OUTPUT,
                                  abs(leftPwmValue));
  m_hardwareInterface.analogWrite(pinMapping::RIGHT_PWM_OUTPUT,
                                  abs(rightPwmValue));

  if (m_pingTimer.hasTimerFired(currentTime)) {
    m_commsObject.addPing();
    m_pingTimer.reset(currentTime);
  }

  m_commsObject.sendCurrentBuffer();
}

// ---------- Private Methods --------------

/**
 * Checks whether the deadman switch has toggled. If it has it proceeds
 * to check if the switch toggles off within a specified time
 * (see DEADMAN_TIMEOUT_DELAY). If the switch is safe true is returned.
 * If the switch is failed and on the Arduino an infinite loop is entered
 * after making the vehicle safe.
 * If the switch is failed and unit testing the function returns false
 *
 * @return True if the switch is safe, false if unit testing and the
 * switch has failed.
 */
bool ArgoRc::checkDeadmanSwitch() {

  if (m_hardwareInterface.digitalRead(RC_DEADMAN) == digitalIO::E_HIGH) {
    return true;
  }

  unsigned long startingTime = m_hardwareInterface.millis();

  while ((m_hardwareInterface.millis() - startingTime) <
         DEADMAN_TIMEOUT_DELAY) {
    // Check its not a switch bounce
    if (m_hardwareInterface.digitalRead(RC_DEADMAN) == digitalIO::E_HIGH)
      return true;
  }

  // Pin still has not driven high after DEADMAN_TIMEOUT seconds
  enterDeadmanFail();

  // For unit testing
  return false;
} // namespace ArgoRcLib

/**
 * Sets the PWM values for the left and right wheels based on
 * the target speed and steering values. This is used when the
 * vehicle is under remote control
 *
 * @param speed The target speed of the vehicle
 * @param steer The target steering angle of the vehicle
 *
 * @return An object representing the left and right PWM targets
 */
PwmTargets ArgoRc::setMotorTarget(int speed, int steer) {
  PwmTargets targetPwmVals;

  if (abs(speed) < 20 && abs(steer) > 2) {
    targetPwmVals.leftPwm = steer;
    targetPwmVals.rightPwm = -steer;
  } else {
    targetPwmVals.leftPwm = speed * ((-255 - steer) / -255.0);
    targetPwmVals.rightPwm = speed * ((255 - steer) / 255.0);
  }
  return targetPwmVals;
}

/**
 * Enters a safety mode when the deadman switch has toggled. The
 * remote control input is set to 0, the vehicle is stopped and
 * all relays are switched to a safe position.
 * If this is executed on the Arduino it enters an infinite loop
 * which emits a fatal message.
 * If this is executed whilst unit testing it returns
 */
void ArgoRc::enterDeadmanFail() {
  InterruptData::g_pinData[0].lastGoodWidth = 0;
  InterruptData::g_pinData[1].lastGoodWidth = 0;
  m_hardwareInterface.analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0);
  m_hardwareInterface.analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0);

  // turn off all direction relays and footswitch
  footswitch_off();
  direction_relays_off();
  m_hardwareInterface.enterDeadmanSafetyMode();
}

/**
 * Reads the PWM input from the remote control based on the last
 * interrupt data received
 *
 * @return An object representing the left and right wheel PWM values
 */
PwmTargets ArgoRc::readPwmInput() {
  int rcPwmThrottleRaw = InterruptData::g_pinData[0].lastGoodWidth;
  int rcPwmSteeringRaw = InterruptData::g_pinData[1].lastGoodWidth;

  if (rcPwmThrottleRaw == 0 && rcPwmSteeringRaw == 0) {
    // Pin probably not connected or no data. Bail setting the PWM to 0
    return PwmTargets(0, 0);
  }

  constexpr int centerPoint = 1520;
  constexpr int range = 330;

  constexpr int minValue = centerPoint - range;
  constexpr int maxValue = centerPoint + range;

  rcPwmThrottleRaw = constrainInput(rcPwmThrottleRaw, minValue, maxValue);
  rcPwmSteeringRaw = constrainInput(rcPwmSteeringRaw, minValue, maxValue);

  int throttleTarget = map(rcPwmThrottleRaw, minValue, maxValue,
                           -PWM_MAXIMUM_OUTPUT, PWM_MAXIMUM_OUTPUT);
  int steeringTarget = map(rcPwmSteeringRaw, minValue, maxValue,
                           -PWM_MAXIMUM_OUTPUT, PWM_MAXIMUM_OUTPUT);

  return setMotorTarget(throttleTarget, steeringTarget);
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