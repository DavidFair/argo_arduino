
#include <stdint.h>

//#include <Encoder.h>

#include "ArduinoGlobals.hpp"
#include "arduino_enums.hpp"
#include "arduino_interface.hpp"
#include "arduino_lib_wrapper.hpp"

#include "argo_rc_lib.hpp"

//#define TEST_POT_ENABLED
#define RC_PWM_ENABLED

namespace {
const unsigned long DEADMAN_TIMEOUT_DELAY = 500;
const int PWM_MAXIMUM_OUTPUT = 255;

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

ArgoRc::ArgoRc(Hardware::ArduinoInterface &hardwareInterface)
    : m_hardwareInterface(hardwareInterface), m_encoders(),
      m_commsObject(m_hardwareInterface) {}

void ArgoRc::setup() {
  m_hardwareInterface.serialBegin(115200);

  setupDigitalPins();
  footswitch_off();
  direction_relays_off();

  m_hardwareInterface.analogWrite(pinMapping::STEERING_PWM_OUTPUT, 0);
  m_hardwareInterface.analogWrite(pinMapping::BRAKING_PWM_OUTPUT, 128);

  m_hardwareInterface.digitalWrite(pinMapping::TEST_POT_POSITIVE,
                                   digitalIO::E_HIGH);
}

void ArgoRc::direction_relays_off() {

  m_hardwareInterface.serialPrintln("RELAYS OFF");
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                   digitalIO::E_HIGH);
}

void ArgoRc::forward_left() {
  footswitch_on();
  m_hardwareInterface.serialPrintln("forward_left");
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                   digitalIO::E_LOW);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                   digitalIO::E_HIGH);
}

void ArgoRc::forward_right() {
  footswitch_on();
  m_hardwareInterface.serialPrintln("                  forward_right");
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                   digitalIO::E_LOW);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                   digitalIO::E_HIGH);
}

void ArgoRc::reverse_left() {
  footswitch_on();
  m_hardwareInterface.serialPrintln(
      "                                    reverse_left");
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                   digitalIO::E_LOW);
}

void ArgoRc::reverse_right() {
  footswitch_on();
  m_hardwareInterface.serialPrintln(
      "                                                      reverse_right");
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                   digitalIO::E_LOW);
}

void ArgoRc::footswitch_on() {
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                   digitalIO::E_LOW);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                   digitalIO::E_LOW);
}

void ArgoRc::footswitch_off() {
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                   digitalIO::E_HIGH);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                   digitalIO::E_HIGH);
}

  //#define DEBUG_OUTPUT
#define DEBUG_OUTPUT_PWM

void ArgoRc::loop() {
  m_commsObject.sendEncoderRotation(m_encoders.read());

#ifdef RC_PWM_ENABLED

  if (!checkDeadmanSwitch()) {
    // This is for unit testing - enterDeadmanSafetyMode on hardware gets
    // stuck in an infinite loop
    return;
  }

  // Deadman switch is high at this point
  auto targetPwmVals = readPwmInput();
#endif

  int leftPwmValue = targetPwmVals.leftPwm;
  int rightPwmValue = targetPwmVals.rightPwm;

  if ((leftPwmValue > -40 && leftPwmValue < 40) &&
      (rightPwmValue > -40 && rightPwmValue < 40)) {
    footswitch_off();
    direction_relays_off();
  }

  if (leftPwmValue >= 40)
    forward_left();
  if (rightPwmValue >= 40)
    forward_right();
  if (leftPwmValue <= -40)
    reverse_left();
  if (rightPwmValue <= -40)
    reverse_right();

  m_hardwareInterface.analogWrite(pinMapping::LEFT_PWM_OUTPUT, leftPwmValue);
  m_hardwareInterface.analogWrite(pinMapping::RIGHT_PWM_OUTPUT, rightPwmValue);
}

// ---------- Private Methods --------------

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

PwmTargets ArgoRc::setMotorTarget(int speed, int steer) {
  PwmTargets targetPwmVals;

  if (abs(speed) < 40 && abs(steer) > 2) {
    targetPwmVals.leftPwm = steer;
    targetPwmVals.rightPwm = -steer;
  } else {
    targetPwmVals.leftPwm = speed * ((-255 - steer) / -255.0);
    targetPwmVals.rightPwm = speed * ((255 - steer) / 255.0);
  }
  return targetPwmVals;
}

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

PwmTargets ArgoRc::readPwmInput() {
  int rcPwmThrottleRaw = InterruptData::g_pinData[0].lastGoodWidth;
  int rcPwmSteeringRaw = InterruptData::g_pinData[1].lastGoodWidth;

  m_hardwareInterface.serialPrint("rc_pwm_throttle_raw: ");
  m_hardwareInterface.serialPrintln(rcPwmThrottleRaw);

  m_hardwareInterface.serialPrint("rc_pwm_steering_raw: ");
  m_hardwareInterface.serialPrintln(rcPwmSteeringRaw);

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

  m_hardwareInterface.setPinMode(pinMapping::TEST_POT_POSITIVE,
                                 digitalIO::E_OUTPUT);

  m_hardwareInterface.setPinMode(pinMapping::RC_DEADMAN, digitalIO::E_INPUT);
}

} // namespace ArgoRcLib