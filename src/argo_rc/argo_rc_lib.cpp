
#include <stdint.h>

//#include <Encoder.h>

#include "arduino_enums.hpp"
#include "arduino_interface.hpp"
#include "arduino_lib_wrapper.hpp"
#include "pinTimingData.hpp"

#include "argo_rc_lib.hpp"

//#define TEST_POT_ENABLED
#define RC_PWM_ENABLED

// Encoder left_encoder(pinMapping.LEFT_ENCODER_1,pinMapping.LEFT_ENCODER_2);
// Encoder
// right_encoder(pinMapping::RIGHT_ENCODER_1,pinMapping::RIGHT_ENCODER_2);

using namespace ArduinoEnums;
using namespace EncoderLib;
using namespace Hardware;

namespace ArgoRcLib {

ArgoRc::ArgoRc(ArduinoInterface *hardwareInterface)
    : m_hardwareInterface(hardwareInterface),

      m_leftEncoder(EncoderInterface::createEncoder(
          pinMapping::LEFT_ENCODER_1, pinMapping::LEFT_ENCODER_2)),

      m_rightEncoder(EncoderInterface::createEncoder(
          pinMapping::RIGHT_ENCODER_1, pinMapping::RIGHT_ENCODER_2)) {}

void ArgoRc::setup() {
  m_hardwareInterface->serialBegin(115200);

  setupDigitalPins();

  footswitch_off();
  direction_relays_off();

  m_hardwareInterface->digitalWrite(pinMapping::TEST_POT_POSITIVE,
                                    digitalIO::E_HIGH);

#ifdef RC_PWM_ENABLED
  m_hardwareInterface->setPinMode(pinMapping::RC_DEADMAN, digitalIO::E_INPUT);
  setup_rc();
#endif
}

void ArgoRc::direction_relays_off() {

  m_hardwareInterface->serialPrintln("RELAYS OFF");
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                    digitalIO::E_HIGH);
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                    digitalIO::E_HIGH);
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                    digitalIO::E_HIGH);
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                    digitalIO::E_HIGH);
}

void ArgoRc::forward_left() {
  footswitch_on();
  m_hardwareInterface->serialPrintln("forward_left");
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                    digitalIO::E_LOW);
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                    digitalIO::E_HIGH);
}

void ArgoRc::forward_right() {
  footswitch_on();
  m_hardwareInterface->serialPrintln("                  forward_right");
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                    digitalIO::E_LOW);
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                    digitalIO::E_HIGH);
}

void ArgoRc::reverse_left() {
  footswitch_on();
  m_hardwareInterface->serialPrintln(
      "                                    reverse_left");
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,
                                    digitalIO::E_HIGH);
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_REVERSE_RELAY,
                                    digitalIO::E_LOW);
}

void ArgoRc::reverse_right() {
  footswitch_on();
  m_hardwareInterface->serialPrintln(
      "                                                      reverse_right");
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,
                                    digitalIO::E_HIGH);
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,
                                    digitalIO::E_LOW);
}

void ArgoRc::footswitch_on() {
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                    digitalIO::E_LOW);
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                    digitalIO::E_LOW);
}

void ArgoRc::footswitch_off() {
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                    digitalIO::E_HIGH);
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                    digitalIO::E_HIGH);
}

//#define DEBUG_OUTPUT
#define DEBUG_OUTPUT_PWM

void ArgoRc::loop() {
  readEncoderOutput();

#ifdef RC_PWM_ENABLED

  if (m_hardwareInterface->digitalRead(pinMapping::RC_DEADMAN) ==
      digitalIO::E_LOW) {
    enterDeadmanFail();

    // This is for unit testing - enterDeadmanSafetyMode on hardware gets
    // stuck in an infinite loop
    return;
  }

  // Deadman switch is high at this point

  int rc_pwm_left = timingData::g_pinData[0].lastGoodWidth;
  int rc_pwm_right = timingData::g_pinData[1].lastGoodWidth;

  int left_pwm = map(rc_pwm_left, 1520, 1850, 0, 255);
  int right_pwm = map(rc_pwm_right, 1520, 1850, 0, 255);

  readPwmInput(left_pwm, right_pwm);
  left_pwm = constrainPwmInput(left_pwm);
  right_pwm = constrainPwmInput(right_pwm);

#endif

#ifdef TEST_POT_ENABLED
  left_pwm = map(test_pot_value, 0, 1023, 0, 255);
  right_pwm = map(test_pot_value, 0, 1023, 0, 255);
#endif

#ifdef DEBUG_OUTPUT_PWM
  m_hardwareInterface->serialPrint("ENABLED  ");
  m_hardwareInterface->serialPrint("LEFT PWM: ");
  m_hardwareInterface->serialPrint(left_pwm);
  m_hardwareInterface->serialPrint("  RIGHT PWM: ");
  m_hardwareInterface->serialPrintln(right_pwm);
#endif

  m_hardwareInterface->analogWrite(pinMapping::LEFT_PWM_OUTPUT, left_pwm);
  m_hardwareInterface->analogWrite(pinMapping::RIGHT_PWM_OUTPUT, right_pwm);

#ifdef DEBUG_OUTPUT
  m_hardwareInterface->serialPrint("  LPOS: ");
  m_hardwareInterface->serialPrint(left_newPosition);
  m_hardwareInterface->serialPrint("  RPOS: ");
  m_hardwareInterface->serialPrintln(right_newPosition);
#endif
}

// ---------- Private Methods --------------

int ArgoRc::constrainPwmInput(int initialValue) {
  int returnValue = initialValue;
  if (returnValue < 0)
    returnValue = -returnValue;

  if (returnValue > 255)
    returnValue = 255;

  return returnValue;
}

void ArgoRc::enterDeadmanFail() {
  timingData::g_pinData[0].lastGoodWidth = 0;
  timingData::g_pinData[1].lastGoodWidth = 0;
  m_hardwareInterface->analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0);
  m_hardwareInterface->analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0);

  // turn off all direction relays and footswitch
  footswitch_off();
  direction_relays_off();
  m_hardwareInterface->enterDeadmanSafetyMode();
}

void ArgoRc::readEncoderOutput() {
  m_lastEncoderVal.leftEncoderVal = m_leftEncoder->read();
  m_lastEncoderVal.rightEncoderVal = m_rightEncoder->read();
}

void ArgoRc::readPwmInput(const int leftPwmValue, const int rightPwmValue) {

  if ((leftPwmValue > -40 && leftPwmValue < 40) &&
      (rightPwmValue > -40 && rightPwmValue < 40))
    footswitch_off();

  if (leftPwmValue >= 40)
    forward_left();
  if (rightPwmValue >= 40)
    forward_right();
  if (leftPwmValue <= -40)
    reverse_left();
  if (rightPwmValue <= -40)
    reverse_right();
}

void ArgoRc::setupDigitalPins() {
  m_hardwareInterface->setPinMode(pinMapping::LEFT_FORWARD_RELAY,
                                  digitalIO::E_OUTPUT);
  m_hardwareInterface->setPinMode(pinMapping::LEFT_REVERSE_RELAY,
                                  digitalIO::E_OUTPUT);

  m_hardwareInterface->setPinMode(pinMapping::RIGHT_FORWARD_RELAY,
                                  digitalIO::E_OUTPUT);
  m_hardwareInterface->setPinMode(pinMapping::RIGHT_REVERSE_RELAY,
                                  digitalIO::E_OUTPUT);

  m_hardwareInterface->setPinMode(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                  digitalIO::E_OUTPUT);
  m_hardwareInterface->setPinMode(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                  digitalIO::E_OUTPUT);

  m_hardwareInterface->setPinMode(pinMapping::LEFT_ENCODER_1,
                                  digitalIO::E_INPUT_PULLUP);
  m_hardwareInterface->setPinMode(pinMapping::LEFT_ENCODER_2,
                                  digitalIO::E_INPUT_PULLUP);

  m_hardwareInterface->setPinMode(pinMapping::RIGHT_ENCODER_1,
                                  digitalIO::E_INPUT_PULLUP);
  m_hardwareInterface->setPinMode(pinMapping::RIGHT_ENCODER_2,
                                  digitalIO::E_INPUT_PULLUP);

  m_hardwareInterface->setPinMode(pinMapping::TEST_POT_POSITIVE,
                                  digitalIO::E_OUTPUT);
}

void ArgoRc::setup_rc() {
  // Set ADC8 - ADC15 to input (0) using the port register
  m_hardwareInterface->setPortBitmask(portMapping::E_DDRK, 0);

  // enable PCINT which allows us to use the PCMSK register by bitshifting
  // 1 into the correct register location
  const auto pcie2EnableBitmask =
      1 << static_cast<uint8_t>(portControlValues::E_PCIE2);
  m_hardwareInterface->orPortBitmask(portMapping::E_PCICR, pcie2EnableBitmask);

  // Set pins 18 to 23 as ISR triggers
  m_hardwareInterface->setPortBitmask(portMapping::E_PCMSK2, 0xFC);

  // The ISR is defined in the hardware implementation
}

} // namespace ArgoRcLib