
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

ArgoRc::ArgoRc(Argo::unique_ptr<ArduinoInterface> &&hardwareInterface,
               Argo::unique_ptr<ArgoEncoder> &&encoders)
    : m_hardwareInterface(Argo::move(hardwareInterface)),
      m_encoders(Argo::move(encoders)), m_commsObject(*m_hardwareInterface) {
  // Initialise encoders before any function attempts to use them
  setupEncoders();
}

void ArgoRc::setup() {
  m_hardwareInterface->serialBegin(115200);

  setupDigitalPins();
  footswitch_off();
  direction_relays_off();

  m_hardwareInterface->digitalWrite(pinMapping::TEST_POT_POSITIVE,
                                    digitalIO::E_HIGH);

  m_hardwareInterface->setPinMode(pinMapping::RC_DEADMAN, digitalIO::E_INPUT);
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
  m_commsObject.sendEncoderRotation(m_encoders->read());

  for (int i = 18; i < 23; i++) {
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(digitalRead(i));
  }
#ifdef RC_PWM_ENABLED

#define DEADMAN_TIMEOUT_DELAY 500

  if (digitalRead(RC_DEADMAN) == LOW) {

    unsigned long timout_deadman = millis();
    while ((millis() - timout_deadman) < DEADMAN_TIMEOUT_DELAY) {
      if (digitalRead(RC_DEADMAN) == HIGH)
        break;
    }

    if (m_hardwareInterface->digitalRead(pinMapping::RC_DEADMAN) ==
        digitalIO::E_LOW) {
      enterDeadmanFail();

      // This is for unit testing - enterDeadmanSafetyMode on hardware gets
      // stuck in an infinite loop
      return;
    }
  }

  // Deadman switch is high at this point

  int rc_pwm_left = timingData::g_pinData[0].lastGoodWidth;
  int rc_pwm_right = timingData::g_pinData[1].lastGoodWidth;

  m_hardwareInterface->serialPrint("rc_pwm_left_raw: ");
  m_hardwareInterface->serialPrintln(rc_pwm_left);

  m_hardwareInterface->serialPrint("rc_pwm_right_raw: ");
  m_hardwareInterface->serialPrintln(rc_pwm_right);

  int throttle_pwm = map(rc_pwm_left, 1520, 1850, 0, 255);
  int steering_pwm = map(rc_pwm_right, 1520, 1850, 0, 255);

  // throttle_pwm = constrainPwmInput(throttle_pwm);
  // steeringPwm = constrainPwmInput(steeringPwm);

  int left_pwm;
  int right_pwm;

  setMotorTarget(throttle_pwm, steering_pwm, left_pwm, right_pwm);
  readPwmInput(left_pwm, right_pwm);

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

void ArgoRc::setMotorTarget(int speed, int steer, int &left_pwm,
                            int &right_pwm) {
  if (abs(speed) < 40 && abs(steer) > 2) {
    left_pwm = steer;
    right_pwm = -steer;
  } else {
    left_pwm = speed * ((-255 - steer) / -255.0);
    right_pwm = speed * ((255 - steer) / 255.0);
  }
}

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

  m_hardwareInterface->setPinMode(pinMapping::STEERING_PWM_OUTPUT,
                                  digitalIO::E_OUTPUT);
  m_hardwareInterface->setPinMode(pinMapping::BRAKING_PWM_OUTPUT,
                                  digitalIO::E_OUTPUT);

  m_hardwareInterface->analogWrite(pinMapping::STEERING_PWM_OUTPUT, 0);
  m_hardwareInterface->analogWrite(pinMapping::BRAKING_PWM_OUTPUT, 128);
}

void ArgoRc::setupEncoders() {
  m_encoders->setEncoderPins(
      ArgoEncoderPositions::LEFT_ENCODER,
      Argo::pair<pinMapping, pinMapping>(pinMapping::LEFT_ENCODER_1,
                                         pinMapping::LEFT_ENCODER_2));

  m_encoders->setEncoderPins(
      ArgoEncoderPositions::RIGHT_ENCODER,
      Argo::pair<pinMapping, pinMapping>(pinMapping::RIGHT_ENCODER_1,
                                         pinMapping::RIGHT_ENCODER_2));
}

} // namespace ArgoRcLib