#include <Arduino.h>
#include <stdint.h>

#include "arduino_hardware.hpp"

using namespace ArduinoEnums;

namespace Hardware {

int ArduinoHardware::analogRead(pinMapping pin) const {
  return ::analogRead(convertPinEnumToArduino(pin));
}

void ArduinoHardware::analogWrite(pinMapping pin, int value) const {
  ::analogWrite(convertPinEnumToArduino(pin), value);
}

void ArduinoHardware::delay(unsigned long milliseconds) const {
  ::delay(milliseconds);
}

digitalIO ArduinoHardware::digitalRead(pinMapping pin) const {
  int result = ::digitalRead(convertPinEnumToArduino(pin));

  switch (result) {
  case HIGH:
    return digitalIO::E_HIGH;
  case LOW:
    return digitalIO::E_LOW;
  default:
    serialPrintln("Result was unexpected in digitalRead, aborting!");
    exit(-1);
  }
}

void ArduinoHardware::digitalWrite(pinMapping pin, digitalIO mode) const {
  const auto outputPin = convertPinEnumToArduino(pin);

  switch (mode) {
  case digitalIO::E_HIGH:
    return ::digitalWrite(outputPin, HIGH);
  case digitalIO::E_LOW:
    return ::digitalWrite(outputPin, LOW);
  default:
    serialPrintln("Attempted to use a pin state as output");
    break;
  }
}

void ArduinoHardware::enterDeadmanSafetyMode() const {
  while (1) {
    // wait here forever - requires a reset
    serialPrintln(" DEADMAN SWITCH RELEASED - RESET ARDUINO! ");
    delay(500);
  }
}

void ArduinoHardware::setPinMode(pinMapping pin, digitalIO mode) const {
  const auto destPin = convertPinEnumToArduino(pin);
  switch (mode) {
  case digitalIO::E_INPUT:
    ::pinMode(destPin, INPUT);
    break;
  case digitalIO::E_INPUT_PULLUP:
    ::pinMode(destPin, INPUT_PULLUP);
    break;
  case digitalIO::E_OUTPUT:
    ::pinMode(destPin, OUTPUT);
    break;
  default:
    serialPrintln("Attempted to set a pin to a state instead of a mode");
    break;
  }
}

uint8_t ArduinoHardware::convertPinEnumToArduino(pinMapping pinToConvert) {
  /* It is not possible to assign a value to an enum class after
   * the definition. However to pull in pins A6/A7 we must include
   * the Arduino header which means we must target the Mega.
   *
   * Instead we use a switch case to "assign" values within
   * this class instead. Whilst there will be a slight performance
   * impact the compiler should optimise it to the point where there
   * won't be much of a difference compared to assigning values to the
   * enum class instead.
   */

  // GCC will warn us if we miss a case off the enum

  switch (pinToConvert) {
  case pinMapping::LEFT_FORWARD_RELAY:
    return 23;
  case pinMapping::LEFT_REVERSE_RELAY:
    return 25;

  case pinMapping::RIGHT_FORWARD_RELAY:
    return 27;
  case pinMapping::RIGHT_REVERSE_RELAY:
    return 29;

  case pinMapping::LEFT_FOOTSWITCH_RELAY:
    return 42;
  case pinMapping::RIGHT_FOOTSWITCH_RELAY:
    return 40;

  case pinMapping::LEFT_PWM_OUTPUT:
    return 44;
  case pinMapping::RIGHT_PWM_OUTPUT:
    return 46;

  case pinMapping::STEERING_PWM_OUTPUT:
    return 4;
  case pinMapping::BRAKING_PWM_OUTPUT:
    return 6;

  case pinMapping::LEFT_ENCODER_1:
    return 19;
  case pinMapping::LEFT_ENCODER_2:
    return 18;
  case pinMapping::RIGHT_ENCODER_1:
    return 20;
  case pinMapping::RIGHT_ENCODER_2:
    return 21;

  case pinMapping::TEST_POT_POSITIVE:
    return A6;
  case pinMapping::TEST_POT_WIPER:
    return A7;

  case pinMapping::RC_PWM_IN_L:
    return A10;
    // TODO A11 for RC_PWM_IN_R

  case pinMapping::RC_DEADMAN:
    return 2;
  default:
    ::Serial.println("Unknown pin in pin mapping. Aborting!");
    exit(-1);
  }
}

} // namespace Hardware
