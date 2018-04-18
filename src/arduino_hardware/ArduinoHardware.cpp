#include <Arduino.h>
#include <stdint.h>

#include "ArduinoHardware.hpp"

using namespace ArduinoEnums;

namespace Hardware {

/**
 * Called when the robot hits an unrecoverable position such as
 * falling out of a switch statement
 * Directly toggles the left and right footswitches to the off
 * position, emits a fatal error then enters an infinite loop
 * by calling exit
 */
void ArduinoHardware::robotSafetyAbort() {
  /* This should only ever be called if there was a serious bug. We don't call
   * enterDeadmanSafety mode as the program is no longer in a known sane state.
   *
   * On the AVR-GCC creates an infinite loop in assembly when we call exit(-1)
   */

  // The compiler should inline this as both calls are static
  auto leftRelayPin = ArduinoHardware::convertPinEnumToArduino(
      pinMapping::LEFT_FOOTSWITCH_RELAY);
  auto rightRelayPin = ArduinoHardware::convertPinEnumToArduino(
      pinMapping::RIGHT_FOOTSWITCH_RELAY);

  // Remember as we are connected to a relay E_HIGH = relay off
  // call direct to Arduino lib for best chances
  ::digitalWrite(leftRelayPin, HIGH);
  ::digitalWrite(rightRelayPin, HIGH);

  ::Serial.println("!f Aborting program. Robot should be in a safe state now.");
  exit(-1);
} // namespace Hardware

/**
 * Reads an int from the given analog pin
 *
 * @param pin The pin to read from
 * @return The int value read
 */
int ArduinoHardware::analogRead(pinMapping pin) const {
  return ::analogRead(convertPinEnumToArduino(pin));
}

/**
 * Writes an int to the given analog pin
 *
 * @param pin The pin to write to
 * @param value The integer value to write
 */
void ArduinoHardware::analogWrite(pinMapping pin, int value) const {
  ::analogWrite(convertPinEnumToArduino(pin), value);
}

/**
 * Reads a digital state from the given pin
 *
 * @param pin The pin to read a state from
 * @return digitalIO value representing the pin state
 */
digitalIO ArduinoHardware::digitalRead(pinMapping pin) const {
  int result = ::digitalRead(convertPinEnumToArduino(pin));

  switch (result) {
  case HIGH:
    return digitalIO::E_HIGH;
  case LOW:
    return digitalIO::E_LOW;
  default:
    serialPrintln("!f Result was unexpected in digitalRead");
    ArduinoHardware::robotSafetyAbort();
  }
}

/**
 * Writes a given state to a digital pin
 *
 * @param pin The pin to write the state to
 * @param mode The digitaIO value to write to the pin
 */
void ArduinoHardware::digitalWrite(pinMapping pin, digitalIO mode) const {
  const auto outputPin = convertPinEnumToArduino(pin);

  switch (mode) {
  case digitalIO::E_HIGH:
    return ::digitalWrite(outputPin, HIGH);
  case digitalIO::E_LOW:
    return ::digitalWrite(outputPin, LOW);
  default:
    serialPrintln("!f Attempted to use a pin state as output");
    ArduinoHardware::robotSafetyAbort();
  }
}

/*
 * Sets the specified pin to the given digitalIO mode
 *
 * @param pin The pin to set the mode on
 * @param mode The digitalIO mode to set the pin to
 */
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
    serialPrintln("!f Attempted to set a pin to a state instead of a mode");
    ArduinoHardware::robotSafetyAbort();
  }
}

/**
 * Converts a pinMapping value to the actual pin used on the Argo
 *
 * @param pinToConvert The pinMapping value of the requested pin
 * @return An unsigned int representing the pin number on the Arduino
 */
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

  case pinMapping::LEFT_ENCODER:
    return 19;
  case pinMapping::RIGHT_ENCODER:
    return 20;

  case pinMapping::RC_DEADMAN:
    return 2;
  default:
    ::Serial.println("!f Unknown pin in pin mapping");
    ArduinoHardware::robotSafetyAbort();
  }
}

} // namespace Hardware
