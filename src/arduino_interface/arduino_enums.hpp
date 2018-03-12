#ifndef ARDUINO_ENUM_HPP_
#define ARDUINO_ENUM_HPP_

#include <stdint.h>

namespace ArduinoEnums {
/* Wherever the Arduino header pollutes the global namespace with
 * these defines we prefix them with "E_"
 */

enum digitalIO {
  E_INPUT,
  E_INPUT_PULLUP,
  E_OUTPUT,

  E_HIGH,
  E_LOW
};

enum Direction { FORWARD = 1, REVERSE = -1 };

enum pinMapping {
  // Digital pins
  LEFT_FORWARD_RELAY,
  LEFT_REVERSE_RELAY,

  RIGHT_FORWARD_RELAY,
  RIGHT_REVERSE_RELAY,

  LEFT_FOOTSWITCH_RELAY,
  RIGHT_FOOTSWITCH_RELAY,

  LEFT_ENCODER_1,
  LEFT_ENCODER_2,

  RIGHT_ENCODER_1,
  RIGHT_ENCODER_2,

  TEST_POT_POSITIVE,

  RC_DEADMAN,

  // Analogue pins
  LEFT_PWM_OUTPUT,
  RIGHT_PWM_OUTPUT,

  STEERING_PWM_OUTPUT,
  BRAKING_PWM_OUTPUT,

  RC_PWM_IN_L,
  TEST_POT_WIPER
};

static const pinMapping allDigitalPins[] = {
    pinMapping::LEFT_FORWARD_RELAY,    pinMapping::LEFT_REVERSE_RELAY,
    pinMapping::RIGHT_FORWARD_RELAY,   pinMapping::RIGHT_REVERSE_RELAY,
    pinMapping::LEFT_FOOTSWITCH_RELAY, pinMapping::RIGHT_FOOTSWITCH_RELAY,
    pinMapping::LEFT_ENCODER_1,        pinMapping::LEFT_ENCODER_2,
    pinMapping::RIGHT_ENCODER_1,       pinMapping::RIGHT_ENCODER_2,
    pinMapping::TEST_POT_POSITIVE,     pinMapping::RC_DEADMAN};

} // namespace ArduinoEnums

#endif // ARDUINO_ENUM_HPP_