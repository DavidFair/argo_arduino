#ifndef ARDUINO_ENUM_HPP_
#define ARDUINO_ENUM_HPP_

#include <stdint.h>

namespace ArduinoEnums{
    /* Wherever the Arduino header pollutes the global namespace with
     * these defines we prefix them with "E_"
     */

enum class digitalIO{
    E_INPUT_PULLUP,
    E_OUTPUT,

    E_HIGH,
    E_LOW
};

enum class pinMapping {
    LEFT_FORWARD_RELAY,
    LEFT_REVERSE_RELAY,

    RIGHT_FORWARD_RELAY,
    RIGHT_REVERSE_RELAY,

    LEFT_FOOTSWITCH_RELAY,
    RIGHT_FOOTSWITCH_RELAY,

    LEFT_PWM_OUTPUT,
    RIGHT_PWM_OUTPUT,

    LEFT_ENCODER_1,
    LEFT_ENCODER_2,

    RIGHT_ENCODER_1,
    RIGHT_ENCODER_2,

    TEST_POT_POSITIVE,
    TEST_POT_WIPER,

    RC_PWM_IN_L,
    RC_DEADMAN
};

enum class portMapping {
    E_DDRK, // DDRK controls ADC8-ADC15 direction
    E_PCICR, // Controls interrupt enable pins
    E_PCMSK2, // Bitwise controls PCINT 23 - 16 to trigger interupts
    E_PINK // PIN K which is A8-A15 on the Mega 2560
};

enum class portControlValues : uint8_t {
    // PCIE2 sets enables the PCMSK2 register which allows us to use PCINT 23-16 for ISR
    E_PCIE2 = 0x04
};

} // End of namespace

#endif //ARDUINO_ENUM_HPP_