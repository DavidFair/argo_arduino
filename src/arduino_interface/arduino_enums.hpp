#ifndef ARDUINO_ENUM_HPP_
#define ARDUINO_ENUM_HPP_

#include <stdint.h>

enum class digitalIO{
    INPUT_PULLUP,
    OUTPUT,
    HIGH,
    LOW
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
    DDRK, // DDRK controls ADC8-ADC15 direction
    PCICR, // Controls interrupt enable pins
    PCMSK2 // Bitwise controls PCINT 23 - 16 to trigger interupts

}

enum class portControlValues : uint8_t {
    // PCIE2 sets bit 2 to 1 for the PCMSK2 register
    PCIE2 = 0x04 // Enables PCINT 23-16 to serice interrupts
}

#endif //ARDUINO_ENUM_HPP_