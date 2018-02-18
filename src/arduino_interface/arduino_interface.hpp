#ifndef ARDUINO_INTERFACE_HPP_
#define ARDUINO_INTERFACE_HPP_

#include <stdint.h>
#include <string>

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

class ArduinoInterface {
    // Abstract class which defines the methods that we use on our Arduino
    // for unit testing and on a real device

public:

    virtual ~ArduinoInterface() = default;

    virtual int analogRead(pinMapping pin);
    virtual void digitalWrite(pinMapping pin, digitalIO mode);
    virtual void pinMode(pinMapping pin, digitalIO mode);
    
    virtual void serialBegin(unsigned long baudRate);
    virtual void serialPrintln(const std::string &s);

};


#endif //ARDUINO_INTERFACE_HPP_