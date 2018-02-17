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

struct pinMapping {
    uint8_t LEFT_FORWARD_RELAY;
    uint8_t LEFT_REVERSE_RELAY;

    uint8_t RIGHT_FORWARD_RELAY;
    uint8_t RIGHT_REVERSE_RELAY;

    uint8_t LEFT_FOOTSWITCH_RELAY;
    uint8_t RIGHT_FOOTSWITCH_RELAY;

    uint8_t LEFT_PWM_OUTPUT;
    uint8_t RIGHT_PWM_OUTPUT;

    uint8_t LEFT_ENCODER_1;
    uint8_t LEFT_ENCODER_2;

    uint8_t RIGHT_ENCODER_1;
    uint8_t RIGHT_ENCODER_2;

    uint8_t TEST_POT_POSITIVE;
    uint8_t TEST_POT_WIPER;

    uint8_t RC_PWM_IN_L;
    uint8_t RC_DEADMAN;

};

class ArduinoInterface {
    // Abstract class which defines the methods that we use on our Arduino
    // for unit testing and on a real device

public:

    virtual ~ArduinoInterface() = default;

    virtual int analogRead(pinMapping pin);
    virtual void digitalWrite(uint8_t pin, digitalIO mode);
    virtual void pinMode(uint8_t pin, digitalIO mode);
    
    virtual void serialBegin(unsigned long baudRate);
    virtual void serialPrintln(const std::string &s);

};


#endif //ARDUINO_INTERFACE_HPP_