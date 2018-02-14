#ifndef ARDUINO_INTERFACE_HPP_
#define ARDUINO_INTERFACE_HPP_

#include <stdint.h>

enum class digitalIO{
    INPUT_PULLUP,
    OUTPUT,
    HIGH,
    LOW
};

class ArduinoInterface {
    // Abstract class which defines the methods that we use on our Arduino
    // for unit testing and on a real device

    virtual ~ArduinoInterface() = default;
    virtual void pinMode(uint8_t pin, digitalIO mode);
    virtual void digitalWrite(uint8_t pin, digitalIO mode);
    
};


#endif //ARDUINO_INTERFACE_HPP_