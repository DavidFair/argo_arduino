#ifndef ARDUINO_INTERFACE_HPP_
#define ARDUINO_INTERFACE_HPP_

#include <stdint.h>

#include "arduino_enums.hpp"
#include "arduino_string.hpp"



class ArduinoInterface {
    // Abstract class which defines the methods that we use on our Arduino
    // for unit testing and on a real device

public:
    ArduinoInterface() = default;
    virtual ~ArduinoInterface() = default;

    virtual int analogRead(ArduinoEnums::pinMapping pin) const = 0;

    virtual void analogWrite(ArduinoEnums::pinMapping pin, int value) const = 0;

    virtual void delay(unsigned long milliseconds) const = 0;
    
    virtual ArduinoEnums::digitalIO digitalRead(ArduinoEnums::pinMapping pin) const = 0;

    virtual void digitalWrite(ArduinoEnums::pinMapping pin, ArduinoEnums::digitalIO mode) const = 0;

    virtual unsigned long micros() const = 0;

    virtual void orPortBitmask(ArduinoEnums::portMapping port, uint8_t bitmask) const = 0;

    virtual uint8_t readPortBits(ArduinoEnums::portMapping port) const = 0;
    
    virtual void serialBegin(unsigned long baudRate) const = 0;
    
    virtual void setPortBitmask(ArduinoEnums::portMapping port, uint8_t bitmask) const = 0;

    virtual void serialPrint(const String &s) const = 0;
    
    virtual void serialPrint(int i) const = 0;
    
    virtual void serialPrintln(const String &s) const = 0;
    
    virtual void serialPrintln(int i) const = 0;

    virtual void setPinMode(ArduinoEnums::pinMapping pin, ArduinoEnums::digitalIO mode) const = 0;
};



#endif //ARDUINO_INTERFACE_HPP_