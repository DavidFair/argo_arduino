#ifndef ARDUINO_INTERFACE_HPP_
#define ARDUINO_INTERFACE_HPP_

#include <stdint.h>
#include <string>

#include "arduino_enums.hpp"

class ArduinoInterface {
    // Abstract class which defines the methods that we use on our Arduino
    // for unit testing and on a real device

public:

    virtual ~ArduinoInterface() = default;

    virtual int analogRead(pinMapping pin) = 0;

    virtual void analogWrite(pinMapping pin, int value) = 0;

    virtual void delay(unsigned long milliseconds) = 0;
    
    virtual digitalIO digitalRead(pinMapping pin) = 0;

    virtual void digitalWrite(pinMapping pin, digitalIO mode) = 0;

    virtual void pinMode(pinMapping pin, digitalIO mode) = 0;
    
    virtual void serialBegin(unsigned long baudRate) = 0;
    
    virtual void serialPrint(const std::string &s) = 0;
    virtual void serialPrint(int i) = 0;
    
    virtual void serialPrintln(const std::string &s) = 0;
    virtual void serialPrintln(int i) = 0;

    virtual void setPortBitmask(portMapping port, uint8_t bitmask);
    virtual void orPortBitmask(portMapping port, uint8_t bitmask);


};


#endif //ARDUINO_INTERFACE_HPP_