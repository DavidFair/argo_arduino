#ifndef ARDUINO_INTERFACE_HPP_
#define ARDUINO_INTERFACE_HPP_

#include <stdint.h>

#include "arduino_enums.hpp"
#include "arduino_string.hpp"

// Forward declarations
class ArduinoInterface;

// Typedef the ISR function pointer in the header for easy caller access
using isrFuncPtr = void (*)(ArduinoInterface &hardwareInterface);

// --- RC PWM input ---------------------------------
struct tPinTimingData{
  uint8_t edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int  lastGoodWidth;
};


class ArduinoInterface {
    // Abstract class which defines the methods that we use on our Arduino
    // for unit testing and on a real device

public:
    ArduinoInterface() = default;
    virtual ~ArduinoInterface() = default;

    virtual int analogRead(ArduinoEnums::pinMapping pin) = 0;

    virtual void analogWrite(ArduinoEnums::pinMapping pin, int value) = 0;

    virtual void createIsr(ArduinoEnums::portMapping isrTarget, isrFuncPtr isrFunction) = 0;

    virtual void delay(unsigned long milliseconds) = 0;
    
    virtual ArduinoEnums::digitalIO digitalRead(ArduinoEnums::pinMapping pin) = 0;

    virtual void digitalWrite(ArduinoEnums::pinMapping pin, ArduinoEnums::digitalIO mode) = 0;

    virtual unsigned long micros() = 0;

    virtual void orPortBitmask(ArduinoEnums::portMapping port, uint8_t bitmask);
    
    virtual void pinMode(ArduinoEnums::pinMapping pin, ArduinoEnums::digitalIO mode) = 0;

    virtual uint8_t readPortBits(ArduinoEnums::portMapping port) = 0;
    
    virtual void serialBegin(unsigned long baudRate) = 0;
    
    virtual void setPortBitmask(ArduinoEnums::portMapping port, uint8_t bitmask);

    virtual void serialPrint(const String &s) = 0;
    
    virtual void serialPrint(int i) = 0;
    
    virtual void serialPrintln(const String &s) = 0;
    
    virtual void serialPrintln(int i) = 0;


    volatile tPinTimingData m_pinData[6 + 1];
    volatile uint8_t m_PCintLast;

};



#endif //ARDUINO_INTERFACE_HPP_