#ifndef ARDUINO_HARDWARE_HPP_
#define ARDUINO_HARDWARE_HPP_

#include <stdint.h>

#include "arduino_interface.hpp"



class ArduinoHardware : ArduinoInterface {

public:
    ArduinoHardware();

private:

    pinMapping m_pinMapping;

    uint8_t assignPinMapping(pinMapping pinToConvert);

};


#endif //ARDUINO_HARDWARE_HPP_