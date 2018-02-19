#ifndef ARDUINO_HARDWARE_HPP_
#define ARDUINO_HARDWARE_HPP_

#include <stdint.h>

#include "arduino_interface.hpp"

class ArduinoHardware : ArduinoInterface {

public:
    ArduinoHardware() = default;
    virtual ~ArduinoHardware() override = default;

private:

    ArduinoEnums::pinMapping m_pinMapping;

    uint8_t convertPinEnumToArduino(ArduinoEnums::pinMapping pinToConvert);

};


#endif //ARDUINO_HARDWARE_HPP_