#ifndef ARDUINO_HARDWARE_HPP_
#define ARDUINO_HARDWARE_HPP_

#include <stdint.h>
#include <unordered_map>

#include "arduino_interface.hpp"



class ArduinoHardware : ArduinoInterface {

public:
    ArduinoHardware();

private:
    std::unordered_map<pinMapping, uint8_t> hardwarePinning{};

    void assignPinMapping();

};


#endif //ARDUINO_HARDWARE_HPP_