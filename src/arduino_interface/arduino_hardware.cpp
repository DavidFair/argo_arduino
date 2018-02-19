#include <Arduino.h>

#include "arduino_hardware.hpp"

using namespace ArduinoEnums;


uint8_t ArduinoHardware::convertPinEnumToArduino(pinMapping pinToConvert){
    /* It is not possible to assign a value to an enum class after
     * the definition. However to pull in pins A6/A7 we must include
     * the Arduino header which means we must target the Mega.
     *
     * Instead we use a switch case to "assign" values within
     * this class instead. Whilst there will be a slight performance
     * impact the compiler should optimise it to the point where there
     * won't be much of a difference compared to assigning values to the
     * enum class instead.
     */ 
    
    // GCC will warn us if we miss a case off the enum
    
    switch(pinToConvert){
        case pinMapping::LEFT_FORWARD_RELAY : return 23;
        case pinMapping::LEFT_REVERSE_RELAY : return 25;
        
        case pinMapping::RIGHT_FORWARD_RELAY : return 27;
        case pinMapping::RIGHT_REVERSE_RELAY : return 29;

        case pinMapping::LEFT_FOOTSWITCH_RELAY : return 42;
        case pinMapping::RIGHT_FOOTSWITCH_RELAY : return 40;

        case pinMapping::LEFT_PWM_OUTPUT : return 44;
        case pinMapping::RIGHT_PWM_OUTPUT : return 46;

        case pinMapping::LEFT_ENCODER_1: return 19;
        case pinMapping::LEFT_ENCODER_2: return 18;
        case pinMapping::RIGHT_ENCODER_1: return 20;
        case pinMapping::RIGHT_ENCODER_2: return 21;

        case pinMapping::TEST_POT_POSITIVE: return A6;
        case pinMapping::TEST_POT_WIPER: return A7;

        case pinMapping::RC_PWM_IN_L: return A11;
        case pinMapping::RC_DEADMAN: return 2;
    }
}


