#ifndef ARDUINO_LIB_WRAPPER_HPP
#define ARDUINO_LIB_WRAPPER_HPP

// Compatibility shim as Arduino uses String whilst 
// GCC uses std::string. This allows us to use both with the same header

#ifdef UNIT_TESTING
    #include <string>
    // Whenever we encounter a String use the processor to convert to std::string
    #define String std::string

    // Copy implementation for missing map function from
    // https://www.arduino.cc/reference/en/language/functions/math/map/
    long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
#endif

#ifndef UNIT_TESTING
    // Bring in the Arduino header which has String in it
    #include <Arduino.h>
#endif

#endif //ARDUINO_LIB_WRAPPER_HPP