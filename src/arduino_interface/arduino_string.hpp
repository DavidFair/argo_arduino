#ifndef ARDUINO_STRING_HPP_
#define ARDUINO_STRING_HPP_

// Compatibility shim as Arduino uses String whilst 
// GCC uses std::string. This allows us to use both with the same header

#ifdef UNIT_TESTING
    #include <string>
    // Whenever we encounter a String use the processor to convert to std::string
    #define String std::string
#endif

#ifndef UNIT_TESTING
    // Bring in the Arduino header which has String in it
    #include <Arduino.h>
#endif

#endif // ARDUINO_STRING_HPP_