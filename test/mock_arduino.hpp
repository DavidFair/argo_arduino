#ifndef MOCK_ARDUINO_HPP_
#define MOCK_ARDUINO_HPP_

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stdint.h>

#include "arduino_enums.hpp"
#include "arduino_interface.hpp"
#include "arduino_lib_wrapper.hpp"

class MockArduino : public Hardware::ArduinoInterface {
public:
  MOCK_CONST_METHOD1(analogRead, int(ArduinoEnums::pinMapping));

  MOCK_CONST_METHOD2(analogWrite, void(ArduinoEnums::pinMapping, int));

  MOCK_CONST_METHOD1(delay, void(unsigned long));

  MOCK_CONST_METHOD1(digitalRead,
                     ArduinoEnums::digitalIO(ArduinoEnums::pinMapping));

  MOCK_CONST_METHOD2(digitalWrite,
                     void(ArduinoEnums::pinMapping, ArduinoEnums::digitalIO));

  MOCK_CONST_METHOD0(micros, unsigned long());

  MOCK_CONST_METHOD2(orPortBitmask, void(ArduinoEnums::portMapping, uint8_t));

  MOCK_CONST_METHOD1(readPortBits, uint8_t(ArduinoEnums::portMapping));

  MOCK_CONST_METHOD1(serialBegin, void(unsigned long));

  MOCK_CONST_METHOD2(setPortBitmask,
                     void(ArduinoEnums::portMapping, uint8_t bitmask));

  MOCK_CONST_METHOD1(serialPrint, void(const std::string &));

  MOCK_CONST_METHOD1(serialPrint, void(int));

  MOCK_CONST_METHOD1(serialPrintln, void(const std::string &));

  MOCK_CONST_METHOD1(serialPrintln, void(int));

  MOCK_CONST_METHOD2(setPinMode,
                     void(ArduinoEnums::pinMapping, ArduinoEnums::digitalIO));
};

#endif // MOCK_ARDUINO_HPP_