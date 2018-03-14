#include <gtest/gtest.h>
#include <string>

#include "Encoder.hpp"
#include "SerialComms.hpp"
#include "mock_arduino.hpp"


using ::testing::Test;

using namespace Hardware;
using namespace ArgoRcLib;

namespace {
class SerialCommsFixture : public ::testing::Test {
protected:
  SerialCommsFixture()
      : mockObj(), testInstance(static_cast<ArduinoInterface &>(mockObj)) {}

  MockArduino mockObj;
  SerialComms testInstance;
};

} // Anonymous namespace

TEST_F(SerialCommsFixture, writesEncoderData) {
  const EncoderData expectedData = {101, 102};
  const std::string expectedString = "!D L_ENC_1:101 R_ENC_1:102 ";

  EXPECT_CALL(mockObj, serialPrintln(expectedString));
  testInstance.sendEncoderRotation(expectedData);
}