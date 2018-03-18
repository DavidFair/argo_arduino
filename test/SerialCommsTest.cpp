#include <gtest/gtest.h>
#include <string>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "SerialComms.hpp"
#include "Speed.hpp"
#include "mock_arduino.hpp"

using ::testing::Test;

using namespace ArgoRcLib;
using namespace Hardware;
using namespace Libs;

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
  EncoderPulses expectedData;
  expectedData.leftEncoderVal = 101;
  expectedData.rightEncoderVal = 102;

  const std::string expectedString = "!D L_ENC:101 R_ENC:102 \n";

  EXPECT_CALL(mockObj, serialPrintln(expectedString));
  testInstance.addEncoderRotation(expectedData);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, writesSpeedData) {
  const Distance oneMeter = 1.0_m;
  const auto oneSecond = 1_s;
  const Speed oneMeterSecond(oneMeter, oneSecond);
  const Speed twoMeterSecond(oneMeter * 2, oneSecond);

  WheelSpeeds expectedSpeeds{oneMeterSecond, twoMeterSecond};

  const std::string expectedString = "!D L_SPEED:1000 R_SPEED:2000 \n";

  EXPECT_CALL(mockObj, serialPrintln(expectedString));
  testInstance.addVehicleSpeed(expectedSpeeds);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, parseSpeedCommand) {
  const std::string inputString = "!C L_SPEED:1200, R_SPEED:1200 \n";

  // EXPECT_CALL()
}