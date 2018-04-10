#include <gtest/gtest.h>
#include <string>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "SerialComms.hpp"
#include "Speed.hpp"
#include "mock_arduino.hpp"

using ::testing::InSequence;
using ::testing::InvokeWithoutArgs;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Test;

using namespace ArgoRcLib;
using namespace Hardware;
using namespace Libs;

namespace {

const unsigned long SERIAL_DELAY = 500;

unsigned long time = 0;
unsigned long incrementMillis() {
  time += SERIAL_DELAY;
  return time;
}

class SerialCommsFixture : public ::testing::Test {
protected:
  SerialCommsFixture()
      : mockObj(), testInstance(static_cast<ArduinoInterface &>(mockObj)) {
    time = 0;
    ON_CALL(mockObj, millis())
        .WillByDefault(InvokeWithoutArgs(&incrementMillis));
  }

  NiceMock<MockArduino> mockObj;
  SerialComms testInstance;
};

} // Anonymous namespace

TEST_F(SerialCommsFixture, writesEncoderData) {
  EncoderPulses expectedData;
  expectedData.leftEncoderVal = 101;
  expectedData.rightEncoderVal = 102;

  const std::string expectedString = "!e L_ENC:101 R_ENC:102 \n";

  EXPECT_CALL(mockObj, serialPrint(expectedString));
  testInstance.addEncoderRotation(expectedData);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, writesSpeedData) {
  const Distance oneMeter = 1.0_m;
  const auto oneSecond = 1_s;
  const Speed oneMeterSecond(oneMeter, oneSecond);
  const Speed twoMeterSecond(oneMeter * 2, oneSecond);

  WheelSpeeds expectedSpeeds{oneMeterSecond, twoMeterSecond};

  const std::string expectedString = "!s L_SPEED:1000 R_SPEED:2000 \n";

  EXPECT_CALL(mockObj, serialPrint(expectedString));
  testInstance.addVehicleSpeed(expectedSpeeds);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, parseSpeedCommand) {
  const std::string speedCommand = "!T L_SPEED:1200 R_SPEED:1300 \n";

  InSequence s;
  EXPECT_CALL(mockObj, serialAvailable())
      .Times(speedCommand.length())
      .WillRepeatedly(Return(1));
  EXPECT_CALL(mockObj, serialAvailable()).WillOnce(Return(0));

  int currentBufferPos = 0;
  ON_CALL(mockObj, serialRead())
      .WillByDefault(InvokeWithoutArgs([&currentBufferPos, &speedCommand]() {
        char nextChar = speedCommand[currentBufferPos];
        currentBufferPos++;
        return nextChar;
      }));

  testInstance.parseIncomingBuffer();
  auto targetSpeeds = testInstance.getTargetSpeeds();

  Speed leftExpectedSpeed(1.2_m, 1_s);
  Speed rightExpectedSpeed(1.3_m, 1_s);

  EXPECT_EQ(targetSpeeds.leftWheel, leftExpectedSpeed);
  EXPECT_EQ(targetSpeeds.rightWheel, rightExpectedSpeed);
}