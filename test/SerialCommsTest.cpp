#include <string>

#include <gtest/gtest.h>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "SerialComms.hpp"
#include "Speed.hpp"
#include "mock_arduino.hpp"

using ::testing::HasSubstr;
using ::testing::InSequence;
using ::testing::InvokeWithoutArgs;
using ::testing::Matcher;
using ::testing::MatchesRegex;
using ::testing::Mock;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Test;

using namespace ArgoRcLib;
using namespace Hardware;
using namespace Libs;

namespace {

const unsigned long SERIAL_DELAY = 200; // Milliseconds

// Allows expectations to be set without handling checksums too
void checkPrintWithoutChecksum(NiceMock<MockArduino> &mockObj,
                               std::string expectedStr) {
  expectedStr.append(" chk:[0-9]{1,3}\n");

  EXPECT_CALL(mockObj, serialPrint(Matcher<const std::string &>(
                           MatchesRegex(expectedStr))));
}

unsigned long millisTime = 0;
unsigned long incrementMillis() {
  millisTime += SERIAL_DELAY;
  return millisTime;
}

class SerialCommsFixture : public ::testing::Test {
protected:
  SerialCommsFixture()
      : mockObj(), testInstance(static_cast<ArduinoInterface &>(mockObj)) {
    millisTime = 0;
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

  const std::string outString = "!e L_ENC:101 R_ENC:102";
  checkPrintWithoutChecksum(mockObj, outString);

  testInstance.addEncoderRotation(expectedData);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, writesSpeedData) {
  const Distance oneMeter = 1.0_m;
  const auto oneSecond = 1_s;
  const Speed oneMeterSecond(oneMeter, oneSecond);
  const Speed twoMeterSecond(oneMeter * 2, oneSecond);

  WheelSpeeds expectedSpeeds{oneMeterSecond, twoMeterSecond};

  const std::string outString = "!s L_SPEED:1000 R_SPEED:2000";
  checkPrintWithoutChecksum(mockObj, outString);

  testInstance.addVehicleSpeed(expectedSpeeds);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, parseSpeedCommand) {
  const std::string speedCommand = "!T L_SPEED:1200 R_SPEED:1300\n";

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

  EXPECT_EQ(targetSpeeds.leftWheel.getUnitDistance().millimeters(),
            leftExpectedSpeed.getUnitDistance().millimeters());
  EXPECT_EQ(targetSpeeds.rightWheel.getUnitDistance().millimeters(),
            rightExpectedSpeed.getUnitDistance().millimeters());
}

TEST_F(SerialCommsFixture, goodPingIsDetected) {
  const std::string pingCommand{"!P\n"};

  InSequence s;
  EXPECT_CALL(mockObj, serialAvailable())
      .Times(pingCommand.length())
      .WillRepeatedly(Return(1));
  EXPECT_CALL(mockObj, serialAvailable()).WillOnce(Return(0));

  int currentBufferPos = 0;
  ON_CALL(mockObj, serialRead())
      .WillByDefault(InvokeWithoutArgs([&currentBufferPos, &pingCommand]() {
        char nextChar = pingCommand[currentBufferPos];
        currentBufferPos++;

        return nextChar;
      }));

  // Each of these calls increment millis by the set amount so order them
  // that we would fail the test IF we did not send the ping command
  ASSERT_TRUE(testInstance.isPingGood()); // millisTime is below threshold
  testInstance.parseIncomingBuffer();     // We would now be over if ping fails
  EXPECT_TRUE(testInstance.isPingGood());
}

TEST_F(SerialCommsFixture, missedPingDetected) {
  const unsigned long pingTimeout = 500 + 1;

  ASSERT_TRUE(testInstance.isPingGood()); // millisTime is below threshold
  millisTime += pingTimeout;
  EXPECT_FALSE(testInstance.isPingGood()); // We should be over now
}

TEST_F(SerialCommsFixture, warningIsAdded) {
  const char *warningStr = "Test warning";
  std::string outString{"!w "};
  outString.append(warningStr);

  checkPrintWithoutChecksum(mockObj, outString);
  testInstance.addWarning(warningStr);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, partialReadHandled) {
  const std::string partOne("!T L_SPEED:1000 R_SPEED:2000\n!T L_SPEED:2000");
  const std::string partTwo(" R_SPEED:3000\n");

  InSequence s;
  EXPECT_CALL(mockObj, serialAvailable())
      .Times(partOne.length())
      .WillRepeatedly(Return(1));

  EXPECT_CALL(mockObj, serialAvailable()).WillOnce(Return(0));

  int partOneBufPos = 0;
  ON_CALL(mockObj, serialRead())
      .WillByDefault(InvokeWithoutArgs([&partOneBufPos, &partOne]() {
        char nextChar = partOne[partOneBufPos];
        partOneBufPos++;

        return nextChar;
      }));

  testInstance.parseIncomingBuffer();
  auto parsedSpeeds = testInstance.getTargetSpeeds();
  ASSERT_EQ(parsedSpeeds.leftWheel.getUnitDistance().millimeters(), 1000);
  ASSERT_EQ(parsedSpeeds.rightWheel.getUnitDistance().millimeters(), 2000);

  // Reset and send buffer again
  ASSERT_TRUE(Mock::VerifyAndClear(&mockObj));

  EXPECT_CALL(mockObj, serialAvailable())
      .Times(partTwo.length())
      .WillRepeatedly(Return(1));

  EXPECT_CALL(mockObj, serialAvailable()).WillOnce(Return(0));

  int partTwoBufPos = 0;
  ON_CALL(mockObj, serialRead())
      .WillByDefault(InvokeWithoutArgs([&partTwoBufPos, &partTwo]() {
        char nextChar = partTwo[partTwoBufPos];
        partTwoBufPos++;

        return nextChar;
      }));

  testInstance.parseIncomingBuffer();
  auto secondSpeeds = testInstance.getTargetSpeeds();

  EXPECT_EQ(secondSpeeds.leftWheel.getUnitDistance().millimeters(), 2000);
  EXPECT_EQ(secondSpeeds.rightWheel.getUnitDistance().millimeters(), 3000);
}

TEST_F(SerialCommsFixture, outgoingChecksumIsValid) {
  auto sumChars = [](const std::string &s) {
    uint8_t sum = 0;
    for (auto &character : s) {
      sum += (int)character;
    }
    return sum;
  };

  const Distance oneMeter = 1.0_m;
  const auto oneSecond = 1_s;
  const Speed oneMeterSecond(oneMeter, oneSecond);
  const Speed twoMeterSecond(oneMeter * 2, oneSecond);

  WheelSpeeds expectedSpeeds{oneMeterSecond, twoMeterSecond};

  std::string outString = "!s L_SPEED:1000 R_SPEED:2000 ";
  uint8_t expectedChecksum = sumChars(outString);

  std::string expectedString = "chk:" + std::to_string(expectedChecksum);

  EXPECT_CALL(mockObj, serialPrint(Matcher<const std::string &>(
                           HasSubstr(expectedString))));
  testInstance.addVehicleSpeed(expectedSpeeds);
  testInstance.sendCurrentBuffer();
}