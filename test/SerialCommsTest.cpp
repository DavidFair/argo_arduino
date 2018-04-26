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

uint8_t calculateChecksum(const std::string &s) {
  uint8_t sum = 0;
  for (auto character : s) {
    sum += (int)character;
  }
  return sum;
}

std::string appendChecksumToString(std::string inputString) {
  auto checksumVal = calculateChecksum(inputString);
  inputString.append(" chk:").append(std::to_string(checksumVal)).append("\n");
  return inputString;
}

// Allows expectations to be set without handling checksums too
void checkPrintWithoutChecksum(NiceMock<MockArduino> &mockObj,
                               std::string expectedStr) {
  expectedStr.append(" chk:[0-9]{1,3}\n");

  EXPECT_CALL(mockObj, serialPrint(Matcher<const std::string &>(
                           MatchesRegex(expectedStr))));
}

// Sets up a serial read, the caller is responsible for tracking the
// current position of the given string
void setupSerialRead(MockArduino &mockObj, int &bufferPos,
                     const std::string &incomingString) {

  InSequence s;
  EXPECT_CALL(mockObj, serialAvailable())
      .Times(incomingString.length())
      .WillRepeatedly(Return(1));

  EXPECT_CALL(mockObj, serialAvailable()).WillOnce(Return(0));

  ON_CALL(mockObj, serialRead())
      .WillByDefault(InvokeWithoutArgs([&bufferPos, &incomingString]() {
        char nextChar = incomingString[bufferPos];
        bufferPos++;

        return nextChar;
      }));
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
  auto speedCommand = appendChecksumToString("!T L_SPEED:1200 R_SPEED:1300");

  int bufferPos = 0;
  setupSerialRead(mockObj, bufferPos, speedCommand);

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

  int bufferPos = 0;
  setupSerialRead(mockObj, bufferPos, pingCommand);

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
  outString.append(warningStr).append("\n");

  EXPECT_CALL(mockObj, serialPrint(outString));
  testInstance.addWarning(warningStr);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, partialReadHandled) {
  const std::string fullCommandOne =
      appendChecksumToString("!T L_SPEED:1000 R_SPEED:2000");
  const std::string fullCommandTwo =
      appendChecksumToString("!T L_SPEED:2000 R_SPEED:3000");

  int divide = fullCommandTwo.length() / 2;
  auto lastIter = fullCommandTwo.cbegin() + divide;

  // Split the second command in two
  const std::string partCommandOne(fullCommandTwo.cbegin(), lastIter);
  const std::string partCommandTwo{lastIter, fullCommandTwo.cend()};

  const std::string partOne(fullCommandOne + partCommandOne);
  const std::string partTwo(partCommandTwo);

  int partOnePos = 0;
  setupSerialRead(mockObj, partOnePos, partOne);

  testInstance.parseIncomingBuffer();
  auto parsedSpeeds = testInstance.getTargetSpeeds();
  ASSERT_EQ(parsedSpeeds.leftWheel.getUnitDistance().millimeters(), 1000);
  ASSERT_EQ(parsedSpeeds.rightWheel.getUnitDistance().millimeters(), 2000);

  // Reset and send buffer again
  ASSERT_TRUE(Mock::VerifyAndClear(&mockObj));

  int partTwoPos = 0;
  setupSerialRead(mockObj, partTwoPos, partTwo);

  testInstance.parseIncomingBuffer();
  auto secondSpeeds = testInstance.getTargetSpeeds();

  EXPECT_EQ(secondSpeeds.leftWheel.getUnitDistance().millimeters(), 2000);
  EXPECT_EQ(secondSpeeds.rightWheel.getUnitDistance().millimeters(), 3000);
}

TEST_F(SerialCommsFixture, outgoingChecksumIsValid) {
  const Distance oneMeter = 1.0_m;
  const auto oneSecond = 1_s;
  const Speed oneMeterSecond(oneMeter, oneSecond);
  const Speed twoMeterSecond(oneMeter * 2, oneSecond);

  WheelSpeeds expectedSpeeds{oneMeterSecond, twoMeterSecond};

  std::string outString = "!s L_SPEED:1000 R_SPEED:2000";
  uint8_t expectedChecksum = calculateChecksum(outString);

  std::string expectedString = "chk:" + std::to_string(expectedChecksum);

  EXPECT_CALL(mockObj, serialPrint(Matcher<const std::string &>(
                           HasSubstr(expectedString))));
  testInstance.addVehicleSpeed(expectedSpeeds);
  testInstance.sendCurrentBuffer();
}

TEST_F(SerialCommsFixture, missingChecksumDetected) {
  const std::string missingChecksumCommand("!T L_SPEED:1000 R_SPEED:2000\n");

  const std::string expectedWarning(
      "No checksum found in the following command");

  int bufferPos = 0;
  setupSerialRead(mockObj, bufferPos, missingChecksumCommand);

  EXPECT_CALL(mockObj, serialPrint(Matcher<const std::string &>(
                           HasSubstr(expectedWarning))));

  testInstance.parseIncomingBuffer();
  testInstance.sendCurrentBuffer();

  // Check the command has no effect
  WheelSpeeds defaultSpeeds;
  EXPECT_EQ(testInstance.getTargetSpeeds().leftWheel.getUnitDistance(),
            defaultSpeeds.leftWheel.getUnitDistance());
}

TEST_F(SerialCommsFixture, partChecksumDetected) {
  const std::string partChecksum("!T L_SPEED:1000 R_SPEED:2000 chk:\n");

  const std::string expectedWarning("Checksum was missing value in");

  int bufferPos = 0;
  setupSerialRead(mockObj, bufferPos, partChecksum);

  EXPECT_CALL(mockObj, serialPrint(Matcher<const std::string &>(
                           HasSubstr(expectedWarning))));

  testInstance.parseIncomingBuffer();
  testInstance.sendCurrentBuffer();

  // Check the command has no effect
  WheelSpeeds defaultSpeeds;
  EXPECT_EQ(testInstance.getTargetSpeeds().leftWheel.getUnitDistance(),
            defaultSpeeds.leftWheel.getUnitDistance());
}

TEST_F(SerialCommsFixture, corruptChecksumDetected) {
  const std::string corruptChecksum("!T L_SPEED:1000 R_SPEED:2000 chk:a3\n");

  const std::string expectedWarning("Could not convert checksum");

  int bufferPos = 0;
  setupSerialRead(mockObj, bufferPos, corruptChecksum);

  EXPECT_CALL(mockObj, serialPrint(Matcher<const std::string &>(
                           HasSubstr(expectedWarning))));

  testInstance.parseIncomingBuffer();
  testInstance.sendCurrentBuffer();

  // Check the command has no effect
  WheelSpeeds defaultSpeeds;
  EXPECT_EQ(testInstance.getTargetSpeeds().leftWheel.getUnitDistance(),
            defaultSpeeds.leftWheel.getUnitDistance());
}

TEST_F(SerialCommsFixture, mismatchedChecksumDetected) {
  const std::string incorrectChecksum("!T L_SPEED:1000 R_SPEED:2000 chk:11\n");

  const std::string expectedWarning("checksum did not match");

  int bufferPos = 0;
  setupSerialRead(mockObj, bufferPos, incorrectChecksum);

  EXPECT_CALL(mockObj, serialPrint(Matcher<const std::string &>(
                           HasSubstr(expectedWarning))));

  testInstance.parseIncomingBuffer();
  testInstance.sendCurrentBuffer();

  // Check the command has no effect
  WheelSpeeds defaultSpeeds;
  EXPECT_EQ(testInstance.getTargetSpeeds().leftWheel.getUnitDistance(),
            defaultSpeeds.leftWheel.getUnitDistance());
}