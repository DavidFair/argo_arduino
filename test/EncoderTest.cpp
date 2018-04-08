#include <cmath>
#include <gtest/gtest.h>

#include "ArduinoGlobals.hpp"
#include "Distance.hpp"
#include "Encoder.hpp"
#include "Speed.hpp"
#include "Time.hpp"
#include "mock_arduino.hpp"
#include "move.hpp"
#include "unique_ptr.hpp"

using ::testing::Return;
using ::testing::StrictMock;
using ::testing::Test;

using namespace Globals;
using namespace Hardware;
using namespace Libs;

namespace {

const Time oneSecond = 1_s;
const Time halfSecond = 0.5_s;

unique_ptr<StrictMock<MockArduino>> expectMillisCall() {

  unique_ptr<StrictMock<MockArduino>> newMock(new StrictMock<MockArduino>());
  EXPECT_CALL(*newMock, millis()).WillOnce(Return(0)).RetiresOnSaturation();
  return Libs::move(newMock);
}

class EncoderFixture : public ::testing::Test {
protected:
  EncoderFixture()
      : mockHardware(expectMillisCall()), testInstance(*mockHardware) {}

  Libs::unique_ptr<StrictMock<MockArduino>> mockHardware;
  Encoder testInstance;
};
} // Anonymous Namespace

TEST_F(EncoderFixture, startsAtZero) {
  // Note: This test must be first before we manipulate the global struct

  auto returnedVals = testInstance.read();

  EXPECT_EQ(0, returnedVals.leftEncoderVal);
  EXPECT_EQ(0, returnedVals.rightEncoderVal);
}

constexpr Distance wheelCirc = 1.91_m;
constexpr int countsPerWheelRotation = 490;
constexpr Distance distPerCount = wheelCirc / countsPerWheelRotation;

TEST_F(EncoderFixture, calculatesSpeed) {
  const auto expectedSpeed = distPerCount * countsPerWheelRotation;

  InterruptData::g_pinEncoderData.leftEncoderCount = countsPerWheelRotation;
  InterruptData::g_pinEncoderData.rightEncoderCount =
      countsPerWheelRotation * 2;

  EXPECT_CALL(*mockHardware, millis()).WillOnce(Return(oneSecond.millis()));

  auto speed = testInstance.calculateSpeed();

  EXPECT_EQ(speed.leftWheel.getUnitDistance(), expectedSpeed);
  EXPECT_EQ(speed.rightWheel.getUnitDistance(), expectedSpeed * 2);
}

TEST_F(EncoderFixture, calculatesSpeedAsTimeVaries) {
  EXPECT_CALL(*mockHardware, millis())
      .WillOnce(Return((oneSecond + halfSecond).millis()));

  EXPECT_CALL(*mockHardware, millis())
      .WillOnce(Return(halfSecond.millis()))
      .RetiresOnSaturation();

  InterruptData::g_pinEncoderData.leftEncoderCount = countsPerWheelRotation;
  InterruptData::g_pinEncoderData.rightEncoderCount = countsPerWheelRotation;

  const Distance expectedDistance = distPerCount * countsPerWheelRotation;
  const Speed expectedHalfSec(expectedDistance, halfSecond);

  auto halfSecSpeed = testInstance.calculateSpeed();

  EXPECT_EQ(halfSecSpeed.leftWheel.getUnitDistance(),
            expectedHalfSec.getUnitDistance());
  EXPECT_EQ(halfSecSpeed.rightWheel.getUnitDistance(),
            expectedHalfSec.getUnitDistance());

  // Vehicle reverses in one second
  InterruptData::g_pinEncoderData.leftEncoderCount = 0;
  InterruptData::g_pinEncoderData.rightEncoderCount = 0;

  const Speed expectedOneSec(expectedDistance, oneSecond);
  auto oneSecSpeed = testInstance.calculateSpeed();

  EXPECT_EQ(oneSecSpeed.leftWheel.getUnitDistance().millimeters(),
            -(expectedOneSec.getUnitDistance().millimeters()));
  EXPECT_EQ(oneSecSpeed.rightWheel.getUnitDistance().millimeters(),
            -(expectedOneSec.getUnitDistance().millimeters()));
}

TEST_F(EncoderFixture, read) {
  const int32_t leftKnownVal = 123;
  const int32_t rightKnownVal = 234;

  InterruptData::g_pinEncoderData.leftEncoderCount = leftKnownVal;
  InterruptData::g_pinEncoderData.rightEncoderCount = rightKnownVal;

  auto returnedVals = testInstance.read();

  EXPECT_EQ(leftKnownVal, returnedVals.leftEncoderVal);
  EXPECT_EQ(rightKnownVal, returnedVals.rightEncoderVal);
}

TEST_F(EncoderFixture, reset) {

  InterruptData::g_pinEncoderData.leftEncoderCount = 1;
  InterruptData::g_pinEncoderData.rightEncoderCount = 1;

  // Check these are not 0 before testing
  ASSERT_NE(InterruptData::g_pinEncoderData.leftEncoderCount, 0);
  ASSERT_NE(InterruptData::g_pinEncoderData.rightEncoderCount, 0);

  testInstance.reset();
  auto vals = testInstance.read();
  EXPECT_EQ(0, vals.leftEncoderVal);
  EXPECT_EQ(0, vals.rightEncoderVal);
}
