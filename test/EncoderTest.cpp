#include <cmath>
#include <gtest/gtest.h>

#include "ArduinoGlobals.hpp"
#include "Encoder.hpp"
#include "Length.hpp"
#include "Speed.hpp"
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
Libs::unique_ptr<StrictMock<MockArduino>> expectMillisCall() {
  Libs::unique_ptr<StrictMock<MockArduino>> newMock(
      new StrictMock<MockArduino>());
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

const Length wheelRadius = 0.58_m * M_PI;
// 25 counts : 1 motor rotation
// 20 motor rotations : 1 wheel rotation
const int countsPerWheelRotation = 25 * 20;

TEST_F(EncoderFixture, calculatesSpeed) {
  const unsigned long ONE_SEC = 1000;
  const Speed wheelTurnPerSec(wheelRadius, ONE_SEC);
  const auto expectedSpeed = wheelTurnPerSec.getMilliMetersPerSecond();

  ASSERT_EQ(expectedSpeed, wheelRadius.getMilliMeters());

  InterruptData::g_pinEncoderData.leftEncoderCount = countsPerWheelRotation;
  InterruptData::g_pinEncoderData.rightEncoderCount =
      2 * countsPerWheelRotation;

  EXPECT_CALL(*mockHardware, millis()).WillOnce(Return(ONE_SEC));

  auto speed = testInstance.calculateSpeed();

  EXPECT_EQ(speed.leftWheel.getMilliMetersPerSecond(), expectedSpeed);
  EXPECT_EQ(speed.rightWheel.getMilliMetersPerSecond(), 2 * expectedSpeed);
}

TEST_F(EncoderFixture, calculatesSpeedAsTimeVaries) {
  const unsigned long ONE_SEC = 1000;
  const unsigned long HALF_SECOND = 500;

  EXPECT_CALL(*mockHardware, millis()).WillOnce(Return(ONE_SEC + HALF_SECOND));

  EXPECT_CALL(*mockHardware, millis())
      .WillOnce(Return(HALF_SECOND))
      .RetiresOnSaturation();

  InterruptData::g_pinEncoderData.leftEncoderCount = countsPerWheelRotation;
  InterruptData::g_pinEncoderData.rightEncoderCount = countsPerWheelRotation;

  const Speed expectedHalfSec(wheelRadius, HALF_SECOND);
  auto halfSecSpeed = testInstance.calculateSpeed();

  EXPECT_EQ(halfSecSpeed.leftWheel.getMilliMetersPerSecond(),
            expectedHalfSec.getMilliMetersPerSecond());
  EXPECT_EQ(halfSecSpeed.rightWheel.getMilliMetersPerSecond(),
            expectedHalfSec.getMilliMetersPerSecond());

  // Vehicle reverses in one second
  InterruptData::g_pinEncoderData.leftEncoderCount = 0;
  InterruptData::g_pinEncoderData.rightEncoderCount = 0;

  const Speed expectedOneSec(wheelRadius, ONE_SEC);
  auto oneSecSpeed = testInstance.calculateSpeed();

  EXPECT_EQ(oneSecSpeed.leftWheel.getMilliMetersPerSecond(),
            -(expectedOneSec.getMilliMetersPerSecond()));
  EXPECT_EQ(oneSecSpeed.rightWheel.getMilliMetersPerSecond(),
            -(expectedOneSec.getMilliMetersPerSecond()));
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
