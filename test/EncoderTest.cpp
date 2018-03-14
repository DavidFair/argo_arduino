#include <gtest/gtest.h>

#include "ArduinoGlobals.hpp"
#include "Encoder.hpp"

using namespace Globals;
using namespace Hardware;

TEST(EncoderTest, startsAtZero) {
  // Note: This test must be first before we manipulate the global struct
  Encoder encoderObj;
  auto returnedVals = encoderObj.read();

  EXPECT_EQ(0, returnedVals.leftEncoderVal);
  EXPECT_EQ(0, returnedVals.rightEncoderVal);
}

TEST(EncoderTest, read) {
  const int32_t leftKnownVal = 123;
  const int32_t rightKnownVal = 234;

  InterruptData::g_pinEncoderData.leftEncoderCount = leftKnownVal;
  InterruptData::g_pinEncoderData.rightEncoderCount = rightKnownVal;

  Encoder encoderObj;

  auto returnedVals = encoderObj.read();

  EXPECT_EQ(leftKnownVal, returnedVals.leftEncoderVal);
  EXPECT_EQ(rightKnownVal, returnedVals.rightEncoderVal);
}

TEST(EncoderTest, reset) {
  Encoder encoderObj;

  InterruptData::g_pinEncoderData.leftEncoderCount = 1;
  InterruptData::g_pinEncoderData.rightEncoderCount = 1;

  // Check these are not 0 before testing
  ASSERT_NE(InterruptData::g_pinEncoderData.leftEncoderCount, 0);
  ASSERT_NE(InterruptData::g_pinEncoderData.rightEncoderCount, 0);

  encoderObj.reset();
  auto vals = encoderObj.read();
  EXPECT_EQ(0, vals.leftEncoderVal);
  EXPECT_EQ(0, vals.rightEncoderVal);
}
