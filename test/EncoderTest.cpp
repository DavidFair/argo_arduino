#include <gtest/gtest.h>

#include "ArduinoGlobals.hpp"
#include "Encoder.hpp"

using namespace Globals;
using namespace Hardware;

TEST(EncoderTest, startsAtZero) {
  // Note: This test must be first before we manipulate the global struct
  Encoder encoderObj;
  auto returnedVals = encoderObj.read();

  EXPECT_EQ(0, returnedVals.encoderVal[ArgoEncoderPositions::LEFT_ENCODER]);
  EXPECT_EQ(0, returnedVals.encoderVal[ArgoEncoderPositions::RIGHT_ENCODER]);
}

TEST(EncoderTest, read) {
  const int32_t leftKnownVal = 123;
  const int32_t rightKnownVal = 234;

  InterruptData::g_pinEncoderData.leftEncoderCount = leftKnownVal;
  InterruptData::g_pinEncoderData.rightEncoderCount = rightKnownVal;

  Encoder encoderObj;

  EncoderData returnedVals = encoderObj.read();

  EXPECT_EQ(leftKnownVal,
            returnedVals.encoderVal[ArgoEncoderPositions::LEFT_ENCODER]);
  EXPECT_EQ(rightKnownVal,
            returnedVals.encoderVal[ArgoEncoderPositions::RIGHT_ENCODER]);
}

TEST(EncoderTest, reset) {
  Encoder encoderObj;

  encoderObj.write(ArgoEncoderPositions::LEFT_ENCODER, 1);
  encoderObj.write(ArgoEncoderPositions::RIGHT_ENCODER, 1);

  // Check these are not 0 before testing
  ASSERT_NE(InterruptData::g_pinEncoderData.leftEncoderCount, 0);
  ASSERT_NE(InterruptData::g_pinEncoderData.rightEncoderCount, 0);

  encoderObj.reset();
  auto vals = encoderObj.read();
  EXPECT_EQ(0, vals.encoderVal[ArgoEncoderPositions::LEFT_ENCODER]);
  EXPECT_EQ(0, vals.encoderVal[ArgoEncoderPositions::RIGHT_ENCODER]);
}

TEST(EncoderTest, write) {
  const int32_t leftExpectedVal = 456;
  const int32_t rightExpectedVal = 789;

  Encoder encoderObj;

  encoderObj.write(ArgoEncoderPositions::LEFT_ENCODER, leftExpectedVal);
  encoderObj.write(ArgoEncoderPositions::RIGHT_ENCODER, rightExpectedVal);

  auto returnedVals = encoderObj.read();

  EXPECT_EQ(leftExpectedVal,
            returnedVals.encoderVal[ArgoEncoderPositions::LEFT_ENCODER]);
  EXPECT_EQ(rightExpectedVal,
            returnedVals.encoderVal[ArgoEncoderPositions::RIGHT_ENCODER]);
}