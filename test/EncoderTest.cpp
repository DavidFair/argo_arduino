#include <gtest/gtest.h>

#include "ArduinoGlobals.hpp"
#include "Encoder.hpp"
#include "mock_arduino.hpp"

using ::testing::StrictMock;
using ::testing::Test;

using namespace Globals;
using namespace Hardware;

namespace {
class EncoderFixture : public ::testing::Test {
protected:
  EncoderFixture() : mockHardware(), testInstance(mockHardware) {}

  StrictMock<MockArduino> mockHardware;
  Encoder testInstance;
};
} // Anonymous Namespace

TEST_F(EncoderFixture, startsAtZero) {
  // Note: This test must be first before we manipulate the global struct

  auto returnedVals = testInstance.read();

  EXPECT_EQ(0, returnedVals.leftEncoderVal);
  EXPECT_EQ(0, returnedVals.rightEncoderVal);
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
