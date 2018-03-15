#include <gtest/gtest.h>

#include "PidController.hpp"
#include "mock_arduino.hpp"
#include "unique_ptr.hpp"

using ::testing::Return;
using ::testing::StrictMock;
using ::testing::Test;

using namespace ArgoRcLib;
using namespace Libs;

namespace {
const unsigned long ONE_SEC = 1000;

unique_ptr<StrictMock<MockArduino>> createMockHardware() {

  unique_ptr<StrictMock<MockArduino>> hardware(new StrictMock<MockArduino>());

  EXPECT_CALL(*hardware, millis()).WillOnce(Return(0)).RetiresOnSaturation();
  return Libs::move(hardware);
}

class PidControllerFixture : public ::testing::Test {
protected:
  PidControllerFixture()
      : mockHardware(createMockHardware()), testInstance(*mockHardware) {}

  unique_ptr<StrictMock<MockArduino>> mockHardware;
  PidController testInstance;
};

TEST_F(PidControllerFixture, proportionalLowerMin) {
  // Set the current error to 0.1 meter - second
  constexpr Distance error = 0.1_m;
  constexpr int16_t expectedResult =
      PID_CONSTANTS::propLower * error.getMilliMeters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
  EXPECT_GE(result, 1);
}

TEST_F(PidControllerFixture, proportionalLowerMax) {
  // Set the current error to one mm below boundary speed for lower
  constexpr Distance error(
      0, static_cast<int32_t>(PID_CONSTANTS::boundarySpeed - 1));
  constexpr int16_t expectedResult =
      PID_CONSTANTS::propLower * error.getMilliMeters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, proportionalUpperBottom) {
  // Set the current error to one mm to boundary speed for upper
  constexpr Distance error(0,
                           static_cast<int32_t>(PID_CONSTANTS::boundarySpeed));

  constexpr int16_t expectedResult =
      PID_CONSTANTS::propUpper * error.getMilliMeters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, proportionalUpperMaxVal) {
  // At 5 m/s difference we should always get max acceleration
  constexpr Distance error = 5_m;

  constexpr int16_t expectedResult =
      PID_CONSTANTS::propUpper * error.getMilliMeters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
  EXPECT_GE(result, 255);
}

TEST_F(PidControllerFixture, integralLowerBottom) {
  constexpr Distance error = 0.1_m;

  constexpr int16_t expectedResult =
      PID_CONSTANTS::integralLower * error.getMilliMeters();

  auto result = testInstance.calcIntegral(error);
  EXPECT_EQ(result, expectedResult);
  EXPECT_GE(result, 1);
}

TEST_F(PidControllerFixture, integralLowerMax) {
  constexpr Distance error(
      0, static_cast<int32_t>(PID_CONSTANTS::boundarySpeed - 1));

  constexpr int16_t expectedResult =
      PID_CONSTANTS::integralLower * error.getMilliMeters();

  auto result = testInstance.calcIntegral(error);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, integralUpperBottom) {
  constexpr Distance error(0,
                           static_cast<int32_t>(PID_CONSTANTS::boundarySpeed));

  constexpr int16_t expectedResult =
      PID_CONSTANTS::integralUpper * error.getMilliMeters();

  auto result = testInstance.calcIntegral(error);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, integralUpperMaxVal) {
  constexpr Distance error = 20_m;

  auto result = testInstance.calcIntegral(error);
  ASSERT_EQ(result, 255);
}

TEST_F(PidControllerFixture, integralIsCumulative) {
  constexpr Distance error = 2_m;

  constexpr int16_t expectedResult =
      PID_CONSTANTS::integralUpper * error.getMilliMeters();

  auto result = testInstance.calcIntegral(error);
  ASSERT_EQ(result, expectedResult);
  // Now it should be cumulative
  auto secondResult = testInstance.calcIntegral(error);
  EXPECT_EQ(secondResult, 2 * expectedResult);
}

} // namespace