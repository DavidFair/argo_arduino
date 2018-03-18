#include <gtest/gtest.h>

#include "Encoder.hpp"
#include "PidController.hpp"
#include "mock_arduino.hpp"
#include "unique_ptr.hpp"

using ::testing::Return;
using ::testing::StrictMock;
using ::testing::Test;

using namespace ArgoRcLib;
using namespace Libs;
using namespace Hardware;

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

// --------- P - I - D step tests --------------

TEST_F(PidControllerFixture, proportionalLowerMin) {
  // Set the current error to 0.1 meter - second
  constexpr Distance error = 0.1_m;
  constexpr int16_t expectedResult =
      PID_CONSTANTS::propLower * error.millimeters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
  EXPECT_GE(result, 1);
}

TEST_F(PidControllerFixture, proportionalLowerMax) {
  // Set the current error to one mm below boundary speed for lower
  constexpr Distance error(
      0, static_cast<int32_t>(PID_CONSTANTS::boundarySpeed - 1));
  constexpr int16_t expectedResult =
      PID_CONSTANTS::propLower * error.millimeters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, proportionalUpperBottom) {
  // Set the current error to one mm to boundary speed for upper
  constexpr Distance error(0,
                           static_cast<int32_t>(PID_CONSTANTS::boundarySpeed));

  constexpr int16_t expectedResult =
      PID_CONSTANTS::propUpper * error.millimeters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, proportionalUpperMaxVal) {
  // At 5 m/s difference we should always get max acceleration
  constexpr Distance error = 5_m;

  constexpr int16_t expectedResult =
      PID_CONSTANTS::propUpper * error.millimeters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
  EXPECT_GE(result, 255);
}

TEST_F(PidControllerFixture, integralLowerBottom) {
  constexpr Distance error = 0.1_m;

  constexpr int16_t expectedResult =
      PID_CONSTANTS::integralLower * error.millimeters();

  auto result =
      testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER);
  EXPECT_EQ(result, expectedResult);
  EXPECT_GE(result, 1);
}

TEST_F(PidControllerFixture, integralLowerMax) {
  constexpr Distance error(
      0, static_cast<int32_t>(PID_CONSTANTS::boundarySpeed - 1));

  constexpr int16_t expectedResult =
      PID_CONSTANTS::integralLower * error.millimeters();

  auto result =
      testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, integralUpperBottom) {
  constexpr Distance error(0,
                           static_cast<int32_t>(PID_CONSTANTS::boundarySpeed));

  constexpr int16_t expectedResult =
      PID_CONSTANTS::integralUpper * error.millimeters();

  auto result =
      testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, integralUpperMaxVal) {
  constexpr Distance error = 20_m;

  auto result =
      testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER);
  ASSERT_EQ(result, 255);
}

TEST_F(PidControllerFixture, integralIsCumulative) {
  constexpr Distance error = 2_m;

  constexpr int16_t expectedResult =
      PID_CONSTANTS::integralUpper * error.millimeters();

  auto result =
      testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER);
  ASSERT_EQ(result, expectedResult);
  // Now it should be cumulative
  auto secondResult =
      testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER);
  EXPECT_EQ(secondResult, 2 * expectedResult);
}

TEST_F(PidControllerFixture, derivLowerBottom) {
  constexpr Distance error = 0.1_m;

  constexpr int16_t expectedResult =
      PID_CONSTANTS::derivLower * error.millimeters() * -1;

  auto result = testInstance.calcDeriv(error, EncoderPositions::LEFT_ENCODER);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, derivTracksPrevious) {
  constexpr Distance errorOne = 0.1_m;
  constexpr Distance errorTwo = 0.5_m;

  constexpr Distance difference = errorTwo - errorOne;

  constexpr int16_t expectedResult =
      PID_CONSTANTS::derivLower * difference.millimeters() * -1;

  testInstance.calcDeriv(errorOne, EncoderPositions::LEFT_ENCODER);
  auto result =
      testInstance.calcDeriv(errorTwo, EncoderPositions::LEFT_ENCODER);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, derivLowerMax) {
  constexpr Distance error(
      0, static_cast<int32_t>(PID_CONSTANTS::boundarySpeed - 1));

  constexpr int16_t expectedResult =
      PID_CONSTANTS::derivLower * error.millimeters() * -1;

  auto result = testInstance.calcDeriv(error, EncoderPositions::LEFT_ENCODER);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, derivUpperBottom) {
  constexpr Distance error(0,
                           static_cast<int32_t>(PID_CONSTANTS::boundarySpeed));

  constexpr int16_t expectedResult =
      PID_CONSTANTS::derivUpper * error.millimeters() * -1;

  auto result = testInstance.calcDeriv(error, EncoderPositions::LEFT_ENCODER);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, derivUpperMax) {
  constexpr Distance error = 10_m;

  auto result = testInstance.calcDeriv(error, EncoderPositions::LEFT_ENCODER);
  EXPECT_GE(-255, result);
}

// -------- Other methods on class --------------

TEST_F(PidControllerFixture, calculatePwmTargets) {
  Speed zeroSpeed;
  constexpr Distance oneMeter = 1_m;
  constexpr Speed oneMeterSecond(Libs::move(1_m), Libs::move(1_s));

  Hardware::WheelSpeeds zeroSpeedCurrent(zeroSpeed, zeroSpeed);
  Hardware::WheelSpeeds targetSpeed(oneMeterSecond, oneMeterSecond);

  int16_t expectedVal = 0;
  expectedVal += testInstance.calcProportional(oneMeter);
  expectedVal +=
      testInstance.calcIntegral(oneMeter, EncoderPositions::LEFT_ENCODER);
  expectedVal +=
      testInstance.calcDeriv(oneMeter, EncoderPositions::LEFT_ENCODER);

  // Create a new instance so we have a fresh state
  PidController newController(*createMockHardware());

  PwmTargets val =
      newController.calculatePwmTargets(zeroSpeedCurrent, targetSpeed);
  EXPECT_EQ(val.leftPwm, expectedVal);
  EXPECT_EQ(val.rightPwm, expectedVal);
}

TEST_F(PidControllerFixture, resetPid) {}

} // namespace