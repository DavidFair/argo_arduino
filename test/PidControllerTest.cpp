#include <gtest/gtest.h>

#include "Encoder.hpp"
#include "PidController.hpp"
#include "Time.hpp"
#include "mock_arduino.hpp"
#include "unique_ptr.hpp"

using ::testing::AnyNumber;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;
using ::testing::StrictMock;
using ::testing::Test;

using namespace ArgoRcLib;
using namespace Libs;
using namespace Hardware;

namespace {
constexpr auto ONE_UNIT_SEC = 1_s;

unsigned long time = 0;
unsigned long incrementMillis() {
  time += ONE_UNIT_SEC.millis();
  return time;
}

unique_ptr<StrictMock<MockArduino>> createMockHardware() {

  unique_ptr<StrictMock<MockArduino>> hardware(new StrictMock<MockArduino>());
  time = 0;

  EXPECT_CALL(*hardware, millis()).Times(AnyNumber());
  ON_CALL(*hardware, millis())
      .WillByDefault(InvokeWithoutArgs(&incrementMillis));

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

TEST_F(PidControllerFixture, proportionalMin) {
  // Set the current error to 0.1 meter - second
  constexpr Distance error = 0.1_m;

  auto result = testInstance.calcProportional(error);
  EXPECT_GT(result, 0);
}

TEST_F(PidControllerFixture, proportionalMax) {
  constexpr Distance error = 1_m;

  constexpr float expectedResult = PID_CONSTANTS::prop * error.meters();

  auto result = testInstance.calcProportional(error);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, integralMin) {
  constexpr Distance error = 1_m;
  auto result = testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER,
                                          ONE_UNIT_SEC);
  EXPECT_GT(result, 0);
}

TEST_F(PidControllerFixture, integral) {
  constexpr Distance error = 20_m;

  constexpr float expectedResult = PID_CONSTANTS::integral * error.meters();

  auto result = testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER,
                                          ONE_UNIT_SEC);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, integralIsCumulative) {
  constexpr Distance error = 2_m;

  constexpr float expectedResult = PID_CONSTANTS::integral * error.meters();

  auto result = testInstance.calcIntegral(error, EncoderPositions::LEFT_ENCODER,
                                          ONE_UNIT_SEC);
  ASSERT_EQ(result, expectedResult);
  // Now it should be cumulative
  auto secondResult = testInstance.calcIntegral(
      error, EncoderPositions::LEFT_ENCODER, ONE_UNIT_SEC);
  EXPECT_EQ(secondResult, 2 * expectedResult);
}

TEST_F(PidControllerFixture, derivMin) {
  constexpr Distance error = 0.1_m;

  constexpr float expectedResult = PID_CONSTANTS::deriv * error.meters() * -1;

  auto result = testInstance.calcDeriv(error, EncoderPositions::LEFT_ENCODER,
                                       ONE_UNIT_SEC);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, derivTracksPrevious) {
  constexpr Distance errorOne = 0.1_m;
  constexpr Distance errorTwo = 0.5_m;

  constexpr Distance difference = errorTwo - errorOne;

  constexpr float expectedResult =
      PID_CONSTANTS::deriv * difference.meters() * -1;

  testInstance.calcDeriv(errorOne, EncoderPositions::LEFT_ENCODER,
                         ONE_UNIT_SEC);
  auto result = testInstance.calcDeriv(errorTwo, EncoderPositions::LEFT_ENCODER,
                                       ONE_UNIT_SEC);
  EXPECT_EQ(result, expectedResult);
}

TEST_F(PidControllerFixture, derivMax) {
  constexpr Distance error = 10_m;

  auto result = testInstance.calcDeriv(error, EncoderPositions::LEFT_ENCODER,
                                       ONE_UNIT_SEC);
  // We should see a resistive contribution
  EXPECT_LT(result, 0);
}

// -------- Other methods on class --------------

TEST_F(PidControllerFixture, calculatePwmTargets) {
  Speed zeroSpeed;
  const Distance oneMeter = 1_m;
  const Time oneSecond = 1_s;
  const Speed oneMeterSecond(oneMeter, oneSecond);

  Hardware::WheelSpeeds zeroSpeedCurrent(zeroSpeed, zeroSpeed);
  Hardware::WheelSpeeds targetSpeed(oneMeterSecond, oneMeterSecond);

  float expectedVal = 0;
  expectedVal += testInstance.calcProportional(oneMeter);
  expectedVal += testInstance.calcIntegral(
      oneMeter, EncoderPositions::LEFT_ENCODER, ONE_UNIT_SEC);
  expectedVal += testInstance.calcDeriv(
      oneMeter, EncoderPositions::LEFT_ENCODER, ONE_UNIT_SEC);

  // Create a new instance so we have a fresh state
  auto mockHardware = createMockHardware();
  PidController newController(*mockHardware);

  PwmTargets val =
      newController.calculatePwmTargets(zeroSpeedCurrent, targetSpeed);
  EXPECT_EQ(val.leftPwm, expectedVal);
  EXPECT_EQ(val.rightPwm, expectedVal);
}

TEST_F(PidControllerFixture, calculateNegPwmTargets) {
  Speed zeroSpeed;
  const Distance oneMeter = -1_m;
  const Time oneSecond = 1_s;
  const Speed oneMeterSecond(oneMeter, oneSecond);

  Hardware::WheelSpeeds zeroSpeedCurrent(zeroSpeed, zeroSpeed);
  Hardware::WheelSpeeds targetSpeed(oneMeterSecond, oneMeterSecond);

  float expectedVal = 0;
  expectedVal += testInstance.calcProportional(oneMeter);
  expectedVal += testInstance.calcIntegral(
      oneMeter, EncoderPositions::LEFT_ENCODER, ONE_UNIT_SEC);
  expectedVal += testInstance.calcDeriv(
      oneMeter, EncoderPositions::LEFT_ENCODER, ONE_UNIT_SEC);

  // Create a new instance so we have a fresh state
  auto mockHardware = createMockHardware();
  PidController newController(*mockHardware);

  PwmTargets val =
      newController.calculatePwmTargets(zeroSpeedCurrent, targetSpeed);
  EXPECT_EQ(val.leftPwm, expectedVal);
  EXPECT_EQ(val.rightPwm, expectedVal);
}

TEST_F(PidControllerFixture, resetPid) {
  Speed zeroSpeed;
  const Distance oneMeter = 1_m;
  const Time oneSecond = 1_s;
  const Speed oneMeterSecond(oneMeter, oneSecond);

  Hardware::WheelSpeeds zeroSpeedCurrent(zeroSpeed, zeroSpeed);
  Hardware::WheelSpeeds targetSpeed(oneMeterSecond, oneMeterSecond);

  // Create a new instance so we have a fresh state
  auto mockHardware = createMockHardware();
  PidController newController(*mockHardware);

  PwmTargets val =
      newController.calculatePwmTargets(zeroSpeedCurrent, targetSpeed);
  ASSERT_NE(val.leftPwm, 0);
  ASSERT_NE(val.rightPwm, 0);

  newController.resetPid();
  auto newTarget =
      newController.calculatePwmTargets(zeroSpeedCurrent, targetSpeed);

  // If the PID reset these should be equal now
  EXPECT_EQ(newTarget.leftPwm, val.leftPwm);
  EXPECT_EQ(newTarget.rightPwm, val.rightPwm);
}

TEST_F(PidControllerFixture, speedUnderThresholdStops) {
  Speed zeroSpeed;
  const Distance thresholdDistance(0, 100);
  const Time oneSecond = 1_s;
  const Speed stopThreshold(thresholdDistance, oneSecond);

  Hardware::WheelSpeeds zeroSpeedCurrent(zeroSpeed, zeroSpeed);
  Hardware::WheelSpeeds targetSpeed(stopThreshold, stopThreshold);

  PwmTargets val =
      testInstance.calculatePwmTargets(zeroSpeedCurrent, targetSpeed);
  EXPECT_EQ(val.leftPwm, 0);
  EXPECT_EQ(val.rightPwm, 0);
}

} // namespace