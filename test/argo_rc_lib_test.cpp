#include <gtest/gtest.h>
#include <string>

#include "ArduinoGlobals.hpp"
#include "Encoder.hpp"
#include "arduino_interface.hpp"
#include "argo_rc_lib.hpp"
#include "mock_arduino.hpp"
#include "unique_ptr.hpp"

using ::testing::An;
using ::testing::AnyNumber;
using ::testing::Ge;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Test;
using ::testing::_;

using namespace ArduinoEnums;
using namespace ArgoRcLib;
using namespace Globals;
using namespace Hardware;

// Anonymous namespace
namespace {
// ---------- Functions to setup various checks --------

// RetiresOnSaturation() ensures we can setup multiple expectations
// of the same type i.e. we can check the footswitch on is called 2x1 times

void checkFootSwitchesAreOff(MockArduino &hardwareInterface) {
  EXPECT_CALL(hardwareInterface, digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                              digitalIO::E_HIGH))
      .Times(1)
      .RetiresOnSaturation();

  EXPECT_CALL(
      hardwareInterface,
      digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_HIGH))
      .Times(1)
      .RetiresOnSaturation();
}

void checkFootSwitchesAreOn(MockArduino &hardwareInterface) {
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY, digitalIO::E_LOW))
      .Times(1)
      .RetiresOnSaturation();

  EXPECT_CALL(
      hardwareInterface,
      digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_LOW))
      .Times(1)
      .RetiresOnSaturation();
}

void checkDirectionRelaysAreOff(MockArduino &hardwareInterface) {
  // And the direction relays are switched off too
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::E_HIGH))
      .Times(1);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::E_HIGH))
      .Times(1);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_FORWARD_RELAY, digitalIO::E_HIGH))
      .Times(1);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_REVERSE_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkForwardLeftIsOn(MockArduino &hardwareInterface) {
  // Check the relays are switched on by dropping the circuit low
  checkFootSwitchesAreOn(hardwareInterface);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_FORWARD_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_REVERSE_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkForwardRightIsOn(MockArduino &hardwareInterface) {
  // Check the relays are switched on by dropping the circuit low
  checkFootSwitchesAreOn(hardwareInterface);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkReverseLeftIsOn(MockArduino &hardwareInterface) {
  // Check the relays are switched on by dropping the circuit low
  checkFootSwitchesAreOn(hardwareInterface);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_REVERSE_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_FORWARD_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkReverseRightIsOn(MockArduino &hardwareInterface) {
  checkFootSwitchesAreOn(hardwareInterface);
  // Check the relays are switched on by dropping the circuit low
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void returnDeadmanSafe(MockArduino &hardwareInterface) {
  ON_CALL(hardwareInterface, digitalRead(pinMapping::RC_DEADMAN))
      .WillByDefault(Return(digitalIO::E_HIGH));
}

// ----------- Test fixture --------------

class ArgoRcTest : public ::testing::Test {
protected:
  ArgoRcTest()
      : _forwardedPtr(new NiceMock<MockArduino>),
        hardwareMock(static_cast<NiceMock<MockArduino> &>(*_forwardedPtr)),
        argoRcLib(*_forwardedPtr) {
    returnDeadmanSafe(hardwareMock);
  }

  // This pointer has its ownership transfered to ArgoRc however we still
  // hold a reference so we can set expectations
  Libs::unique_ptr<ArduinoInterface> _forwardedPtr;
  NiceMock<MockArduino> &hardwareMock;
  ArgoRc argoRcLib;
};

} // End of anonymous namespace

// ----- Setup Tests ------

TEST_F(ArgoRcTest, configuresDigitalIO) {
  EXPECT_CALL(hardwareMock, serialBegin(Ge(1000))).Times(1);

  // Check all output Pins
  for (auto pin : ArduinoEnums::allDigitalPins) {
    EXPECT_CALL(hardwareMock, setPinMode(pin, _));
  }

  argoRcLib.setup();
}

TEST_F(ArgoRcTest, resetRelays) {

  // Ignore all other writes
  EXPECT_CALL(hardwareMock, digitalWrite(_, _)).Times(AnyNumber());

  checkFootSwitchesAreOff(hardwareMock);
  checkDirectionRelaysAreOff(hardwareMock);

  argoRcLib.setup();
}

// ------ Direction Tests ---------

TEST_F(ArgoRcTest, forwardLeftSetsRelays) {
  checkForwardLeftIsOn(hardwareMock);
  argoRcLib.forward_left();
  ASSERT_EQ(ArgoData::g_currentVehicleDirection.leftWheelDirection,
            Direction::FORWARD);
}

TEST_F(ArgoRcTest, forwardRightSetsRelays) {
  checkForwardRightIsOn(hardwareMock);
  argoRcLib.forward_right();
  ASSERT_EQ(ArgoData::g_currentVehicleDirection.rightWheelDirection,
            Direction::FORWARD);
}

TEST_F(ArgoRcTest, reverseLeftSetRelays) {
  checkReverseLeftIsOn(hardwareMock);
  argoRcLib.reverse_left();
  ASSERT_EQ(ArgoData::g_currentVehicleDirection.leftWheelDirection,
            Direction::REVERSE);
}

TEST_F(ArgoRcTest, reverseRightSetsRelays) {
  checkReverseRightIsOn(hardwareMock);
  argoRcLib.reverse_right();
  ASSERT_EQ(ArgoData::g_currentVehicleDirection.rightWheelDirection,
            Direction::REVERSE);
}

// ----- Footswitch tests -------

TEST_F(ArgoRcTest, footSwitchOn) {

  checkFootSwitchesAreOn(hardwareMock);

  argoRcLib.footswitch_on();
}

TEST_F(ArgoRcTest, footSwitchOff) {

  checkFootSwitchesAreOff(hardwareMock);

  argoRcLib.footswitch_off();
}

// ------- Deadman switch tests ------
TEST_F(ArgoRcTest, deadmanSwitchTriggers) {
  const unsigned long LOOP_MAX_DELAY = 500;
  // We return 0, 499 and 500 to the loop
  EXPECT_CALL(hardwareMock, millis()).WillOnce(Return(LOOP_MAX_DELAY));
  EXPECT_CALL(hardwareMock, millis())
      .WillOnce(Return(LOOP_MAX_DELAY - 1))
      .RetiresOnSaturation();
  EXPECT_CALL(hardwareMock, millis()).WillOnce(Return(0)).RetiresOnSaturation();

  // At least 2 calls - one before and in the loop
  EXPECT_CALL(hardwareMock, digitalRead(pinMapping::RC_DEADMAN))
      .Times(2)
      .WillRepeatedly(Return(digitalIO::E_LOW));

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0));
  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0));

  checkFootSwitchesAreOff(hardwareMock);
  checkDirectionRelaysAreOff(hardwareMock);

  EXPECT_CALL(hardwareMock, enterDeadmanSafetyMode());

  argoRcLib.loop();
}

// -------- PWM input tests ----------

// Maps to 20/-20 through the PWM mapping
const unsigned int negBoundary = 1495;
const unsigned int posBounds = 1546;

const unsigned int zeroValue = 1520;
const unsigned int minValue = 1190;
const unsigned int maxValue = 1850;

const int mappedValue = 20;

const size_t throttlePwmIndex = 0;
const size_t steeringPwmIndex = 1;

TEST_F(ArgoRcTest, throttleIsForward) {
  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = posBounds;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = zeroValue;

  checkForwardLeftIsOn(hardwareMock);
  checkForwardRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, throttleIsReverse) {
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = negBoundary;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = zeroValue;

  checkReverseLeftIsOn(hardwareMock);
  checkReverseRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, leftReverse) {
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = negBoundary;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = minValue;

  checkReverseRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 2 * mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, rightReverse) {
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = negBoundary;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = maxValue;

  checkReverseLeftIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, 2 * mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, rightForward) {
  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = posBounds;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = minValue;

  checkForwardRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 2 * mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, leftForward) {
  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = posBounds;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = maxValue;

  checkForwardLeftIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, 2 * mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0))
      .Times(1);

  argoRcLib.loop();
}

// ------------- PWM switch off tests --------------

TEST_F(ArgoRcTest, pwmNegativeOff) {
  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = negBoundary + 1;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = negBoundary + 1;

  checkDirectionRelaysAreOff(hardwareMock);
  checkFootSwitchesAreOff(hardwareMock);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, pwmPositiveOff) {
  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = posBounds - 1;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = posBounds - 1;

  checkDirectionRelaysAreOff(hardwareMock);
  checkFootSwitchesAreOff(hardwareMock);

  argoRcLib.loop();
}

// ---------------- PWM turn on the spot ------------
TEST_F(ArgoRcTest, leftTurnOnSpot) {
  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = zeroValue;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = negBoundary;

  checkReverseLeftIsOn(hardwareMock);
  checkForwardRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, rightTurnOnSpot) {
  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = zeroValue;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = posBounds;

  checkForwardLeftIsOn(hardwareMock);
  checkReverseRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, mappedValue))
      .Times(1);

  argoRcLib.loop();
}

// --------- Check Encoder data is printed over serial -------
// As we need to setup our own on call for the encoder mock don't use text
// fixtures
TEST_F(ArgoRcTest, encoderDataIsSent) {
  const int32_t expectedVal = 123;

  InterruptData::g_pinEncoderData.leftEncoderCount = expectedVal;
  InterruptData::g_pinEncoderData.rightEncoderCount = expectedVal;

  // Ignore other calls to serialPrintln
  EXPECT_CALL(hardwareMock, serialPrintln(An<int>())).Times(AnyNumber());
  EXPECT_CALL(hardwareMock, serialPrintln(An<const std::string &>()))
      .Times(AnyNumber());

  // Set the deadman switch to safe
  returnDeadmanSafe(hardwareMock);

  const std::string expectedOutput("!D L_ENC:123 R_ENC:123 ");
  EXPECT_CALL(hardwareMock, serialPrintln(expectedOutput));

  argoRcLib.loop();
}