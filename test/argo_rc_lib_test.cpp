#include <gtest/gtest.h>
#include <string>

#include "ArduinoGlobals.hpp"
#include "ArduinoInterface.hpp"
#include "Encoder.hpp"
#include "argo_rc_lib.hpp"
#include "mock_arduino.hpp"
#include "unique_ptr.hpp"

using namespace ::testing;

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

unsigned long time = 0;
unsigned long incrementMillis() {
  time += 100;
  return time;
}

void setToRcControl() {
  const int RC_PWM_VAL = 2000;
  InterruptData::g_pinData[2].lastGoodWidth = RC_PWM_VAL;
}

// void setToRosControl() {
//   const int ROS_PWM_VAL = 0;
//   InterruptData::g_pinData[2].lastGoodWidth = ROS_PWM_VAL;
// }

// ----------- Test fixture --------------

bool usePingTimeout = false;

class ArgoRcTest : public ::testing::Test {
protected:
  ArgoRcTest()
      : _forwardedPtr(new NiceMock<MockArduino>),
        hardwareMock(static_cast<NiceMock<MockArduino> &>(*_forwardedPtr)),
        argoRcLib(*_forwardedPtr, usePingTimeout) {
    time = 0;
    ON_CALL(hardwareMock, millis())
        .WillByDefault(InvokeWithoutArgs(&incrementMillis));
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

// -------- PWM input tests ----------

const unsigned int negBoundary = 1200;
const unsigned int posBounds = 1700;

// The RC interrupt timings
const unsigned int zeroValue = 1520;
const unsigned int range = 500;
const unsigned int minValue = zeroValue - range;
const unsigned int maxValue = zeroValue + range;

const size_t throttlePwmIndex = 0;
const size_t steeringPwmIndex = 1;

const int mappedValue = 0;

TEST_F(ArgoRcTest, throttleIsForward) {
  setToRcControl();

  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = posBounds;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = zeroValue;

  checkForwardLeftIsOn(hardwareMock);
  checkForwardRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, Gt(0)))
      .Times(1);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, Gt(0)))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, throttleIsReverse) {
  setToRcControl();

  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = negBoundary;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = zeroValue;

  checkReverseLeftIsOn(hardwareMock);
  checkReverseRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, Gt(0)))
      .Times(1);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, Gt(0)))
      .Times(1);

  argoRcLib.loop();
}

// ------------- PWM switch off tests --------------

TEST_F(ArgoRcTest, pwmOff) {
  setToRcControl();

  // Set timing data to the boundary value
  InterruptData::g_pinData[throttlePwmIndex].lastGoodWidth = zeroValue;
  InterruptData::g_pinData[steeringPwmIndex].lastGoodWidth = zeroValue;

  checkDirectionRelaysAreOff(hardwareMock);
  checkFootSwitchesAreOff(hardwareMock);

  argoRcLib.loop();
}
