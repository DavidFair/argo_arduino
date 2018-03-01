#include <gtest/gtest.h>

#include "argo_rc_lib.hpp"
#include "mock_arduino.hpp"
#include "pinTimingData.hpp"

using ::testing::AnyNumber;
using ::testing::AtLeast;
using ::testing::Ge;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::_;

using namespace ArduinoEnums;
using ArgoRcLib::ArgoRc;
using Hardware::ArduinoInterface;

ArgoRc createArgoRcLibObject(MockArduino &hardware)
{
  auto hardwarePtr = static_cast<ArduinoInterface *>(&hardware);
  return ArgoRc(hardwarePtr);
}

// ---------- Functions to setup various checks --------

void checkFootSwitchesAreOff(MockArduino &hardwareInterface)
{
  EXPECT_CALL(hardwareInterface, digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                              digitalIO::E_HIGH))
      .Times(1);

  EXPECT_CALL(
      hardwareInterface,
      digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkFootSwitchesAreOn(MockArduino &hardwareInterface)
{
  EXPECT_CALL(hardwareInterface, digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                              digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(
      hardwareInterface,
      digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_LOW))
      .Times(1);
}

void checkDirectionRelaysAreOff(MockArduino &hardwareInterface)
{
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

void checkForwardLeftIsOn(MockArduino &hardwareInterface)
{
  // Check the relays are switched on by dropping the circuit low
  checkFootSwitchesAreOn(hardwareInterface);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_FORWARD_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_REVERSE_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkForwardRightIsOn(MockArduino &hardwareInterface)
{
  // Check the relays are switched on by dropping the circuit low
  checkFootSwitchesAreOn(hardwareInterface);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkReverseLeftIsOn(MockArduino &hardwareInterface)
{
  // Check the relays are switched on by dropping the circuit low
  checkFootSwitchesAreOn(hardwareInterface);
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_REVERSE_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_FORWARD_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkReverseRightIsOn(MockArduino &hardwareInterface)
{
  checkFootSwitchesAreOn(hardwareInterface);
  // Check the relays are switched on by dropping the circuit low
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void returnDeadmanSafe(MockArduino &hardwareInterface)
{
  ON_CALL(hardwareInterface,
          digitalRead(pinMapping::RC_DEADMAN))
      .WillByDefault(Return(digitalIO::E_HIGH));
}

// ----- Setup Tests ------

TEST(ArgoRcLibSetup, configuresDigitalIO)
{
  NiceMock<MockArduino> hardwareMock;

  EXPECT_CALL(hardwareMock, serialBegin(Ge(1000))).Times(1);

  // Check all output Pins
  for (auto pin : ArduinoEnums::allDigitalPins)
  {
    EXPECT_CALL(hardwareMock, setPinMode(pin, _));
  }

  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.setup();
}

TEST(ArgoRcLibSetup, resetRelays)
{
  NiceMock<MockArduino> hardwareMock;

  // Ignore all other writes
  EXPECT_CALL(hardwareMock, digitalWrite(_, _)).Times(AnyNumber());

  checkFootSwitchesAreOff(hardwareMock);
  checkDirectionRelaysAreOff(hardwareMock);

  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.setup();
}

TEST(ArgoRcLibSetup, configuresIsr)
{
  NiceMock<MockArduino> hardwareMock;

  // Check ADC8-15 are set to inputs (all bits 0)
  EXPECT_CALL(hardwareMock, setPortBitmask(portMapping::E_DDRK, 0));

  // this is done by setting PCICR to PCIE2 bitmask to
  // enable interrupts
  EXPECT_CALL(hardwareMock, orPortBitmask(portMapping::E_PCICR, _));
  // And then enabling the specific pin triggers
  EXPECT_CALL(hardwareMock, setPortBitmask(portMapping::E_PCMSK2, _));

  // Trigger the call
  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.setup();
}

// ------ Direction Tests ---------

TEST(ArgoRcRuntime, forwardLeftSetsRelays)
{
  NiceMock<MockArduino> hardwareMock;
  auto argoRcLib = createArgoRcLibObject(hardwareMock);

  checkForwardLeftIsOn(hardwareMock);
  argoRcLib.forward_left();
}

TEST(ArgoRcRuntime, forwardRightSetsRelays)
{
  NiceMock<MockArduino> hardwareMock;
  auto argoRcLib = createArgoRcLibObject(hardwareMock);

  checkForwardRightIsOn(hardwareMock);
  argoRcLib.forward_right();
}

TEST(ArgoRcRuntime, reverseLeftSetRelays)
{
  NiceMock<MockArduino> hardwareMock;
  auto argoRcLib = createArgoRcLibObject(hardwareMock);

  checkReverseLeftIsOn(hardwareMock);
  argoRcLib.reverse_left();
}

TEST(ArgoRcRuntime, reverseRightSetsRelays)
{
  NiceMock<MockArduino> hardwareMock;
  auto argoRcLib = createArgoRcLibObject(hardwareMock);

  checkReverseRightIsOn(hardwareMock);
  argoRcLib.reverse_right();
}

// ----- Footswitch tests -------

TEST(ArgoRcRuntime, footSwitchOn)
{
  NiceMock<MockArduino> hardwareMock;
  checkFootSwitchesAreOn(hardwareMock);
  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.footswitch_on();
}

TEST(ArgoRcRuntime, footSwitchOff)
{
  NiceMock<MockArduino> hardwareMock;
  checkFootSwitchesAreOff(hardwareMock);
  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.footswitch_off();
}

// ------- Deadman switch tests ------
TEST(ArgoRcDeadman, deadmanSwitchTriggers)
{
  NiceMock<MockArduino> hardwareMock;
  auto argoRcLib = createArgoRcLibObject(hardwareMock);

  ON_CALL(hardwareMock,
          digitalRead(pinMapping::RC_DEADMAN))
      .WillByDefault(Return(digitalIO::E_LOW));

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0));
  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0));

  checkFootSwitchesAreOff(hardwareMock);
  checkDirectionRelaysAreOff(hardwareMock);

  EXPECT_CALL(hardwareMock, enterDeadmanSafetyMode());

  argoRcLib.loop();
}

// -------- PWM input tests

// This maps to -40 when the bounds are set from 1520-1850 : 0-255
const unsigned int reverseBoundValue = 1468;
// This maps to 40 when the bounds are set from 1520-1850 : 0-255
const unsigned int forwardBoundValue = 1572;
// This maps to 0
const unsigned int zeroValue = 1520;

const size_t leftPwmIndex = 0;
const size_t rightPwmIndex = 1;

TEST(ArgoRcPwmIn, pwmLeftIsForward)
{
  NiceMock<MockArduino> hardwareMock;
  auto argoRcLib = createArgoRcLibObject(hardwareMock);

  returnDeadmanSafe(hardwareMock);

  // Set timing data to the boundary value
  timingData::g_pinData[leftPwmIndex].lastGoodWidth = forwardBoundValue;
  timingData::g_pinData[rightPwmIndex].lastGoodWidth = zeroValue;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
