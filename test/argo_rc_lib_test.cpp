#include <gtest/gtest.h>

#include "argo_rc_lib.hpp"
#include "mock_arduino.hpp"

using ::testing::AtLeast;
using ::testing::Ge;
using ::testing::NiceMock;
using ::testing::_;

using namespace ArduinoEnums;
using ArgoRcLib::ArgoRc;
using Hardware::ArduinoInterface;

ArgoRc createArgoRcLibObject(MockArduino &hardware) {
  auto hardwarePtr = static_cast<ArduinoInterface *>(&hardware);
  return ArgoRc(hardwarePtr);
}

void checkFootSwitchesAreOff(const MockArduino &hardwareInterface) {
  EXPECT_CALL(hardwareInterface, digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                              digitalIO::E_HIGH));
  EXPECT_CALL(
      hardwareInterface,
      digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_HIGH));
}

void checkDirectionRelaysAreOff(const MockArduino &hardwareInterface) {
  // And the direction relays are switched off too
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::E_HIGH));
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::E_HIGH));
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_FORWARD_RELAY, digitalIO::E_HIGH));
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_REVERSE_RELAY, digitalIO::E_HIGH));
}

TEST(ArgoRcLib, setupConfiguresDigitalIO) {
  NiceMock<MockArduino> hardwareMock;

  EXPECT_CALL(hardwareMock, serialBegin(Ge(1000))).Times(1);

  // Check all output Pins
  for (auto pin : ArduinoEnums::allDigitalPins) {
    EXPECT_CALL(hardwareMock, setPinMode(pin, _));
  }

  // Check there is a state set to the test potentiometer
  EXPECT_CALL(hardwareMock, digitalWrite(pinMapping::TEST_POT_POSITIVE, _));

  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.setup();
}

TEST(ArgoRcLib, setupResetRelays) {
  NiceMock<MockArduino> hardwareMock;
  checkFootSwitchesAreOff(hardwareMock);
  checkDirectionRelaysAreOff(hardwareMock);

  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.setup();
}

TEST(ArgoRcLib, setupConfiguresIsr) {
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