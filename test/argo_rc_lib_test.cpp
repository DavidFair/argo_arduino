#include <gtest/gtest.h>

#include "argo_rc_lib.hpp"
#include "mock_arduino.hpp"

using ::testing::AnyNumber;
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

void checkFootSwitchesAreOff(MockArduino &hardwareInterface) {
  EXPECT_CALL(hardwareInterface, digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                              digitalIO::E_HIGH))
      .Times(1);

  EXPECT_CALL(
      hardwareInterface,
      digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_HIGH))
      .Times(1);
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

TEST(ArgoRcLibSetup, configuresDigitalIO) {
  NiceMock<MockArduino> hardwareMock;

  EXPECT_CALL(hardwareMock, serialBegin(Ge(1000))).Times(1);

  // Check all output Pins
  for (auto pin : ArduinoEnums::allDigitalPins) {
    EXPECT_CALL(hardwareMock, setPinMode(pin, _));
  }

  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.setup();
}

TEST(ArgoRcLibSetup, resetRelays) {
  NiceMock<MockArduino> hardwareMock;

  // Ignore all other writes
  EXPECT_CALL(hardwareMock, digitalWrite(_, _)).Times(AnyNumber());

  checkFootSwitchesAreOff(hardwareMock);
  checkDirectionRelaysAreOff(hardwareMock);

  auto argoRcLib = createArgoRcLibObject(hardwareMock);
  argoRcLib.setup();
}

TEST(ArgoRcLibSetup, configuresIsr) {
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

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
