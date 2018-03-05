#include <gtest/gtest.h>
#include <string>

#include "argo_encoder.hpp"
#include "argo_rc_lib.hpp"
#include "encoder_interface.hpp"
#include "mock_arduino.hpp"
#include "mock_encoder.hpp"
#include "pinTimingData.hpp"
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
using namespace EncoderLib;
using namespace Mocks;
using Hardware::ArduinoInterface;

// Anonymous namespace
namespace {
Argo::unique_ptr<EncoderInterface> blankEncoderFactoryMethod(pinMapping,
                                                             pinMapping) {
  // Returns a default mock object with no expectations
  return Argo::unique_ptr<EncoderInterface>(new NiceMock<MockEncoder>());
}

Argo::unique_ptr<EncoderFactory> blankEncoderFactory() {
  // Creates a factory which embeds the default mock object creation
  return Argo::unique_ptr<EncoderFactory>(
      new EncoderFactory(&blankEncoderFactoryMethod));
}

Argo::unique_ptr<ArgoEncoder> createEncoderDep(MockArduino &hardwareMock) {
  // Passes the factory to ArgoEncoder to create instances on demand
  auto encoderFactory = blankEncoderFactory();
  Argo::unique_ptr<ArgoEncoder> newEncoder(
      new ArgoEncoder(static_cast<ArduinoInterface &>(hardwareMock),
                      Argo::move(encoderFactory)));
  return newEncoder;
}

class ArgoRcTest : public ::testing::Test {
protected:
  ArgoRcTest()
      : _forwardedPtr(new NiceMock<MockArduino>),
        hardwareMock(static_cast<NiceMock<MockArduino> &>(*_forwardedPtr)),
        argoRcLib(Argo::move(_forwardedPtr), createEncoderDep(hardwareMock)) {}

  // This pointer has its ownership transfered to ArgoRc however we still
  // hold a reference so we can set expectations
  Argo::unique_ptr<ArduinoInterface> _forwardedPtr;
  NiceMock<MockArduino> &hardwareMock;
  ArgoRc argoRcLib;
};

} // namespace

// ---------- Functions to setup various checks --------

void checkFootSwitchesAreOff(MockArduino &hardwareInterface) {
  EXPECT_CALL(hardwareInterface, digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                              digitalIO::E_HIGH))
      .Times(1);

  EXPECT_CALL(
      hardwareInterface,
      digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_HIGH))
      .Times(1);
}

void checkFootSwitchesAreOn(MockArduino &hardwareInterface) {
  EXPECT_CALL(hardwareInterface,
              digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY, digitalIO::E_LOW))
      .Times(1);

  EXPECT_CALL(
      hardwareInterface,
      digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_LOW))
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

TEST_F(ArgoRcTest, configuresIsr) {

  // Check ADC8-15 are set to inputs (all bits 0)
  EXPECT_CALL(hardwareMock, setPortBitmask(portMapping::E_DDRK, 0));

  // this is done by setting PCICR to PCIE2 bitmask to
  // enable interrupts
  EXPECT_CALL(hardwareMock, orPortBitmask(portMapping::E_PCICR, _));
  // And then enabling the specific pin triggers
  EXPECT_CALL(hardwareMock, setPortBitmask(portMapping::E_PCMSK2, _));

  // Trigger the call

  argoRcLib.setup();
}

// ------ Direction Tests ---------

TEST_F(ArgoRcTest, forwardLeftSetsRelays) {

  checkForwardLeftIsOn(hardwareMock);
  argoRcLib.forward_left();
}

TEST_F(ArgoRcTest, forwardRightSetsRelays) {

  checkForwardRightIsOn(hardwareMock);
  argoRcLib.forward_right();
}

TEST_F(ArgoRcTest, reverseLeftSetRelays) {

  checkReverseLeftIsOn(hardwareMock);
  argoRcLib.reverse_left();
}

TEST_F(ArgoRcTest, reverseRightSetsRelays) {

  checkReverseRightIsOn(hardwareMock);
  argoRcLib.reverse_right();
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

  ON_CALL(hardwareMock, digitalRead(pinMapping::RC_DEADMAN))
      .WillByDefault(Return(digitalIO::E_LOW));

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0));
  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0));

  checkFootSwitchesAreOff(hardwareMock);
  checkDirectionRelaysAreOff(hardwareMock);

  EXPECT_CALL(hardwareMock, enterDeadmanSafetyMode());

  argoRcLib.loop();
}

// -------- PWM input tests ----------

// This maps to -40 when the bounds are set from 1520-1850 : 0-255
const unsigned int reverseBoundValue = 1468;
// This maps to 40 when the bounds are set from 1520-1850 : 0-255
const unsigned int forwardBoundValue = 1572;
// This maps to 0
const unsigned int zeroValue = 1520;
const int mappedValue = 40;

const size_t leftPwmIndex = 0;
const size_t rightPwmIndex = 1;

TEST_F(ArgoRcTest, pwmLeftIsForward) {

  returnDeadmanSafe(hardwareMock);

  // Set timing data to the boundary value
  timingData::g_pinData[leftPwmIndex].lastGoodWidth = forwardBoundValue;
  timingData::g_pinData[rightPwmIndex].lastGoodWidth = zeroValue;

  checkForwardLeftIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, pwmRightIsForward) {

  returnDeadmanSafe(hardwareMock);

  // Set timing data to the boundary value
  timingData::g_pinData[rightPwmIndex].lastGoodWidth = forwardBoundValue;
  timingData::g_pinData[leftPwmIndex].lastGoodWidth = zeroValue;

  checkForwardRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, pwmLeftIsReverse) {

  returnDeadmanSafe(hardwareMock);

  // Set timing data to the boundary value
  timingData::g_pinData[leftPwmIndex].lastGoodWidth = reverseBoundValue;
  timingData::g_pinData[rightPwmIndex].lastGoodWidth = zeroValue;

  checkReverseLeftIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, pwmRightIsReverse) {

  returnDeadmanSafe(hardwareMock);

  // Set timing data to the boundary value
  timingData::g_pinData[leftPwmIndex].lastGoodWidth = zeroValue;
  timingData::g_pinData[rightPwmIndex].lastGoodWidth = reverseBoundValue;

  checkReverseRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, pwmPositiveOff) {

  returnDeadmanSafe(hardwareMock);

  // Set timing data to the boundary value
  timingData::g_pinData[leftPwmIndex].lastGoodWidth = forwardBoundValue - 1;
  timingData::g_pinData[rightPwmIndex].lastGoodWidth = forwardBoundValue - 1;

  // TODO : Make sure direction relays switch off
  // checkDirectionRelaysAreOff(hardwareMock);
  checkFootSwitchesAreOff(hardwareMock);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, pwmNegativeOff) {

  returnDeadmanSafe(hardwareMock);

  // Set timing data to the boundary value
  timingData::g_pinData[leftPwmIndex].lastGoodWidth = reverseBoundValue + 1;
  timingData::g_pinData[rightPwmIndex].lastGoodWidth = reverseBoundValue + 1;

  // TODO : Make sure direction relays switch off
  // checkDirectionRelaysAreOff(hardwareMock);
  checkFootSwitchesAreOff(hardwareMock);

  argoRcLib.loop();
}

// --------- Check Encoder data is printed over serial -------
// As we need to setup our own on call for the encoder mock don't use text
// fixtures
TEST(SerialComms, encoderDataIsSent) {
  const int32_t expectedVal = 123;

  // Create a factory that sets expectations for us
  auto factoryMethod = [](pinMapping, pinMapping) {
    auto mockedEncoder = new NiceMock<MockEncoder>();

    ON_CALL(*mockedEncoder, read()).WillByDefault(Return(expectedVal));
    return Argo::unique_ptr<EncoderInterface>(mockedEncoder);
  };

  // Wrap the factory and put it in an object ArgoRc understands
  Argo::unique_ptr<EncoderFactory> encoderFactory(
      new EncoderFactory(factoryMethod));

  auto mockArduino = new NiceMock<MockArduino>();
  Argo::unique_ptr<ArgoEncoder> encoderImpl =
      (new ArgoEncoder(static_cast<ArduinoInterface &>(*mockArduino),
                       std::move(encoderFactory)));

  // Ignore other calls to serialPrintln
  EXPECT_CALL(*mockArduino, serialPrintln(An<int>())).Times(AnyNumber());
  EXPECT_CALL(*mockArduino, serialPrintln(An<const std::string &>()))
      .Times(AnyNumber());

  const std::string expectedOutput("!D L_ENC_1:123 R_ENC_1:123 ");
  EXPECT_CALL(*mockArduino, serialPrintln(expectedOutput));

  ArgoRc testInstance(Argo::unique_ptr<ArduinoInterface>(mockArduino),
                      std::move(encoderImpl));

  testInstance.loop();
}