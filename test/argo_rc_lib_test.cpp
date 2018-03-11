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
// Mock object creation helpers
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
        argoRcLib(Argo::move(_forwardedPtr), createEncoderDep(hardwareMock)) {
    returnDeadmanSafe(hardwareMock);
  }

  // This pointer has its ownership transfered to ArgoRc however we still
  // hold a reference so we can set expectations
  Argo::unique_ptr<ArduinoInterface> _forwardedPtr;
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

// This maps to -40 when the bounds are set from 1520-1850 : 0-255
const unsigned int negFortyBounds = 1469;
// This maps to 40 when the bounds are set from 1520-1850 : 0-255
const unsigned int posFortyBounds = 1572;

const unsigned int zeroValue = 1520;
const unsigned int minValue = 1190;
const unsigned int maxValue = 1850;

const int mappedValue = 40;

const size_t throttlePwmIndex = 0;
const size_t steeringPwmIndex = 1;

TEST_F(ArgoRcTest, throttleIsForward) {
  // Set timing data to the boundary value
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = posFortyBounds;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = zeroValue;

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
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = negFortyBounds;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = zeroValue;

  checkReverseLeftIsOn(hardwareMock);
  checkReverseRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, -mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, -mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, leftReverse) {
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = negFortyBounds;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = minValue;

  checkReverseRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::LEFT_PWM_OUTPUT, 0))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, -2 * mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, rightReverse) {
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = negFortyBounds;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = maxValue;

  checkReverseLeftIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, -2 * mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock, analogWrite(pinMapping::RIGHT_PWM_OUTPUT, 0))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, rightForward) {
  // Set timing data to the boundary value
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = posFortyBounds;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = minValue;

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
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = posFortyBounds;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = maxValue;

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
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = negFortyBounds + 1;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = negFortyBounds + 1;

  checkDirectionRelaysAreOff(hardwareMock);
  checkFootSwitchesAreOff(hardwareMock);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, pwmPositiveOff) {
  // Set timing data to the boundary value
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = posFortyBounds - 1;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = posFortyBounds - 1;

  checkDirectionRelaysAreOff(hardwareMock);
  checkFootSwitchesAreOff(hardwareMock);

  argoRcLib.loop();
}

// ---------------- PWM turn on the spot ------------
TEST_F(ArgoRcTest, leftTurnOnSpot) {
  // Set timing data to the boundary value
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = zeroValue;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = negFortyBounds;

  checkReverseLeftIsOn(hardwareMock);
  checkForwardRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, -mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, mappedValue))
      .Times(1);

  argoRcLib.loop();
}

TEST_F(ArgoRcTest, rightTurnOnSpot) {
  // Set timing data to the boundary value
  timingData::g_pinData[throttlePwmIndex].lastGoodWidth = zeroValue;
  timingData::g_pinData[steeringPwmIndex].lastGoodWidth = posFortyBounds;

  checkForwardLeftIsOn(hardwareMock);
  checkReverseRightIsOn(hardwareMock);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::LEFT_PWM_OUTPUT, mappedValue))
      .Times(1);

  EXPECT_CALL(hardwareMock,
              analogWrite(pinMapping::RIGHT_PWM_OUTPUT, -mappedValue))
      .Times(1);

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

  // Set the deadman switch to safe
  returnDeadmanSafe(*mockArduino);

  const std::string expectedOutput("!D L_ENC_1:123 R_ENC_1:123 ");
  EXPECT_CALL(*mockArduino, serialPrintln(expectedOutput));

  ArgoRc testInstance(Argo::unique_ptr<ArduinoInterface>(mockArduino),
                      std::move(encoderImpl));

  testInstance.loop();
}