#include <gtest/gtest.h>

#include "arduino_enums.hpp"
#include "argo_rc_lib.hpp"
#include "encoder_interface.hpp"
#include "mock_arduino.hpp"
#include "mock_encoder.hpp"
#include "move.hpp"
#include "pair.hpp"
#include "unique_ptr.hpp"


using ::testing::NiceMock;
using ::testing::Return;
using ::testing::StrictMock;
using ::testing::Test;

using namespace ArduinoEnums;
using namespace ArgoRcLib;
using namespace EncoderLib;
using namespace Mocks;
using namespace Hardware;

namespace { // Anonymous namespace

ArgoRc createArgoObject(EncoderFactoryFunction mockFunc) {
  // The function expects the expectations are in the factory function

  // Set deadman switch safe
  auto mockHardwarePtr = new NiceMock<MockArduino>;
  ON_CALL(*mockHardwarePtr, digitalRead(pinMapping::RC_DEADMAN))
      .WillByDefault(Return(digitalIO::E_HIGH));

  Argo::unique_ptr<ArduinoInterface> mockHardware(Argo::move(mockHardwarePtr));

  Argo::unique_ptr<EncoderFactory> encoderFactory(new EncoderFactory(mockFunc));

  // As ArgoRc takes ownership of the pointer to the mock object this is safe
  Argo::unique_ptr<ArgoEncoder> mockEncoder(
      new ArgoEncoder(*mockHardware, Argo::move(encoderFactory)));

  return ArgoRc(Argo::move(mockHardware), Argo::move(mockEncoder));
}

ArgoEncoder setupEncoderObj(MockArduino &mockHardware,
                            Argo::unique_ptr<EncoderFactory> &&mockFactory) {
  // We have to setup some pins for it to actually try to read. These
  // are aribtary in unit tests as the mocking framework will not use them

  Argo::pair<pinMapping, pinMapping> pinsToRead(pinMapping::LEFT_ENCODER_1,
                                                pinMapping::LEFT_ENCODER_2);

  ArgoEncoder encoderFixture(static_cast<ArduinoInterface &>(mockHardware),
                             Argo::move(mockFactory));

  encoderFixture.setEncoderPins(ArgoEncoderPositions::LEFT_ENCODER, pinsToRead);
  encoderFixture.setEncoderPins(ArgoEncoderPositions::RIGHT_ENCODER,
                                pinsToRead);

  return encoderFixture;
}

} // namespace

TEST(ArgoEncoderTest, loopReadsFromEncoder) {
  auto argoLib = createArgoObject([](pinMapping, pinMapping) {
    auto mock = new NiceMock<MockEncoder>();

    // Expect a call for each encoder once
    EXPECT_CALL(*mock, read()).Times(1);
    return Argo::unique_ptr<EncoderInterface>(mock);
  });

  argoLib.loop();
}

TEST(EncoderTest, readReturnsValue) {
  int32_t knownVal = 543;
  auto mockFactoryMethod = [](pinMapping, pinMapping) {
    // We have to duplicate the knownVal as capturing it turns this
    // from a function pointer to a lambda, which the AVR-GCC compiler
    // does not have the std::functional signature for
    int32_t knownVal = 543;

    auto mock = new StrictMock<MockEncoder>();

    EXPECT_CALL(*mock, read()).Times(1).WillOnce(Return(knownVal));
    return Argo::unique_ptr<EncoderInterface>(mock);
  };

  Argo::unique_ptr<EncoderFactory> mockFactory(
      new EncoderFactory(mockFactoryMethod));

  // We should not have any calls to the Arduino library
  StrictMock<MockArduino> mockHardware;
  auto encoderFixture = setupEncoderObj(mockHardware, Argo::move(mockFactory));

  auto returnedVal = encoderFixture.read();
  ASSERT_EQ(knownVal, returnedVal.encoderVal[0]);
}

TEST(EncoderTest, writeSetsValue) {
  const int32_t expectedWriteVal = 499;

  auto mockFactoryMethod = [](pinMapping, pinMapping) {
    auto mock = new StrictMock<MockEncoder>();

    EXPECT_CALL(*mock, write(expectedWriteVal)).Times(1);
    return Argo::unique_ptr<EncoderInterface>(mock);
  };

  Argo::unique_ptr<EncoderFactory> mockFactory(
      new EncoderFactory(mockFactoryMethod));

  StrictMock<MockArduino> mockHardware;

  auto encoderFixture = setupEncoderObj(mockHardware, Argo::move(mockFactory));

  // As the factory will have created 2 encoders we need to trigger both
  encoderFixture.write(ArgoEncoderPositions::LEFT_ENCODER, expectedWriteVal);
  encoderFixture.write(ArgoEncoderPositions::RIGHT_ENCODER, expectedWriteVal);
}