#include <gtest/gtest.h>
#include <stdexcept>

#include "argo_rc_lib.hpp"
#include "encoder_interface.hpp"
#include "mock_arduino.hpp"
#include "mock_encoder.hpp"
#include "unique_ptr.hpp"

using ::testing::NiceMock;
using ::testing::Test;

using namespace ArduinoEnums;
using namespace ArgoRcLib;
using namespace EncoderLib;
using namespace Mocks;
using namespace Hardware;

namespace { // Anonymous namespace

ArgoRc createArgoObject(EncoderFactoryFunction mockFunc) {
  // At this point of call the expectations should have been set on the mock
  Argo::unique_ptr<ArduinoInterface> mockHardware(new MockArduino);
  Argo::unique_ptr<EncoderFactory> encoderFactory(new EncoderFactory(mockFunc));

  // As ArgoRc takes ownership of the pointer to the mock object this is safe
  Argo::unique_ptr<ArgoEncoder> mockEncoder(
      new ArgoEncoder(*mockHardware, Argo::move(encoderFactory)));

  return ArgoRc(Argo::move(mockHardware), Argo::move(mockEncoder));
}

} // namespace

TEST(EncoderTest, loopReadsFromEncoder) {
  auto argoLib = createArgoObject([](pinMapping, pinMapping) {
    auto mock = new NiceMock<MockEncoder>();

    // Expect a call for each encoder once
    EXPECT_CALL(*mock, read()).Times(1);
    return Argo::unique_ptr<EncoderInterface>(mock);
  });

  argoLib.loop();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
