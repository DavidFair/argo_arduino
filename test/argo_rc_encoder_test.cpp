#include <gtest/gtest.h>
#include <stdexcept>

#include "argo_rc_lib.hpp"
#include "encoder_interface.hpp"
#include "mock_arduino.hpp"
#include "mock_encoder.hpp"
#include "unique_ptr.hpp"

using ::testing::NiceMock;
using ::testing::Test;

using namespace Mocks;
using namespace Hardware;
using ArgoRcLib::ArgoRc;

namespace { // Anonymous namespace

class EncoderTest : public ::testing::Test {
protected:
  EncoderTest()
      : hardwareMock(), argoLib(static_cast<ArduinoInterface *>(&hardwareMock)),
        encoderMock(new MockEncoder()){};

  virtual void SetUp() { encoderMock->injectMockObj(); }

  MockArduino hardwareMock;
  ArgoRc argoLib;
  Argo::unique_ptr<MockEncoder> encoderMock{nullptr};
};

} // namespace

TEST_F(EncoderTest, loopReadsFromEncoder) {

  // Expect a call for each encoder
  EXPECT_CALL(*encoderMock, read()).Times(2);
}