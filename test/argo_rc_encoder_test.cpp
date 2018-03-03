#include <gtest/gtest.h>
#include <stdexcept>

#include "argo_rc_lib.hpp"
#include "encoder_interface.hpp"
#include "mock_arduino.hpp"
#include "mock_encoder.hpp"
#include "unique_ptr.hpp"

using ::testing::NiceMock;

using namespace Mocks;
using namespace Hardware;
using ArgoRcLib::ArgoRc;

static ArgoRc createArgoRcLibObject(MockArduino &hardware) {
  // Inject the current mock object to the createEncoder method

  auto hardwarePtr = static_cast<ArduinoInterface *>(&hardware);
  return ArgoRc(hardwarePtr);
}

static Argo::unique_ptr<MockEncoder> injectMockEncoder() {
  Argo::unique_ptr<MockEncoder> mockEncoder(new MockEncoder());

  // Inject the current mock object to be used
  mockEncoder->injectMockObj();
  return mockEncoder;
}

TEST(ArgoRcEncoder, loopReadsFromEncoder) {
  NiceMock<MockArduino> hardwareMock;

  auto argoObj = createArgoRcLibObject(hardwareMock);
  auto encoderMock = injectMockEncoder();

  // Expect a call for each encoder
  EXPECT_CALL(*encoderMock, read()).Times(2);
}