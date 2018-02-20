#include <gtest/gtest.h>

#include "argo_rc_lib.hpp"
#include "mock_arduino.hpp"

using ::testing::AtLeast;
using ::testing::_;

using ArgoRcLib::ArgoRc;

class ArgoRcTest : public testing::Test {
public:
  MockArduino m_hardwareMock;
  ArgoRc m_argoRc;

  virtual void SetUp() { m_argoRc.setup(&m_hardwareMock); }
};

TEST_F(ArgoRcTest, setupConfiguresDeviceCorrectly) {
  EXPECT_CALL(m_hardwareMock, serialBegin(_)).Times(1);

  // Trigger the call
  m_argoRc.setup(&m_hardwareMock);
}