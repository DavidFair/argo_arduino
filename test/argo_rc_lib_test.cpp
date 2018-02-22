#include <gtest/gtest.h>

#include "argo_rc_lib.hpp"
#include "mock_arduino.hpp"

using ::testing::AtLeast;
using ::testing::Ge;
using ::testing::_;

using namespace ArduinoEnums;
using ArgoRcLib::ArgoRc;

class ArgoRcTest : public testing::Test {
public:
  MockArduino m_hardwareMock;
  ArgoRc m_argoRc;

  // virtual void SetUp() { m_argoRc.setup(&m_hardwareMock); }
};

TEST_F(ArgoRcTest, setupConfiguresDevice) {
  EXPECT_CALL(m_hardwareMock, serialBegin(Ge(1000))).Times(1);

  // Output Pins
  EXPECT_CALL(m_hardwareMock,
              setPinMode(pinMapping::LEFT_FORWARD_RELAY, digitalIO::E_OUTPUT))
      .Times(1);
  EXPECT_CALL(m_hardwareMock,
              setPinMode(pinMapping::LEFT_REVERSE_RELAY, digitalIO::E_OUTPUT))
      .Times(1);

  EXPECT_CALL(m_hardwareMock,
              setPinMode(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::E_OUTPUT))
      .Times(1);
  EXPECT_CALL(m_hardwareMock,
              setPinMode(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::E_OUTPUT))
      .Times(1);

  EXPECT_CALL(m_hardwareMock, setPinMode(pinMapping::LEFT_FOOTSWITCH_RELAY,
                                         digitalIO::E_OUTPUT))
      .Times(1);
  EXPECT_CALL(m_hardwareMock, setPinMode(pinMapping::RIGHT_FOOTSWITCH_RELAY,
                                         digitalIO::E_OUTPUT))
      .Times(1);

  EXPECT_CALL(m_hardwareMock,
              setPinMode(pinMapping::TEST_POT_POSITIVE, digitalIO::E_OUTPUT))
      .Times(1);

  // Input Pullup Pins
  EXPECT_CALL(m_hardwareMock,
              setPinMode(pinMapping::LEFT_ENCODER_1, digitalIO::E_INPUT_PULLUP))
      .Times(1);
  EXPECT_CALL(m_hardwareMock,
              setPinMode(pinMapping::LEFT_ENCODER_2, digitalIO::E_INPUT_PULLUP))
      .Times(1);

  EXPECT_CALL(m_hardwareMock, setPinMode(pinMapping::RIGHT_ENCODER_1,
                                         digitalIO::E_INPUT_PULLUP))
      .Times(1);

  EXPECT_CALL(m_hardwareMock, setPinMode(pinMapping::RIGHT_ENCODER_2,
                                         digitalIO::E_INPUT_PULLUP))
      .Times(1);

  // Trigger the call
  m_argoRc.setup(&m_hardwareMock);
}