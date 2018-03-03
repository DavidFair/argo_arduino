#ifndef MOCK_ENCODER_HPP_
#define MOCK_ENCODER_HPP_

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stdint.h>

#include "arduino_enums.hpp"
#include "encoder_interface.hpp"

namespace Mocks {

class MockEncoder : public EncoderLib::EncoderInterface {
public:
  MOCK_METHOD0(read, int32_t());
  MOCK_METHOD1(write, void(int32_t));

  void injectMockObj() { injectMockDep(this); }
};

} // Namespace Mocks

#endif // MOCK_ENCODER_HPP_