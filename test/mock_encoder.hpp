#ifndef MOCK_ENCODER_HPP_
#define MOCK_ENCODER_HPP_

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "arduino_enums.hpp"
#include "encoder_interface.hpp"

namespace Mocks {

class MockEncoder : public EncoderInterface {};

} // Namespace Mocks

#endif // MOCK_ENCODER_HPP_