#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "Encoder.hpp"

using namespace Globals;

namespace Hardware {

Encoder::Encoder(ArduinoInterface &hardware) : m_hardware(hardware) {}

EncoderData Encoder::read() const {
  EncoderData encoderData;
  encoderData.encoderVal[ArgoEncoderPositions::LEFT_ENCODER] =
      InterruptData::g_pinEncoderData.leftEncoderCount;

  encoderData.encoderVal[ArgoEncoderPositions::RIGHT_ENCODER] =
      InterruptData::g_pinEncoderData.rightEncoderCount;

  return encoderData;
}

void write(ArgoEncoderPositions targetEncoder, int32_t val) {
  (void)targetEncoder;
  (void)val;
}

} // namespace Hardware