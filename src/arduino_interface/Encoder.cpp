#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "Encoder.hpp"

using namespace Globals;

namespace Hardware {
EncoderData Encoder::read() const {
  EncoderData encoderData;
  encoderData.encoderVal[ArgoEncoderPositions::LEFT_ENCODER] =
      InterruptData::g_pinEncoderData.leftEncoderCount;

  encoderData.encoderVal[ArgoEncoderPositions::RIGHT_ENCODER] =
      InterruptData::g_pinEncoderData.rightEncoderCount;

  return encoderData;
}

void Encoder::reset() const {
  write(ArgoEncoderPositions::LEFT_ENCODER, 0);
  write(ArgoEncoderPositions::RIGHT_ENCODER, 0);
}

void Encoder::write(ArgoEncoderPositions targetEncoder, int32_t val) const {
  if (targetEncoder == ArgoEncoderPositions::LEFT_ENCODER) {
    InterruptData::g_pinEncoderData.leftEncoderCount = val;
  } else if (targetEncoder == ArgoEncoderPositions::RIGHT_ENCODER) {
    InterruptData::g_pinEncoderData.rightEncoderCount = val;
  }
}

} // namespace Hardware