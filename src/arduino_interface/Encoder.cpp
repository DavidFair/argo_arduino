#include <stdint.h>

#include "ArduinoGlobals.hpp"
#include "Length.hpp"

#include "Encoder.hpp"

using namespace Globals;

namespace {
// constexpr int

} // End of anonymous namespace

namespace Hardware {
EncoderPulses Encoder::read() const {
  EncoderPulses encoderData;
  encoderData.leftEncoderVal = InterruptData::g_pinEncoderData.leftEncoderCount;

  encoderData.rightEncoderVal =
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