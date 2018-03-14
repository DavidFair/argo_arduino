#ifndef ENCODER_HPP_
#define ENCODER_HPP_
#include <stdint.h>

#include "move.hpp"

#include "arduino_interface.hpp"

namespace Hardware {

enum ArgoEncoderPositions { LEFT_ENCODER, RIGHT_ENCODER, _NUM_OF_ENCODERS };

struct EncoderData {
  int32_t encoderVal[ArgoEncoderPositions::_NUM_OF_ENCODERS];
};

class Encoder {
public:
  EncoderData read() const;
  void reset() const;
  void write(ArgoEncoderPositions targetEncoder, int32_t val) const;

}; // namespace Hardware

} // namespace Hardware

#endif // ENCODER_HPP_