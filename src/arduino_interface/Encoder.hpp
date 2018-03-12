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
  Encoder(Hardware::ArduinoInterface &hardware);

  Encoder(Encoder &&other) : m_hardware(other.m_hardware) {}

  Encoder &operator=(Encoder &&other) {
    m_hardware = other.m_hardware;
    return *this;
  }

  EncoderData read() const;
  void write(ArgoEncoderPositions targetEncoder, int32_t val) const;

private:
  Hardware::ArduinoInterface &m_hardware;
}; // namespace Hardware

} // namespace Hardware

#endif // ENCODER_HPP_