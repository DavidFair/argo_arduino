#ifndef ENCODER_HPP_
#define ENCODER_HPP_
#include <stdint.h>

#include "move.hpp"

#include "arduino_interface.hpp"

namespace Hardware {

enum ArgoEncoderPositions { LEFT_ENCODER, RIGHT_ENCODER, _NUM_OF_ENCODERS };

struct EncoderPulses {
  int32_t leftEncoderVal{0};
  int32_t rightEncoderVal{0};
};

struct WheelSpeeds {
  int32_t leftMmPerSec{0};
  int32_t rightMmPerSec{0};
};

class Encoder {
public:
  Encoder(ArduinoInterface &hardware);

  // Copy constructors
  Encoder(Encoder &other);
  Encoder &operator=(Encoder &other);

  // Move constructors
  Encoder(Encoder &&other);
  Encoder &operator=(Encoder &&other);

  EncoderPulses read() const;
  void reset() const;

private:
  void write(ArgoEncoderPositions targetEncoder, int32_t val) const;

  ArduinoInterface &m_hardware;
  unsigned long m_lastReadTime{0};

}; // namespace Hardware

} // namespace Hardware

#endif // ENCODER_HPP_