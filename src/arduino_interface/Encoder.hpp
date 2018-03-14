#ifndef ENCODER_HPP_
#define ENCODER_HPP_
#include <stdint.h>

#include "move.hpp"

#include "Speed.hpp"
#include "arduino_interface.hpp"

namespace Hardware {

enum EncoderPositions { LEFT_ENCODER, RIGHT_ENCODER, _NUM_OF_ENCODERS };

struct EncoderPulses {
  int32_t leftEncoderVal{0};
  int32_t rightEncoderVal{0};
};

struct WheelSpeeds {
  Libs::Speed leftWheel;
  Libs::Speed rightWheel;
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

  WheelSpeeds calculateSpeed();

  EncoderPulses read() const;
  void reset() const;

private:
  void write(EncoderPositions targetEncoder, int32_t val) const;

  ArduinoInterface &m_hardware;
  unsigned long m_lastReadTime{0};
  EncoderPulses m_lastEncValues;

}; // namespace Hardware

} // namespace Hardware

#endif // ENCODER_HPP_