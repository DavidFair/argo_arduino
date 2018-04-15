#ifndef ENCODER_HPP_
#define ENCODER_HPP_
#include <stdint.h>

#include "move.hpp"

#include "ArduinoInterface.hpp"
#include "Speed.hpp"

namespace Hardware {

/// Enum which represents the left and right encoders
enum EncoderPositions { LEFT_ENCODER, RIGHT_ENCODER, _NUM_OF_ENCODERS };

/// Struct which holds the number of encoder pulses at a given point
struct EncoderPulses {
  int32_t leftEncoderVal{0};
  int32_t rightEncoderVal{0};
};

/// Struct which holds the current speed at a given point
struct WheelSpeeds {
  WheelSpeeds() : leftWheel(), rightWheel() {}
  WheelSpeeds(Libs::Speed left, Libs::Speed right)
      : leftWheel(left), rightWheel(right) {}

  Libs::Speed leftWheel;
  Libs::Speed rightWheel;
};

/// Class which handles getting and setting encoder pulses from the global
class Encoder {
public:
  /// Creates a new encoder object which interfaces with the reference hardware
  Encoder(ArduinoInterface &hardware);

  // Copy constructors
  Encoder(Encoder &other);
  Encoder &operator=(Encoder &other);

  // Move constructors
  Encoder(Encoder &&other);
  Encoder &operator=(Encoder &&other);

  /// Calculates the current speed based on the time since last call
  WheelSpeeds calculateSpeed();

  /// Returns the raw encoder pulse counts
  EncoderPulses read() const;

  /// Resets the encoder pulse counts
  void reset() const;

private:
  /// Writes a given value to the target encoder
  void write(EncoderPositions targetEncoder, int32_t val) const;

  /// Reference to the Arduino hardware
  ArduinoInterface &m_hardware;
  /// Holds the time since the encoders pulses were last read
  unsigned long m_lastReadTime{0};
  /// Holds the value of the encoders on last read
  EncoderPulses m_lastEncValues;

  // Milliseconds between each speed calculation
  /// The minimum time between calculating a new speed or returning
  /// the previous speed instead
  const static uint8_t m_minTimeBetweenSpeedCalc{100};
  /// Holds the previously calculated speed
  WheelSpeeds m_previousCalcSpeeds;

}; // namespace Hardware

} // namespace Hardware

#endif // ENCODER_HPP_