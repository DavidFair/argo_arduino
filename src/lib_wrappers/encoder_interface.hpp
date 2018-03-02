#ifndef ENCODER_INTERFACE_HPP_
#define ENCODER_INTERFACE_HPP_

#include "arduino_enums.hpp"

namespace Encoder {

class EncoderInterface {
public:
  virtual ~EncoderInterface() = default;

  static *EncoderInterface createEncoder(ArduinoEnums::pinMapping pinOne,
                                         ArduinoEnums::pinMapping pinTwo);

  virtual int32_t read() = 0;

  virtual void write(int32_t val) = 0;

protected:
  EncoderInterface();
};

} // Namespace Encoder

#endif // ENCODER_INTERFACE_HPP_