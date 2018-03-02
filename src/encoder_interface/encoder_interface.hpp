#ifndef ENCODER_INTERFACE_HPP_
#define ENCODER_INTERFACE_HPP_

namespace Encoder {

class EncoderInterface {
public:
  virtual ~EncoderInterface() = default;

protected:
  EncoderInterface() = default;
};

} // Namespace Encoder

#endif // ENCODER_INTERFACE_HPP_