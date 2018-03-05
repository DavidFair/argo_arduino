#ifndef ENCODER_IMPL_HPP_
#define ENCODER_IMPL_HPP_

#include <Encoder.h>

#include "arduino_enums.hpp"
#include "encoder_interface.hpp"
#include "unique_ptr.hpp"

namespace EncoderLib {

class EncoderImpl : public EncoderInterface {
public:
  EncoderImpl(uint8_t pinOne, uint8_t pinTwo)
      : m_encoder(new Encoder(pinOne, pinTwo)) {}

  virtual ~EncoderImpl() = default;

  virtual int32_t read() override { return m_encoder->read(); }

  virtual void write(int32_t val) override { m_encoder->write(val); }

private:
  Argo::unique_ptr<Encoder> m_encoder;
};

} // namespace EncoderLib

#endif // ENCODER_IMPL_HPP_