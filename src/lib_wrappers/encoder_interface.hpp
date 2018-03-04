#ifndef ENCODER_INTERFACE_HPP_
#define ENCODER_INTERFACE_HPP_

#include "arduino_enums.hpp"
#include "unique_ptr.hpp"

namespace EncoderLib {

class EncoderInterface {
public:
  virtual ~EncoderInterface() = default;

  virtual int32_t read() = 0;

  virtual void write(int32_t val) = 0;

protected:
  EncoderInterface() = default;
};

// This factory class was adapted from the idea by Bryan Chen
// https://stackoverflow.com/a/24295925 (Accessed: 2017-03-03)

using EncoderFactoryFunction = Argo::unique_ptr<EncoderInterface> (*)(
    ArduinoEnums::pinMapping pinOne, ArduinoEnums::pinMapping pinTwo);

class EncoderFactory {
public:
  EncoderFactory();
  EncoderFactory(EncoderFactoryFunction funcPtr);

  Argo::unique_ptr<EncoderInterface>
  createEncoder(ArduinoEnums::pinMapping pinOne,
                ArduinoEnums::pinMapping pinTwo) const;

private:
  EncoderFactoryFunction m_currentFactory;
};

} // Namespace EncoderLib

#endif // ENCODER_INTERFACE_HPP_