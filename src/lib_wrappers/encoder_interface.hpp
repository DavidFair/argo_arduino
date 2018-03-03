#ifndef ENCODER_INTERFACE_HPP_
#define ENCODER_INTERFACE_HPP_

#include "arduino_enums.hpp"
#include "unique_ptr.hpp"

namespace EncoderLib {

class EncoderInterface {
public:
  virtual ~EncoderInterface() = default;

  static Argo::unique_ptr<EncoderInterface>
  createEncoder(ArduinoEnums::pinMapping pinOne,
                ArduinoEnums::pinMapping pinTwo);

  virtual int32_t read() = 0;

  virtual void write(int32_t val) = 0;

protected:
  EncoderInterface();

  static void injectMockDep(EncoderInterface *mockedObject) {
    m_injectedMock = mockedObject;
  }

private:
  static EncoderInterface *m_injectedMock;
};

} // Namespace EncoderLib

#endif // ENCODER_INTERFACE_HPP_