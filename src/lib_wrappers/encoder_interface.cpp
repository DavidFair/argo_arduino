#include "encoder_interface.hpp"

#include "arduino_enums.hpp"
#include "unique_ptr.hpp"

#ifdef UNIT_TESTING
#include <stdexcept>
#else
#include "arduino_hardware.hpp"
#include "encoder_impl.hpp"
#endif

using namespace ArduinoEnums;

namespace EncoderLib {

// Initialise the pointer for DI to nullptr
EncoderInterface *EncoderInterface::m_injectedMock = nullptr;

Argo::unique_ptr<EncoderInterface>
EncoderInterface::createEncoder(pinMapping pinOne, pinMapping pinTwo) {

#ifdef UNIT_TESTING
  // Unit testing impl
  if (!m_injectedMock) {
    throw std::runtime_error("The mock object was not injected before use");
  }

  // Supress unused warning for params as we are mocking
  (void)pinOne;
  (void)pinTwo;

  Argo::unique_ptr<EncoderInterface> mockedEncoder(m_injectedMock);
  m_injectedMock = nullptr;
  return mockedEncoder;
#else
  const auto pinOneNumber =
      Hardware::ArduinoHardware::convertPinEnumToArduino(pinOne);
  const auto pinTwoNumber =
      Hardware::ArduinoHardware::convertPinEnumToArduino(pinTwo);
  Argo::unique_ptr<EncoderInterface> argoPtr(static_cast<EncoderInterface *>(
      new EncoderImpl(pinOneNumber, pinTwoNumber)));
  return argoPtr;
#endif
}

} // namespace EncoderLib