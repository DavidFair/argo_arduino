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
using namespace EncoderLib;

// Anonymous namespace

namespace {
// This holds the method used when we are on actual Arduino Hardware
Argo::unique_ptr<EncoderInterface> createEncoderOnHardware(pinMapping pinOne,
                                                           pinMapping pinTwo) {

#ifdef UNIT_TESTING
  // Mark params as used
  (void)pinOne;
  (void)pinTwo;

  throw std::runtime_error(
      "An implementation should be provided for the encoder factory");

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
} // namespace

namespace EncoderLib {

EncoderFactory::EncoderFactory() : m_currentFactory(&createEncoderOnHardware) {}

EncoderFactory::EncoderFactory(EncoderFactoryFunction funcPtr)
    : m_currentFactory(funcPtr) {}

Argo::unique_ptr<EncoderInterface>
EncoderFactory::createEncoder(pinMapping pinOne, pinMapping pinTwo) const {
  return m_currentFactory(pinOne, pinTwo);
}

} // namespace EncoderLib