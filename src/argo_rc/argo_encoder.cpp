#include <stdint.h>

#include "argo_encoder.hpp"
#include "move.hpp"

using namespace EncoderLib;
using namespace ArduinoEnums;

namespace ArgoRcLib {

// Delegate onto the main constructor with the default factory function ptr
ArgoEncoder::ArgoEncoder(Hardware::ArduinoInterface &hardwarePtr)
    : ArgoEncoder(hardwarePtr, Argo::move(Argo::unique_ptr<EncoderFactory>(
                                   new EncoderFactory()))) {}

ArgoEncoder::ArgoEncoder(
    Hardware::ArduinoInterface &hardwarePtr,
    Argo::unique_ptr<EncoderLib::EncoderFactory> &&factoryObj)
    : m_hardwareInterface(hardwarePtr),
      m_factoryObj(Argo::move(factoryObj)), m_encoders{nullptr, nullptr} {}

EncoderData ArgoEncoder::read() {
  EncoderData readValues;

  // Start at the first element which is the left encoder
  for (uint8_t i = 0; i < static_cast<uint8_t>(_NUM_OF_ENCODERS); i++) {
    checkEncoderIsValid(i);
    readValues.encoderVal[i] = m_encoders[i]->read();
  }
  return readValues;
}

void ArgoEncoder::setEncoderPins(
    ArgoEncoderPositions targetEncoder,
    const Argo::pair<pinMapping, pinMapping> &encoderPins) {

  m_encoders[targetEncoder] =
      m_factoryObj->createEncoder(encoderPins.first, encoderPins.second);
}

void ArgoEncoder::write(ArgoEncoderPositions targetEncoder, int32_t val) {
  auto encoderIndex = static_cast<uint8_t>(targetEncoder);
  checkEncoderIsValid(encoderIndex);
  m_encoders[encoderIndex]->write(val);
}

// Private methods

void ArgoEncoder::checkEncoderIsValid(uint8_t encPosition) {
  if (!m_encoders[encPosition].get()) {
    m_hardwareInterface.serialPrint("The encoder at position: ");
    m_hardwareInterface.serialPrint(encPosition);
    m_hardwareInterface.serialPrint(" was not set.");
  }
}

} // Namespace ArgoRcLib