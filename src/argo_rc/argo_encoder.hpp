#ifndef ARGO_ENCODER_HPP_
#define ARGO_ENCODER_HPP_

#include <stdint.h>

#include "arduino_interface.hpp"
#include "encoder_interface.hpp"
#include "pair.hpp"
#include "unique_ptr.hpp"

namespace ArgoRcLib {

enum ArgoEncoderPositions { LEFT_ENCODER, RIGHT_ENCODER, _NUM_OF_ENCODERS };

struct EncoderData {
  int32_t encoderVal[ArgoEncoderPositions::_NUM_OF_ENCODERS];
};

class ArgoEncoder {
public:
  ArgoEncoder(Hardware::ArduinoInterface &hardwarePtr);
  ArgoEncoder(Hardware::ArduinoInterface &hardwarePtr,
              Argo::unique_ptr<EncoderLib::EncoderFactory> &&factoryObj);

  EncoderData read();

  void setEncoderPins(ArgoEncoderPositions targetEncoder,
                      const Argo::pair<ArduinoEnums::pinMapping,
                                       ArduinoEnums::pinMapping> &encoderPins);

  void write(ArgoEncoderPositions targetEncoder, int32_t val);

  static const uint8_t NUM_OF_ENCODERS = ArgoEncoderPositions::_NUM_OF_ENCODERS;

private:
  void checkEncoderIsValid(uint8_t encPosition);

  Hardware::ArduinoInterface &m_hardwareInterface;

  Argo::unique_ptr<EncoderLib::EncoderFactory> m_factoryObj;
  Argo::unique_ptr<EncoderLib::EncoderInterface> m_encoders[NUM_OF_ENCODERS];
};

} // Namespace ArgoRcLib

#endif // ARGO_ENCODER_HPP_