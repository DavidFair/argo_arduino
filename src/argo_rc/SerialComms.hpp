#ifndef SERIAL_COMMS_HPP
#define SERIAL_COMMS_HPP

#include "arduino_interface.hpp"
#include "argo_encoder.hpp"

namespace ArgoRcLib {

class SerialComms {
public:
  SerialComms(Hardware::ArduinoInterface &hardware);

  void sendEncoderRotation(const EncoderData &data);

private:
  void appendKVPair(const char *key, const char *value);
  void appendToOutputBuf(const char *s);

  void sendCurrentBuffer();

  // Set the output buffer to 100 characters this is resonable for all
  // commands
  static const uint8_t OUT_BUFFER_SIZE = 100;

  uint8_t m_currentIndex{0};
  char m_outBuffer[OUT_BUFFER_SIZE]{'\0'};

  Hardware::ArduinoInterface &m_hardwareInterface;
};

} // Namespace ArgoRcLib

#endif // SERIAL_COMMS_HPP