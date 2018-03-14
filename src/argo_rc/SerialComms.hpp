#ifndef SERIAL_COMMS_HPP
#define SERIAL_COMMS_HPP

#include "Encoder.hpp"
#include "arduino_interface.hpp"

namespace ArgoRcLib {

class SerialComms {
public:
  SerialComms(Hardware::ArduinoInterface &hardware);

  void sendEncoderRotation(const Hardware::EncoderPulses &data);

  void sendVehicleSpeed(const Hardware::WheelSpeeds &speeds);

private:
  void appendKVPair(const char *key, const char *value);

  void appendToOutputBuf(const char c);
  void appendToOutputBuf(const char *s);

  void convertValue(char *buf, int bufSize, int32_t val);

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