#ifndef SERIAL_COMMS_HPP
#define SERIAL_COMMS_HPP

#include "Encoder.hpp"
#include "MinString.hpp"
#include "arduino_interface.hpp"
#include "move.hpp"
#include "pair.hpp"

namespace ArgoRcLib {

enum class CommandType { NONE, PING, SPEED };

class SerialComms {
public:
  SerialComms(Hardware::ArduinoInterface &hardware);

  SerialComms(SerialComms &) = delete;
  SerialComms &operator=(SerialComms &) = delete;

  SerialComms(SerialComms &&other)
      : m_hardwareInterface(other.m_hardwareInterface){};

  SerialComms &operator=(SerialComms &&other) {
    m_hardwareInterface = other.m_hardwareInterface;
    return *this;
  }

  // Write methods
  void addEncoderRotation(const Hardware::EncoderPulses &data);

  void addPing();

  void addVehicleSpeed(const Hardware::WheelSpeeds &speeds);

  // Read methods
  Hardware::WheelSpeeds getTargetSpeeds() { return m_currentTargetSpeeds; }

  // Buffer Management
  void parseIncomingBuffer();
  void sendCurrentBuffer();

private:
  void appendKVPair(const char *key, const char *value);

  void appendToOutputBuf(const char c);
  void appendToOutputBuf(const Libs::MinString &s);

  bool convertBufStrToInt(uint8_t startingPos, int &result);
  void convertValue(char *buf, int bufSize, int32_t val);

  void findInputCommands();

  void parseTargetSpeed(const Libs::pair<uint8_t, uint8_t> &charRange);

  void
  processIndividualCommand(const Libs::pair<uint8_t, uint8_t> &charPosition);

  void resetBuffer(char *targetBuffer, uint8_t bufferSize);

  // Set the output buffer to 64 characters this is reasonable for all
  // commands and matches the Arduino buffer
  static const uint8_t BUFFER_SIZE = 64;

  // Buffer management
  uint8_t m_outIndex{0};
  uint8_t m_inputIndex{0};
  char m_outBuffer[BUFFER_SIZE]{'\0'};
  char m_inputBuffer[BUFFER_SIZE]{'\0'};

  // Previous results
  Hardware::WheelSpeeds m_currentTargetSpeeds;

  Hardware::ArduinoInterface &m_hardwareInterface;
};

} // Namespace ArgoRcLib

#endif // SERIAL_COMMS_HPP