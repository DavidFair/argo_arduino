#ifndef SERIAL_COMMS_HPP
#define SERIAL_COMMS_HPP

#include "ArduinoInterface.hpp"
#include "Encoder.hpp"
#include "MinString.hpp"
#include "move.hpp"
#include "pair.hpp"

namespace ArgoRcLib {

// Forward declarations
struct PwmTargets;

/// Class that implements handling serial communications
class SerialComms {
public:
  /// Constructs a new object with the given Arduino hardware
  SerialComms(Hardware::ArduinoInterface &hardware);

  // Copy constructors
  SerialComms(SerialComms &) = delete;
  SerialComms &operator=(SerialComms &) = delete;

  // Move constructors
  SerialComms(SerialComms &&other)
      : m_hardwareInterface(other.m_hardwareInterface){};
  SerialComms &operator=(SerialComms &&other) {
    m_hardwareInterface = other.m_hardwareInterface;
    return *this;
  }

  // Write methods
  /// Adds the given encoder rotation to the outgoing buffer
  void addEncoderRotation(const Hardware::EncoderPulses &data);
  /// Adds a ping to the outgoing buffer
  void addPing();
  /// Adds PWM values to the outgoing buffer
  void addPwmValues(const PwmTargets &targetVals);
  /// Adds the given wheel speeds to the outgoing buffer
  void addVehicleSpeed(const Hardware::WheelSpeeds &speeds);
  /// Adds a warning from a string literal to the outgoing buffer
  void addWarning(const char *warningText);

  /// Calculates a checksum for a given message in the buffer
  uint8_t calcMsgChecksum(const char *msgStart, const char *msgEnd) const;

  // Read methods
  /// Returns the last parsed target wheel speeds
  Hardware::WheelSpeeds getTargetSpeeds() const {
    return m_currentTargetSpeeds;
  }

  /// Returns whether a ping has been received within the elapsed time
  bool isPingGood() const;

  // Buffer Management
  /// Reads and parses any incoming commands over serial
  void parseIncomingBuffer();
  /// Sends the current outgoing buffer over serial
  void sendCurrentBuffer();

private:
  /// Appends a checksum value to the end of the outgoing buffer position
  void appendChecksumValue(const int commandStart, const int commandEnd);

  /// Appends a key and value pair to the outgoing buffer
  void appendKVPair(const char *key, const char *value);

  /// Appends a single character to the outgoing buffer
  void appendToOutputBuf(const char c);
  /// Appends a MinString to the current outgoing buffer
  void appendToOutputBuf(const Libs::MinString &s);

  /// Checks an incoming command's checksum
  bool checkIncomingChecksum(const Libs::pair<uint8_t, uint8_t> &commandRange);

  /// Converts a value in the incoming buffer into an integer
  bool convertBufStrToInt(uint8_t startingPos, int &result);
  /// Converts a value into a char array whose buffer provided by the caller
  void convertValue(char *buf, int bufSize, int32_t val);
  /// Finds any input command in the incoming buffer and parses them
  void findInputCommands();
  /// Parses a new target speed at the given position in the buffer
  void parseTargetSpeed(const Libs::pair<uint8_t, uint8_t> &charRange);
  /// Splits and processes internal commands within the internal buffer
  void
  processIndividualCommand(const Libs::pair<uint8_t, uint8_t> &charPosition);
  /// Resets the target buffer to null characters
  void resetBuffer(char *targetBuffer, uint8_t bufferSize);

  // Set the output buffer to 100 characters which should be reasonable
  /// Size of the input and output buffers
  static const uint8_t BUFFER_SIZE = 100;

  // Buffer management
  /// Tracks the current position of the output buffer
  uint8_t m_outIndex{0};
  /// Tracks the current position of the input buffer
  uint8_t m_inputIndex{0};
  /// The output serial buffer
  char m_outBuffer[BUFFER_SIZE]{'\0'};
  /// The input serial buffer
  char m_inputBuffer[BUFFER_SIZE]{'\0'};

  // Previous results
  /// Holds the most recent target wheel speeds
  Hardware::WheelSpeeds m_currentTargetSpeeds;
  /// Holds a reference to the Arduino hardware
  Hardware::ArduinoInterface &m_hardwareInterface;

  /// Holds the time that the last ping was received at
  unsigned long m_lastPingTime;
};

} // Namespace ArgoRcLib

#endif // SERIAL_COMMS_HPP