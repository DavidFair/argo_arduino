#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>

#include "Distance.hpp"
#include "Encoder.hpp"
#include "MinString.hpp"
#include "Speed.hpp"
#include "arduino_interface.hpp"
#include "cstring_wrapper.hpp"

#include "SerialComms.hpp"

using namespace Hardware;
using namespace Libs;

/* We use char * arrays instead of Strings on the Arduino. This
 * helps reduce memory fragmentation and overhead associated with
 * strings, and keeps a consistent interface with the unit tests too. */
namespace {

// Whitespace seperates various data fields we are transmitting
// A ':' character indicates a key - value pair
constexpr char K_V_SEPERATOR = ':';
constexpr char SEPERATOR = ' ';
constexpr char EOL = '\n';

constexpr MinString DATA_RECIEVE_PREFIX = "!C ";
constexpr MinString DATA_TRANSMIT_PREFIX = "!D ";

// Function specific data
constexpr int NUM_ENCODER = EncoderPositions::_NUM_OF_ENCODERS;

constexpr MinString ENCODER_NAMES[NUM_ENCODER] = {"L_ENC", "R_ENC"};
constexpr MinString SPEED_PREFIX[NUM_ENCODER] = {"L_SPEED", "R_SPEED"};

} // namespace

namespace ArgoRcLib {

SerialComms::SerialComms(Hardware::ArduinoInterface &hardware)
    : m_currentTargetSpeeds(), m_hardwareInterface(hardware) {}

void SerialComms::addEncoderRotation(const EncoderPulses &data) {
  // Prepare our output buffer - prepend that we are sending data
  appendToOutputBuf(DATA_TRANSMIT_PREFIX);

  // The maximum number of digits in either encoder output
  constexpr int NUM_DEC_PLACES = 6;
  char convertedNumber[NUM_DEC_PLACES]{0};

  // Convert each number and forward as a K-V pair
  convertValue(convertedNumber, NUM_DEC_PLACES, data.leftEncoderVal);
  appendKVPair(ENCODER_NAMES[EncoderPositions::LEFT_ENCODER].str(),
               convertedNumber);

  convertValue(convertedNumber, NUM_DEC_PLACES, data.rightEncoderVal);
  appendKVPair(ENCODER_NAMES[EncoderPositions::RIGHT_ENCODER].str(),
               convertedNumber);
  appendToOutputBuf(EOL);
}

void SerialComms::addVehicleSpeed(const Hardware::WheelSpeeds &speeds) {
  appendToOutputBuf(DATA_TRANSMIT_PREFIX);

  constexpr int NUM_DEC_PLACES = 10;
  char convertedNumber[NUM_DEC_PLACES];

  convertValue(convertedNumber, NUM_DEC_PLACES,
               speeds.leftWheel.getUnitDistance().millimeters());
  appendKVPair(SPEED_PREFIX[EncoderPositions::LEFT_ENCODER].str(),
               convertedNumber);

  convertValue(convertedNumber, NUM_DEC_PLACES,
               speeds.rightWheel.getUnitDistance().millimeters());
  appendKVPair(SPEED_PREFIX[EncoderPositions::RIGHT_ENCODER].str(),
               convertedNumber);
  appendToOutputBuf(EOL);
}

void SerialComms::parseIncomingBuffer() {
  while (m_hardwareInterface.serialAvailable() > 0) {
    if ((m_inputIndex + 1) >= BUFFER_SIZE) {
      m_hardwareInterface.serialPrintln(
          "Input buffer not large enough - ignoring");
      m_inputIndex = 0;
      return;
    }

    m_inputBuffer[m_inputIndex++] = m_hardwareInterface.serialRead();
  }

  bool containsEOLChar = (strchr(m_inputBuffer, EOL) != NULL);

  if (m_inputIndex > 0 && containsEOLChar) {
    findInputCommands();
    resetBuffer(m_inputBuffer, BUFFER_SIZE);
    m_inputIndex = 0;
  }
}

// --------- Private methods ------------

void SerialComms::appendKVPair(const char *key, const char *value) {
  appendToOutputBuf(key);
  appendToOutputBuf(K_V_SEPERATOR);
  appendToOutputBuf(value);
  appendToOutputBuf(SEPERATOR);
}

void SerialComms::appendToOutputBuf(const char c) {
  // Wrap in null terminated array and forward
  const char toAppend[2] = {c, '\0'};
  appendToOutputBuf(toAppend);
}

void SerialComms::appendToOutputBuf(const MinString &s) {
  auto sLength = s.length();
  if (sLength + m_outIndex >= BUFFER_SIZE) {
    // Flush the current buffer regardless
    sendCurrentBuffer();
  }
  strcat(m_outBuffer, s.str());
  m_outIndex += sLength;
}

void SerialComms::convertValue(char *buf, int bufSize, int32_t val) {
// Abuse snprintf to convert our value
#ifdef UNIT_TESTING
  snprintf(buf, bufSize, "%d", val);
#else
  // On Arduino a int32_t is equivalent to a long decimal
  snprintf(buf, bufSize, "%ld", val);
#endif
}

void SerialComms::findInputCommands() {
  Libs::pair<uint8_t, uint8_t> commandIndexes(0, m_inputIndex);

  if (DATA_RECIEVE_PREFIX.equalsCString(&m_inputBuffer[commandIndexes.first])) {
    // We have a command
    parseTargetSpeed(Libs::move(commandIndexes));
  }
}

void SerialComms::parseTargetSpeed(Libs::pair<uint8_t, uint8_t> charRange) {
  char *foundPtr = strchr(&m_inputBuffer[charRange.first], K_V_SEPERATOR);
  uint8_t currentPos = foundPtr - m_inputBuffer + 1; // Move to next char

  const uint8_t maxLength = 8;

  // Get left number
  uint8_t leftNumDigits = 0;
  char leftBuf[maxLength]{0};
  while (isdigit(m_inputBuffer[currentPos]) && leftNumDigits < maxLength) {
    leftBuf[leftNumDigits] = m_inputBuffer[currentPos];
    leftNumDigits++;
    currentPos++;
  }
  // Move forward to the next numbers
  foundPtr = strchr(&m_inputBuffer[currentPos + 1], K_V_SEPERATOR);
  currentPos = foundPtr - m_inputBuffer + 1;

  if (currentPos > charRange.second) {
    return;
  }

  uint8_t rightNumDigits = 0;
  char rightBuf[maxLength]{0};
  while (isdigit(m_inputBuffer[currentPos]) && rightNumDigits < maxLength) {
    rightBuf[rightNumDigits] = m_inputBuffer[currentPos];
    rightNumDigits++;
    currentPos++;
  }

  bool isValid = leftNumDigits != 0 && rightNumDigits != 0;
  if (!isValid) {
    return;
  }

  // Parse and store as millimeters
  Distance leftDist(0, atoi(leftBuf)), rightDist(0, atoi(rightBuf));

  Speed leftVal(leftDist, 1_s), rightVal(rightDist, 1_s);

  // Store the new target
  WheelSpeeds newTargets(leftVal, rightVal);
  m_currentTargetSpeeds = newTargets;
}

void SerialComms::resetBuffer(char *targetBuffer, uint8_t bufferSize) {
  memset(targetBuffer, 0, sizeof(targetBuffer[0]) * bufferSize);
}

void SerialComms::sendCurrentBuffer() {
  m_hardwareInterface.serialPrint(m_outBuffer);
  m_outBuffer[0] = '\0';
  m_outIndex = 0;
}

} // namespace ArgoRcLib