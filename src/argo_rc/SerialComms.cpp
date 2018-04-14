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

const unsigned long PING_TIMEOUT = 250; // milliseconds

// Whitespace seperates various data fields we are transmitting
// A ':' character indicates a key - value pair
constexpr char K_V_SEPERATOR = ':';
constexpr char SEPERATOR = ' ';
constexpr char EOL = '\n';

// Transmit prefixes
constexpr MinString ENCODER_TRANSMIT_PRE = "!e ";
constexpr MinString PING_TRANSMIT_PRE = "!p";
constexpr MinString SPEED_TRANSMIT_PRE = "!s ";

// Command prefixes
constexpr MinString PING_COMMAND_PRE = "!P";
constexpr MinString SPEED_COMMAND_PRE = "!T"; // As in 'T'arget speed

// Function specific data
constexpr int NUM_ENCODER = EncoderPositions::_NUM_OF_ENCODERS;

constexpr MinString ENCODER_NAMES[NUM_ENCODER] = {"L_ENC", "R_ENC"};
constexpr MinString SPEED_PREFIX[NUM_ENCODER] = {"L_SPEED", "R_SPEED"};

} // namespace

namespace ArgoRcLib {

SerialComms::SerialComms(Hardware::ArduinoInterface &hardware)
    : m_currentTargetSpeeds(), m_hardwareInterface(hardware),
      m_lastPingTime(hardware.millis()) {}

void SerialComms::addEncoderRotation(const EncoderPulses &data) {
  // Prepare our output buffer - prepend that we are sending data
  appendToOutputBuf(ENCODER_TRANSMIT_PRE);

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

void SerialComms::addPing() {
  appendToOutputBuf(PING_TRANSMIT_PRE);
  appendToOutputBuf(EOL);
}

void SerialComms::addVehicleSpeed(const Hardware::WheelSpeeds &speeds) {
  appendToOutputBuf(SPEED_TRANSMIT_PRE);

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

bool SerialComms::isPingGood() const {
  const auto timeSinceLastPing = m_hardwareInterface.millis() - m_lastPingTime;
  return (timeSinceLastPing < PING_TIMEOUT);
}

void SerialComms::parseIncomingBuffer() {
  while (m_hardwareInterface.serialAvailable() > 0) {
    if ((m_inputIndex + 1) >= BUFFER_SIZE) {
      m_hardwareInterface.serialPrintln(
          "Input buffer not large enough - ignoring");
      m_inputIndex = 0;
      return;
    }

    m_inputBuffer[m_inputIndex] = m_hardwareInterface.serialRead();
    m_inputIndex++;
  }

  // Check we have a single complete command before parsing
  bool containsEOLChar = (strchr(m_inputBuffer, EOL) != NULL);

  if (m_inputIndex > 0 && containsEOLChar) {
    findInputCommands();
    resetBuffer(m_inputBuffer, BUFFER_SIZE);
    m_inputIndex = 0;
  }
}

void SerialComms::sendCurrentBuffer() {
  m_hardwareInterface.serialPrint(m_outBuffer);
  m_outBuffer[0] = '\0';
  m_outIndex = 0;
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

bool SerialComms::convertBufStrToInt(uint8_t startingPos, int &result) {
  const uint8_t maxLength = 8;

  uint8_t numDigits = 0;

  char foundChars[maxLength]{0};
  while (isdigit(m_inputBuffer[startingPos]) && numDigits < maxLength) {
    foundChars[numDigits] = m_inputBuffer[startingPos];
    numDigits++;
    startingPos++;
  }

  // Check there were any digits
  if (numDigits == 0) {
    // Todo warn
    return false;
  }

  result = atoi(foundChars);
  return true;
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
  // Each time we find a new EOL character split and parse
  uint8_t startingSearchPos = 0;
  bool allEolParsed = false;

  do {
    char *foundPtr = strchr(&m_inputBuffer[startingSearchPos], EOL);
    if (foundPtr != nullptr) {
      const uint8_t positionOfEol = (uint8_t)(foundPtr - m_inputBuffer);
      // The next char is +1 so create a pair of command index and
      Libs::pair<uint8_t, uint8_t> foundPair(startingSearchPos, positionOfEol);
      processIndividualCommand(foundPair);

      startingSearchPos = positionOfEol + 1;
    } else {
      allEolParsed = true;
      const auto finalCharPos = strlen(m_inputBuffer);
      if (startingSearchPos != finalCharPos) {
        // There is another outstanding buffer to process
        Libs::pair<uint8_t, uint8_t> foundPair(startingSearchPos, finalCharPos);
        processIndividualCommand(foundPair);
      }
    }
  } while (!allEolParsed);
}

void SerialComms::parseTargetSpeed(
    const Libs::pair<uint8_t, uint8_t> &charRange) {

  char *foundLeftPtr = strchr(&m_inputBuffer[charRange.first], K_V_SEPERATOR);
  if (foundLeftPtr == nullptr) {
    // Todo warn
    return;
  }

  uint8_t firstLeftDigitPos = (foundLeftPtr - m_inputBuffer) + 1;
  if (firstLeftDigitPos >= charRange.second) {
    // Todo warn that there was nothing after K-V seperator
    return;
  }

  int leftDigit = 0;
  if (!convertBufStrToInt(firstLeftDigitPos, leftDigit)) {
    // Todo warn about failed converstion
    return;
  }

  // Same again for right wheel now:

  // Move forward to the next K_V seperator and numbers
  char *foundRightPtr =
      strchr(&m_inputBuffer[firstLeftDigitPos], K_V_SEPERATOR);
  if (foundRightPtr == nullptr) {
    // Todo warn
    return;
  }

  uint8_t firstRightDigitPos = (foundRightPtr - m_inputBuffer) + 1;
  if (firstRightDigitPos >= charRange.second) {
    // Todo warn that there was nothing after K-V seperator
    return;
  }

  int rightDigit = 0;
  if (!convertBufStrToInt(firstRightDigitPos, rightDigit)) {
    // Todo warn about failed converstion
    return;
  }

  // Parse and store as millimeters which is the second arg in the constructor
  Distance leftDist(0, leftDigit), rightDist(0, rightDigit);

  // This should have already been normalised to a second by ROS
  Speed leftVal(leftDist, 1_s), rightVal(rightDist, 1_s);

  // Store the new target
  WheelSpeeds newTargets(leftVal, rightVal);
  m_currentTargetSpeeds = newTargets;
}

void SerialComms::processIndividualCommand(
    const Libs::pair<uint8_t, uint8_t> &charPosition) {
  const char *strPtr = &m_inputBuffer[charPosition.first];

  if (PING_COMMAND_PRE.isPresentInString(strPtr)) {
    m_lastPingTime = m_hardwareInterface.millis();
  } else if (SPEED_COMMAND_PRE.isPresentInString(strPtr)) {
    parseTargetSpeed(charPosition);
  } else {
    // TODO add printing a warning here
  }
}

void SerialComms::resetBuffer(char *targetBuffer, uint8_t bufferSize) {
  memset(targetBuffer, 0, sizeof(targetBuffer[0]) * bufferSize);
}

} // namespace ArgoRcLib