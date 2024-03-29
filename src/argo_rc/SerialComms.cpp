#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>

#include "ArduinoInterface.hpp"
#include "Distance.hpp"
#include "Encoder.hpp"
#include "MinString.hpp"
#include "PidController.hpp"
#include "SerialComms.hpp"
#include "Speed.hpp"
#include "cstring_wrapper.hpp"

using namespace Hardware;
using namespace Libs;

/* We use char * arrays instead of Strings on the Arduino. This
 * helps reduce memory fragmentation and overhead associated with
 * strings, and keeps a consistent interface with the unit tests too. */

namespace {
/// Time between pings before we consider comms lost
const unsigned long PING_TIMEOUT = 500; // milliseconds

// Whitespace seperates various data fields we are transmitting
// A ':' character indicates a key - value pair
/// The character separating a Key-Value pair
constexpr char K_V_SEPARATOR = ':';
/// Character separating tokens in a command
constexpr char SEPARATOR = ' ';
/// Character separating commands in a buffer
constexpr char EOL = '\n';

// Transmit prefixes
/// Prefix for transmitting the current encoder data
const MinString ENCODER_TRANSMIT_PRE = "!e ";
/// Prefix for transmitting an outgoing ping
const MinString PING_TRANSMIT_PRE = "!p";
/// Prefix for PWM transmitting
const MinString PWM_TRANSMIT_PRE = "!m "; // As in pw'M' or M for motor

/// Prefix for transmitting the current vehicle speed
const MinString SPEED_TRANSMIT_PRE = "!s ";
/// Prefix for transmitting a warning
const MinString WARN_TRANSMIT_PRE = "!w";

// Command prefixes
/// Prefix for an incoming ping
const MinString PING_COMMAND_PRE = "!P";
/// Prefix to a target speed command
const MinString SPEED_COMMAND_PRE = "!T"; // As in 'T'arget speed

/// Checksum prefix for K-V pair
const MinString CHECKSUM_PRE = "chk";

// Function specific data
constexpr int NUM_ENCODER = EncoderPositions::_NUM_OF_ENCODERS;

/// Names of the left and right PWM when sending encoder PWM values
const MinString PWM_NAMES[NUM_ENCODER] = {"L_PWM", "R_PWM"};
/// Names of the left and right encoders when sending encoder data
const MinString ENCODER_NAMES[NUM_ENCODER] = {"L_ENC", "R_ENC"};
/// Names of the left and right wheels when sending speed data
const MinString SPEED_NAMES[NUM_ENCODER] = {"L_SPEED", "R_SPEED"};

} // namespace

namespace ArgoRcLib {

/**
 * Constructs a new SerialComms object which uses the referenced
 * hardware. It sets the last ping time to the current time of the
 * device
 *
 * @param hardware Reference to the Arduino hardware
 */
SerialComms::SerialComms(Hardware::ArduinoInterface &hardware)
    : m_currentTargetSpeeds(), m_hardwareInterface(hardware),
      m_lastPingTime(hardware.millis()) {}

/**
 * Adds the current encoder pulses for both wheels to the
 * outgoing buffer
 *
 * @param data The current encoder pulses for both wheels
 */
void SerialComms::addEncoderRotation(const EncoderPulses &data) {
  // Prepare our output buffer - prepend that we are sending data
  auto startOfCommand = m_outIndex;

  appendToOutputBuf(ENCODER_TRANSMIT_PRE);

  // The maximum number of digits in either encoder output
  constexpr int NUM_DEC_PLACES = 8;
  char convertedNumber[NUM_DEC_PLACES]{0};

  // Convert each number and forward as a K-V pair
  convertValue(convertedNumber, NUM_DEC_PLACES, data.leftEncoderVal);
  appendKVPair(ENCODER_NAMES[EncoderPositions::LEFT_ENCODER].str(),
               convertedNumber);
  appendToOutputBuf(SEPARATOR);

  convertValue(convertedNumber, NUM_DEC_PLACES, data.rightEncoderVal);
  appendKVPair(ENCODER_NAMES[EncoderPositions::RIGHT_ENCODER].str(),
               convertedNumber);

  auto endOfCommand = m_outIndex;
  appendChecksumValue(startOfCommand, endOfCommand);
  appendToOutputBuf(EOL);
}

/// Adds a ping to the outgoing buffer
void SerialComms::addPing() {
  appendToOutputBuf(PING_TRANSMIT_PRE);
  // Pings do not require checksumming
  appendToOutputBuf(EOL);
}

void SerialComms::addPwmValues(const PwmTargets &targetVals) {
  auto startOfCommand = m_outIndex;

  appendToOutputBuf(PWM_TRANSMIT_PRE);
  constexpr int NUM_DEC_PLACES = 8;
  char convertedNumber[NUM_DEC_PLACES];

  convertValue(convertedNumber, NUM_DEC_PLACES, targetVals.leftPwm);
  appendKVPair(PWM_NAMES[EncoderPositions::LEFT_ENCODER].str(),
               convertedNumber);
  appendToOutputBuf(SEPARATOR);

  convertValue(convertedNumber, NUM_DEC_PLACES, targetVals.rightPwm);
  appendKVPair(PWM_NAMES[EncoderPositions::RIGHT_ENCODER].str(),
               convertedNumber);

  auto endOfCommand = m_outIndex;
  appendChecksumValue(startOfCommand, endOfCommand);
  appendToOutputBuf(EOL);
}

/**
 * Adds the vehicles current speed for both wheels to the outgoing buffer
 *
 * @param speeds The vehicles current speed
 */
void SerialComms::addVehicleSpeed(const Hardware::WheelSpeeds &speeds) {
  auto startOfCommand = m_outIndex;

  appendToOutputBuf(SPEED_TRANSMIT_PRE);

  constexpr int NUM_DEC_PLACES = 9;
  char convertedNumber[NUM_DEC_PLACES];

  convertValue(convertedNumber, NUM_DEC_PLACES,
               speeds.leftWheel.getUnitDistance().millimeters());
  appendKVPair(SPEED_NAMES[EncoderPositions::LEFT_ENCODER].str(),
               convertedNumber);
  appendToOutputBuf(SEPARATOR);

  convertValue(convertedNumber, NUM_DEC_PLACES,
               speeds.rightWheel.getUnitDistance().millimeters());
  appendKVPair(SPEED_NAMES[EncoderPositions::RIGHT_ENCODER].str(),
               convertedNumber);

  auto endOfCommand = m_outIndex;
  appendChecksumValue(startOfCommand, endOfCommand);

  appendToOutputBuf(EOL);
}

/**
 * Adds a warning provided by a string literal or pointer to
 * char array to the outgoing buffer.
 *
 * The contents of the pointer must be null terminated and is
 * copied
 *
 * @param warningText Null terminated warning to send
 */
void SerialComms::addWarning(const char *warningText) {
  appendToOutputBuf(WARN_TRANSMIT_PRE);
  appendToOutputBuf(SEPARATOR);
  appendToOutputBuf(warningText);
  appendToOutputBuf(EOL);
}

/**
 * Calculates the message checksum between the two given pointers
 * of the string. It is the callers responsibility to ensure
 * the pointers refer to the same string.
 *
 * The end pointer must not include the EOL character or the
 * preceeding checksum values.
 *
 * @param msgStart Pointer to start of string
 * @param msgEnd Pointer to the end of string
 * @return A checksum value of uint8_t for the current string
 */
uint8_t SerialComms::calcMsgChecksum(const char *msgStart,
                                     const char *msgEnd) const {
  // This MUST be a unsigned type - overflow on signed types is UB
  uint8_t sum = 0;

  const int lengthOfString = msgEnd - msgStart;
  for (int i = 0; i < lengthOfString; i++) {
    sum += (int)msgStart[i];
  }

  return sum;
}

/**
 * Returns whether a ping has been received within the
 * timeout period (see PING_TIMEOUT)
 *
 * @return True if ping was received in timeout period, otherwise false
 */
bool SerialComms::isPingGood() const {
  const auto timeSinceLastPing = m_hardwareInterface.millis() - m_lastPingTime;
  return (timeSinceLastPing < PING_TIMEOUT);
}

/**
 * Parses the incoming buffer to find any pending commands and process
 * them. The buffer is then reset with any partial commands preserved
 * for future calls.
 */
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
  }

  m_inputIndex = strlen(m_inputBuffer);
}

/**
 * Sends the current outgoing buffer to over serial then resets
 * it to clear
 */
void SerialComms::sendCurrentBuffer() {
  m_hardwareInterface.serialPrint(m_outBuffer);
  resetBuffer(m_outBuffer, BUFFER_SIZE);
  m_outIndex = 0;
}

// --------- Private methods ------------

/**
 * Appends the calculated checksum for the buffer after final char
 * in the outgoing message. This must be called before the EOL
 * character is appended. The caller must then append the EOL
 * character afterwards
 *
 * @param startOfMsg Index to the start of the command
 * @param endOfMsg Index to the end of the command
 */
void SerialComms::appendChecksumValue(const int commandStart,
                                      const int commandEnd) {
  const char *startOfMsg = &m_outBuffer[commandStart];
  const char *endOfMsg = &m_outBuffer[commandEnd];

  uint8_t checksumVal = calcMsgChecksum(startOfMsg, endOfMsg);

  constexpr int CHECKSUM_LEN = 11; // Min 11 bytes is required
  char serialisedChecksum[CHECKSUM_LEN] = {0};

  convertValue(serialisedChecksum, CHECKSUM_LEN, checksumVal);

  // The checksum does not include the space before "chk"
  appendToOutputBuf(SEPARATOR);
  appendKVPair(CHECKSUM_PRE.str(), serialisedChecksum);
}

/**
 * Appends a copy of the given key and value to a
 * key-value pair in the output buffer
 *
 * @param key The key to append to the output buffer
 * @param value The value to append to the output buffer
 */
void SerialComms::appendKVPair(const char *key, const char *value) {
  appendToOutputBuf(key);
  appendToOutputBuf(K_V_SEPARATOR);
  appendToOutputBuf(value);
}

/// Appends a single character to the output buffer
void SerialComms::appendToOutputBuf(const char c) {
  // Wrap in null terminated array and forward
  const char toAppend[2] = {c, '\0'};
  appendToOutputBuf(toAppend);
}

/// Appends a copy of a string to the output buffer.
void SerialComms::appendToOutputBuf(const MinString &s) {
  auto sLength = s.length();

  if (sLength + m_outIndex >= BUFFER_SIZE) {
    // Flush the current buffer regardless
    sendCurrentBuffer();
  }
  strcat(m_outBuffer, s.str());
  m_outIndex += sLength;
}

/**
 * Checks an incoming command's checksum. If it is correct and matches
 * the string contents a true is returned. Otherwise a specific warning
 * is added to the outgoing buffer
 *
 * @param commandRange Start and end position in the outgoing buffer
 * @return True if checksum is correct, otherwise false in all other cases
 */
bool SerialComms::checkIncomingChecksum(
    const Libs::pair<uint8_t, uint8_t> &commandRange) {
  // Temporaily swap the \n char will null terminator so that strstr
  // respects our bounds
  const char *startOfCommand = &m_inputBuffer[commandRange.first];
  char *lastDigitOfCommand = &m_inputBuffer[commandRange.second];
  m_inputBuffer[commandRange.second] = '\0';

  const int LEN_OF_CHKSUM = 4; // For 'chk:'
  const char *foundChecksumKey = strstr(startOfCommand, "chk:");

  auto failedChk = [this, startOfCommand, lastDigitOfCommand]() {
    addWarning(startOfCommand);
    *lastDigitOfCommand = '\n';
    return false;
  };

  if (foundChecksumKey == nullptr) {
    addWarning("No checksum found in the following command");
    return failedChk();
  } else if (foundChecksumKey + LEN_OF_CHKSUM >= lastDigitOfCommand) {
    addWarning("Checksum was missing value in the following command");
    return failedChk();
  }

  const char *ptrToChecksumVal = foundChecksumKey + LEN_OF_CHKSUM;
  const int posOfChecksumVal = ptrToChecksumVal - startOfCommand;

  // Attempt to convert now
  int checksumValue = 0;
  if (!convertBufStrToInt(posOfChecksumVal, checksumValue)) {
    addWarning("Could not convert checksum this command");
    return failedChk();
  }

  // Dont include checksum in the calculation
  uint8_t calculatedChecksum =
      calcMsgChecksum(startOfCommand, (foundChecksumKey - 1));

  if (checksumValue != calculatedChecksum) {
    addWarning("Calculated checksum did not match");
    return failedChk();
  }
  // Ensure null terminator is reset
  *lastDigitOfCommand = '\n';
  return true;
}

/**
 * Converts the input buffer characters at a given position to an
 * integer returned through a referenced parameters. If the buffer fails
 * to parse a false is returned to the caller and the value of result
 * is undefined
 *
 * @param startingPos The position in the input buffer where the value
 * starts
 * @param result The integer to write the parsed value to on success
 * @return True if successful, otherwise false
 */
bool SerialComms::convertBufStrToInt(uint8_t startingPos, int &result) {
  const uint8_t maxLength = 8;

  uint8_t numDigits = 0;

  char foundChars[maxLength]{0};

  const bool isNegative = m_inputBuffer[startingPos] == '-';
  if (isNegative) {
    startingPos++;
  }

  while (isdigit(m_inputBuffer[startingPos]) && numDigits < maxLength) {
    foundChars[numDigits] = m_inputBuffer[startingPos];
    numDigits++;
    startingPos++;
  }

  // Check there were any digits
  if (numDigits == 0) {
    return false;
  }

  result = atoi(foundChars);
  if (isNegative) {
    result = -result;
  }
  return true;
}

/**
 * Converts a value from the given integer into a char representation.
 * The caller is responsible for allocating a buffer and passing the
 * maximum buffer size within this function.
 *
 * @param buf Pointer to character array to write the result to
 * @param bufSize Maximum size of the characters to write
 * @param val The integer to convert to characters
 */
void SerialComms::convertValue(char *buf, int bufSize, int32_t val) {
// Abuse snprintf to convert our value
#ifdef UNIT_TESTING
  snprintf(buf, bufSize, "%d", val);
#else
  // On Arduino a int32_t is equivalent to a long decimal
  snprintf(buf, bufSize, "%ld", val);
#endif
}

/**
 * Finds input commands as delimeted by the EOL character
 * and parses each one in turn. If there are any partial commands
 * if moves them to the front of the input buffer.
 * It then resets the buffer of all parsed commands.
 */
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
        // There is an incomplete command pending
        int length = finalCharPos - startingSearchPos;

        // Shift the data back to the beginning of the buffer
        memmove(&m_inputBuffer[0], &m_inputBuffer[startingSearchPos], length);
        m_inputBuffer[length] = '\0';
        resetBuffer(&m_inputBuffer[length], (BUFFER_SIZE - length));
      } else {
        // Nothing in the buffer to restore to clean state
        resetBuffer(m_inputBuffer, BUFFER_SIZE);
      }
    }
  } while (!allEolParsed);
}

/**
 * Parses the target speed from the input buffer. The parameter
 * indicates the start and end of the command in the input buffer.
 *
 * The result is stored in m_currentTargetSpeeds if successful.
 * If the input buffer cannot be parsed a warning is emitted
 * with a reason why.
 *
 * @param charRange A pair representing the index of the input buffer where
 * the command starts and finishes
 */
void SerialComms::parseTargetSpeed(
    const Libs::pair<uint8_t, uint8_t> &charRange) {

  const char *strStart = &m_inputBuffer[charRange.first];
  const char *foundLeftPtr = strchr(strStart, K_V_SEPARATOR);

  if (foundLeftPtr == nullptr) {
    addWarning("Parsing speed: Could not find l. K-V sep");
    addWarning(strStart);
    return;
  }

  uint8_t firstLeftDigitPos = (foundLeftPtr - m_inputBuffer) + 1;
  if (firstLeftDigitPos >= charRange.second) {
    addWarning("Parsing speed: No digits after l. K-V sep");
    addWarning(strStart);
    return;
  }

  int leftDigit = 0;
  if (!convertBufStrToInt(firstLeftDigitPos, leftDigit)) {
    addWarning("Parsing speed: could not convert l. digit");
    addWarning(strStart);
    return;
  }

  // Same again for right wheel now:

  // Move forward to the next K_V SEPARATOR and numbers
  char *foundRightPtr =
      strchr(&m_inputBuffer[firstLeftDigitPos], K_V_SEPARATOR);
  if (foundRightPtr == nullptr) {
    addWarning("Parsing speed: Could not find r. K-V sep");
    addWarning(strStart);
    return;
  }

  uint8_t firstRightDigitPos = (foundRightPtr - m_inputBuffer) + 1;
  if (firstRightDigitPos >= charRange.second) {
    addWarning("Parsing speed: No digits after r. K-V sep");
    addWarning(strStart);
    return;
  }

  int rightDigit = 0;
  if (!convertBufStrToInt(firstRightDigitPos, rightDigit)) {
    addWarning("Parsing speed: could not convert r. digit");
    addWarning(strStart);
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

/**
 * Determines the individual command type and calls the
 * appropriate method on the object. If the command is
 * unknown a warning is emitted instead.
 *
 * @param charPosition A pair representing the start and end position
 * of the command in the internal buffer
 */
void SerialComms::processIndividualCommand(
    const Libs::pair<uint8_t, uint8_t> &charPosition) {
  const char *strPtr = &m_inputBuffer[charPosition.first];

  // Ping commands do not contain checksums
  if (PING_COMMAND_PRE.isPresentInString(strPtr)) {
    m_lastPingTime = m_hardwareInterface.millis();
    return;
  }

  // Ensure checksum works correctly
  if (!checkIncomingChecksum(charPosition)) {
    return;
  }

  if (SPEED_COMMAND_PRE.isPresentInString(strPtr)) {
    parseTargetSpeed(charPosition);
  } else {
    addWarning("The following command string was unknown:\n");
    addWarning(strPtr);
  }
}

/**
 * Resets the target buffer as pointed to with n number of
 * null characters as indicated by the buffer size
 *
 * @param targetBuffer The buffer to reset to nulls
 * @param bufferSize The size of the buffer to reset
 */
void SerialComms::resetBuffer(char *targetBuffer, uint8_t bufferSize) {
  for (uint8_t i = 0; i < bufferSize; i++) {
    targetBuffer[i] = 0;
  }
}

} // namespace ArgoRcLib