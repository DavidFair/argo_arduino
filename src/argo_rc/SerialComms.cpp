#include "cstring_wrapper.hpp"
#include <stdint.h>
#include <stdlib.h>

#include "Encoder.hpp"
#include "arduino_interface.hpp"

#include "SerialComms.hpp"

using namespace Hardware;

/* We use char * arrays instead of Strings on the Arduino. This
 * helps reduce memory fragmentation and overhead associated with
 * strings, and keeps a consistent interface with the unit tests too. */
namespace {

// Whitespace seperates various data fields we are transmitting
// A ':' character indicates a key - value pair
const char K_V_SEPERATOR = ':';
const char SEPERATOR = ' ';
const char *DATA_TRANSMIT_PREFIX = "!D ";

// Function specific data
const char *ENCODER_NAMES[ArgoEncoderPositions::_NUM_OF_ENCODERS] = {"L_ENC_1",
                                                                     "R_ENC_1"};

} // namespace

namespace ArgoRcLib {

SerialComms::SerialComms(Hardware::ArduinoInterface &hardware)
    : m_hardwareInterface(hardware) {}

void SerialComms::sendEncoderRotation(const EncoderPulses &data) {

  // Prepare our output buffer - prepend that we are sending data
  appendToOutputBuf(DATA_TRANSMIT_PREFIX);

  addEncoderValToBuffer(ENCODER_NAMES[0], data.leftEncoderVal);
  addEncoderValToBuffer(ENCODER_NAMES[1], data.rightEncoderVal);

  sendCurrentBuffer();
}

void SerialComms::addEncoderValToBuffer(const char *encoderName,
                                        int32_t encoderVal) {
  // The maximum number of digits in either encoder output
  constexpr int NUM_DEC_PLACES = 10;
  char convertedNumber[NUM_DEC_PLACES];
// Abuse snprintf to convert our value
#ifdef UNIT_TESTING
  snprintf(convertedNumber, sizeof(convertedNumber), "%d", encoderVal);
#else
  // On Arduino a int32_t is equivalent to a long decimal
  snprintf(convertedNumber, sizeof(convertedNumber), "%ld", encoderVal);
#endif

  // Convert each number and forward as a K-V pair
  appendKVPair(encoderName, convertedNumber);
  appendToOutputBuf(SEPERATOR);
}

void SerialComms::appendKVPair(const char *key, const char *value) {
  appendToOutputBuf(key);
  appendToOutputBuf(K_V_SEPERATOR);
  appendToOutputBuf(value);
}

void SerialComms::appendToOutputBuf(const char c) {
  // Wrap in null terminated array and forward
  const char toAppend[2] = {c, '\0'};
  appendToOutputBuf(toAppend);
}

void SerialComms::appendToOutputBuf(const char *s) {
  auto length = strlen(s);
  if (length + m_currentIndex >= OUT_BUFFER_SIZE) {
    m_hardwareInterface.serialPrintln(
        "Serial Comm Buffer was not large enough. Dropping output");
  } else {
    strcat(m_outBuffer, s);
    m_currentIndex += length;
  }
}

void SerialComms::sendCurrentBuffer() {
  m_hardwareInterface.serialPrintln(m_outBuffer);
  m_outBuffer[0] = '\0';
  m_currentIndex = 0;
}

} // namespace ArgoRcLib