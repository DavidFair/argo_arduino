#include <stdint.h>
#include <stdlib.h>

#include "Encoder.hpp"
#include "arduino_interface.hpp"
#include "cstring_wrapper.hpp"

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
constexpr int NUM_ENCODER = EncoderPositions::_NUM_OF_ENCODERS;

const char *ENCODER_NAMES[NUM_ENCODER] = {"L_ENC", "R_ENC"};
const char *SPEED_PREFIX[NUM_ENCODER] = {"L_SPEED", "R_SPEED"};

} // namespace

namespace ArgoRcLib {

SerialComms::SerialComms(Hardware::ArduinoInterface &hardware)
    : m_hardwareInterface(hardware) {}

void SerialComms::sendEncoderRotation(const EncoderPulses &data) {
  // Prepare our output buffer - prepend that we are sending data
  appendToOutputBuf(DATA_TRANSMIT_PREFIX);

  // The maximum number of digits in either encoder output
  constexpr int NUM_DEC_PLACES = 6;
  char convertedNumber[NUM_DEC_PLACES];

  // Convert each number and forward as a K-V pair
  convertValue(convertedNumber, NUM_DEC_PLACES, data.leftEncoderVal);
  appendKVPair(ENCODER_NAMES[EncoderPositions::LEFT_ENCODER], convertedNumber);

  convertValue(convertedNumber, NUM_DEC_PLACES, data.rightEncoderVal);
  appendKVPair(ENCODER_NAMES[EncoderPositions::RIGHT_ENCODER], convertedNumber);

  sendCurrentBuffer();
}

void SerialComms::sendVehicleSpeed(const Hardware::WheelSpeeds &speeds) {
  appendToOutputBuf(DATA_TRANSMIT_PREFIX);

  constexpr int NUM_DEC_PLACES = 10;
  char convertedNumber[NUM_DEC_PLACES];

  convertValue(convertedNumber, NUM_DEC_PLACES,
               speeds.leftWheel.getMilliMetersPerSecond());
  appendKVPair(SPEED_PREFIX[EncoderPositions::LEFT_ENCODER], convertedNumber);

  convertValue(convertedNumber, NUM_DEC_PLACES,
               speeds.rightWheel.getMilliMetersPerSecond());
  appendKVPair(SPEED_PREFIX[EncoderPositions::RIGHT_ENCODER], convertedNumber);

  sendCurrentBuffer();
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

void SerialComms::convertValue(char *buf, int bufSize, int32_t val) {
// Abuse snprintf to convert our value
#ifdef UNIT_TESTING
  snprintf(buf, bufSize, "%d", val);
#else
  // On Arduino a int32_t is equivalent to a long decimal
  snprintf(buf, bufSize, "%ld", val);
#endif
}

void SerialComms::sendCurrentBuffer() {
  m_hardwareInterface.serialPrintln(m_outBuffer);
  m_outBuffer[0] = '\0';
  m_currentIndex = 0;
}

} // namespace ArgoRcLib