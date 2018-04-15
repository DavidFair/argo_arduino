#ifndef ARDUINO_INTERFACE_HPP_
#define ARDUINO_INTERFACE_HPP_

#include <stdint.h>

#include "arduino_enums.hpp"
#include "arduino_lib_wrapper.hpp"

namespace Hardware {

/// Abstract interface which provides an interface to the Arduino
class ArduinoInterface {
  // Abstract class which defines the methods that we use on our Arduino
  // for unit testing and on a real device

public:
  virtual ~ArduinoInterface() = default;

  /// Reads from the specified analog pin
  virtual int analogRead(ArduinoEnums::pinMapping pin) const = 0;

  /// Writes the given value to the specified analog pin
  virtual void analogWrite(ArduinoEnums::pinMapping pin, int value) const = 0;

  /// Delays execution by the given number of milliseconds
  virtual void delay(unsigned long milliseconds) const = 0;

  /// Reads from the given digital pin and returns a digital state
  virtual ArduinoEnums::digitalIO
  digitalRead(ArduinoEnums::pinMapping pin) const = 0;

  /// Writes a given digital state to a digital pin
  virtual void digitalWrite(ArduinoEnums::pinMapping pin,
                            ArduinoEnums::digitalIO mode) const = 0;

  /// Enters an infinite loop when the deadman switch triggers
  virtual void enterDeadmanSafetyMode() const = 0;

  /// Gets the current microsecond elapsed since start
  virtual unsigned long micros() const = 0;

  /// Gets the current milliseconds elapsed since start
  virtual unsigned long millis() const = 0;

  /// Returns the number of characters available in the serial buffer
  virtual int serialAvailable() const = 0;

  /// Starts the serial communications at the given baud rate
  virtual void serialBegin(unsigned long baudRate) const = 0;

  /// Prints the given int to the serial line
  virtual void serialPrint(int i) const = 0;

  /// Prints the given int with an EOL char to the serial line
  virtual void serialPrintln(int i) const = 0;

  /// Reads a single character from the serial line
  virtual char serialRead() const = 0;

  /// Sets the specified pin to a given input or output mode
  virtual void setPinMode(ArduinoEnums::pinMapping pin,
                          ArduinoEnums::digitalIO mode) const = 0;

#ifndef UNIT_TESTING
  /// Prints the given string with an EOL char to the serial line
  virtual void serialPrintln(const String &s) const = 0;
  /// Prints the given string to the serial line
  virtual void serialPrint(const String &s) const = 0;
#else
  /// Prints the given string with an EOL char to the serial line
  virtual void serialPrintln(const std::string &s) const = 0;
  /// Prints the given string to the serial line
  virtual void serialPrint(const std::string &s) const = 0;
#endif

protected:
  ArduinoInterface() = default;
};

} // namespace Hardware

#endif // ARDUINO_INTERFACE_HPP_