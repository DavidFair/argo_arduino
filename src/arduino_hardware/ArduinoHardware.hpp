#ifndef ARDUINO_HARDWARE_HPP_
#define ARDUINO_HARDWARE_HPP_

#include <stdint.h>

#include <Arduino.h>

#include "ArduinoInterface.hpp"

namespace Hardware {

/// Provides a concrete implementation to the Arduino hardware

// By specifying final we hint to the compiler to de-virtualise
// This saves 100 bytes in the final program overhead
class ArduinoHardware final : public ArduinoInterface {

public:
  /// Constructs a new ArduinoHardware object
  ArduinoHardware() = default;
  /// Destructs a new ArduinoHardware object
  ~ArduinoHardware() override = default;

  /// Executed if a fatal error occurs. Stops the vehicle and enters an infinite
  /// loop
  static void robotSafetyAbort();

  // Overrides

  /// Reads from the specified analog pin
  virtual int analogRead(ArduinoEnums::pinMapping pin) const override;

  /// Writes the given value to the specified analog pin
  virtual void analogWrite(ArduinoEnums::pinMapping pin,
                           int value) const override;

  /// Delays execution by the given number of milliseconds
  virtual void delay(unsigned long milliseconds) const override {
    return ::delay(milliseconds);
  }

  /// Reads from the given digital pin and returns a digital state
  virtual ArduinoEnums::digitalIO
  digitalRead(ArduinoEnums::pinMapping pin) const override;

  /// Writes a given digital state to a digital pin
  virtual void digitalWrite(ArduinoEnums::pinMapping pin,
                            ArduinoEnums::digitalIO mode) const override;

  /// Gets the current microsecond elapsed since start
  virtual unsigned long micros() const override { return ::micros(); };

  /// Gets the current milliseconds elapsed since start
  virtual unsigned long millis() const override { return ::millis(); };

  /// Returns the number of characters available in the serial buffer
  virtual int serialAvailable() const override { return ::Serial.available(); }

  /// Starts the serial communications at the given baud rate
  virtual void serialBegin(unsigned long baudRate) const override {
    ::Serial.begin(baudRate);
  };

  /// Prints the given string to the serial line
  virtual void serialPrint(const String &s) const override {
    ::Serial.print(s);
  };

  /// Prints the given integer to the serial line
  virtual void serialPrint(int i) const override { ::Serial.print(i); };

  /// Prints the given string with an EOL char to the serial line
  virtual void serialPrintln(const String &s) const override {
    ::Serial.println(s);
  };

  /// Prints the given string with an EOL char to the serial line
  virtual void serialPrintln(int i) const override { ::Serial.println(i); };

  /// Reads a single character from the serial line
  virtual char serialRead() const override { return ::Serial.read(); }

  /// Sets the specified pin to a given input or output mode
  virtual void setPinMode(ArduinoEnums::pinMapping pin,
                          ArduinoEnums::digitalIO mode) const override;

  // Static methods
  /// Used to map between hardware pins and the program pin enum
  static uint8_t convertPinEnumToArduino(ArduinoEnums::pinMapping pinToConvert);
};

} // namespace Hardware

#endif // ARDUINO_HARDWARE_HPP_