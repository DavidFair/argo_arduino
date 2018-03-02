#ifndef ARDUINO_HARDWARE_HPP_
#define ARDUINO_HARDWARE_HPP_

#include <stdint.h>

#include <Arduino.h>

#include "arduino_interface.hpp"

namespace Hardware {

class ArduinoHardware : public ArduinoInterface {

public:
  ArduinoHardware() = default;
  ~ArduinoHardware() override = default;

  virtual int analogRead(ArduinoEnums::pinMapping pin) const override;

  virtual void analogWrite(ArduinoEnums::pinMapping pin,
                           int value) const override;

  virtual uint8_t
  convertPinEnumToArduino(ArduinoEnums::pinMapping pinToConvert) const override;

  virtual void delay(unsigned long milliseconds) const override;

  virtual ArduinoEnums::digitalIO
  digitalRead(ArduinoEnums::pinMapping pin) const override;

  virtual void digitalWrite(ArduinoEnums::pinMapping pin,
                            ArduinoEnums::digitalIO mode) const override;

  virtual void enterDeadmanSafetyMode() const override;

  virtual unsigned long micros() const override { return ::micros(); };

  virtual void orPortBitmask(ArduinoEnums::portMapping port,
                             uint8_t bitmask) const override;

  virtual uint8_t readPortBits(ArduinoEnums::portMapping port) const override;

  virtual void serialBegin(unsigned long baudRate) const override {
    ::Serial.begin(baudRate);
  };

  virtual void setPortBitmask(ArduinoEnums::portMapping port,
                              uint8_t bitmask) const override;

  virtual void serialPrint(const String &s) const override {
    ::Serial.print(s);
  };

  virtual void serialPrint(int i) const override { ::Serial.print(i); };

  virtual void serialPrintln(const String &s) const override {
    ::Serial.println(s);
  };

  virtual void serialPrintln(int i) const override { ::Serial.println(i); };

  virtual void setPinMode(ArduinoEnums::pinMapping pin,
                          ArduinoEnums::digitalIO mode) const override;
};

} // namespace Hardware

#endif // ARDUINO_HARDWARE_HPP_