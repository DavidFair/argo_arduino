#ifndef ARDUINO_GLOBALS_HPP_
#define ARDUINO_GLOBALS_HPP_

#include <stdint.h>

#include "arduino_enums.hpp"

namespace Globals {
namespace InterruptData {

/// Struct holding remote control timing data
struct PinTimingData {
  uint8_t edge{0};
  unsigned long riseTime{0};
  unsigned long fallTime{0};
  unsigned int lastGoodWidth{0};
};

/// Struct holding the current encoder counts
struct PinEncoderData {
  int32_t leftEncoderCount{0};
  int32_t rightEncoderCount{0};
};

// These must be global for both the Interrupt routine (which cannot exist in
// any class) and the arduino_rc which is a class to access them

/// Holds the current PWM timing data for remote control
extern volatile PinEncoderData g_pinEncoderData;
/// Holds a bit mask for the last interrupt serviced
extern volatile PinTimingData g_pinData[];
/// Holds the current encoder counts for the vehicle
extern volatile uint8_t g_pcIntLast;

} // namespace InterruptData

namespace ArgoData {

/// Struct holding each wheels current direction
struct VehicleDirection {
  ArduinoEnums::Direction leftWheelDirection;
  ArduinoEnums::Direction rightWheelDirection;
};

/// Represents the current vehicle direction
extern volatile VehicleDirection g_currentVehicleDirection;

} // namespace ArgoData

} // namespace Globals

#endif // ARDUINO_GLOBALS_HPP_