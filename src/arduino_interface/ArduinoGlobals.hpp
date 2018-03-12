#ifndef ARDUINO_GLOBALS_HPP_
#define ARDUINO_GLOBALS_HPP_

#include <stdint.h>

#include "arduino_enums.hpp"

namespace Globals {
namespace InterruptData {

// --- RC PWM input ---------------------------------
struct PinTimingData {
  uint8_t edge{0};
  unsigned long riseTime{0};
  unsigned long fallTime{0};
  unsigned int lastGoodWidth{0};
};

struct PinEncoderData {
  uint8_t previousBitReading{0};
  long leftEncoderCount{0};
  long rightEncoderCount{0};
};

// These must be global for both the Interrupt routine (which cannot exist in
// any class) and the arduino_rc which is a class to access them
extern volatile PinEncoderData g_pinEncoderData;
extern volatile PinTimingData g_pinData[];
extern volatile uint8_t g_pcIntLast;

} // namespace InterruptData

namespace ArgoData {

struct VehicleDirection {
  ArduinoEnums::Direction leftWheelDirection;
  ArduinoEnums::Direction rightWheelDirection;
};

extern volatile VehicleDirection g_currentVehicleDirection;

} // namespace ArgoData

} // namespace Globals

#endif // ARDUINO_GLOBALS_HPP_