#include <stdint.h>

#include "ArduinoGlobals.hpp"

namespace Globals {
namespace InterruptData {

// This is the portable standard way of defining the declared global variable
volatile PinTimingData g_pinData[6 + 1];
volatile uint8_t g_pcIntLast = 0;
volatile PinEncoderData g_pinEncoderData;

} // namespace InterruptData

namespace ArgoData {
volatile VehicleDirection g_currentVehicleDirection;

} // namespace ArgoData

} // namespace Globals