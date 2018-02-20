#include "pinTimingData.hpp"

namespace timingData{

// This is the portable standard way of defining the declared global variable
volatile PinTimingData g_pinData[6 + 1];
volatile uint8_t g_pcIntLast = 0;
}