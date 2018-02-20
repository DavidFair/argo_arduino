#ifndef PINTIMINGDATA_HPP_
#define PINTIMINGDATA_HPP_

#include <stdint.h>

namespace timingData{

// --- RC PWM input ---------------------------------
struct PinTimingData{
  uint8_t edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int  lastGoodWidth;


};

// These must be global for both the ISR (which cannot exist in any class)
// and the arduino_rc which is a class to access them
extern volatile PinTimingData g_pinData[];
extern volatile uint8_t g_pcIntLast;

} // End of namespace

#endif // PINTIMINGDATA_HPP_