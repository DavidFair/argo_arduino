
#include <Arduino.h>

#include "arduino_hardware.hpp"
#include "argo_rc_lib.hpp"
#include "move.hpp"
#include "pinTimingData.hpp"
#include "unique_ptr.hpp"

// Forward decleration
void setupInterrupts();

// Instantiate argo_rc library at the global level so it doesn't drop out of
// scope
// The hardware abstraction must live as long as the program runs so there
// is no corresponding free
Argo::unique_ptr<Hardware::ArduinoInterface>
    hardwareImpl(new Hardware::ArduinoHardware());

Argo::unique_ptr<ArgoRcLib::ArgoEncoder>
    encoderFactoryImpl(new ArgoRcLib::ArgoEncoder(*hardwareImpl));

ArgoRcLib::ArgoRc argoRcLib(Argo::move(hardwareImpl),
                            Argo::move(encoderFactoryImpl));

void setup() {
  argoRcLib.setup();
  // Setup interrupts last so they aren't overwritten
  setupInterrupts();
}

void loop() { argoRcLib.loop(); }

// ----- Interrupt Handling - Cannot (easily) be mocked -----

void setupInterrupts() {

  // Set ADC8 - ADC15 to input (0) using the port register
  DDRK = 0;

  // enable PCINT which allows us to use the PCMSK register by bitshifting
  // 1 into the correct register location
  PCICR |= (1 << PCIE2);

  // Set PCINT18 to 23 as ISR triggers
  // On the Mega 2560 these are pins A10-A15 as ISR
  PCMSK2 = 0xFC;

  // The ISR is defined below
}

// Due to the nature of an ISR we cannot unit test it. Place it within this file
// as it is hardware specific
ISR(PCINT2_vect) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint32_t currentTime;
  uint32_t time;

  // get the pin states for the indicated port.
  curr = PINK & 0xFC;
  mask = curr ^ timingData::g_pcIntLast;
  timingData::g_pcIntLast = curr;

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i = 0; i < 6; i++) {
    bit = 0x04 << i;
    if (bit & mask) {
      // for each pin changed, record time of change
      if (bit & timingData::g_pcIntLast) {
        time = currentTime - timingData::g_pinData[i].fallTime;
        timingData::g_pinData[i].riseTime = currentTime;
        if ((time >= 10000) && (time <= 26000))
          timingData::g_pinData[i].edge = 1;
        else
          timingData::g_pinData[i].edge = 0; // invalid rising edge detected
      } else {
        time = currentTime - timingData::g_pinData[i].riseTime;
        timingData::g_pinData[i].fallTime = currentTime;
        if ((time >= 800) && (time <= 2200) &&
            (timingData::g_pinData[i].edge == 1)) {
          timingData::g_pinData[i].lastGoodWidth = time;
          timingData::g_pinData[i].edge = 0;
        }
      }
    }
  }
}