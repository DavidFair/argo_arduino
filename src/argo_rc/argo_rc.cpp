
#include <Arduino.h>

#include "ArduinoGlobals.hpp"
#include "arduino_hardware.hpp"
#include "argo_rc_lib.hpp"
#include "move.hpp"

using namespace Globals;

// Forward deceleration
void setupInterrupts();

// Instantiate argo_rc library at the global level so it doesn't drop out of
// scope
// The hardware abstraction must live as long as the program runs so there
// is no corresponding free
Libs::unique_ptr<Hardware::ArduinoInterface>
    hardwareImpl(new Hardware::ArduinoHardware());

ArgoRcLib::ArgoRc argoRcLib(*hardwareImpl);

void setup()
{
  argoRcLib.setup();
  // Setup interrupts last so they aren't overwritten
  setupInterrupts();
}

void loop() { argoRcLib.loop(); }

// ----- Interrupt Handling - Cannot (easily) be mocked -----

void leftEncoderInterrupt()
{
  InterruptData::g_pinEncoderData.leftEncoderCount += 1;
  // ArgoData::g_currentVehicleDirection.leftWheelDirection;
}

void rightEncoderInterrupt()
{
  InterruptData::g_pinEncoderData.rightEncoderCount += 1;
  //      ArgoData::g_currentVehicleDirection.rightWheelDirection;
}

void setupInterrupts()
{
  constexpr int leftEncPin = 5;
  constexpr int rightEncPin = 3;
  attachInterrupt(leftEncPin, leftEncoderInterrupt, FALLING);
  attachInterrupt(rightEncPin, rightEncoderInterrupt, FALLING);

  // Set ADC8 - ADC15 to input (0) using the port register
  DDRK = 0;

  // enable PCINT2 which allows us to use the PCMSK registers
  // to enable interrupts on the specified pins
  PCICR = (1 << PCIE2);

  // Set PCINT18:23 as ISR triggers for RC controls
  // On the Mega 2560 these are pins A10-A15 as ISR
  PCMSK2 = 0xFC;

  // The ISR is defined below
}

// ISR for the remote control
ISR(PCINT2_vect)
{
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint32_t currentTime;
  uint32_t time;

  // get the pin states for the indicated port.
  curr = PINK & 0xFC;
  mask = curr ^ InterruptData::g_pcIntLast;
  InterruptData::g_pcIntLast = curr;

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i = 0; i < 6; i++)
  {
    bit = 0x04 << i;
    if (bit & mask)
    {
      // for each pin changed, record time of change
      if (bit & InterruptData::g_pcIntLast)
      {
        time = currentTime - InterruptData::g_pinData[i].fallTime;
        InterruptData::g_pinData[i].riseTime = currentTime;
        if ((time >= 10000) && (time <= 26000))
          InterruptData::g_pinData[i].edge = 1;
        else
          InterruptData::g_pinData[i].edge = 0; // invalid rising edge detected
      }
      else
      {
        time = currentTime - InterruptData::g_pinData[i].riseTime;
        InterruptData::g_pinData[i].fallTime = currentTime;
        if ((time >= 800) && (time <= 2200) &&
            (InterruptData::g_pinData[i].edge == 1))
        {
          InterruptData::g_pinData[i].lastGoodWidth = time;
          InterruptData::g_pinData[i].edge = 0;
        }
      }
    }
  }
}