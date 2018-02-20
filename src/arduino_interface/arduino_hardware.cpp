#include <stdint.h>
#include <Arduino.h>

#include "arduino_hardware.hpp"
#include "pinTimingData.hpp"

using namespace ArduinoEnums;

namespace Hardware{

// Due to the nature of an ISR we cannot unit test it. Place it within this file
// as it is hardware specific
ISR(PCINT2_vect)
{
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
  for (uint8_t i=0; i < 6; i++) {
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
      }
      else {
        time = currentTime - timingData::g_pinData[i].riseTime;
        timingData::g_pinData[i].fallTime = currentTime;
        if ((time >= 800) && (time <= 2200) && (timingData::g_pinData[i].edge == 1)) {
          timingData::g_pinData[i].lastGoodWidth = time;
          timingData::g_pinData[i].edge = 0;
        }
      }
    }
  }
}


int ArduinoHardware::analogRead(pinMapping pin) const{
    return ::analogRead(convertPinEnumToArduino(pin));
}

void ArduinoHardware::analogWrite(pinMapping pin, int value) const{
    ::analogWrite(convertPinEnumToArduino(pin), value);
}

void ArduinoHardware::delay(unsigned long milliseconds) const{
    ::delay(milliseconds);
}

digitalIO ArduinoHardware::digitalRead(pinMapping pin) const{
    int result = ::digitalRead(convertPinEnumToArduino(pin));

    switch (result){
        case HIGH: return digitalIO::E_HIGH;
        case LOW: return digitalIO::E_LOW;
    }
}

void ArduinoHardware::digitalWrite(pinMapping pin, digitalIO mode) const{
    const auto outputPin = convertPinEnumToArduino(pin);

    switch (mode){
        case digitalIO::E_HIGH: return ::digitalWrite(outputPin, HIGH);
        case digitalIO::E_LOW: return ::digitalWrite(outputPin, LOW);
        default:
            serialPrintln("Attempted to use a pin state as output");
            break;
    }
}

void ArduinoHardware::orPortBitmask(portMapping port, uint8_t bitmask) const{
    switch(port){
        case portMapping::E_DDRK:
            DDRK |= bitmask;
            break;
        case portMapping::E_PCICR:
            PCICR |= bitmask;
            break;
        case portMapping::E_PCMSK2:
            PCMSK2 |= bitmask;
            break;
        case portMapping::E_PINK:
            PINK |= bitmask;
            break;
    }
}

uint8_t ArduinoHardware::readPortBits(portMapping port) const{
    switch(port){
        case portMapping::E_DDRK: return DDRK;
        case portMapping::E_PCICR: return PCICR;
        case portMapping::E_PCMSK2: return PCMSK2;
        case portMapping::E_PINK: return PINK;
    }
}

void ArduinoHardware::setPortBitmask(portMapping port, uint8_t bitmask) const{
    switch(port){
        case portMapping::E_DDRK:
            DDRK = bitmask;
            break;
        case portMapping::E_PCICR:
            PCICR = bitmask;
            break;
        case portMapping::E_PCMSK2:
            PCMSK2 = bitmask;
            break;
        case portMapping::E_PINK:
            PINK = bitmask;
            break;
    }
}

void ArduinoHardware::setPinMode(pinMapping pin, digitalIO mode) const{
    const auto destPin = convertPinEnumToArduino(pin);
    switch (mode){
        case digitalIO::E_INPUT:
            ::pinMode(destPin, INPUT);
            break;
        case digitalIO::E_INPUT_PULLUP:
            ::pinMode(destPin, INPUT_PULLUP);
            break;
        case digitalIO::E_OUTPUT:
            ::pinMode(destPin, OUTPUT);
            break;
        default:
            serialPrintln("Attempted to set a pin to a state instead of a mode");
            break;
    }
}


uint8_t ArduinoHardware::convertPinEnumToArduino(pinMapping pinToConvert){
    /* It is not possible to assign a value to an enum class after
     * the definition. However to pull in pins A6/A7 we must include
     * the Arduino header which means we must target the Mega.
     *
     * Instead we use a switch case to "assign" values within
     * this class instead. Whilst there will be a slight performance
     * impact the compiler should optimise it to the point where there
     * won't be much of a difference compared to assigning values to the
     * enum class instead.
     */ 
    
    // GCC will warn us if we miss a case off the enum
    
    
    switch(pinToConvert){
        case pinMapping::LEFT_FORWARD_RELAY : return 23;
        case pinMapping::LEFT_REVERSE_RELAY : return 25;
        
        case pinMapping::RIGHT_FORWARD_RELAY : return 27;
        case pinMapping::RIGHT_REVERSE_RELAY : return 29;

        case pinMapping::LEFT_FOOTSWITCH_RELAY : return 42;
        case pinMapping::RIGHT_FOOTSWITCH_RELAY : return 40;

        case pinMapping::LEFT_PWM_OUTPUT : return 44;
        case pinMapping::RIGHT_PWM_OUTPUT : return 46;

        case pinMapping::LEFT_ENCODER_1: return 19;
        case pinMapping::LEFT_ENCODER_2: return 18;
        case pinMapping::RIGHT_ENCODER_1: return 20;
        case pinMapping::RIGHT_ENCODER_2: return 21;

        case pinMapping::TEST_POT_POSITIVE: return A6;
        case pinMapping::TEST_POT_WIPER: return A7;

        case pinMapping::RC_PWM_IN_L: return A11;
        case pinMapping::RC_DEADMAN: return 2;
    }
}

} // End of namespace
