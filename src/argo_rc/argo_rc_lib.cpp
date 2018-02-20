
//#include <Encoder.h>
#include <stdint.h>

#include "arduino_enums.hpp"
#include "arduino_lib_wrapper.hpp"
#include "arduino_interface.hpp"
#include "pinTimingData.hpp"

#include "argo_rc_lib.hpp"

//#define TEST_POT_ENABLED 
#define RC_PWM_ENABLED

//Encoder left_encoder(pinMapping.LEFT_ENCODER_1,pinMapping.LEFT_ENCODER_2);
//Encoder right_encoder(pinMapping::RIGHT_ENCODER_1,pinMapping::RIGHT_ENCODER_2);

using namespace ArduinoEnums;
using namespace Hardware;

namespace ArgoRcLib{

long left_oldPosition  = -999;
long right_oldPosition  = -999;


void ArgoRc::setup(ArduinoInterface *hardwareInterface) 
{
  m_hardwareInterface = hardwareInterface;
  m_hardwareInterface->serialBegin(115200);

  m_hardwareInterface->setPinMode(pinMapping::LEFT_ENCODER_1, digitalIO::E_INPUT_PULLUP);
  m_hardwareInterface->setPinMode(pinMapping::LEFT_ENCODER_2, digitalIO::E_INPUT_PULLUP);

  m_hardwareInterface->setPinMode(pinMapping::RIGHT_ENCODER_1, digitalIO::E_INPUT_PULLUP);
  m_hardwareInterface->setPinMode(pinMapping::RIGHT_ENCODER_2, digitalIO::E_INPUT_PULLUP);

  
  // put your setup code here, to run once:
  m_hardwareInterface->setPinMode(pinMapping::LEFT_FORWARD_RELAY, digitalIO::E_OUTPUT);
  m_hardwareInterface->setPinMode(pinMapping::LEFT_REVERSE_RELAY, digitalIO::E_OUTPUT);
  m_hardwareInterface->setPinMode(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::E_OUTPUT);
  m_hardwareInterface->setPinMode(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::E_OUTPUT);

  m_hardwareInterface->setPinMode(pinMapping::LEFT_FOOTSWITCH_RELAY, digitalIO::E_OUTPUT);
  m_hardwareInterface->setPinMode(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::E_OUTPUT);
  footswitch_off(); 
  

  m_hardwareInterface->setPinMode(pinMapping::TEST_POT_POSITIVE, digitalIO::E_OUTPUT);
  m_hardwareInterface->digitalWrite(pinMapping::TEST_POT_POSITIVE, digitalIO::E_HIGH);

  direction_relays_off();

#ifdef RC_PWM_ENABLED
  m_hardwareInterface->setPinMode(pinMapping::RC_DEADMAN, digitalIO::E_INPUT);
  setup_rc();
#endif

  m_hardwareInterface->delay(1000);

  // Hardwire direction relays to forward
//#ifdef RC_PWM_ENABLED
//      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::E_LOW);
//      m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::E_LOW);
//#endif

//forward_left();
//reverse_right();

}


void ArgoRc::forward_left()
{
  footswitch_on();
  m_hardwareInterface->serialPrintln("forward_left");
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::E_LOW);
  m_hardwareInterface->digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::E_HIGH);
  
}

void ArgoRc::forward_right()
{
  footswitch_on();
  m_hardwareInterface->serialPrintln("                  forward_right");
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::E_LOW);
  m_hardwareInterface->digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::E_HIGH);
  
}

void ArgoRc::reverse_left()
{
  footswitch_on();
  m_hardwareInterface->serialPrintln("                                    reverse_left");
      m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::E_HIGH);
      m_hardwareInterface->digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::E_LOW);
}

void ArgoRc::reverse_right()
{
  footswitch_on();
  m_hardwareInterface->serialPrintln("                                                      reverse_right");
      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::E_HIGH);
      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::E_LOW);
  
}

void ArgoRc::footswitch_on()
{
     m_hardwareInterface->digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,digitalIO::E_LOW);
     m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,digitalIO::E_LOW);
}

void ArgoRc::footswitch_off()
{
     m_hardwareInterface->digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,digitalIO::E_HIGH);
     m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,digitalIO::E_HIGH);
}

//#define DEBUG_OUTPUT
#define DEBUG_OUTPUT_PWM

int rc_pwm_left = 0;
int rc_pwm_right = 0;


void ArgoRc::loop() 
{
  // put your main code here, to run repeatedly:
  int left_pwm = 0;  
  int right_pwm = 0;  
  
  int test_pot_value = m_hardwareInterface->analogRead(pinMapping::TEST_POT_WIPER);

#ifdef RC_PWM_ENABLED
  if(m_hardwareInterface->digitalRead(pinMapping::RC_DEADMAN) == digitalIO::E_HIGH)
  {
    rc_pwm_left = timingData::g_pinData[0].lastGoodWidth;
    rc_pwm_right = timingData::g_pinData[1].lastGoodWidth;
    
    left_pwm = map(rc_pwm_left,1520,1850,0,255);
    right_pwm = map(rc_pwm_right,1520,1850,0,255);

    if(left_pwm > -40 && left_pwm < 40 & right_pwm > -40 & right_pwm < 40)
      footswitch_off();

    if(left_pwm > 40) forward_left();
    if(right_pwm > 40) forward_right();
    if(left_pwm < -40) reverse_left();
    if(right_pwm < -40) reverse_right();

    if(left_pwm < 0)left_pwm = -left_pwm;
    if(right_pwm < 0)right_pwm = -right_pwm;

    if(left_pwm > 255) left_pwm = 255;
    if(right_pwm > 255) right_pwm = 255;
  }
  else
  {
    rc_pwm_left = 0;
    timingData::g_pinData[0].lastGoodWidth = 0;
    rc_pwm_right = 0;
    timingData::g_pinData[1].lastGoodWidth = 0;
    left_pwm = 0;
    right_pwm = 0;
    m_hardwareInterface->analogWrite(pinMapping::LEFT_PWM_OUTPUT, left_pwm);
    m_hardwareInterface->analogWrite(pinMapping::RIGHT_PWM_OUTPUT, right_pwm);
    // turn off all direction relays and footswitch
    footswitch_off(); 
    direction_relays_off();
    while(1)
    {
      // wait here forever - requires a reset
       m_hardwareInterface->serialPrintln(" DEADMAN SWITCH RELEASED - RESET ARDUINO! ");
       m_hardwareInterface->delay(500);
    }
  }
#endif

#ifdef TEST_POT_ENABLED
  left_pwm = map(test_pot_value,0,1023,0,255);
  right_pwm = map(test_pot_value,0,1023,0,255);
//#ifdef DEBUG_OUTPUT
//  m_hardwareInterface->serialPrint("  RIGHT PWM: ");
//  m_hardwareInterface->serialPrint(right_pwm);
//#endif
#endif


#ifdef DEBUG_OUTPUT_PWM
  if(m_hardwareInterface->digitalRead(pinMapping::RC_DEADMAN) == digitalIO::E_HIGH)
      m_hardwareInterface->serialPrint("ENABLED  ");
  m_hardwareInterface->serialPrint("LEFT PWM: ");
  m_hardwareInterface->serialPrint(left_pwm);
  m_hardwareInterface->serialPrint("  RIGHT PWM: ");
  m_hardwareInterface->serialPrintln(right_pwm);
#endif

   m_hardwareInterface->analogWrite(pinMapping::LEFT_PWM_OUTPUT, left_pwm);
   m_hardwareInterface->analogWrite(pinMapping::RIGHT_PWM_OUTPUT, right_pwm);

/*
  long left_newPosition = left_encoder.read();
  if (left_newPosition != left_oldPosition) 
  {
    left_oldPosition = left_newPosition;
  }

  long right_newPosition = right_encoder.read();
  if (right_newPosition != right_oldPosition) 
  {
    right_oldPosition = right_newPosition;
  }
*/

#ifdef DEBUG_OUTPUT
  m_hardwareInterface->serialPrint("  LPOS: ");
  m_hardwareInterface->serialPrint(left_newPosition);
  m_hardwareInterface->serialPrint("  RPOS: ");
  m_hardwareInterface->serialPrintln(right_newPosition);
#endif
  
//  delay(200);

/*
  if(Serial.available())
  {
    byte c = Serial.read();
    if(c == 'l')
      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::E_HIGH);
    else if(c == 'p')
      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::E_LOW);

    if(c == 'k')
      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::E_HIGH);
    else if(c == 'o')
      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::E_LOW);
    

    if(c == 'a')
      m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::E_HIGH);
    else if(c == 'q')
      m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::E_LOW);

    if(c == 's')
      m_hardwareInterface->digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::E_HIGH);
    else if(c == 'w')
      m_hardwareInterface->digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::E_LOW);
  
  
  }    


*/
  
}


void ArgoRc::direction_relays_off()
{

      m_hardwareInterface->serialPrintln("RELAYS OFF");
      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::E_HIGH);
      m_hardwareInterface->digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::E_HIGH);
      m_hardwareInterface->digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::E_HIGH);
      m_hardwareInterface->digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::E_HIGH);
}

void ArgoRc::setup_rc()
{
  // Set ADC8 - ADC15 to input (0) using the port register
  m_hardwareInterface->setPortBitmask(portMapping::E_DDRK, 0);

  // enable PCINT which allows us to use the PCMSK register by bitshifting
  // 1 into the correct register location
  const auto pcie2EnableBitmask = 1 << static_cast<uint8_t>(portControlValues::E_PCIE2);
  m_hardwareInterface->orPortBitmask(portMapping::E_PCICR, pcie2EnableBitmask);
  
  // Set pins 18 to 23 as ISR triggers
  m_hardwareInterface->setPortBitmask(portMapping::E_PCMSK2, 0xFC);

  // The ISR is defined in the hardware implementation
}

} // End of namespace