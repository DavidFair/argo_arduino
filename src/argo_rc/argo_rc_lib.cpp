
//#include <Encoder.h>
#include <stdint.h>

#include "arduino_enums.hpp"
#include "arduino_interface.hpp"

#include "argo_rc_lib.hpp"

//#define TEST_POT_ENABLED 
//#define RC_PWM_ENABLED
//
//#define pinMapping::LEFT_FORWARD_RELAY 23
//#define LEFT_REVERSE_RELAY 25
//#define pinMapping::RIGHT_FORWARD_RELAY 27
//#define pinMapping::RIGHT_REVERSE_RELAY 29
//
//#define pinMapping::LEFT_FOOTSWITCH_RELAY 42
//#define pinMapping::RIGHT_FOOTSWITCH_RELAY 40
//
//
//#define pinMapping::LEFT_PWM_OUTPUT 44
//#define pinMapping::RIGHT_PWM_OUTPUT 46
//
//#define pinMapping.LEFT_ENCODER_1 19
//#define pinMapping.LEFT_ENCODER_2 18
//#define pinMapping::RIGHT_ENCODER_1 20
//#define pinMapping::RIGHT_ENCODER_2 21
//
//#define pinMapping::TEST_POT_POSITIVE A6
//#define pinMapping::TEST_POT_WIPER A7
//
////#define pinMapping::RC_PWM_IN_L A10
//#define pinMapping::RC_PWM_IN_L A11
//#define pinMapping::RC_DEADMAN 2

//Encoder left_encoder(pinMapping.LEFT_ENCODER_1,pinMapping.LEFT_ENCODER_2);
//Encoder right_encoder(pinMapping::RIGHT_ENCODER_1,pinMapping::RIGHT_ENCODER_2);

long left_oldPosition  = -999;
long right_oldPosition  = -999;

// --- RC PWM input ---------------------------------
typedef struct {
  uint8_t edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int  lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[6 + 1];
volatile static uint8_t PCintLast;
// --- RC PWM input ---------------------------------


ArgoRc::ArgoRc(ArduinoInterface &hardwareInterface) : 
  m_hardwareInterface(hardwareInterface)
  {}



void ArgoRc::setup() 
{
  m_hardwareInterface.serialBegin(115200);

  m_hardwareInterface.pinMode(pinMapping::LEFT_ENCODER_1, digitalIO::INPUT_PULLUP);
  m_hardwareInterface.pinMode(pinMapping::LEFT_ENCODER_2, digitalIO::INPUT_PULLUP);

  m_hardwareInterface.pinMode(pinMapping::RIGHT_ENCODER_1, digitalIO::INPUT_PULLUP);
  m_hardwareInterface.pinMode(pinMapping::RIGHT_ENCODER_2, digitalIO::INPUT_PULLUP);

  
  // put your setup code here, to run once:
  m_hardwareInterface.pinMode(pinMapping::LEFT_FORWARD_RELAY, digitalIO::OUTPUT);
  m_hardwareInterface.pinMode(pinMapping::LEFT_REVERSE_RELAY, digitalIO::OUTPUT);
  m_hardwareInterface.pinMode(pinMapping::RIGHT_FORWARD_RELAY, digitalIO::OUTPUT);
  m_hardwareInterface.pinMode(pinMapping::RIGHT_REVERSE_RELAY, digitalIO::OUTPUT);

  m_hardwareInterface.pinMode(pinMapping::LEFT_FOOTSWITCH_RELAY, digitalIO::OUTPUT);
  m_hardwareInterface.pinMode(pinMapping::RIGHT_FOOTSWITCH_RELAY, digitalIO::OUTPUT);
  footswitch_off(); 
  

  m_hardwareInterface.pinMode(pinMapping::TEST_POT_POSITIVE, digitalIO::OUTPUT);
  m_hardwareInterface.digitalWrite(pinMapping::TEST_POT_POSITIVE, digitalIO::HIGH);

  direction_relays_off();

#ifdef RC_PWM_ENABLED
  m_hardwareInterface.pinMode(pinMapping::RC_DEADMAN, digitalIO::INPUT);
  setup_rc();
#endif

  m_hardwareInterface.delay(1000);

  // Hardwire direction relays to forward
//#ifdef RC_PWM_ENABLED
//      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::LOW);
//      m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::LOW);
//#endif

//forward_left();
//reverse_right();

}


void ArgoRc::forward_left()
{
  footswitch_on();
  m_hardwareInterface.serialPrintln("forward_left");
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::LOW);
  m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::HIGH);
  
}

void ArgoRc::forward_right()
{
  footswitch_on();
  m_hardwareInterface.serialPrintln("                  forward_right");
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::LOW);
  m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::HIGH);
  
}

void ArgoRc::reverse_left()
{
  footswitch_on();
  m_hardwareInterface.serialPrintln("                                    reverse_left");
      m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::HIGH);
      m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::LOW);
}

void ArgoRc::reverse_right()
{
  footswitch_on();
  m_hardwareInterface.serialPrintln("                                                      reverse_right");
      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::HIGH);
      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::LOW);
  
}

void ArgoRc::footswitch_on()
{
     m_hardwareInterface.digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,digitalIO::LOW);
     m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,digitalIO::LOW);
}

void ArgoRc::footswitch_off()
{
     m_hardwareInterface.digitalWrite(pinMapping::LEFT_FOOTSWITCH_RELAY,digitalIO::HIGH);
     m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FOOTSWITCH_RELAY,digitalIO::HIGH);
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
  
  int test_pot_value = m_hardwareInterface.analogRead(pinMapping::TEST_POT_WIPER);

#ifdef RC_PWM_ENABLED
  if(m_hardwareInterface.digitalRead(pinMapping::RC_DEADMAN) == digitalIO::HIGH)
  {
    rc_pwm_left = pinData[0].lastGoodWidth;
    rc_pwm_right = pinData[1].lastGoodWidth;
//    if(rc_pwm_left < 1520 || rc_pwm_left > 1850)
  //    left_pwm = 0;
  //  else
      left_pwm = map(rc_pwm_left,1520,1850,0,255);
 //   if(rc_pwm_right < 1520 || rc_pwm_right > 1850)
  //    right_pwm = 0;
  //  else
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
    pinData[0].lastGoodWidth = 0;
    rc_pwm_right = 0;
    pinData[1].lastGoodWidth = 0;
    left_pwm = 0;
    right_pwm = 0;
     m_hardwareInterface.analogWrite(pinMapping::LEFT_PWM_OUTPUT, left_pwm);
     m_hardwareInterface.analogWrite(pinMapping::RIGHT_PWM_OUTPUT, right_pwm);
    // turn off all direction relays and footswitch
    footswitch_off(); 
    direction_relays_off();
    while(1)
    {
      // wait here forever - requires a reset
       m_hardwareInterface.serialPrintln(" DEADMAN SWITCH RELEASED - RESET ARDUINO! ");
       delay(500);
    }
  }
#endif

#ifdef TEST_POT_ENABLED
  left_pwm = map(test_pot_value,0,1023,0,255);
  right_pwm = map(test_pot_value,0,1023,0,255);
//#ifdef DEBUG_OUTPUT
//  m_hardwareInterface.serialPrint("  RIGHT PWM: ");
//  m_hardwareInterface.serialPrint(right_pwm);
//#endif
#endif


#ifdef DEBUG_OUTPUT_PWM
  if(m_hardwareInterface.digitalRead(pinMapping::RC_DEADMAN) == digitalIO::HIGH)
      m_hardwareInterface.serialPrint("ENABLED  ");
  m_hardwareInterface.serialPrint("LEFT PWM: ");
  m_hardwareInterface.serialPrint(left_pwm);
  m_hardwareInterface.serialPrint("  RIGHT PWM: ");
  m_hardwareInterface.serialPrintln(right_pwm);
#endif

   m_hardwareInterface.analogWrite(pinMapping::LEFT_PWM_OUTPUT, left_pwm);
   m_hardwareInterface.analogWrite(pinMapping::RIGHT_PWM_OUTPUT, right_pwm);

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
  m_hardwareInterface.serialPrint("  LPOS: ");
  m_hardwareInterface.serialPrint(left_newPosition);
  m_hardwareInterface.serialPrint("  RPOS: ");
  m_hardwareInterface.serialPrintln(right_newPosition);
#endif
  
//  delay(200);

/*
  if(Serial.available())
  {
    byte c = Serial.read();
    if(c == 'l')
      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::HIGH);
    else if(c == 'p')
      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::LOW);

    if(c == 'k')
      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::HIGH);
    else if(c == 'o')
      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::LOW);
    

    if(c == 'a')
      m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::HIGH);
    else if(c == 'q')
      m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::LOW);

    if(c == 's')
      m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::HIGH);
    else if(c == 'w')
      m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::LOW);
  
  
  }    


*/
  
}


void ArgoRc::direction_relays_off()
{

      m_hardwareInterface.serialPrintln("RELAYS OFF");
      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_FORWARD_RELAY,digitalIO::HIGH);
      m_hardwareInterface.digitalWrite(pinMapping::RIGHT_REVERSE_RELAY,digitalIO::HIGH);
      m_hardwareInterface.digitalWrite(pinMapping::LEFT_REVERSE_RELAY,digitalIO::HIGH);
      m_hardwareInterface.digitalWrite(pinMapping::LEFT_FORWARD_RELAY,digitalIO::HIGH);
}


void ArgoRc::setup_rc()
{
  // Set ADC8 - ADC15 to input (0) using the port register
  m_hardwareInterface.setPortBitmask(portMapping::DDRK, 0);
  
  // enable PCINT 18 to 23 - the individual pins are
  // PCMSK2 register
  m_hardwareInterface.orPortBitmask(portMapping::PCICR, (1 << portControlValues::PCIE2));
  m_hardwareInterface.setPortBitmask(portMapping::PCMSK2, 0xFC);
}

ISR(PCINT2_vect)
{
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint32_t currentTime;
  uint32_t time;
  
  // get the pin states for the indicated port.
  curr = PINK & 0xFC;
  mask = curr ^ PCintLast;
  PCintLast = curr;
  
  currentTime = micros();
  
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 6; i++) {
    bit = 0x04 << i;
    if (bit & mask) {
      // for each pin changed, record time of change
      if (bit & PCintLast) {
        time = currentTime - pinData[i].fallTime;
        pinData[i].riseTime = currentTime;
        if ((time >= 10000) && (time <= 26000))
          pinData[i].edge = 1;
        else
          pinData[i].edge = 0; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[i].riseTime;
        pinData[i].fallTime = currentTime;
        if ((time >= 800) && (time <= 2200) && (pinData[i].edge == 1)) {
          pinData[i].lastGoodWidth = time;
          pinData[i].edge = 0;
        }
      }
    }
  }
}



