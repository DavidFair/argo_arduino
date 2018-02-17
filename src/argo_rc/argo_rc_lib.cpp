
//#include <Encoder.h>
#include <stdint.h>

#include "arduino_interface.hpp"

#include "argo_rc_lib.hpp"

//#define TEST_POT_ENABLED 
//#define RC_PWM_ENABLED
//
//#define LEFT_FORWARD_RELAY 23
//#define LEFT_REVERSE_RELAY 25
//#define RIGHT_FORWARD_RELAY 27
//#define RIGHT_REVERSE_RELAY 29
//
//#define LEFT_FOOTSWITCH_RELAY 42
//#define RIGHT_FOOTSWITCH_RELAY 40
//
//
//#define LEFT_PWM_OUTPUT 44
//#define RIGHT_PWM_OUTPUT 46
//
//#define LEFT_ENCODER_1 19
//#define LEFT_ENCODER_2 18
//#define RIGHT_ENCODER_1 20
//#define RIGHT_ENCODER_2 21
//
//#define TEST_POT_POSITIVE A6
//#define TEST_POT_WIPER A7
//
////#define RC_PWM_IN_L A10
//#define RC_PWM_IN_L A11
//#define RC_DEADMAN 2

//Encoder left_encoder(LEFT_ENCODER_1,LEFT_ENCODER_2);
//Encoder right_encoder(RIGHT_ENCODER_1,RIGHT_ENCODER_2);

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

  m_hardwareInterface.pinMode(LEFT_ENCODER_1, digitalIO::INPUT_PULLUP);
  m_hardwareInterface.pinMode(LEFT_ENCODER_2, digitalIO::INPUT_PULLUP);

  m_hardwareInterface.pinMode(RIGHT_ENCODER_1, digitalIO::INPUT_PULLUP);
  m_hardwareInterface.pinMode(RIGHT_ENCODER_2, digitalIO::INPUT_PULLUP);

  
  // put your setup code here, to run once:
  m_hardwareInterface.pinMode(LEFT_FORWARD_RELAY, digitalIO::OUTPUT);
  m_hardwareInterface.pinMode(LEFT_REVERSE_RELAY, digitalIO::OUTPUT);
  m_hardwareInterface.pinMode(RIGHT_FORWARD_RELAY, digitalIO::OUTPUT);
  m_hardwareInterface.pinMode(RIGHT_REVERSE_RELAY, digitalIO::OUTPUT);

  m_hardwareInterface.pinMode(LEFT_FOOTSWITCH_RELAY, digitalIO::OUTPUT);
  m_hardwareInterface.pinMode(RIGHT_FOOTSWITCH_RELAY, digitalIO::OUTPUT);
  footswitch_off(); 
  

  m_hardwareInterface.pinMode(TEST_POT_POSITIVE, digitalIO::OUTPUT);
  digitalWrite(TEST_POT_POSITIVE,HIGH);

  direction_relays_off();

#ifdef RC_PWM_ENABLED
  m_hardwareInterface.pinMode(RC_DEADMAN, digitalIO::INPUT);
  setup_rc();
#endif

delay(1000);

  // Hardwire direction relays to forward
//#ifdef RC_PWM_ENABLED
//      digitalWrite(RIGHT_FORWARD_RELAY,LOW);
//      digitalWrite(LEFT_FORWARD_RELAY,LOW);
//#endif

//forward_left();
//reverse_right();

}


void forward_left()
{
  footswitch_on();
  Serial.println("forward_left");
      digitalWrite(LEFT_FORWARD_RELAY,LOW);
      digitalWrite(LEFT_REVERSE_RELAY,HIGH);
  
}

void forward_right()
{
  footswitch_on();
  Serial.println("                  forward_right");
      digitalWrite(RIGHT_FORWARD_RELAY,LOW);
      digitalWrite(RIGHT_REVERSE_RELAY,HIGH);
  
}

void reverse_left()
{
  footswitch_on();
  Serial.println("                                    reverse_left");
      digitalWrite(LEFT_FORWARD_RELAY,HIGH);
      digitalWrite(LEFT_REVERSE_RELAY,LOW);
}

void reverse_right()
{
  footswitch_on();
  Serial.println("                                                      reverse_right");
      digitalWrite(RIGHT_FORWARD_RELAY,HIGH);
      digitalWrite(RIGHT_REVERSE_RELAY,LOW);
  
}

void footswitch_on()
{
     digitalWrite(LEFT_FOOTSWITCH_RELAY,LOW);
     digitalWrite(RIGHT_FOOTSWITCH_RELAY,LOW);
}

void footswitch_off()
{
     digitalWrite(LEFT_FOOTSWITCH_RELAY,HIGH);
     digitalWrite(RIGHT_FOOTSWITCH_RELAY,HIGH);
}

//#define DEBUG_OUTPUT
#define DEBUG_OUTPUT_PWM

int rc_pwm_left = 0;
int rc_pwm_right = 0;


void loop() 
{
  // put your main code here, to run repeatedly:
  int left_pwm = 0;  
  int right_pwm = 0;  
  
  int test_pot_value = analogRead(TEST_POT_WIPER);

#ifdef RC_PWM_ENABLED
  if(digitalRead(RC_DEADMAN) == HIGH)
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
    analogWrite(LEFT_PWM_OUTPUT, left_pwm);
    analogWrite(RIGHT_PWM_OUTPUT, right_pwm);
    // turn off all direction relays and footswitch
    footswitch_off(); 
    direction_relays_off();
    while(1)
    {
      // wait here forever - requires a reset
       Serial.println(" DEADMAN SWITCH RELEASED - RESET ARDUINO! ");
       delay(500);
    }
  }
#endif

#ifdef TEST_POT_ENABLED
  left_pwm = map(test_pot_value,0,1023,0,255);
  right_pwm = map(test_pot_value,0,1023,0,255);
//#ifdef DEBUG_OUTPUT
//  Serial.print("  RIGHT PWM: ");
//  Serial.print(right_pwm);
//#endif
#endif


#ifdef DEBUG_OUTPUT_PWM
  if(digitalRead(RC_DEADMAN) == HIGH)
      Serial.print("ENABLED  ");
  Serial.print("LEFT PWM: ");
  Serial.print(left_pwm);
  Serial.print("  RIGHT PWM: ");
  Serial.println(right_pwm);
#endif

  analogWrite(LEFT_PWM_OUTPUT, left_pwm);
  analogWrite(RIGHT_PWM_OUTPUT, right_pwm);

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
  Serial.print("  LPOS: ");
  Serial.print(left_newPosition);
  Serial.print("  RPOS: ");
  Serial.println(right_newPosition);
#endif
  
//  delay(200);

/*
  if(Serial.available())
  {
    byte c = Serial.read();
    if(c == 'l')
      digitalWrite(RIGHT_FORWARD_RELAY,HIGH);
    else if(c == 'p')
      digitalWrite(RIGHT_FORWARD_RELAY,LOW);

    if(c == 'k')
      digitalWrite(RIGHT_REVERSE_RELAY,HIGH);
    else if(c == 'o')
      digitalWrite(RIGHT_REVERSE_RELAY,LOW);
    

    if(c == 'a')
      digitalWrite(LEFT_FORWARD_RELAY,HIGH);
    else if(c == 'q')
      digitalWrite(LEFT_FORWARD_RELAY,LOW);

    if(c == 's')
      digitalWrite(LEFT_REVERSE_RELAY,HIGH);
    else if(c == 'w')
      digitalWrite(LEFT_REVERSE_RELAY,LOW);
  
  
  }    


*/
  
}


void direction_relays_off()
{

      Serial.println("RELAYS OFF");
      digitalWrite(RIGHT_FORWARD_RELAY,HIGH);
      digitalWrite(RIGHT_REVERSE_RELAY,HIGH);
      digitalWrite(LEFT_REVERSE_RELAY,HIGH);
      digitalWrite(LEFT_FORWARD_RELAY,HIGH);
}


void setup_rc()
{
  DDRK = 0; // pins as input
  
  // enable PCINT 18 to 23
  PCICR |= (1 << PCIE2);
  PCMSK2 = 0xFC;
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



