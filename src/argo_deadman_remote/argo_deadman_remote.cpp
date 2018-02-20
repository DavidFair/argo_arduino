// remote control for dead man for argo
//

#define VERSION 1.0

//#define DEBUG

#include <RFM69.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h> //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR
//HARDWARE *************
//*********************************************************************************************
#define NODEID                                                                 \
  7 // must be unique for each node on same network (range up to 254, 255 is
    // used for broadcast)
#define NETWORKID                                                              \
  200 // the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID 1
// Match frequency to the hardware version of the radio on your Moteino
// (uncomment one):
#define FREQUENCY RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY                                                             \
  "thisisargodeadma" // exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW // uncomment only for RFM69HW! Leave out if you have RFM69W!
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION
//CONTROL
//*********************************************************************************************

#ifdef __AVR_ATmega1284P__
#define LED 15      // Moteino MEGAs have LEDs on D15
#define FLASH_SS 23 // and FLASH SS on D23
#else
#define LED 9      // Moteinos have LEDs on D9
#define FLASH_SS 8 // and FLASH SS on D8
#endif

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

unsigned char payload[12];

// --------------------------------------------------------
// SETUP
// SETUP
// SETUP
// --------------------------------------------------------

void setup(void) {
  Serial.begin(115200);
  Serial.print(F("REMOTE Starting!"));
  Serial.print(F("   Rev: "));
  Serial.println(VERSION);

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); // uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
#ifdef ENABLE_ATC
  radio.enableAutoPower(-70);
#endif
#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif
}

void loop() {
  delay(100);

  payload[0] = 'q';

  radio.send(GATEWAYID, payload, 1);
}
