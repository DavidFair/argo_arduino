
#include <Arduino.h>

#include "arduino_hardware.hpp"
#include "argo_rc_lib.hpp"

// Instantiate argo_rc library at the global level so it doesn't drop out of
// scope
ArgoRcLib::ArgoRc argoRcLib;

void setup() {
  // The hardware abstraction must live as long as the program runs so there
  // is no corresponding free
  Hardware::ArduinoInterface *hardwareImpl = new Hardware::ArduinoHardware();
  if (!hardwareImpl) {
    exit(-1);
  }
  argoRcLib.setup(hardwareImpl);
}

void loop() { argoRcLib.loop(); }