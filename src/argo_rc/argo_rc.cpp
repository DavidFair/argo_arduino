
#include <Arduino.h>

#include "arduino_hardware.hpp"
#include "argo_rc_lib.hpp"
#include "move.hpp"
#include "unique_ptr.hpp"

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

void setup() { argoRcLib.setup(); }

void loop() { argoRcLib.loop(); }