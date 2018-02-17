#ifndef ARGO_RC_LIB_H_
#define ARGO_RC_LIB_H_

#include "arduino_interface.hpp"

void direction_relays_off();

void footswitch_on();
void footswitch_off();

void setup_rc();

class ArgoRc {
public:
    ArgoRc(ArduinoInterface &hardwareInterface);
    
    void setup();

private:
    ArduinoInterface m_hardwareInterface;

};

#endif //ARGO_RC_LIB_H_