#ifndef ARGO_RC_LIB_H_
#define ARGO_RC_LIB_H_

#include "arduino_interface.hpp"

namespace ArgoRcLib{

class ArgoRc {
public:
    ArgoRc() = default;
    ~ArgoRc() = default;
    
    void setup(Hardware::ArduinoInterface *hardwareInterface);

    void forward_left();

    void forward_right();

    void reverse_left();

    void reverse_right();

    void footswitch_on();

    void footswitch_off();

    void loop();

    void direction_relays_off();

    void setup_rc();

private:

    Hardware::ArduinoInterface *m_hardwareInterface {nullptr};
};

}

#endif //ARGO_RC_LIB_H_