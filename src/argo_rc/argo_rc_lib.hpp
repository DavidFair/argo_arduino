#ifndef ARGO_RC_LIB_H_
#define ARGO_RC_LIB_H_

#include "arduino_interface.hpp"

namespace ArgoRcLib {

class ArgoRc {
public:
  ArgoRc(Hardware::ArduinoInterface *hardwareInterface);
  ~ArgoRc() = default;

  void setup();

  void forward_left();

  void forward_right();

  void reverse_left();

  void reverse_right();

  void footswitch_on();

  void footswitch_off();

  void loop();

  void direction_relays_off();

private:
  void setupDigitalPins();

  void setup_rc();

  Hardware::ArduinoInterface *m_hardwareInterface{nullptr};
};

} // namespace ArgoRcLib

#endif // ARGO_RC_LIB_H_