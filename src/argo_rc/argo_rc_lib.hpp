#ifndef ARGO_RC_LIB_H_
#define ARGO_RC_LIB_H_

#include "arduino_interface.hpp"
#include "encoder_interface.hpp"
#include "unique_ptr.hpp"

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
  int constrainPwmInput(int initialValue);

  void enterDeadmanFail();

  void readPwmInput(const int leftPwmValue, const int rightPwmValue);

  void setupDigitalPins();

  void setup_rc();

  Hardware::ArduinoInterface *m_hardwareInterface;
  // Argo::unique_ptr<EncoderLib::EncoderInterface> m_leftEncoder;
  // Argo::unique_ptr<EncoderLib::EncoderInterface> m_rightEncoder;
};

} // namespace ArgoRcLib

#endif // ARGO_RC_LIB_H_