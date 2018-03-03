#ifndef ARGO_RC_LIB_H_
#define ARGO_RC_LIB_H_

#include <stdint.h>

#include "arduino_interface.hpp"
#include "encoder_interface.hpp"
#include "unique_ptr.hpp"

namespace ArgoRcLib {

struct EncoderData {
  int32_t leftEncoderVal{0};
  int32_t rightEncoderVal{0};
};

class ArgoRc {
public:
  ArgoRc(Hardware::ArduinoInterface *hardwareInterface);
  ~ArgoRc() = default;

  // We cannot copy due to the fact we hold unique ptrs to hardware and encoders
  ArgoRc(ArgoRc &) = delete;
  ArgoRc operator=(ArgoRc &) = delete;

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

  void readEncoderOutput();

  void setupDigitalPins();

  void setup_rc();

  Hardware::ArduinoInterface *m_hardwareInterface;
  Argo::unique_ptr<EncoderLib::EncoderInterface> m_leftEncoder;
  Argo::unique_ptr<EncoderLib::EncoderInterface> m_rightEncoder;
  EncoderData m_lastEncoderVal;
};

} // namespace ArgoRcLib

#endif // ARGO_RC_LIB_H_