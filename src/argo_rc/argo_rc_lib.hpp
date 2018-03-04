#ifndef ARGO_RC_LIB_H_
#define ARGO_RC_LIB_H_

#include <stdint.h>

#include "arduino_interface.hpp"
#include "argo_encoder.hpp"
#include "move.hpp"
#include "unique_ptr.hpp"

namespace ArgoRcLib {

class ArgoRc {
public:
  explicit ArgoRc(
      Argo::unique_ptr<Hardware::ArduinoInterface> &&hardwareInterface,
      Argo::unique_ptr<ArgoEncoder> &&encoder);
  ~ArgoRc() = default;

  // We cannot copy due to the fact we hold unique ptrs to hardware and encoders
  ArgoRc(ArgoRc &) = delete;
  ArgoRc operator=(ArgoRc &) = delete;

  // Move constructors - used in unit tests
  ArgoRc(ArgoRc &&other)
      : m_hardwareInterface(Argo::move(other.m_hardwareInterface)),
        m_encoders(Argo::move(other.m_encoders)) {}

  ArgoRc &operator=(ArgoRc &&other) {
    m_hardwareInterface.reset(other.m_hardwareInterface.release());
    m_encoders.reset(other.m_encoders.release());
    return *this;
  }

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

  void setupEncoders();

  void setup_rc();

  Argo::unique_ptr<Hardware::ArduinoInterface> m_hardwareInterface;
  Argo::unique_ptr<ArgoEncoder> m_encoders;
};

} // namespace ArgoRcLib

#endif // ARGO_RC_LIB_H_