#ifndef ARGO_RC_LIB_H_
#define ARGO_RC_LIB_H_

#include <stdint.h>

#include "Encoder.hpp"
#include "PidController.hpp"
#include "SerialComms.hpp"
#include "arduino_interface.hpp"
#include "move.hpp"
#include "unique_ptr.hpp"

namespace ArgoRcLib {

struct PwmTargets {
  PwmTargets() : PwmTargets(0, 0) {}

  PwmTargets(int leftPwmTarget, int rightPwmTarget)
      : leftPwm(leftPwmTarget), rightPwm(rightPwmTarget) {}

  int leftPwm{0};
  int rightPwm{0};
};

class ArgoRc {
public:
  explicit ArgoRc(Hardware::ArduinoInterface &hardwareInterface);

  ~ArgoRc() = default;

  // Disable copy constructors that way we don't get temporaries
  ArgoRc(ArgoRc &) = delete;
  ArgoRc operator=(ArgoRc &) = delete;

  // Move constructors - used in unit tests
  ArgoRc(ArgoRc &&other)
      : m_hardwareInterface(other.m_hardwareInterface),
        m_encoders(other.m_encoders), m_commsObject(other.m_commsObject) {}

  ArgoRc &operator=(ArgoRc &&other) {
    m_hardwareInterface = other.m_hardwareInterface;
    m_encoders = Libs::move(other.m_encoders);
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
  bool checkDeadmanSwitch();

  void enterDeadmanFail();

  PwmTargets readPwmInput();

  void setupDigitalPins();

  PwmTargets setMotorTarget(int speed, int steer);

  Hardware::ArduinoInterface &m_hardwareInterface;
  Hardware::Encoder m_encoders;

  SerialComms m_commsObject;
};

} // namespace ArgoRcLib

#endif // ARGO_RC_LIB_H_