#ifndef ARGO_RC_LIB_H_
#define ARGO_RC_LIB_H_

#include <stdint.h>

#include "ArduinoInterface.hpp"
#include "Encoder.hpp"
#include "PidController.hpp"
#include "SerialComms.hpp"
#include "Timer.hpp"
#include "move.hpp"

namespace ArgoRcLib {

/// Implements main loop to be executed on the vehicle
class ArgoRc {
public:
  /// Constructs an object which uses the given hardwareInterface
  explicit ArgoRc(Hardware::ArduinoInterface &hardwareInterface,
                  bool usePingTimeout = true);

  ~ArgoRc() = default;

  // Disable copy constructors that way we don't get temporaries
  ArgoRc(ArgoRc &) = delete;
  ArgoRc operator=(ArgoRc &) = delete;

  // Move constructors - used in unit tests
  ArgoRc(ArgoRc &&other)
      : m_usePingTimeout(other.m_usePingTimeout),
        m_pingTimer(Libs::move(other.m_pingTimer)),
        m_serialOutputTimer(Libs::move(other.m_serialOutputTimer)),
        m_hardwareInterface(other.m_hardwareInterface),
        m_encoders(Libs::move(other.m_encoders)),
        m_commsObject(Libs::move(other.m_commsObject)),
        m_pidController(Libs::move(other.m_pidController)) {}

  ArgoRc &operator=(ArgoRc &&other) {
    m_hardwareInterface = other.m_hardwareInterface;
    m_encoders = Libs::move(other.m_encoders);
    m_commsObject = Libs::move(other.m_commsObject);
    m_pidController = Libs::move(other.m_pidController);
    return *this;
  }
  /// Sets the various pins and serial line up
  void setup();

  /// Switches relays ready to move forward left
  void forward_left();

  /// Switches relays ready to move forward right
  void forward_right();

  /// Switches relays ready to move reverse left
  void reverse_left();

  /// Switches relays ready to move reverse right
  void reverse_right();

  /// Switches relays to engage the footswitch on for driving
  void footswitch_on();

  /// Switches relays to disengage the footswitch
  void footswitch_off();

  /// Main loop which controls the vehicle
  void loop();

  /// Switches all relays off to a safe state
  void direction_relays_off();

private:
  /// Applies the specified PWM targets to the motors
  void applyPwmOutput(const PwmTargets &pwmValues);

  /// Calculates the target speeds based on RC velocity and angular momentum
  Hardware::WheelSpeeds calculateVelocities(int velocity, int angularMomentum);

  /// Reads the current speed targets whilst using remote control
  Hardware::WheelSpeeds mapRcInput();

  /// Sets the digital pin modes to their apppropriate values
  void setupDigitalPins();

  /// Holds whether pings are considered for stopping the vehicle
  const bool m_usePingTimeout;

  /// Time till the next ping is sent
  Libs::Timer m_pingTimer;
  /// Time until the next serial output is sent
  Libs::Timer m_serialOutputTimer;

  /// Reference to the Arduino hardware
  Hardware::ArduinoInterface &m_hardwareInterface;
  /// Object which interfaces to the device encoders
  Hardware::Encoder m_encoders;

  /// Object which handles serial communications for the vehicle
  SerialComms m_commsObject;
  /// Object which handles calculating PWM values under ROS control
  PidController m_pidController;
};

} // namespace ArgoRcLib

#endif // ARGO_RC_LIB_H_