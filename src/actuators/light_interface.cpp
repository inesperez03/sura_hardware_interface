#include "sura_hardware_interface/actuators/light_interface.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

#include <cstdint>
#include <string>

namespace sura_hardware_interface
{

bool LightInterface::initialize(
  const hardware_interface::HardwareInfo &,
  const char * environment,
  int status_light_channel)
{
  if (initialized_) {
    return true;
  }

  environment_ = environment;
  status_light_channel_ = status_light_channel;

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    set_raspberry_pi_version(Raspberry::Pi4);
    set_navigator_version(NavigatorVersion::Version1);

    init();
    navigator_initialized_ = true;

    // Share the Navigator PWM block with the thruster stack. We keep the same
    // 50 Hz frequency so channel 1 can be used like a binary relay output.
    set_pwm_freq_hz(pwm_frequency_hz_);
    set_pwm_enable(true);
    pwm_enabled_ = true;
    set_pwm_channel_duty_cycle(static_cast<uintptr_t>(status_light_channel_), 1.0F);
#else
    return false;
#endif
  } else if (environment_ != "sim") {
    return false;
  }

  initialized_ = true;
  active_ = false;
  return true;
}

bool LightInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return write(true);
}

bool LightInterface::deactivate()
{
  active_ = false;
  return true;
}

bool LightInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  navigator_initialized_ = false;
  pwm_enabled_ = false;
  return true;
}

bool LightInterface::write(bool enabled)
{
  if (!initialized_ || !active_) {
    return false;
  }

  if (environment_ == "sim") {
    return true;
  }

#ifdef TARGET_RASPBERRY
  if (!navigator_initialized_ || !pwm_enabled_) {
    return false;
  }

  // NavLight turns off when the signal is pulled to ground. We model the
  // logical command as enabled=true, so channel HIGH means light ON.
  set_pwm_channel_duty_cycle(
    static_cast<uintptr_t>(status_light_channel_),
    enabled ? 1.0F : 0.0F);
  return true;
#else
  (void)enabled;
  return false;
#endif
}

}  // namespace sura_hardware_interface
