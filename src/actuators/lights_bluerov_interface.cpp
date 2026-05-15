#include "sura_hardware_interface/actuators/lights_bluerov_interface.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace sura_hardware_interface
{

namespace
{

constexpr double kMinLightsPwmUs = 1100.0;
constexpr double kMaxLightsPwmUs = 1900.0;

#ifdef TARGET_RASPBERRY
uint16_t pulse_us_to_counts(double pulse_us, double freq_hz)
{
  const double period_us = 1e6 / freq_hz;
  const double counts = pulse_us * 4096.0 / period_us;

  const long rounded = std::lround(counts);
  return static_cast<uint16_t>(std::clamp(rounded, 0L, 4095L));
}
#endif

double clamp_lights_pwm(double pwm_us)
{
  if (!std::isfinite(pwm_us)) {
    return kMinLightsPwmUs;
  }
  return std::clamp(pwm_us, kMinLightsPwmUs, kMaxLightsPwmUs);
}

}  // namespace

bool LightsBluerovInterface::initialize(
  const hardware_interface::HardwareInfo &,
  const char * environment,
  int lights_channel)
{
  if (initialized_) {
    return true;
  }

  environment_ = environment;
  lights_channel_ = lights_channel;

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    set_raspberry_pi_version(Raspberry::Pi4);
    set_navigator_version(NavigatorVersion::Version1);

    init();
    navigator_initialized_ = true;

    set_pwm_freq_hz(pwm_frequency_hz_);
    set_pwm_enable(true);
    pwm_enabled_ = true;
    write_pwm_us(kMinLightsPwmUs);
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

bool LightsBluerovInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return write(kMinLightsPwmUs);
}

bool LightsBluerovInterface::deactivate()
{
  if (initialized_ && active_) {
    (void)write(kMinLightsPwmUs);
  }

  active_ = false;
  return true;
}

bool LightsBluerovInterface::cleanup()
{
  if (initialized_) {
    const bool was_active = active_;
    active_ = true;
    if (was_active || environment_ == "real") {
      (void)write(kMinLightsPwmUs);
    }
  }

  active_ = false;
  initialized_ = false;
  navigator_initialized_ = false;
  pwm_enabled_ = false;
  return true;
}

bool LightsBluerovInterface::write(double pwm_us)
{
  if (!initialized_ || !active_) {
    return false;
  }

  if (environment_ == "sim") {
    return true;
  }

  return write_pwm_us(pwm_us);
}

bool LightsBluerovInterface::write_pwm_us(double pwm_us)
{
#ifdef TARGET_RASPBERRY
  if (!navigator_initialized_ || !pwm_enabled_) {
    return false;
  }

  const uint16_t counts = pulse_us_to_counts(clamp_lights_pwm(pwm_us), pwm_frequency_hz_);
  set_pwm_channel_value(static_cast<uintptr_t>(lights_channel_), counts);
  return true;
#else
  (void)pwm_us;
  return environment_ == "sim";
#endif
}

}  // namespace sura_hardware_interface
