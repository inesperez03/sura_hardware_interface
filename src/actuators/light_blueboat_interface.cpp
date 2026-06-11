#include "sura_hardware_interface/actuators/light_blueboat_interface.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sura_hardware_interface/navigator_access.hpp"

namespace sura_hardware_interface
{

namespace
{

bool has_single_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & name)
{
  return interfaces.size() == 1 && interfaces[0].name == name;
}

int required_channel(const hardware_interface::ComponentInfo & actuator_info)
{
  const auto it = actuator_info.parameters.find("channel");

  if (it == actuator_info.parameters.end()) {
    throw std::runtime_error(
      "Actuator joint '" + actuator_info.name + "' is missing required parameter 'channel'");
  }

  return std::stoi(it->second);
}

bool command_enabled(const std::unordered_map<std::string, double> & commands)
{
  const auto it = commands.find("enabled");
  return it != commands.end() && std::isfinite(it->second) && it->second >= 0.5;
}

}  // namespace

bool LightBlueboatInterface::initialize(
  const hardware_interface::ComponentInfo & actuator_info,
  const hardware_interface::HardwareInfo &,
  const std::string & environment)
{
  if (initialized_) {
    return true;
  }

  if (
    !has_single_interface(actuator_info.command_interfaces, "enabled") ||
    !has_single_interface(actuator_info.state_interfaces, "enabled"))
  {
    return false;
  }

  environment_ = environment;

  try {
    status_light_channel_ = required_channel(actuator_info);
  } catch (const std::exception &) {
    return false;
  }

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    navigator_access::initialize_once();
    navigator_initialized_ = true;

    // Share the Navigator PWM block with the thruster stack. We keep the same
    // 50 Hz frequency so channel 1 can be used like a binary relay output.
    navigator_access::call(
      [&]()
      {
        set_pwm_freq_hz(pwm_frequency_hz_);
        set_pwm_enable(true);
        set_pwm_channel_duty_cycle(static_cast<uintptr_t>(status_light_channel_), 1.0F);
      });
    pwm_enabled_ = true;
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

bool LightBlueboatInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return write_enabled(true);
}

bool LightBlueboatInterface::deactivate()
{
  active_ = false;
  return true;
}

bool LightBlueboatInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  navigator_initialized_ = false;
  pwm_enabled_ = false;
  return true;
}

bool LightBlueboatInterface::read(
  const std::unordered_map<std::string, double> & commands,
  std::unordered_map<std::string, double> & states)
{
  const auto it = states.find("enabled");

  if (it != states.end()) {
    it->second = command_enabled(commands) ? 1.0 : 0.0;
  }

  return true;
}

bool LightBlueboatInterface::write(
  const std::unordered_map<std::string, double> & commands,
  std::unordered_map<std::string, double> & states)
{
  const bool enabled = command_enabled(commands);
  read(commands, states);
  return write_enabled(enabled);
}

bool LightBlueboatInterface::write_enabled(bool enabled)
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
  navigator_access::call(
    [&]()
    {
      set_pwm_channel_duty_cycle(
        static_cast<uintptr_t>(status_light_channel_),
        enabled ? 1.0F : 0.0F);
    });
  return true;
#else
  (void)enabled;
  return false;
#endif
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::LightBlueboatInterface,
  sura_hardware_interface::ActuatorInterfaceBase)
