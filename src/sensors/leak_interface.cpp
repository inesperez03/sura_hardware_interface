#include "sura_hardware_interface/sensors/leak_interface.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "sura_hardware_interface/navigator_access.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

namespace sura_hardware_interface
{

namespace
{

double get_double_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const double default_value)
{
  const auto it = sensor_info.parameters.find(name);

  if (it == sensor_info.parameters.end()) {
    return default_value;
  }

  return std::stod(it->second);
}

void set_state_if_exists(
  std::unordered_map<std::string, double> & states,
  const std::string & name,
  const double value)
{
  const auto it = states.find(name);

  if (it != states.end()) {
    it->second = value;
  }
}

}  // namespace

bool LeakInterface::initialize(
  const hardware_interface::ComponentInfo & sensor_info,
  const hardware_interface::HardwareInfo &,
  const std::string & environment,
  const rclcpp::Node::SharedPtr &)
{
  if (initialized_) {
    return true;
  }

  sensor_name_ = sensor_info.name;
  environment_ = environment;

  if (environment_ != "real" && environment_ != "sim") {
    std::cerr << "[Leak] Unsupported environment: " << environment_ << std::endl;
    return false;
  }

  try {
    read_rate_hz_ = get_double_param_or(
      sensor_info,
      "read_rate_hz",
      read_rate_hz_);
  } catch (const std::exception & e) {
    std::cerr << "[Leak] Error parsing parameters for sensor '"
              << sensor_name_ << "': " << e.what() << std::endl;
    return false;
  }

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    navigator_access::initialize_once();
#endif
  }

  initialized_ = true;
  active_ = false;

  return true;
}

bool LeakInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool LeakInterface::deactivate()
{
  active_ = false;
  return true;
}

bool LeakInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  return true;
}

bool LeakInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  double leak = 0.0;

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    leak = navigator_access::call(
      []()
      {
        return read_leak();
      }) ? 1.0 : 0.0;
#else
    leak = 0.0;
#endif
  } else {
    leak = 0.0;
  }

  set_state_if_exists(states, "leak", leak);

  return true;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::LeakInterface,
  sura_hardware_interface::SensorInterfaceBase)
