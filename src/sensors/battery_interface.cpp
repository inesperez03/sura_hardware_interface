#include "sura_hardware_interface/sensors/battery_interface.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

namespace sura_hardware_interface
{

namespace
{

#ifdef TARGET_RASPBERRY
constexpr AdcChannel kVoltageChannel = AdcChannel::Ch3;
constexpr AdcChannel kCurrentChannel = AdcChannel::Ch2;

constexpr double kPsmVoltageMultiplier = 11.0;
constexpr double kPsmCurrentPerVolt = 37.8788;
constexpr double kPsmCurrentOffset = 0.330;
#endif

bool has_state(
  const std::unordered_map<std::string, double> & states,
  const std::string & name)
{
  return states.find(name) != states.end();
}

}  // namespace

bool BatteryInterface::initialize(
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

  const auto read_rate_it = sensor_info.parameters.find("read_rate_hz");
  if (read_rate_it != sensor_info.parameters.end()) {
    try {
      read_rate_hz_ = std::stod(read_rate_it->second);
    } catch (const std::exception &) {
      return false;
    }
  }

  if (environment_ != "real" && environment_ != "sim") {
    return false;
  }

#ifdef TARGET_RASPBERRY
  if (environment_ == "real") {
    init();
  }
#endif

  initialized_ = true;
  active_ = false;
  return true;
}

bool BatteryInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool BatteryInterface::deactivate()
{
  active_ = false;
  return true;
}

bool BatteryInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  return true;
}

bool BatteryInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  double voltage = std::numeric_limits<double>::quiet_NaN();
  double current = std::numeric_limits<double>::quiet_NaN();
  double present = 0.0;

#ifdef TARGET_RASPBERRY
  if (environment_ == "real") {
    const double voltage_adc = static_cast<double>(read_adc(kVoltageChannel));
    const double current_adc = static_cast<double>(read_adc(kCurrentChannel));

    voltage = voltage_adc * kPsmVoltageMultiplier;
    current = (current_adc - kPsmCurrentOffset) * kPsmCurrentPerVolt;

    if (current < 0.0) {
      current = 0.0;
    }

    present = 1.0;
  }
#endif

  if (has_state(states, "voltage")) {
    states["voltage"] = voltage;
  }

  if (has_state(states, "current")) {
    states["current"] = current;
  }

  if (has_state(states, "present")) {
    states["present"] = present;
  }

  return true;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::BatteryInterface,
  sura_hardware_interface::SensorInterfaceBase)
