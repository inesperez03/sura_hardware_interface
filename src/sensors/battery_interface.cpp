#include "sura_hardware_interface/sensors/battery_interface.hpp"

#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
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

std::string get_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const std::string & default_value)
{
  const auto it = sensor_info.parameters.find(name);

  if (it == sensor_info.parameters.end()) {
    return default_value;
  }

  return it->second;
}

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

}  // namespace

bool BatteryInterface::initialize(
  const hardware_interface::ComponentInfo & sensor_info,
  const hardware_interface::HardwareInfo &,
  const std::string & environment,
  const rclcpp::Node::SharedPtr & sim_node)
{
  if (initialized_) {
    return true;
  }

  sensor_name_ = sensor_info.name;
  environment_ = environment;
  sim_node_ = sim_node;

  try {
    read_rate_hz_ = get_double_param_or(sensor_info, "read_rate_hz", read_rate_hz_);
    stonefish_topic_ = get_param_or(sensor_info, "stonefish_topic", stonefish_topic_);
  } catch (const std::exception & e) {
    std::cerr << "[Battery] Error parsing parameters for sensor '"
              << sensor_name_ << "': " << e.what() << std::endl;
    return false;
  }

  if (environment_ != "real" && environment_ != "sim") {
    std::cerr << "[Battery] Unsupported environment: " << environment_ << std::endl;
    return false;
  }

#ifdef TARGET_RASPBERRY
  if (environment_ == "real") {
    navigator_access::initialize_once();
  }
#endif

  if (environment_ == "sim") {
    if (!sim_node_) {
      std::cerr << "[Battery] sim_node is null in simulation mode" << std::endl;
      return false;
    }

    if (stonefish_topic_.empty()) {
      std::cerr << "[Battery] Missing parameter 'stonefish_topic' in sim mode"
                << std::endl;
      return false;
    }

    battery_sub_ =
      sim_node_->create_subscription<sensor_msgs::msg::BatteryState>(
      stonefish_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg)
      {
        last_voltage_ = msg->voltage;
        last_current_ = msg->current;
        last_percentage_ = msg->percentage;
        last_present_ = msg->present ? 1.0 : 0.0;
      });
  }

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

  battery_sub_.reset();
  sim_node_.reset();

  last_voltage_ = std::numeric_limits<double>::quiet_NaN();
  last_current_ = std::numeric_limits<double>::quiet_NaN();
  last_percentage_ = std::numeric_limits<double>::quiet_NaN();
  last_present_ = 0.0;

  return true;
}

bool BatteryInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  double voltage = std::numeric_limits<double>::quiet_NaN();
  double current = std::numeric_limits<double>::quiet_NaN();
  double percentage = std::numeric_limits<double>::quiet_NaN();
  double present = 0.0;

#ifdef TARGET_RASPBERRY
  if (environment_ == "real") {
    double voltage_adc = 0.0;
    double current_adc = 0.0;
    navigator_access::call(
      [&]()
      {
        voltage_adc = static_cast<double>(read_adc(kVoltageChannel));
        current_adc = static_cast<double>(read_adc(kCurrentChannel));
      });

    voltage = voltage_adc * kPsmVoltageMultiplier;
    current = (current_adc - kPsmCurrentOffset) * kPsmCurrentPerVolt;

    if (current < 0.0) {
      current = 0.0;
    }

    present = 1.0;
  }
#endif

  if (environment_ == "sim") {
    voltage = last_voltage_;
    current = last_current_;
    percentage = last_percentage_;
    present = last_present_;
  }

  if (has_state(states, "voltage")) {
    states["voltage"] = voltage;
  }

  if (has_state(states, "current")) {
    states["current"] = current;
  }

  if (has_state(states, "percentage")) {
    states["percentage"] = percentage;
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
