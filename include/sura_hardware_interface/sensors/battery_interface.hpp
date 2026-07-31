#pragma once

#include <limits>
#include <string>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "sura_hardware_interface/sensors/sensor_interface_base.hpp"

namespace sura_hardware_interface
{

class BatteryInterface : public SensorInterfaceBase
{
public:
  bool initialize(
    const hardware_interface::ComponentInfo & sensor_info,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & environment,
    const rclcpp::Node::SharedPtr & sim_node) override;

  bool activate() override;

  bool deactivate() override;

  bool cleanup() override;

  bool read(std::unordered_map<std::string, double> & states) override;

private:
  bool initialized_{false};
  bool active_{false};

  std::string sensor_name_;
  std::string environment_{"real"};

  double read_rate_hz_{1.0};
  rclcpp::Node::SharedPtr sim_node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  std::string stonefish_topic_;

  double last_voltage_{std::numeric_limits<double>::quiet_NaN()};
  double last_current_{std::numeric_limits<double>::quiet_NaN()};
  double last_present_{0.0};
};

}  // namespace sura_hardware_interface
