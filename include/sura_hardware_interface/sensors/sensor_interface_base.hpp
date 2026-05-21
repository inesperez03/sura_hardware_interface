#pragma once

#include <string>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sura_hardware_interface
{

class SensorInterfaceBase
{
public:
  virtual ~SensorInterfaceBase() = default;

  virtual bool initialize(
    const hardware_interface::ComponentInfo & sensor_info,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & environment,
    const rclcpp::Node::SharedPtr & sim_node) = 0;

  virtual bool activate() = 0;

  virtual bool deactivate() = 0;

  virtual bool cleanup() = 0;

  virtual bool read(std::unordered_map<std::string, double> & states) = 0;
};

}  // namespace sura_hardware_interface
