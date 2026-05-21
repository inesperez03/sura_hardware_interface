#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include "sura_hardware_interface/sensors/sensor_interface_base.hpp"

namespace sura_hardware_interface
{

#ifdef TARGET_RASPBERRY
namespace
{
class MS5837Local;
}
#endif

class PressureInterface : public SensorInterfaceBase
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

  double read_rate_hz_{200.0};
  double pressure_offset_pa_{101325.0};

  int i2c_bus_{6};
  uint8_t i2c_address_{0x76};

  rclcpp::Node::SharedPtr sim_node_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;

  std::string stonefish_topic_;

  double last_pressure_pa_{101325.0};
  bool has_last_pressure_{false};

#ifdef TARGET_RASPBERRY
  std::unique_ptr<MS5837Local> pressure_sensor_;
#endif
};

}  // namespace sura_hardware_interface