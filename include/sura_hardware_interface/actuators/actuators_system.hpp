#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sura_hardware_interface/actuators/actuator_interface_base.hpp"

namespace sura_hardware_interface
{

class ActuatorsSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ActuatorsSystem)

  ActuatorsSystem();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct ActuatorInstance
  {
    hardware_interface::ComponentInfo info;
    pluginlib::UniquePtr<ActuatorInterfaceBase> interface;

    std::unordered_map<std::string, double> command_states;
    std::unordered_map<std::string, double> read_states;
  };

  void reset_states();

  static std::string parameter_or(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & name,
    const std::string & default_value);

  static bool has_parameter(
    const hardware_interface::ComponentInfo & component,
    const std::string & name);

  std::string environment_{"real"};
  bool is_configured_{false};
  bool is_active_{false};

  std::vector<std::unique_ptr<ActuatorInstance>> actuators_;

  pluginlib::ClassLoader<ActuatorInterfaceBase> actuator_interface_loader_;
};

}  // namespace sura_hardware_interface
