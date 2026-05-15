#pragma once

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sura_hardware_interface/actuators/light_interface.hpp"
#include "sura_hardware_interface/actuators/lights_bluerov_interface.hpp"

namespace sura_hardware_interface
{

class ActuatorsSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ActuatorsSystem)

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
  hardware_interface::return_type write_status_light(bool enabled);
  hardware_interface::return_type write_bluerov_lights(double pwm_us);

  std::string environment_{"real"};
  bool is_active_{false};

  bool has_status_light_{false};
  bool has_bluerov_lights_{false};
  std::string status_light_joint_name_;
  std::string bluerov_lights_joint_name_;

  int status_light_channel_{1};
  double status_light_command_{1.0};
  double status_light_state_{1.0};
  LightInterface status_light_;

  int bluerov_lights_channel_{11};
  double bluerov_lights_command_{1100.0};
  double bluerov_lights_state_{1100.0};
  LightsBluerovInterface bluerov_lights_;
};

}  // namespace sura_hardware_interface
