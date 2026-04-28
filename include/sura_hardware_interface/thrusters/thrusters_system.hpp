#pragma once

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "sura_hardware_interface/thrusters/thruster_mapper.hpp"

namespace sura_hardware_interface
{

class ThrustersSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ThrustersSystem)

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
  void publish_zero_command();
  void reset_thruster_filter();

  std::string environment_;
  std::string lookup_csv_path_;
  std::string stonefish_topic_{"/catamaran/controller/thruster_setpoints_sim"};

  bool is_active_{false};

  std::vector<double> force_commands_;
  std::vector<double> force_states_;
  std::vector<double> last_force_commands_;
  std::vector<double> last_outputs_;
  std::vector<double> filtered_force_commands_;
  double thruster_lpf_alpha_{1.0};

  // Per-joint flag: if true, mirror PWM around 1500 us in real hardware mode.
  std::vector<bool> inverted_flags_;

  ThrusterMapper mapper_;

  rclcpp::Node::SharedPtr internal_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_stonefish_pub_;

  bool navigator_initialized_{false};
  bool pwm_enabled_{false};
  double pwm_frequency_hz_{50.0};

  std::vector<int> pwm_channel_indices_;
};

}  // namespace sura_hardware_interface
