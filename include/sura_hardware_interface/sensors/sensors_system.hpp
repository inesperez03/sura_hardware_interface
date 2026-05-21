#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sura_hardware_interface/sensors/sensor_interface_base.hpp"

namespace sura_hardware_interface
{

class SensorsSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorsSystem)

  SensorsSystem();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
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
  struct SensorInstance
  {
    hardware_interface::ComponentInfo info;
    pluginlib::UniquePtr<SensorInterfaceBase> interface;

    std::unordered_map<std::string, double> control_states;
    std::unordered_map<std::string, double> read_states;

    std::mutex states_mutex;

    double read_rate_hz{0.0};
    bool use_thread{false};

    std::atomic_bool read_thread_running{false};
    std::thread read_thread;

    std::atomic_bool last_read_ok{true};
  };

  void start_sensor_threads();
  void stop_sensor_threads();

  void start_sim_spin_thread();
  void stop_sim_spin_thread();

  void sensor_read_loop(SensorInstance * sensor);

  void reset_states();

  static std::string parameter_or(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & name,
    const std::string & default_value);

  static bool has_parameter(
    const hardware_interface::ComponentInfo & component,
    const std::string & name);

  static double component_double_parameter_or(
    const hardware_interface::ComponentInfo & component,
    const std::string & name,
    double default_value);

  std::string environment_{"real"};

  bool is_configured_{false};
  bool is_active_{false};

  std::vector<std::unique_ptr<SensorInstance>> sensors_;

  pluginlib::ClassLoader<SensorInterfaceBase> sensor_interface_loader_;

  rclcpp::Node::SharedPtr sim_node_;

  std::atomic_bool sim_spin_running_{false};
  std::thread sim_spin_thread_;
};

}  // namespace sura_hardware_interface
