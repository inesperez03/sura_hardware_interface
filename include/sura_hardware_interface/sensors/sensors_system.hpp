#pragma once

#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "sura_hardware_interface/sensors/imu_interface.hpp"
#include "sura_hardware_interface/sensors/magnetometer_interface.hpp"
#include "sura_hardware_interface/sensors/pressure_interface.hpp"
#include "sura_hardware_interface/sensors/dvl75_interface.hpp"

namespace sura_hardware_interface
{

class SensorsSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorsSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

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

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void reset_sensor_state();

  hardware_interface::HardwareInfo info_;

  ImuInterface imu_;
  MagnetometerInterface magnetometer_;
  PressureInterface pressure_;
  DvlInterface dvl_;

  bool has_imu_{false};
  bool has_magnetometer_{false};
  bool has_pressure_{false};
  bool has_dvl_{false};
  bool is_active_{false};

  std::string imu_sensor_name_{"imu_sensor"};
  std::string magnetometer_sensor_name_{"magnetometer_sensor"};
  std::string pressure_sensor_name_{"pressure_sensor"};
  std::string dvl_sensor_name_{"dvl_sensor"};

  double orientation_x_{0.0};
  double orientation_y_{0.0};
  double orientation_z_{0.0};
  double orientation_w_{1.0};

  double angular_velocity_x_{0.0};
  double angular_velocity_y_{0.0};
  double angular_velocity_z_{0.0};

  double linear_acceleration_x_{0.0};
  double linear_acceleration_y_{0.0};
  double linear_acceleration_z_{0.0};

  double magnetic_field_x_{0.0};
  double magnetic_field_y_{0.0};
  double magnetic_field_z_{0.0};

  double fluid_pressure_{0.0};

  double dvl_distance_z_{0.0};
  double dvl_confidence_{0.0};

  double dvl_linear_velocity_x_{0.0};
  double dvl_linear_velocity_y_{0.0};
  double dvl_linear_velocity_z_{0.0};

  double dvl_angular_velocity_x_{0.0};
  double dvl_angular_velocity_y_{0.0};
  double dvl_angular_velocity_z_{0.0};

  double dvl_gps_latitude_{0.0};
  double dvl_gps_longitude_{0.0};
  double dvl_gps_altitude_{0.0};
  double dvl_gps_valid_{0.0};
};

}  // namespace sura_hardware_interface