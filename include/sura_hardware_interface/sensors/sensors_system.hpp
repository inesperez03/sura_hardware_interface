#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/range.hpp>
#ifndef TARGET_RASPBERRY
#include <stonefish_ros2/msg/dvl.hpp>
#endif

#include "sura_hardware_interface/sensors/imu_interface.hpp"
#include "sura_hardware_interface/sensors/magnetometer_interface.hpp"
#include "sura_hardware_interface/sensors/pressure_interface.hpp"
#include "sura_hardware_interface/sensors/battery_interface.hpp"
#include "sura_hardware_interface/sensors/dvl75_interface.hpp"
#include "sura_hardware_interface/sensors/gps_interface.hpp"

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
  void configure_sim_subscribers();
  void reset_sim_subscribers();
  std::string parameter_or(const std::string & name, const std::string & default_value) const;
  double parameter_or(const std::string & name, double default_value) const;

  void sim_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void sim_magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
  void sim_pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
#ifndef TARGET_RASPBERRY
  void sim_dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);
#endif
  void sim_dvl_altitude_callback(const sensor_msgs::msg::Range::SharedPtr msg);
  void sim_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  hardware_interface::HardwareInfo info_;

  ImuInterface imu_;
  MagnetometerInterface magnetometer_;
  PressureInterface pressure_;
  BatteryInterface battery_;
  DvlInterface dvl_;
  GpsInterface gps_;

  bool has_imu_{false};
  bool has_magnetometer_{false};
  bool has_pressure_{false};
  bool has_battery_{false};
  bool has_dvl_{false};
  bool has_gps_{false};
  bool is_active_{false};

  std::string environment_{"real"};
  std::string robot_namespace_{"cirtesub"};
  std::string sim_imu_topic_;
  std::string sim_magnetometer_topic_;
  std::string sim_pressure_topic_;
  std::string sim_dvl_topic_;
  std::string sim_dvl_altitude_topic_;
  std::string sim_gps_topic_;
  double pressure_offset_pa_{101325.0};
  double sim_dvl_confidence_{100.0};

  rclcpp::Node::SharedPtr sim_node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sim_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sim_magnetometer_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sim_pressure_sub_;
#ifndef TARGET_RASPBERRY
  rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr sim_dvl_sub_;
#endif
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sim_dvl_altitude_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sim_gps_sub_;

  std::string imu_sensor_name_{"imu_sensor"};
  std::string magnetometer_sensor_name_{"magnetometer_sensor"};
  std::string pressure_sensor_name_{"pressure_sensor"};
  std::string battery_sensor_name_{"battery_sensor"};
  std::string dvl_sensor_name_{"dvl_sensor"};
  std::string gps_sensor_name_{"gps_sensor"};

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

  double battery_voltage_{0.0};
  double battery_current_{0.0};
  double battery_present_{0.0};

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

  double gps_latitude_{0.0};
  double gps_longitude_{0.0};
  double gps_altitude_{0.0};
  double gps_valid_{0.0};
};

}  // namespace sura_hardware_interface
