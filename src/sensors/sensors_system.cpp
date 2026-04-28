#include "sura_hardware_interface/sensors/sensors_system.hpp"

#include <algorithm>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sura_hardware_interface
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("SensorsSystem");
}  // namespace

hardware_interface::CallbackReturn SensorsSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(kLogger, "Failed to initialize base SystemInterface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  reset_sensor_state();
  is_active_ = false;

  const auto has_sensor = [this](const std::string & sensor_name) {
    return std::any_of(
      info_.sensors.begin(),
      info_.sensors.end(),
      [&sensor_name](const auto & sensor) {
        return sensor.name == sensor_name;
      });
  };

  has_imu_ = has_sensor(imu_sensor_name_);
  has_magnetometer_ = has_sensor(magnetometer_sensor_name_);
  has_pressure_ = has_sensor(pressure_sensor_name_);

  if (!has_imu_) {
    RCLCPP_ERROR(
      kLogger,
      "Sensor '%s' not found in ros2_control description",
      imu_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!has_magnetometer_) {
    RCLCPP_ERROR(
      kLogger,
      "Sensor '%s' not found in ros2_control description",
      magnetometer_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!has_pressure_) {
    RCLCPP_ERROR(
      kLogger,
      "Sensor '%s' not found in ros2_control description",
      pressure_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    kLogger,
    "SensorsSystem initialized. IMU='%s', Magnetometer='%s', Pressure='%s' detected.",
    imu_sensor_name_.c_str(),
    magnetometer_sensor_name_.c_str(),
    pressure_sensor_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Configuring SensorsSystem...");

  if (!has_imu_) {
    RCLCPP_ERROR(kLogger, "Cannot configure SensorsSystem because no IMU sensor was detected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!has_magnetometer_) {
    RCLCPP_ERROR(
      kLogger,
      "Cannot configure SensorsSystem because no magnetometer sensor was detected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!has_pressure_) {
    RCLCPP_ERROR(
      kLogger,
      "Cannot configure SensorsSystem because no pressure sensor was detected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  reset_sensor_state();

  if (!imu_.initialize(info_)) {
    RCLCPP_ERROR(kLogger, "Failed to initialize IMU interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  is_active_ = false;

  RCLCPP_INFO(kLogger, "SensorsSystem configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Activating SensorsSystem...");

  if (!has_imu_) {
    RCLCPP_ERROR(kLogger, "Cannot activate SensorsSystem because no IMU sensor was detected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!imu_.activate()) {
    RCLCPP_ERROR(kLogger, "Failed to activate IMU");
    return hardware_interface::CallbackReturn::ERROR;
  }

  is_active_ = true;

  RCLCPP_INFO(kLogger, "SensorsSystem activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Deactivating SensorsSystem...");

  is_active_ = false;

  if (has_imu_ && !imu_.deactivate()) {
    RCLCPP_ERROR(kLogger, "Failed to deactivate IMU");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "SensorsSystem deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SensorsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(
    (has_imu_ ? 10U : 0U) +
    (has_magnetometer_ ? 3U : 0U) +
    (has_pressure_ ? 1U : 0U));

  if (has_imu_) {
    interfaces.emplace_back(imu_sensor_name_, "orientation.x", &orientation_x_);
    interfaces.emplace_back(imu_sensor_name_, "orientation.y", &orientation_y_);
    interfaces.emplace_back(imu_sensor_name_, "orientation.z", &orientation_z_);
    interfaces.emplace_back(imu_sensor_name_, "orientation.w", &orientation_w_);

    interfaces.emplace_back(imu_sensor_name_, "angular_velocity.x", &angular_velocity_x_);
    interfaces.emplace_back(imu_sensor_name_, "angular_velocity.y", &angular_velocity_y_);
    interfaces.emplace_back(imu_sensor_name_, "angular_velocity.z", &angular_velocity_z_);

    interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.x", &linear_acceleration_x_);
    interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.y", &linear_acceleration_y_);
    interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.z", &linear_acceleration_z_);
  }

  if (has_magnetometer_) {
    interfaces.emplace_back(
      magnetometer_sensor_name_, "magnetic_field.x", &magnetic_field_x_);
    interfaces.emplace_back(
      magnetometer_sensor_name_, "magnetic_field.y", &magnetic_field_y_);
    interfaces.emplace_back(
      magnetometer_sensor_name_, "magnetic_field.z", &magnetic_field_z_);
  }

  if (has_pressure_) {
    interfaces.emplace_back(
      pressure_sensor_name_, "fluid_pressure", &fluid_pressure_);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> SensorsSystem::export_command_interfaces()
{
  return {};
}

hardware_interface::CallbackReturn SensorsSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Cleaning up SensorsSystem...");

  is_active_ = false;

  if (has_imu_ && !imu_.cleanup()) {
    RCLCPP_ERROR(kLogger, "Failed to cleanup IMU");
    return hardware_interface::CallbackReturn::ERROR;
  }

  reset_sensor_state();

  RCLCPP_INFO(kLogger, "SensorsSystem cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(kLogger, "Shutting down SensorsSystem...");

  is_active_ = false;

  if (has_imu_) {
    (void)imu_.deactivate();
    (void)imu_.cleanup();
  }

  reset_sensor_state();

  RCLCPP_INFO(kLogger, "SensorsSystem shutdown completed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(kLogger, "SensorsSystem entered error state");

  is_active_ = false;

  if (has_imu_) {
    (void)imu_.deactivate();
    (void)imu_.cleanup();
  }

  reset_sensor_state();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SensorsSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  if (has_imu_) {
    const bool imu_ok = imu_.read(
      orientation_x_, orientation_y_, orientation_z_, orientation_w_,
      angular_velocity_x_, angular_velocity_y_, angular_velocity_z_,
      linear_acceleration_x_, linear_acceleration_y_, linear_acceleration_z_);

    if (!imu_ok) {
      RCLCPP_ERROR(kLogger, "Failed to read IMU data");
      return hardware_interface::return_type::ERROR;
    }
  }

  if (has_magnetometer_) {
    const bool mag_ok = magnetometer_.read(
      magnetic_field_x_, magnetic_field_y_, magnetic_field_z_);

    if (!mag_ok) {
      RCLCPP_ERROR(kLogger, "Failed to read magnetometer data");
      return hardware_interface::return_type::ERROR;
    }
  }

  if (has_pressure_) {
    const bool pressure_ok = pressure_.read(fluid_pressure_);

    if (!pressure_ok) {
      RCLCPP_ERROR(kLogger, "Failed to read pressure data");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorsSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

void SensorsSystem::reset_sensor_state()
{
  orientation_x_ = 0.0;
  orientation_y_ = 0.0;
  orientation_z_ = 0.0;
  orientation_w_ = 1.0;

  angular_velocity_x_ = 0.0;
  angular_velocity_y_ = 0.0;
  angular_velocity_z_ = 0.0;

  linear_acceleration_x_ = 0.0;
  linear_acceleration_y_ = 0.0;
  linear_acceleration_z_ = 0.0;

  magnetic_field_x_ = 0.0;
  magnetic_field_y_ = 0.0;
  magnetic_field_z_ = 0.0;

  fluid_pressure_ = 0.0;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::SensorsSystem,
  hardware_interface::SystemInterface)