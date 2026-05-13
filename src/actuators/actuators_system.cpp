#include "sura_hardware_interface/actuators/actuators_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace sura_hardware_interface
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("sura_hardware_interface");

bool parse_bool_like(double value)
{
  return std::isfinite(value) && value >= 0.5;
}

}  // namespace

hardware_interface::CallbackReturn ActuatorsSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    environment_ = info_.hardware_parameters.at("environment");
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(kLogger, "Missing hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(kLogger, "ActuatorsSystem expects exactly one joint for the status light");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joint = info_.joints.front();
  if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "enabled") {
    RCLCPP_ERROR(
      kLogger,
      "Joint %s must have exactly one command interface named 'enabled'",
      joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != "enabled") {
    RCLCPP_ERROR(
      kLogger,
      "Joint %s must have exactly one state interface named 'enabled'",
      joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto channel_it = info_.hardware_parameters.find("status_light_channel");
  if (channel_it != info_.hardware_parameters.end()) {
    try {
      status_light_channel_ = std::stoi(channel_it->second);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Invalid status_light_channel: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  status_light_command_ = 1.0;
  status_light_state_ = 1.0;
  is_active_ = false;

  RCLCPP_INFO(
    kLogger,
    "ActuatorsSystem initialized for environment='%s', status_light_channel=%d",
    environment_.c_str(),
    status_light_channel_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  status_light_command_ = 1.0;
  status_light_state_ = 1.0;

  try {
    if (!status_light_.initialize(info_, environment_.c_str(), status_light_channel_)) {
      RCLCPP_ERROR(kLogger, "Failed to initialize status light interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Failed to initialize status light interface: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(kLogger, "Failed to initialize status light interface: unknown error");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "ActuatorsSystem configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  status_light_command_ = 1.0;
  status_light_state_ = 1.0;
  status_light_.cleanup();
  RCLCPP_INFO(kLogger, "ActuatorsSystem cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  status_light_command_ = 1.0;
  status_light_state_ = 1.0;
  status_light_.cleanup();
  RCLCPP_INFO(kLogger, "ActuatorsSystem shutdown completed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = true;
  status_light_command_ = 1.0;
  status_light_state_ = 1.0;

  try {
    if (!status_light_.activate()) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Failed to activate status light interface: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(kLogger, "Failed to activate status light interface: unknown error");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "ActuatorsSystem activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  status_light_.deactivate();
  RCLCPP_INFO(kLogger, "ActuatorsSystem deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  status_light_.cleanup();
  RCLCPP_ERROR(kLogger, "ActuatorsSystem entered error state");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ActuatorsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints.front().name, "enabled", &status_light_state_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ActuatorsSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints.front().name, "enabled", &status_light_command_));
  return command_interfaces;
}

hardware_interface::return_type ActuatorsSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  status_light_state_ = parse_bool_like(status_light_command_) ? 1.0 : 0.0;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ActuatorsSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  return write_status_light(parse_bool_like(status_light_command_));
}

hardware_interface::return_type ActuatorsSystem::write_status_light(bool enabled)
{
  status_light_state_ = enabled ? 1.0 : 0.0;
  try {
    if (!status_light_.write(enabled)) {
      RCLCPP_ERROR(kLogger, "Status light interface rejected write");
      return hardware_interface::return_type::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Failed to write status light command: %s", e.what());
    return hardware_interface::return_type::ERROR;
  } catch (...) {
    RCLCPP_ERROR(kLogger, "Failed to write status light command: unknown error");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::ActuatorsSystem,
  hardware_interface::SystemInterface)
