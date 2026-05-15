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
constexpr double kMinBluerovLightsPwmUs = 1100.0;
constexpr double kMaxBluerovLightsPwmUs = 1900.0;

bool parse_bool_like(double value)
{
  return std::isfinite(value) && value >= 0.5;
}

double clamp_bluerov_lights_pwm(double pwm_us)
{
  if (!std::isfinite(pwm_us)) {
    return kMinBluerovLightsPwmUs;
  }
  return std::clamp(pwm_us, kMinBluerovLightsPwmUs, kMaxBluerovLightsPwmUs);
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

  has_status_light_ = false;
  has_bluerov_lights_ = false;
  status_light_joint_name_.clear();
  bluerov_lights_joint_name_.clear();

  for (const auto & joint : info_.joints) {
    const bool is_status_light =
      joint.command_interfaces.size() == 1 &&
      joint.command_interfaces[0].name == "enabled" &&
      joint.state_interfaces.size() == 1 &&
      joint.state_interfaces[0].name == "enabled";
    const bool is_bluerov_lights =
      joint.command_interfaces.size() == 1 &&
      joint.command_interfaces[0].name == "pwm_us" &&
      joint.state_interfaces.size() == 1 &&
      joint.state_interfaces[0].name == "pwm_us";

    if (is_status_light) {
      if (has_status_light_) {
        RCLCPP_ERROR(kLogger, "ActuatorsSystem supports only one status light joint");
        return hardware_interface::CallbackReturn::ERROR;
      }
      has_status_light_ = true;
      status_light_joint_name_ = joint.name;
    } else if (is_bluerov_lights) {
      if (has_bluerov_lights_) {
        RCLCPP_ERROR(kLogger, "ActuatorsSystem supports only one BlueROV lights joint");
        return hardware_interface::CallbackReturn::ERROR;
      }
      has_bluerov_lights_ = true;
      bluerov_lights_joint_name_ = joint.name;
      const auto channel_it = joint.parameters.find("channel");
      if (channel_it == joint.parameters.end()) {
        RCLCPP_ERROR(
          kLogger,
          "Joint %s is missing required parameter 'channel'",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      try {
        bluerov_lights_channel_ = std::stoi(channel_it->second);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          kLogger,
          "Invalid channel parameter for joint %s: %s",
          joint.name.c_str(),
          e.what());
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      RCLCPP_ERROR(
        kLogger,
        "Unsupported actuator joint '%s'. Expected command/state interface 'enabled' or 'pwm_us'",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
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
  bluerov_lights_command_ = kMinBluerovLightsPwmUs;
  bluerov_lights_state_ = kMinBluerovLightsPwmUs;
  is_active_ = false;

  RCLCPP_INFO(
    kLogger,
    "ActuatorsSystem initialized for environment='%s', status_light=%s, bluerov_lights=%s",
    environment_.c_str(),
    has_status_light_ ? status_light_joint_name_.c_str() : "disabled",
    has_bluerov_lights_ ? bluerov_lights_joint_name_.c_str() : "disabled");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  status_light_command_ = 1.0;
  status_light_state_ = 1.0;
  bluerov_lights_command_ = kMinBluerovLightsPwmUs;
  bluerov_lights_state_ = kMinBluerovLightsPwmUs;

  try {
    if (has_status_light_ &&
      !status_light_.initialize(info_, environment_.c_str(), status_light_channel_))
    {
      RCLCPP_ERROR(kLogger, "Failed to initialize status light interface");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_bluerov_lights_ &&
      !bluerov_lights_.initialize(info_, environment_.c_str(), bluerov_lights_channel_))
    {
      RCLCPP_ERROR(kLogger, "Failed to initialize BlueROV lights interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Failed to initialize actuator interfaces: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(kLogger, "Failed to initialize actuator interfaces: unknown error");
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
  bluerov_lights_command_ = kMinBluerovLightsPwmUs;
  bluerov_lights_state_ = kMinBluerovLightsPwmUs;
  if (has_status_light_) {
    status_light_.cleanup();
  }
  if (has_bluerov_lights_) {
    bluerov_lights_.cleanup();
  }
  RCLCPP_INFO(kLogger, "ActuatorsSystem cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  status_light_command_ = 1.0;
  status_light_state_ = 1.0;
  bluerov_lights_command_ = kMinBluerovLightsPwmUs;
  bluerov_lights_state_ = kMinBluerovLightsPwmUs;
  if (has_status_light_) {
    status_light_.cleanup();
  }
  if (has_bluerov_lights_) {
    bluerov_lights_.cleanup();
  }
  RCLCPP_INFO(kLogger, "ActuatorsSystem shutdown completed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = true;
  status_light_command_ = 1.0;
  status_light_state_ = 1.0;
  bluerov_lights_command_ = kMinBluerovLightsPwmUs;
  bluerov_lights_state_ = kMinBluerovLightsPwmUs;

  try {
    if (has_status_light_ && !status_light_.activate()) {
      RCLCPP_ERROR(kLogger, "Failed to activate status light interface");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_bluerov_lights_ && !bluerov_lights_.activate()) {
      RCLCPP_ERROR(kLogger, "Failed to activate BlueROV lights interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Failed to activate actuator interfaces: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(kLogger, "Failed to activate actuator interfaces: unknown error");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "ActuatorsSystem activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  if (has_status_light_) {
    status_light_.deactivate();
  }
  if (has_bluerov_lights_) {
    bluerov_lights_.deactivate();
  }
  RCLCPP_INFO(kLogger, "ActuatorsSystem deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;
  if (has_status_light_) {
    status_light_.cleanup();
  }
  if (has_bluerov_lights_) {
    bluerov_lights_.cleanup();
  }
  RCLCPP_ERROR(kLogger, "ActuatorsSystem entered error state");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ActuatorsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  if (has_status_light_) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        status_light_joint_name_, "enabled", &status_light_state_));
  }
  if (has_bluerov_lights_) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        bluerov_lights_joint_name_, "pwm_us", &bluerov_lights_state_));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ActuatorsSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  if (has_status_light_) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        status_light_joint_name_, "enabled", &status_light_command_));
  }
  if (has_bluerov_lights_) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        bluerov_lights_joint_name_, "pwm_us", &bluerov_lights_command_));
  }
  return command_interfaces;
}

hardware_interface::return_type ActuatorsSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (has_status_light_) {
    status_light_state_ = parse_bool_like(status_light_command_) ? 1.0 : 0.0;
  }
  if (has_bluerov_lights_) {
    bluerov_lights_state_ = clamp_bluerov_lights_pwm(bluerov_lights_command_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ActuatorsSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  if (has_status_light_ &&
    write_status_light(parse_bool_like(status_light_command_)) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  if (has_bluerov_lights_ &&
    write_bluerov_lights(bluerov_lights_command_) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
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

hardware_interface::return_type ActuatorsSystem::write_bluerov_lights(double pwm_us)
{
  const double clamped_pwm_us = clamp_bluerov_lights_pwm(pwm_us);
  bluerov_lights_state_ = clamped_pwm_us;
  try {
    if (!bluerov_lights_.write(clamped_pwm_us)) {
      RCLCPP_ERROR(kLogger, "BlueROV lights interface rejected write");
      return hardware_interface::return_type::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Failed to write BlueROV lights command: %s", e.what());
    return hardware_interface::return_type::ERROR;
  } catch (...) {
    RCLCPP_ERROR(kLogger, "Failed to write BlueROV lights command: unknown error");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::ActuatorsSystem,
  hardware_interface::SystemInterface)
