#include "sura_hardware_interface/actuators/actuators_system.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/exceptions.hpp"

namespace sura_hardware_interface
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("ActuatorsSystem");

}  // namespace

ActuatorsSystem::ActuatorsSystem()
: actuator_interface_loader_(
    "sura_hardware_interface",
    "sura_hardware_interface::ActuatorInterfaceBase")
{
}

std::string ActuatorsSystem::parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  const std::string & default_value)
{
  const auto it = parameters.find(name);
  return it == parameters.end() ? default_value : it->second;
}

bool ActuatorsSystem::has_parameter(
  const hardware_interface::ComponentInfo & component,
  const std::string & name)
{
  return component.parameters.find(name) != component.parameters.end();
}

hardware_interface::CallbackReturn ActuatorsSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  environment_ = parameter_or(info_.hardware_parameters, "environment", "real");

  if (environment_ != "real" && environment_ != "sim") {
    RCLCPP_ERROR(
      kLogger,
      "Unsupported environment '%s'. Use 'real' or 'sim'.",
      environment_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  actuators_.clear();
  actuators_.reserve(info_.joints.size());

  for (const auto & joint_info : info_.joints) {
    if (!has_parameter(joint_info, "interface")) {
      RCLCPP_ERROR(
        kLogger,
        "Actuator joint '%s' has no required parameter 'interface'",
        joint_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string interface_name = joint_info.parameters.at("interface");

    pluginlib::UniquePtr<ActuatorInterfaceBase> actuator_interface;

    try {
      actuator_interface =
        actuator_interface_loader_.createUniqueInstance(interface_name);
    } catch (const pluginlib::PluginlibException & e) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to create interface '%s' for actuator joint '%s': %s",
        interface_name.c_str(),
        joint_info.name.c_str(),
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto actuator = std::make_unique<ActuatorInstance>();
    actuator->info = joint_info;
    actuator->interface = std::move(actuator_interface);

    for (const auto & command_interface : joint_info.command_interfaces) {
      actuator->command_states[command_interface.name] = 0.0;
    }

    for (const auto & state_interface : joint_info.state_interfaces) {
      actuator->read_states[state_interface.name] = 0.0;
    }

    actuators_.push_back(std::move(actuator));
  }

  is_configured_ = false;
  is_active_ = false;

  RCLCPP_INFO(
    kLogger,
    "ActuatorsSystem initialized for environment='%s' with %zu actuator(s)",
    environment_.c_str(),
    actuators_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ActuatorsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & actuator : actuators_) {
    for (const auto & state_interface : actuator->info.state_interfaces) {
      state_interfaces.emplace_back(
        actuator->info.name,
        state_interface.name,
        &actuator->read_states[state_interface.name]);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ActuatorsSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & actuator : actuators_) {
    for (const auto & command_interface : actuator->info.command_interfaces) {
      command_interfaces.emplace_back(
        actuator->info.name,
        command_interface.name,
        &actuator->command_states[command_interface.name]);
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  reset_states();

  for (auto & actuator : actuators_) {
    if (!actuator->interface->initialize(actuator->info, info_, environment_)) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to initialize actuator joint '%s'",
        actuator->info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  is_configured_ = true;
  is_active_ = false;

  RCLCPP_INFO(kLogger, "ActuatorsSystem configured");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!is_configured_) {
    RCLCPP_ERROR(kLogger, "Cannot activate ActuatorsSystem before configure");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & actuator : actuators_) {
    if (!actuator->interface->activate()) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to activate actuator joint '%s'",
        actuator->info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  is_active_ = true;

  RCLCPP_INFO(kLogger, "ActuatorsSystem activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  is_active_ = false;

  for (auto & actuator : actuators_) {
    if (!actuator->interface->deactivate()) {
      RCLCPP_WARN(
        kLogger,
        "Failed to deactivate actuator joint '%s'",
        actuator->info.name.c_str());
    }
  }

  RCLCPP_INFO(kLogger, "ActuatorsSystem deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  is_active_ = false;

  for (auto & actuator : actuators_) {
    if (!actuator->interface->cleanup()) {
      RCLCPP_WARN(
        kLogger,
        "Failed to cleanup actuator joint '%s'",
        actuator->info.name.c_str());
    }
  }

  reset_states();
  is_configured_ = false;

  RCLCPP_INFO(kLogger, "ActuatorsSystem cleaned up");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  is_active_ = false;

  for (auto & actuator : actuators_) {
    if (!actuator->interface->cleanup()) {
      RCLCPP_WARN(
        kLogger,
        "Failed to cleanup actuator joint '%s'",
        actuator->info.name.c_str());
    }
  }

  reset_states();
  is_configured_ = false;

  RCLCPP_INFO(kLogger, "ActuatorsSystem shutdown completed");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  is_active_ = false;

  for (auto & actuator : actuators_) {
    if (!actuator->interface->cleanup()) {
      RCLCPP_WARN(
        kLogger,
        "Failed to cleanup actuator joint '%s'",
        actuator->info.name.c_str());
    }
  }

  reset_states();
  is_configured_ = false;

  RCLCPP_ERROR(kLogger, "ActuatorsSystem entered error state");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ActuatorsSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (auto & actuator : actuators_) {
    if (!actuator->interface->read(actuator->command_states, actuator->read_states)) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to read actuator joint '%s'",
        actuator->info.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ActuatorsSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  for (auto & actuator : actuators_) {
    if (!actuator->interface->write(actuator->command_states, actuator->read_states)) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to write actuator joint '%s'",
        actuator->info.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

void ActuatorsSystem::reset_states()
{
  for (auto & actuator : actuators_) {
    for (auto & command : actuator->command_states) {
      command.second = 0.0;
    }

    for (auto & state : actuator->read_states) {
      state.second = 0.0;
    }
  }
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::ActuatorsSystem,
  hardware_interface::SystemInterface)
