#include "sura_hardware_interface/sensors/sensors_system.hpp"

#include <chrono>
#include <exception>
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

const rclcpp::Logger kLogger = rclcpp::get_logger("SensorsSystem");

}  // namespace

SensorsSystem::SensorsSystem()
: sensor_interface_loader_(
    "sura_hardware_interface",
    "sura_hardware_interface::SensorInterfaceBase")
{
}

std::string SensorsSystem::parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  const std::string & default_value)
{
  const auto it = parameters.find(name);
  return it == parameters.end() ? default_value : it->second;
}

bool SensorsSystem::has_parameter(
  const hardware_interface::ComponentInfo & component,
  const std::string & name)
{
  return component.parameters.find(name) != component.parameters.end();
}

double SensorsSystem::component_double_parameter_or(
  const hardware_interface::ComponentInfo & component,
  const std::string & name,
  const double default_value)
{
  const auto it = component.parameters.find(name);

  if (it == component.parameters.end()) {
    return default_value;
  }

  return std::stod(it->second);
}

hardware_interface::CallbackReturn SensorsSystem::on_init(
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

  sensors_.clear();
  sensors_.reserve(info_.sensors.size());

  for (const auto & sensor_info : info_.sensors) {
    if (!has_parameter(sensor_info, "interface")) {
      RCLCPP_ERROR(
        kLogger,
        "Sensor '%s' has no required parameter 'interface'",
        sensor_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string interface_name = sensor_info.parameters.at("interface");

    pluginlib::UniquePtr<SensorInterfaceBase> sensor_interface;

    try {
      sensor_interface =
        sensor_interface_loader_.createUniqueInstance(interface_name);
    } catch (const pluginlib::PluginlibException & e) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to create interface '%s' for sensor '%s': %s",
        interface_name.c_str(),
        sensor_info.name.c_str(),
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto sensor = std::make_unique<SensorInstance>();
    sensor->info = sensor_info;
    sensor->interface = std::move(sensor_interface);

    try {
      sensor->read_rate_hz = component_double_parameter_or(
        sensor_info,
        "read_rate_hz",
        0.0);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        kLogger,
        "Invalid read_rate_hz for sensor '%s': %s",
        sensor_info.name.c_str(),
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    sensor->use_thread = sensor->read_rate_hz > 0.0;

    for (const auto & state_interface : sensor_info.state_interfaces) {
      sensor->control_states[state_interface.name] = 0.0;
      sensor->read_states[state_interface.name] = 0.0;
    }

    sensors_.push_back(std::move(sensor));
  }

  is_configured_ = false;
  is_active_ = false;

  RCLCPP_INFO(
    kLogger,
    "SensorsSystem initialized for environment='%s' with %zu sensor(s)",
    environment_.c_str(),
    sensors_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SensorsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & sensor : sensors_) {
    for (const auto & state_interface : sensor->info.state_interfaces) {
      state_interfaces.emplace_back(
        sensor->info.name,
        state_interface.name,
        &sensor->control_states[state_interface.name]);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SensorsSystem::export_command_interfaces()
{
  return {};
}

hardware_interface::CallbackReturn SensorsSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  stop_sensor_threads();
  stop_sim_spin_thread();

  reset_states();

  if (environment_ == "sim") {
    sim_node_ = std::make_shared<rclcpp::Node>("sura_sensors_system_sim");
  }

  for (auto & sensor : sensors_) {
    if (!sensor->interface->initialize(
          sensor->info,
          info_,
          environment_,
          sim_node_))
    {
      RCLCPP_ERROR(
        kLogger,
        "Failed to initialize sensor '%s'",
        sensor->info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  is_configured_ = true;
  is_active_ = false;

  RCLCPP_INFO(kLogger, "SensorsSystem configured");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!is_configured_) {
    RCLCPP_ERROR(kLogger, "Cannot activate SensorsSystem before configure");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & sensor : sensors_) {
    if (!sensor->interface->activate()) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to activate sensor '%s'",
        sensor->info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  is_active_ = true;

  if (environment_ == "sim") {
    start_sim_spin_thread();
  }

  start_sensor_threads();

  RCLCPP_INFO(kLogger, "SensorsSystem activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  is_active_ = false;

  stop_sensor_threads();
  stop_sim_spin_thread();

  for (auto & sensor : sensors_) {
    if (!sensor->interface->deactivate()) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to deactivate sensor '%s'",
        sensor->info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(kLogger, "SensorsSystem deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  is_active_ = false;

  stop_sensor_threads();
  stop_sim_spin_thread();

  for (auto & sensor : sensors_) {
    if (!sensor->interface->cleanup()) {
      RCLCPP_ERROR(
        kLogger,
        "Failed to cleanup sensor '%s'",
        sensor->info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  reset_states();

  sim_node_.reset();

  is_configured_ = false;

  RCLCPP_INFO(kLogger, "SensorsSystem cleaned up");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  is_active_ = false;

  stop_sensor_threads();
  stop_sim_spin_thread();

  for (auto & sensor : sensors_) {
    (void)sensor->interface->deactivate();
    (void)sensor->interface->cleanup();
  }

  reset_states();

  sim_node_.reset();

  is_configured_ = false;

  RCLCPP_INFO(kLogger, "SensorsSystem shutdown");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_shutdown(previous_state);
}

void SensorsSystem::start_sensor_threads()
{
  for (auto & sensor : sensors_) {
    if (!sensor->use_thread) {
      continue;
    }

    if (sensor->read_thread_running.load()) {
      continue;
    }

    sensor->read_thread_running.store(true);

    sensor->read_thread = std::thread(
      [this, sensor_ptr = sensor.get()]()
      {
        sensor_read_loop(sensor_ptr);
      });

    RCLCPP_INFO(
      kLogger,
      "Started read thread for sensor '%s' at %.3f Hz",
      sensor->info.name.c_str(),
      sensor->read_rate_hz);
  }
}

void SensorsSystem::stop_sensor_threads()
{
  for (auto & sensor : sensors_) {
    sensor->read_thread_running.store(false);
  }

  for (auto & sensor : sensors_) {
    if (sensor->read_thread.joinable()) {
      sensor->read_thread.join();
    }
  }
}

void SensorsSystem::sensor_read_loop(SensorInstance * sensor)
{
  if (!sensor) {
    return;
  }

  const double rate_hz = sensor->read_rate_hz;

  if (rate_hz <= 0.0) {
    return;
  }

  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  auto next_time = std::chrono::steady_clock::now();

  std::unordered_map<std::string, double> local_states;

  {
    std::lock_guard<std::mutex> lock(sensor->states_mutex);
    local_states = sensor->read_states;
  }

  while (sensor->read_thread_running.load()) {
    const bool ok = sensor->interface->read(local_states);
    sensor->last_read_ok.store(ok);

    {
      std::lock_guard<std::mutex> lock(sensor->states_mutex);
      sensor->read_states = local_states;
    }

    next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      period);

    std::this_thread::sleep_until(next_time);
  }
}

void SensorsSystem::start_sim_spin_thread()
{
  if (!sim_node_) {
    return;
  }

  if (sim_spin_running_.load()) {
    return;
  }

  sim_spin_running_.store(true);

  sim_spin_thread_ = std::thread(
    [this]()
    {
      const auto period = std::chrono::milliseconds(2);
      auto next_time = std::chrono::steady_clock::now();

      while (sim_spin_running_.load()) {
        rclcpp::spin_some(sim_node_);

        next_time += period;
        std::this_thread::sleep_until(next_time);
      }
    });

  RCLCPP_INFO(kLogger, "Started simulation spin thread");
}

void SensorsSystem::stop_sim_spin_thread()
{
  sim_spin_running_.store(false);

  if (sim_spin_thread_.joinable()) {
    sim_spin_thread_.join();
  }
}

hardware_interface::return_type SensorsSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!is_active_) {
    return hardware_interface::return_type::OK;
  }

  for (auto & sensor : sensors_) {
    if (sensor->use_thread) {
      std::lock_guard<std::mutex> lock(sensor->states_mutex);

      for (const auto & state : sensor->read_states) {
        sensor->control_states[state.first] = state.second;
      }

      if (!sensor->last_read_ok.load()) {
        RCLCPP_WARN_THROTTLE(
          kLogger,
          *rclcpp::Clock::make_shared(),
          2000,
          "Last read failed for sensor '%s'",
          sensor->info.name.c_str());
      }

      continue;
    }

    std::unordered_map<std::string, double> local_states;

    {
      std::lock_guard<std::mutex> lock(sensor->states_mutex);
      local_states = sensor->read_states;
    }

    const bool ok = sensor->interface->read(local_states);
    sensor->last_read_ok.store(ok);

    {
      std::lock_guard<std::mutex> lock(sensor->states_mutex);

      sensor->read_states = local_states;

      for (const auto & state : sensor->read_states) {
        sensor->control_states[state.first] = state.second;
      }
    }

    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        kLogger,
        *rclcpp::Clock::make_shared(),
        2000,
        "Failed to read sensor '%s'",
        sensor->info.name.c_str());
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

void SensorsSystem::reset_states()
{
  for (auto & sensor : sensors_) {
    std::lock_guard<std::mutex> lock(sensor->states_mutex);

    for (auto & state : sensor->control_states) {
      state.second = 0.0;
    }

    for (auto & state : sensor->read_states) {
      state.second = 0.0;
    }

    sensor->last_read_ok.store(true);
  }
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::SensorsSystem,
  hardware_interface::SystemInterface)
