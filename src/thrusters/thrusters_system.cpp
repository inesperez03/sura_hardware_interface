#include "sura_hardware_interface/thrusters/thrusters_system.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sura_hardware_interface/navigator_access.hpp"

namespace sura_hardware_interface
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("sura_hardware_interface");
constexpr const char * kThrusterLpfAlphaEnv = "BLUEBOAT_THRUSTER_LPF_ALPHA";

#ifdef TARGET_RASPBERRY
static uint16_t pulse_us_to_counts(double pulse_us, double freq_hz)
{
  const double period_us = 1e6 / freq_hz;
  const double counts = pulse_us * 4096.0 / period_us;

  const long rounded = std::lround(counts);
  return static_cast<uint16_t>(std::clamp(rounded, 0L, 4095L));
}
#endif

bool parse_bool_parameter(const std::string & value)
{
  if (value == "true" || value == "True" || value == "TRUE" || value == "1") {
    return true;
  }

  if (value == "false" || value == "False" || value == "FALSE" || value == "0") {
    return false;
  }

  throw std::invalid_argument("Invalid boolean value: " + value);
}

double parse_double_parameter(const std::string & value)
{
  std::size_t processed = 0;
  const double parsed = std::stod(value, &processed);
  if (processed != value.size()) {
    throw std::invalid_argument("Trailing characters in double value: " + value);
  }
  return parsed;
}

int parse_int_parameter(const std::string & value)
{
  std::size_t processed = 0;
  const int parsed = std::stoi(value, &processed);
  if (processed != value.size()) {
    throw std::invalid_argument("Trailing characters in integer value: " + value);
  }
  return parsed;
}

std::string required_joint_parameter(
  const hardware_interface::ComponentInfo & joint,
  const std::string & parameter_name)
{
  const auto parameter_it = joint.parameters.find(parameter_name);
  if (parameter_it == joint.parameters.end()) {
    throw std::runtime_error(
      "Joint '" + joint.name + "' is missing required parameter '" + parameter_name + "'");
  }

  return parameter_it->second;
}

std::string optional_joint_parameter(
  const hardware_interface::ComponentInfo & joint,
  const std::string & parameter_name)
{
  const auto parameter_it = joint.parameters.find(parameter_name);
  if (parameter_it == joint.parameters.end()) {
    return "";
  }

  return parameter_it->second;
}

std::string resolve_lookup_csv_path(const std::string & lookup_csv_path)
{
  const std::filesystem::path path{lookup_csv_path};

  if (path.is_absolute() || std::filesystem::exists(path)) {
    return lookup_csv_path;
  }

  try {
    const auto package_share =
      ament_index_cpp::get_package_share_directory("sura_hardware_interface");
    const auto package_path = std::filesystem::path(package_share) / path;

    if (std::filesystem::exists(package_path)) {
      return package_path.string();
    }
  } catch (const std::exception &) {
  }

  return lookup_csv_path;
}

}  // namespace

void ThrustersSystem::reset_thruster_filter()
{
  filtered_force_commands_.assign(info_.joints.size(), 0.0);
}

void ThrustersSystem::publish_zero_command()
{
  if (environment_ == "sim" && thruster_stonefish_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(info_.joints.size(), 0.0);
    thruster_stonefish_pub_->publish(msg);
  } else if (environment_ == "real" && navigator_initialized_ && pwm_enabled_) {
#ifdef TARGET_RASPBERRY
    try {
      navigator_access::call(
        [&]()
        {
          for (std::size_t index = 0; index < pwm_channel_indices_.size(); ++index) {
            double neutral_pulse_us = mapper_.forceToPwm(0.0);
            if (index < inverted_flags_.size() && inverted_flags_[index]) {
              neutral_pulse_us = 3000.0 - neutral_pulse_us;
            }
            if (index < pwm_offsets_us_.size()) {
              neutral_pulse_us += pwm_offsets_us_[index];
            }

            set_pwm_channel_value(
              static_cast<uintptr_t>(pwm_channel_indices_[index]),
              pulse_us_to_counts(neutral_pulse_us, pwm_frequency_hz_));
          }
        });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to send neutral PWM command: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(kLogger, "Failed to send neutral PWM command: unknown error");
    }
#else
    RCLCPP_ERROR(
      kLogger,
      "Real environment requested, but hardware backend is not available on this architecture");
#endif
  }
}

hardware_interface::CallbackReturn ThrustersSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto environment_it = info_.hardware_parameters.find("environment");
  if (environment_it == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(kLogger, "Missing hardware parameter: environment");
    return hardware_interface::CallbackReturn::ERROR;
  }
  environment_ = environment_it->second;

  if (info_.joints.empty()) {
    RCLCPP_ERROR(kLogger, "Expected at least one thruster joint");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "effort") {
      RCLCPP_ERROR(
        kLogger,
        "Joint %s must have exactly one command interface: effort",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != "effort") {
      RCLCPP_ERROR(
        kLogger,
        "Joint %s must have exactly one state interface: effort",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  force_commands_.assign(info_.joints.size(), 0.0);
  force_states_.assign(info_.joints.size(), 0.0);
  last_force_commands_.assign(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  last_outputs_.assign(
    info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  reset_thruster_filter();
  inverted_flags_.assign(info_.joints.size(), false);
  pwm_offsets_us_.assign(info_.joints.size(), 0.0);
  pwm_channel_indices_.clear();
  pwm_channel_indices_.reserve(info_.joints.size());
  lookup_csv_path_.clear();
  stonefish_topic_.clear();

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];

    try {
      const std::string joint_lookup_csv = required_joint_parameter(joint, "lookup_csv");

      if (lookup_csv_path_.empty()) {
        lookup_csv_path_ = joint_lookup_csv;
      } else if (lookup_csv_path_ != joint_lookup_csv) {
        RCLCPP_ERROR(
          kLogger,
          "All thruster joints must use the same lookup_csv. Joint %s uses '%s', expected '%s'",
          joint.name.c_str(),
          joint_lookup_csv.c_str(),
          lookup_csv_path_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (environment_ == "sim") {
        const std::string joint_stonefish_topic =
          required_joint_parameter(joint, "stonefish_topic");

        if (stonefish_topic_.empty()) {
          stonefish_topic_ = joint_stonefish_topic;
        } else if (stonefish_topic_ != joint_stonefish_topic) {
          RCLCPP_ERROR(
            kLogger,
            "All thruster joints must use the same stonefish_topic. Joint %s uses '%s', expected "
            "'%s'",
            joint.name.c_str(),
            joint_stonefish_topic.c_str(),
            stonefish_topic_.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }

      if (environment_ == "real") {
        inverted_flags_[index] = parse_bool_parameter(required_joint_parameter(joint, "inverted"));
        pwm_channel_indices_.push_back(
          parse_int_parameter(required_joint_parameter(joint, "channel")));
      } else {
        const std::string inverted_value = optional_joint_parameter(joint, "inverted");
        if (!inverted_value.empty()) {
          inverted_flags_[index] = parse_bool_parameter(inverted_value);
        }
      }

      const std::string pwm_offset_value = optional_joint_parameter(joint, "pwm_offset");
      if (!pwm_offset_value.empty()) {
        pwm_offsets_us_[index] = parse_double_parameter(pwm_offset_value);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "%s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (environment_ == "real") {
      RCLCPP_INFO(
        kLogger,
        "Joint %s channel=%d inverted=%s pwm_offset=%.3f us",
        joint.name.c_str(),
        pwm_channel_indices_[index],
        inverted_flags_[index] ? "true" : "false",
        pwm_offsets_us_[index]);
    } else {
      RCLCPP_INFO(
        kLogger,
        "Joint %s inverted=%s pwm_offset=%.3f us",
        joint.name.c_str(),
        inverted_flags_[index] ? "true" : "false",
        pwm_offsets_us_[index]);
    }
  }

  is_active_ = false;
  navigator_initialized_ = false;
  pwm_enabled_ = false;

  thruster_lpf_alpha_ = 1.0;
  if (const char * alpha_env = std::getenv(kThrusterLpfAlphaEnv); alpha_env != nullptr) {
    try {
      thruster_lpf_alpha_ = std::clamp(parse_double_parameter(alpha_env), 0.0, 1.0);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        kLogger,
        "Invalid %s value '%s': %s. Falling back to alpha=1.0",
        kThrusterLpfAlphaEnv,
        alpha_env,
        e.what());
      thruster_lpf_alpha_ = 1.0;
    }
  } else {
    RCLCPP_INFO(
      kLogger,
      "%s not set. Thruster LPF disabled (alpha=1.0)",
      kThrusterLpfAlphaEnv);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::string resolved_lookup_csv_path = resolve_lookup_csv_path(lookup_csv_path_);

  if (!mapper_.loadCsv(resolved_lookup_csv_path)) {
    RCLCPP_ERROR(
      kLogger,
      "Failed to load thruster lookup CSV: %s",
      resolved_lookup_csv_path.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  is_active_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  navigator_initialized_ = false;
  pwm_enabled_ = false;

  if (environment_ == "sim") {
    internal_node_ = std::make_shared<rclcpp::Node>("sura_hardware_interface_pub");

    thruster_stonefish_pub_ =
      internal_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        stonefish_topic_, 10);

    RCLCPP_INFO(
      kLogger, "Stonefish publisher created successfully for topic %s",
      stonefish_topic_.c_str());
  } else if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    if (pwm_channel_indices_.empty()) {
      RCLCPP_ERROR(kLogger, "Real environment requires one channel parameter per thruster joint");
      return hardware_interface::CallbackReturn::ERROR;
    }

    try {
      navigator_access::initialize_once();
      navigator_initialized_ = true;

      navigator_access::call(
        [&]()
        {
          set_pwm_freq_hz(pwm_frequency_hz_);
          set_pwm_enable(true);

          for (std::size_t index = 0; index < pwm_channel_indices_.size(); ++index) {
            double neutral_pulse_us = mapper_.forceToPwm(0.0);
            if (index < inverted_flags_.size() && inverted_flags_[index]) {
              neutral_pulse_us = 3000.0 - neutral_pulse_us;
            }
            if (index < pwm_offsets_us_.size()) {
              neutral_pulse_us += pwm_offsets_us_[index];
            }

            set_pwm_channel_value(
              static_cast<uintptr_t>(pwm_channel_indices_[index]),
              pulse_us_to_counts(neutral_pulse_us, pwm_frequency_hz_));
          }
        });
      pwm_enabled_ = true;

      RCLCPP_INFO(kLogger, "Navigator initialized successfully");
      RCLCPP_INFO(kLogger, "PWM enabled at %.2f Hz", pwm_frequency_hz_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to initialize Navigator: %s", e.what());
      navigator_initialized_ = false;
      pwm_enabled_ = false;
      return hardware_interface::CallbackReturn::ERROR;
    } catch (...) {
      RCLCPP_ERROR(kLogger, "Failed to initialize Navigator: unknown error");
      navigator_initialized_ = false;
      pwm_enabled_ = false;
      return hardware_interface::CallbackReturn::ERROR;
    }
#else
    RCLCPP_ERROR(
      kLogger,
      "Environment is 'real' but this build does not include Navigator support on this architecture");
    return hardware_interface::CallbackReturn::ERROR;
#endif
  } else {
    RCLCPP_ERROR(kLogger, "Unknown environment: %s", environment_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(force_commands_.begin(), force_commands_.end(), 0.0);
  std::fill(force_states_.begin(), force_states_.end(), 0.0);
  std::fill(
    last_force_commands_.begin(),
    last_force_commands_.end(),
    std::numeric_limits<double>::quiet_NaN());
  std::fill(
    last_outputs_.begin(),
    last_outputs_.end(),
    std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(kLogger, "ThrustersSystem configured successfully");
  RCLCPP_INFO(kLogger, "Environment: %s", environment_.c_str());
  RCLCPP_INFO(kLogger, "Thruster joints: %zu", info_.joints.size());
  RCLCPP_INFO(kLogger, "Loaded CSV samples: %zu", mapper_.size());
  RCLCPP_INFO(kLogger, "Thruster LPF alpha: %.3f", thruster_lpf_alpha_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  std::fill(force_commands_.begin(), force_commands_.end(), 0.0);
  std::fill(force_states_.begin(), force_states_.end(), 0.0);
  reset_thruster_filter();

  publish_zero_command();

#ifdef TARGET_RASPBERRY
  if (environment_ == "real" && navigator_initialized_) {
    try {
      navigator_access::call(
        []()
        {
          set_pwm_enable(false);
        });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to disable PWM during cleanup: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(kLogger, "Failed to disable PWM during cleanup: unknown error");
    }
  }
#endif

  pwm_enabled_ = false;
  navigator_initialized_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  std::fill(
    last_force_commands_.begin(),
    last_force_commands_.end(),
    std::numeric_limits<double>::quiet_NaN());
  std::fill(
    last_outputs_.begin(),
    last_outputs_.end(),
    std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(kLogger, "ThrustersSystem cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  std::fill(force_commands_.begin(), force_commands_.end(), 0.0);
  std::fill(force_states_.begin(), force_states_.end(), 0.0);
  reset_thruster_filter();

  publish_zero_command();

#ifdef TARGET_RASPBERRY
  if (environment_ == "real" && navigator_initialized_) {
    try {
      navigator_access::call(
        []()
        {
          set_pwm_enable(false);
        });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to disable PWM during shutdown: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(kLogger, "Failed to disable PWM during shutdown: unknown error");
    }
  }
#endif

  pwm_enabled_ = false;
  navigator_initialized_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  RCLCPP_INFO(kLogger, "ThrustersSystem shutdown completed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = true;

  std::fill(force_commands_.begin(), force_commands_.end(), 0.0);
  std::fill(force_states_.begin(), force_states_.end(), 0.0);
  reset_thruster_filter();
  std::fill(
    last_force_commands_.begin(),
    last_force_commands_.end(),
    std::numeric_limits<double>::quiet_NaN());
  std::fill(
    last_outputs_.begin(),
    last_outputs_.end(),
    std::numeric_limits<double>::quiet_NaN());

  publish_zero_command();

  RCLCPP_INFO(kLogger, "ThrustersSystem activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  std::fill(force_commands_.begin(), force_commands_.end(), 0.0);
  std::fill(force_states_.begin(), force_states_.end(), 0.0);
  reset_thruster_filter();

  publish_zero_command();

#ifdef TARGET_RASPBERRY
  if (environment_ == "real" && navigator_initialized_) {
    try {
      navigator_access::call(
        []()
        {
          set_pwm_enable(false);
        });
      pwm_enabled_ = false;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to disable PWM during deactivation: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(kLogger, "Failed to disable PWM during deactivation: unknown error");
    }
  }
#endif

  RCLCPP_INFO(kLogger, "ThrustersSystem deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  std::fill(force_commands_.begin(), force_commands_.end(), 0.0);
  std::fill(force_states_.begin(), force_states_.end(), 0.0);
  reset_thruster_filter();

  publish_zero_command();

#ifdef TARGET_RASPBERRY
  if (environment_ == "real" && navigator_initialized_) {
    try {
      navigator_access::call(
        []()
        {
          set_pwm_enable(false);
        });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to disable PWM during error handling: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(kLogger, "Failed to disable PWM during error handling: unknown error");
    }
  }
#endif

  pwm_enabled_ = false;
  navigator_initialized_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  RCLCPP_ERROR(kLogger, "ThrustersSystem entered error state");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ThrustersSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[index].name, "effort", &force_states_[index]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ThrustersSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[index].name, "effort", &force_commands_[index]));
  }

  return command_interfaces;
}

hardware_interface::return_type ThrustersSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  force_states_ = filtered_force_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThrustersSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!is_active_) {
    std::fill(force_states_.begin(), force_states_.end(), 0.0);
    publish_zero_command();
    return hardware_interface::return_type::OK;
  }

  std::vector<double> stonefish_outputs(info_.joints.size(), 0.0);
  if (filtered_force_commands_.size() != info_.joints.size()) {
    reset_thruster_filter();
  }

#ifdef TARGET_RASPBERRY
  std::vector<uint16_t> pwm_counts(info_.joints.size(), 0U);
#endif

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const double raw_force = force_commands_[index];
    const double filtered_force =
      filtered_force_commands_[index] +
      thruster_lpf_alpha_ * (raw_force - filtered_force_commands_[index]);
    filtered_force_commands_[index] = filtered_force;
    stonefish_outputs[index] = mapper_.forceToStonefish(filtered_force);

#ifdef TARGET_RASPBERRY
    double applied_force = filtered_force;

    if (environment_ == "real" && inverted_flags_[index]) {
      applied_force = filtered_force;
    }

    double pulse_us = mapper_.forceToPwm(applied_force);

    if (environment_ == "real" && inverted_flags_[index]) {
      pulse_us = 3000.0 - pulse_us;
    }

    pulse_us += pwm_offsets_us_[index];

    pwm_counts[index] = pulse_us_to_counts(pulse_us, pwm_frequency_hz_);
#endif
  }

  force_states_ = filtered_force_commands_;

  if (environment_ == "real") {
#ifndef TARGET_RASPBERRY
    RCLCPP_ERROR(
      kLogger,
      "Real environment requested, but this build does not support Navigator hardware");
    return hardware_interface::return_type::ERROR;
#else
    if (!navigator_initialized_ || !pwm_enabled_) {
      RCLCPP_ERROR(kLogger, "Navigator PWM not initialized");
      return hardware_interface::return_type::ERROR;
    }

    try {
      navigator_access::call(
        [&]()
        {
          for (std::size_t index = 0; index < pwm_channel_indices_.size(); ++index) {
            set_pwm_channel_value(
              static_cast<uintptr_t>(pwm_channel_indices_[index]),
              pwm_counts[index]);
          }
        });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(kLogger, "Failed to write PWM to Navigator: %s", e.what());
      return hardware_interface::return_type::ERROR;
    } catch (...) {
      RCLCPP_ERROR(kLogger, "Failed to write PWM to Navigator: unknown error");
      return hardware_interface::return_type::ERROR;
    }

    bool changed = false;
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      if (
        std::isnan(last_force_commands_[index]) ||
        std::fabs(filtered_force_commands_[index] - last_force_commands_[index]) > 1e-6 ||
        std::fabs(static_cast<double>(pwm_counts[index]) - last_outputs_[index]) > 1e-6)
      {
        changed = true;
        break;
      }
    }

    if (changed) {
      last_force_commands_ = filtered_force_commands_;
      for (std::size_t index = 0; index < info_.joints.size(); ++index) {
        last_outputs_[index] = static_cast<double>(pwm_counts[index]);
      }
    }
#endif
  } else if (environment_ == "sim") {
    if (!thruster_stonefish_pub_) {
      RCLCPP_ERROR(kLogger, "Stonefish publisher not initialized");
      return hardware_interface::return_type::ERROR;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = stonefish_outputs;
    thruster_stonefish_pub_->publish(msg);

    bool changed = false;
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      if (
        std::isnan(last_force_commands_[index]) ||
        std::fabs(filtered_force_commands_[index] - last_force_commands_[index]) > 1e-6 ||
        std::fabs(stonefish_outputs[index] - last_outputs_[index]) > 1e-6)
      {
        changed = true;
        break;
      }
    }

    if (changed) {
      last_force_commands_ = filtered_force_commands_;
      last_outputs_ = stonefish_outputs;
    }
  } else {
    RCLCPP_ERROR(kLogger, "Unknown environment: %s", environment_.c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::ThrustersSystem,
  hardware_interface::SystemInterface)
