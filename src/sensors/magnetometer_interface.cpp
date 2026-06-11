#include "sura_hardware_interface/sensors/magnetometer_interface.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "sura_hardware_interface/navigator_access.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

namespace sura_hardware_interface
{

namespace
{

std::string get_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const std::string & default_value)
{
  const auto it = sensor_info.parameters.find(name);

  if (it == sensor_info.parameters.end()) {
    return default_value;
  }

  return it->second;
}

double get_double_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const double default_value)
{
  const auto it = sensor_info.parameters.find(name);

  if (it == sensor_info.parameters.end()) {
    return default_value;
  }

  return std::stod(it->second);
}

void set_state_if_exists(
  std::unordered_map<std::string, double> & states,
  const std::string & name,
  const double value)
{
  const auto it = states.find(name);

  if (it != states.end()) {
    it->second = value;
  }
}

}  // namespace

bool MagnetometerInterface::initialize(
  const hardware_interface::ComponentInfo & sensor_info,
  const hardware_interface::HardwareInfo &,
  const std::string & environment,
  const rclcpp::Node::SharedPtr & sim_node)
{
  if (initialized_) {
    return true;
  }

  sensor_name_ = sensor_info.name;
  environment_ = environment;
  sim_node_ = sim_node;

  if (environment_ != "real" && environment_ != "sim") {
    std::cerr << "[Magnetometer] Unsupported environment: "
              << environment_ << std::endl;
    return false;
  }

  try {
    read_rate_hz_ = get_double_param_or(
      sensor_info,
      "read_rate_hz",
      read_rate_hz_);

    stonefish_topic_ = get_param_or(
      sensor_info,
      "stonefish_topic",
      stonefish_topic_);
  } catch (const std::exception & e) {
    std::cerr << "[Magnetometer] Error parsing parameters for sensor '"
              << sensor_name_ << "': " << e.what() << std::endl;
    return false;
  }

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    navigator_access::initialize_once();
#endif
  }

  if (environment_ == "sim") {
    if (!sim_node_) {
      std::cerr << "[Magnetometer] sim_node is null in simulation mode"
                << std::endl;
      return false;
    }

    if (stonefish_topic_.empty()) {
      std::cerr << "[Magnetometer] Missing parameter 'stonefish_topic' in sim mode"
                << std::endl;
      return false;
    }

    magnetometer_sub_ =
      sim_node_->create_subscription<sensor_msgs::msg::MagneticField>(
      stonefish_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::MagneticField::SharedPtr msg)
      {
        last_sample_time_sec_ = msg->header.stamp.sec;
        last_sample_time_nanosec_ = msg->header.stamp.nanosec;

        last_magnetic_field_x_ = msg->magnetic_field.x;
        last_magnetic_field_y_ = msg->magnetic_field.y;
        last_magnetic_field_z_ = msg->magnetic_field.z;
      });
  }

  initialized_ = true;
  active_ = false;

  return true;
}

bool MagnetometerInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool MagnetometerInterface::deactivate()
{
  active_ = false;
  return true;
}

bool MagnetometerInterface::cleanup()
{
  active_ = false;
  initialized_ = false;

  magnetometer_sub_.reset();
  sim_node_.reset();

  last_magnetic_field_x_ = 0.0;
  last_magnetic_field_y_ = 0.0;
  last_magnetic_field_z_ = 0.0;

  last_sample_time_sec_ = 0;
  last_sample_time_nanosec_ = 0;

  return true;
}

bool MagnetometerInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  const auto read_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  int32_t sample_time_sec = static_cast<int32_t>(read_time.seconds());
  uint32_t sample_time_nanosec =
    static_cast<uint32_t>(read_time.nanoseconds() % 1000000000LL);

  double magnetic_field_x = 0.0;
  double magnetic_field_y = 0.0;
  double magnetic_field_z = 0.0;

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    const AxisData mag = navigator_access::call(
      []()
      {
        return read_mag();
      });

    // ROTATION_YAW_270: x' = y, y' = -x, z' = z
    magnetic_field_x = static_cast<double>(mag.y);
    magnetic_field_y = -static_cast<double>(mag.x);
    magnetic_field_z = static_cast<double>(mag.z);
#endif
  } else {
    if (last_sample_time_sec_ != 0 || last_sample_time_nanosec_ != 0) {
      sample_time_sec = last_sample_time_sec_;
      sample_time_nanosec = last_sample_time_nanosec_;
    }

    magnetic_field_x = last_magnetic_field_x_;
    magnetic_field_y = last_magnetic_field_y_;
    magnetic_field_z = last_magnetic_field_z_;
  }

  set_state_if_exists(states, "magnetic_field.x", magnetic_field_x);
  set_state_if_exists(states, "magnetic_field.y", magnetic_field_y);
  set_state_if_exists(states, "magnetic_field.z", magnetic_field_z);
  set_state_if_exists(states, "sample_time.sec", static_cast<double>(sample_time_sec));
  set_state_if_exists(states, "sample_time.nanosec", static_cast<double>(sample_time_nanosec));

  return true;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::MagnetometerInterface,
  sura_hardware_interface::SensorInterfaceBase)
