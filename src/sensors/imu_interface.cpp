#include "sura_hardware_interface/sensors/imu_interface.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "sura_hardware_interface/navigator_access.hpp"
#include "sensor_msgs/msg/imu.hpp"

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

bool ImuInterface::initialize(
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
    std::cerr << "[IMU] Unsupported environment: " << environment_ << std::endl;
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
    std::cerr << "[IMU] Error parsing parameters for sensor '"
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
      std::cerr << "[IMU] sim_node is null in simulation mode" << std::endl;
      return false;
    }

    if (stonefish_topic_.empty()) {
      std::cerr << "[IMU] Missing parameter 'stonefish_topic' in sim mode"
                << std::endl;
      return false;
    }

    imu_sub_ =
      sim_node_->create_subscription<sensor_msgs::msg::Imu>(
      stonefish_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg)
      {
        last_sample_time_sec_ = msg->header.stamp.sec;
        last_sample_time_nanosec_ = msg->header.stamp.nanosec;

        last_orientation_x_ = msg->orientation.x;
        last_orientation_y_ = msg->orientation.y;
        last_orientation_z_ = msg->orientation.z;
        last_orientation_w_ = msg->orientation.w;

        last_angular_velocity_x_ = msg->angular_velocity.x;
        last_angular_velocity_y_ = msg->angular_velocity.y;
        last_angular_velocity_z_ = msg->angular_velocity.z;

        last_linear_acceleration_x_ = msg->linear_acceleration.x;
        last_linear_acceleration_y_ = msg->linear_acceleration.y;
        last_linear_acceleration_z_ = msg->linear_acceleration.z;
      });
  }

  initialized_ = true;
  active_ = false;
  return true;
}

bool ImuInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool ImuInterface::deactivate()
{
  active_ = false;
  return true;
}

bool ImuInterface::cleanup()
{
  active_ = false;
  initialized_ = false;

  imu_sub_.reset();
  sim_node_.reset();

  last_orientation_x_ = 0.0;
  last_orientation_y_ = 0.0;
  last_orientation_z_ = 0.0;
  last_orientation_w_ = 1.0;

  last_angular_velocity_x_ = 0.0;
  last_angular_velocity_y_ = 0.0;
  last_angular_velocity_z_ = 0.0;

  last_linear_acceleration_x_ = 0.0;
  last_linear_acceleration_y_ = 0.0;
  last_linear_acceleration_z_ = 0.0;

  last_sample_time_sec_ = 0;
  last_sample_time_nanosec_ = 0;

  return true;
}

bool ImuInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  double orientation_x = 0.0;
  double orientation_y = 0.0;
  double orientation_z = 0.0;
  double orientation_w = 1.0;

  double angular_velocity_x = 0.0;
  double angular_velocity_y = 0.0;
  double angular_velocity_z = 0.0;

  double linear_acceleration_x = 0.0;
  double linear_acceleration_y = 0.0;
  double linear_acceleration_z = 0.0;

  const auto read_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  int32_t sample_time_sec = static_cast<int32_t>(read_time.seconds());
  uint32_t sample_time_nanosec =
    static_cast<uint32_t>(read_time.nanoseconds() % 1000000000LL);

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    AxisData accel{};
    AxisData gyro{};
    navigator_access::call(
      [&]()
      {
        accel = read_accel();
        gyro = read_gyro();
      });

    // Navigator IMU frame is SWD. Convert to NED by inverting X and Y.
    angular_velocity_x = static_cast<double>(-gyro.x);
    angular_velocity_y = static_cast<double>(-gyro.y);
    angular_velocity_z = static_cast<double>(gyro.z);

    linear_acceleration_x = static_cast<double>(-accel.x);
    linear_acceleration_y = static_cast<double>(-accel.y);
    linear_acceleration_z = static_cast<double>(accel.z);
#endif
  } else {
    if (last_sample_time_sec_ != 0 || last_sample_time_nanosec_ != 0) {
      sample_time_sec = last_sample_time_sec_;
      sample_time_nanosec = last_sample_time_nanosec_;
    }

    orientation_x = last_orientation_x_;
    orientation_y = last_orientation_y_;
    orientation_z = last_orientation_z_;
    orientation_w = last_orientation_w_;

    angular_velocity_x = last_angular_velocity_x_;
    angular_velocity_y = last_angular_velocity_y_;
    angular_velocity_z = last_angular_velocity_z_;

    linear_acceleration_x = last_linear_acceleration_x_;
    linear_acceleration_y = last_linear_acceleration_y_;
    linear_acceleration_z = last_linear_acceleration_z_;
  }

  set_state_if_exists(states, "orientation.x", orientation_x);
  set_state_if_exists(states, "orientation.y", orientation_y);
  set_state_if_exists(states, "orientation.z", orientation_z);
  set_state_if_exists(states, "orientation.w", orientation_w);

  set_state_if_exists(states, "angular_velocity.x", angular_velocity_x);
  set_state_if_exists(states, "angular_velocity.y", angular_velocity_y);
  set_state_if_exists(states, "angular_velocity.z", angular_velocity_z);

  set_state_if_exists(states, "linear_acceleration.x", linear_acceleration_x);
  set_state_if_exists(states, "linear_acceleration.y", linear_acceleration_y);
  set_state_if_exists(states, "linear_acceleration.z", linear_acceleration_z);
  set_state_if_exists(states, "sample_time.sec", static_cast<double>(sample_time_sec));
  set_state_if_exists(states, "sample_time.nanosec", static_cast<double>(sample_time_nanosec));

  return true;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::ImuInterface,
  sura_hardware_interface::SensorInterfaceBase)