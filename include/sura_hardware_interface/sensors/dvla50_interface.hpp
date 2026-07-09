#pragma once

#include <array>
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "sura_hardware_interface/sensors/sensor_interface_base.hpp"

namespace sura_hardware_interface
{

class DvlA50Interface : public SensorInterfaceBase
{
public:
  bool initialize(
    const hardware_interface::ComponentInfo & sensor_info,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & environment,
    const rclcpp::Node::SharedPtr & sim_node) override;

  bool activate() override;

  bool deactivate() override;

  bool cleanup() override;

  bool read(std::unordered_map<std::string, double> & states) override;

private:
  struct BeamData
  {
    double id{0.0};
    double velocity{0.0};
    double distance{0.0};
    double rssi{0.0};
    double nsd{0.0};
    double valid{0.0};
  };

  bool setupConnection();

  void closeConnection();

  bool receiveAndParseAvailable();

  bool parseVelocityReport(const std::string & json);

  void resetTimedOutStates(std::unordered_map<std::string, double> & states);

  void setStates(std::unordered_map<std::string, double> & states);

  bool initialized_{false};
  bool active_{false};

  std::string sensor_name_;
  std::string environment_{"real"};

  double read_rate_hz_{20.0};
  double timeout_s_{1.0};
  double sim_fom_{0.0};

  rclcpp::Node::SharedPtr sim_node_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;

  std::string stonefish_topic_;
  std::string stonefish_altitude_topic_;

  std::string dvl_ip_{"192.168.194.95"};
  int dvl_port_{16171};
  int socket_fd_{-1};
  std::string rx_buffer_;

  std::mutex data_mutex_;

  std::chrono::steady_clock::time_point last_rx_time_{
    std::chrono::steady_clock::now()};

  double last_time_{0.0};
  double last_vx_{0.0};
  double last_vy_{0.0};
  double last_vz_{0.0};
  double last_fom_{0.0};
  double last_altitude_{0.0};
  double last_velocity_valid_{0.0};
  double last_status_{0.0};
  double last_format_code_{0.0};
  std::array<BeamData, 4> last_beams_{};
};

}  // namespace sura_hardware_interface
