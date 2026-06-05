#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "sura_hardware_interface/sensors/sensor_interface_base.hpp"

namespace sura_hardware_interface
{

class DvlInterface : public SensorInterfaceBase
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
  bool setupSocket();

  void closeSocket();

  void configureDvl();

  void sendCommand(const std::string & command);

  bool receiveAndParseOnce();

  bool parseDvpdL(
    const std::string & line,
    double & vx,
    double & vy,
    double & vz,
    double & confidence);

  bool parseDvextDistanceZ(
    const std::string & line,
    double & distance_z,
    bool & valid_lock);

  bool parseGprmc(
    const std::string & line,
    double & latitude,
    double & longitude,
    double & altitude,
    double & valid);

  double nmeaCoordinateToDecimalDegrees(
    const std::string & value,
    const std::string & hemisphere,
    double max_degrees);

  bool hasValidNmeaChecksum(const std::string & line);

  std::string removeChecksum(const std::string & line);

  bool initialized_{false};
  bool active_{false};

  std::string sensor_name_;
  std::string environment_{"real"};

  double read_rate_hz_{20.0};
  double sim_dvl_confidence_{100.0};

  rclcpp::Node::SharedPtr sim_node_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  std::string stonefish_topic_;
  std::string stonefish_altitude_topic_;
  std::string stonefish_gps_topic_;

  std::string dvl_ip_{"192.168.1.236"};
  int listen_port_{27000};
  int command_port_{50000};

  double min_confidence_{50.0};
  double timeout_s_{1.0};

  int socket_fd_{-1};

  std::mutex data_mutex_;

  std::chrono::steady_clock::time_point last_rx_time_{
    std::chrono::steady_clock::now()};

  double last_vx_{0.0};
  double last_vy_{0.0};
  double last_vz_{0.0};

  double last_confidence_{0.0};

  double last_distance_z_{0.0};
  bool last_distance_z_valid_{false};

  double last_gps_latitude_{0.0};
  double last_gps_longitude_{0.0};
  double last_gps_altitude_{0.0};
  double last_gps_valid_{0.0};
};

}  // namespace sura_hardware_interface
