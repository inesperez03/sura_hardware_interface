#pragma once

#include <chrono>
#include <mutex>
#include <string>

#include "hardware_interface/hardware_info.hpp"

namespace sura_hardware_interface
{

class DvlInterface
{
public:
  bool initialize(const hardware_interface::HardwareInfo & info);
  bool activate();
  bool deactivate();
  bool cleanup();

  bool read(
    double & linear_velocity_x,
    double & linear_velocity_y,
    double & linear_velocity_z,
    double & angular_velocity_x,
    double & angular_velocity_y,
    double & angular_velocity_z,
    double & distance_z,
    double & confidence,
    double & gps_latitude,
    double & gps_longitude,
    double & gps_altitude,
    double & gps_valid);

private:
  bool initialized_{false};
  bool active_{false};

  std::string dvl_ip_{"192.168.1.236"};
  int command_port_{50000};
  int listen_port_{27000};

  int socket_fd_{-1};

  double min_confidence_{50.0};
  double timeout_s_{1.0};

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

  std::chrono::steady_clock::time_point last_rx_time_{};

  std::mutex data_mutex_;

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

  static double nmeaCoordinateToDecimalDegrees(
    const std::string & value,
    const std::string & hemisphere);

  static std::string removeChecksum(const std::string & line);
};

}  // namespace sura_hardware_interface