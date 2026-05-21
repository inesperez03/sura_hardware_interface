#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sura_hardware_interface/sensors/sensor_interface_base.hpp"

namespace sura_hardware_interface
{

class GpsInterface : public SensorInterfaceBase
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
  bool open_serial();
  void close_serial();
  bool read_from_serial();
  bool read_line(std::string & line);

  bool parse_gga(
    const std::string & line,
    double & latitude,
    double & longitude,
    double & altitude,
    double & valid) const;

  double nmea_to_decimal(
    const std::string & value,
    char direction) const;

  bool try_extract_ubx_packet(std::vector<uint8_t> & packet);

  bool parse_nav_pvt(
    const std::vector<uint8_t> & packet,
    double & latitude,
    double & longitude,
    double & altitude,
    double & valid) const;

  bool initialized_{false};
  bool active_{false};

  std::string sensor_name_;
  std::string environment_{"real"};

  std::string protocol_{"ubx"};
  std::string serial_port_{"/dev/ttyAMA5"};
  int baudrate_{230400};

  int fd_{-1};
  std::string line_buffer_;
  std::vector<uint8_t> byte_buffer_;

  double last_latitude_{0.0};
  double last_longitude_{0.0};
  double last_altitude_{0.0};
  double last_valid_{0.0};
};

}  // namespace sura_hardware_interface
