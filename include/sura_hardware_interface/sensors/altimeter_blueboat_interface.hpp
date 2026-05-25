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

class AltimeterBlueboatInterface : public SensorInterfaceBase
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
  bool initialize_ping();
  bool configure_ping();
  bool request_distance();
  bool request_message(uint16_t message_id, std::vector<uint8_t> & payload);
  bool send_request(uint16_t message_id);
  bool send_set_message(uint16_t message_id, const std::vector<uint8_t> & payload);
  bool write_message(uint16_t message_id, const std::vector<uint8_t> & payload);
  bool read_from_serial();
  bool take_message(uint16_t expected_message_id, std::vector<uint8_t> & payload);

  bool initialized_{false};
  bool active_{false};

  std::string sensor_name_;
  std::string environment_{"real"};
  std::string serial_port_{"/dev/serial3"};
  int baudrate_{115200};
  int timeout_ms_{500};

  uint32_t speed_of_sound_{1450000};
  uint16_t ping_interval_{100};
  uint8_t gain_setting_{1};
  uint32_t scan_start_mm_{100};
  uint32_t scan_length_mm_{3000};
  uint8_t mode_auto_{0};

  int fd_{-1};
  std::vector<uint8_t> rx_buffer_;
  double last_altitude_{0.0};
  double last_confidence_{0.0};
  double last_scan_start_{0.1};
  double last_scan_length_{3.0};
  double last_gain_setting_{1.0};
};

}  // namespace sura_hardware_interface
