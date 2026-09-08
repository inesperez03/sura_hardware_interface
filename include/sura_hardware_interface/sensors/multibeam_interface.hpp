#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sura_hardware_interface/sensors/multibeam_ping_protocol.hpp"
#include "sura_hardware_interface/sensors/multibeam_tcp_client.hpp"
#include "sura_hardware_interface/sensors/sensor_interface_base.hpp"

namespace sura_hardware_interface
{

class MultibeamInterface : public SensorInterfaceBase
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
  static constexpr std::size_t kMaxDetections = 64;

  struct DetectionState
  {
    double angle_rad{0.0};
    double time_of_flight_s{0.0};
    double power{0.0};
    double point_type{0.0};
    double reserved{0.0};
  };

  void send_initial_configuration();
  void process_frames();
  void handle_frame(const multibeam::Frame & frame);
  void handle_atof_point_data(const multibeam::AtofPointData & data);
  void handle_yz_point_data(const multibeam::YzPointData & data);
  void handle_attitude_report(const multibeam::AttitudeReport & data);
  void handle_water_stats(const multibeam::WaterStats & data);
  void publish_states(std::unordered_map<std::string, double> & states);
  void set_state(std::unordered_map<std::string, double> & states, const std::string & name, double value);
  uint64_t now_utc_ms() const;

  bool initialized_{false};
  bool active_{false};
  bool initialization_sent_{false};

  std::string sensor_name_;
  std::string environment_{"real"};
  std::string ip_address_{"192.168.1.86"};
  int port_{62312};

  multibeam::SetPingParameters ping_parameters_;
  multibeam::TcpClient tcp_;
  multibeam::Parser parser_;

  std::size_t max_rx_bytes_per_read_{65536};
  std::size_t max_messages_per_read_{64};
  std::size_t max_tx_queue_bytes_{65536};
  double silence_timeout_s_{3.0};

  std::chrono::steady_clock::time_point last_rx_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_ping_time_{std::chrono::steady_clock::now()};

  uint64_t ping_sequence_{0};
  uint64_t attitude_sequence_{0};
  uint64_t water_sequence_{0};
  uint64_t valid_pings_{0};
  uint64_t truncated_pings_{0};
  uint64_t malformed_payloads_{0};

  int32_t reception_time_sec_{0};
  uint32_t reception_time_nanosec_{0};
  uint32_t ping_number_{0};
  uint32_t power_up_time_ms_{0};
  uint64_t device_utc_time_ms_{0};
  float listening_time_s_{0.0F};
  float sound_speed_m_s_{1500.0F};
  uint32_t acoustic_frequency_hz_{0};
  float pulse_duration_s_{0.0F};
  uint32_t flags_{0};
  uint16_t reported_detection_count_{0};
  uint16_t stored_detection_count_{0};
  bool truncated_{false};
  std::array<DetectionState, kMaxDetections> detections_{};

  float up_vector_x_{0.0F};
  float up_vector_y_{0.0F};
  float up_vector_z_{1.0F};
  uint64_t attitude_utc_time_ms_{0};
  uint32_t attitude_power_up_time_ms_{0};

  bool water_valid_{false};
  float temperature_c_{0.0F};
  float pressure_bar_{0.0F};
};

}  // namespace sura_hardware_interface
