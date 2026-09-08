#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sura_hardware_interface
{
namespace multibeam
{

// Blue Robotics ping-protocol, surveyor240.json @
// 1746cd03f942d58bcf08253055854caea2e33fda.
constexpr uint16_t kMsgDeviceInformation = 4;
constexpr uint16_t kMsgAck = 1;
constexpr uint16_t kMsgNack = 2;
constexpr uint16_t kMsgProtocolVersion = 5;
constexpr uint16_t kMsgGeneralRequest = 6;
constexpr uint16_t kMsgUtcRequest = 14;
constexpr uint16_t kMsgUtcResponse = 15;
constexpr uint16_t kMsgJsonWrapper = 10;
constexpr uint16_t kMsgWaterStats = 118;
constexpr uint16_t kMsgAttitudeReport = 504;
constexpr uint16_t kMsgYzPointData = 3011;
constexpr uint16_t kMsgAtofPointData = 3012;
constexpr uint16_t kMsgSetPingParameters = 3023;

constexpr std::size_t kFrameHeaderSize = 8;
constexpr std::size_t kFrameChecksumSize = 2;
constexpr std::size_t kAtofPointSize = 16;

struct Frame
{
  uint16_t message_id{0};
  uint8_t source{0};
  uint8_t destination{0};
  std::vector<uint8_t> payload;
};

struct ParserCounters
{
  uint64_t checksum_errors{0};
  uint64_t malformed_packets{0};
  uint64_t unknown_ids{0};
  uint64_t bytes_dropped{0};
};

class Parser
{
public:
  explicit Parser(std::size_t max_payload_size = 65535);

  void append(const uint8_t * data, std::size_t size);
  bool next(Frame & frame);
  void clear();

  const ParserCounters & counters() const;
  std::size_t buffered_size() const;

private:
  std::vector<uint8_t> buffer_;
  std::size_t max_payload_size_;
  ParserCounters counters_;
};

struct SetPingParameters
{
  int32_t start_mm{0};
  int32_t end_mm{0};
  float sound_speed_m_s{1500.0F};
  int16_t gain_index{-1};
  int16_t msec_per_ping{200};
  uint16_t deprecated{0};
  uint8_t diagnostic_injected_signal{0};
  bool ping_enable{false};
  bool enable_channel_data{false};
  bool reserved_for_raw_data{false};
  bool enable_yz_point_data{false};
  bool enable_atof_data{true};
  int32_t target_ping_hz{240000};
  uint16_t n_range_steps{400};
  uint16_t reserved{0};
  float pulse_len_steps{1.5F};
};

struct AtofPoint
{
  float angle_rad{0.0F};
  float time_of_flight_s{0.0F};
  float power{0.0F};
  uint8_t point_type{0};
  std::array<uint8_t, 3> reserved{};
};

struct AtofPointData
{
  uint32_t power_up_time_ms{0};
  uint64_t utc_time_ms{0};
  float listening_time_s{0.0F};
  float sound_speed_m_s{0.0F};
  uint32_t ping_number{0};
  uint32_t acoustic_frequency_hz{0};
  float pulse_duration_s{0.0F};
  uint32_t flags{0};
  uint16_t reported_point_count{0};
  uint16_t reserved{0};
  std::vector<AtofPoint> points;
};

struct YzPoint
{
  float y_m{0.0F};
  float z_m{0.0F};
};

struct YzPointData
{
  uint32_t power_up_time_ms{0};
  uint32_t ping_number{0};
  float sound_speed_m_s{0.0F};
  float up_vector_x{0.0F};
  float up_vector_y{0.0F};
  float up_vector_z{0.0F};
  float water_temperature_c{0.0F};
  float water_pressure_bar{0.0F};
  float heave_m{0.0F};
  float start_m{0.0F};
  float end_m{0.0F};
  uint16_t reported_point_count{0};
  std::vector<YzPoint> points;
};

struct AttitudeReport
{
  float up_vector_x{0.0F};
  float up_vector_y{0.0F};
  float up_vector_z{0.0F};
  float reserved_1{0.0F};
  float reserved_2{0.0F};
  float reserved_3{0.0F};
  uint64_t utc_time_ms{0};
  uint32_t power_up_time_ms{0};
};

struct WaterStats
{
  float temperature_c{0.0F};
  float pressure_bar{0.0F};
};

std::vector<uint8_t> encode_frame(uint16_t message_id, const std::vector<uint8_t> & payload);
std::vector<uint8_t> encode_general_request(uint16_t requested_id);
std::vector<uint8_t> encode_set_ping_parameters(const SetPingParameters & parameters);
std::vector<uint8_t> encode_utc_response(uint64_t utc_time_ms, uint32_t accuracy_ms);

bool decode_atof_point_data(const std::vector<uint8_t> & payload, AtofPointData & data);
bool decode_yz_point_data(const std::vector<uint8_t> & payload, YzPointData & data);
bool decode_attitude_report(const std::vector<uint8_t> & payload, AttitudeReport & data);
bool decode_water_stats(const std::vector<uint8_t> & payload, WaterStats & data);

}  // namespace multibeam
}  // namespace sura_hardware_interface
