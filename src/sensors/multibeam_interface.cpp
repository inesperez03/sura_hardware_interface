#include "sura_hardware_interface/sensors/multibeam_interface.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace sura_hardware_interface
{
namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("MultibeamInterface");

std::string get_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const std::string & default_value)
{
  const auto it = sensor_info.parameters.find(name);
  return it == sensor_info.parameters.end() ? default_value : it->second;
}

int get_int_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const int default_value)
{
  const auto value = get_param_or(sensor_info, name, std::to_string(default_value));
  return std::stoi(value);
}

double get_double_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const double default_value)
{
  const auto value = get_param_or(sensor_info, name, std::to_string(default_value));
  return std::stod(value);
}

uint32_t hi32(const uint64_t value)
{
  return static_cast<uint32_t>((value >> 32U) & 0xffffffffULL);
}

uint32_t lo32(const uint64_t value)
{
  return static_cast<uint32_t>(value & 0xffffffffULL);
}

}  // namespace

bool MultibeamInterface::initialize(
  const hardware_interface::ComponentInfo & sensor_info,
  const hardware_interface::HardwareInfo &,
  const std::string & environment,
  const rclcpp::Node::SharedPtr &)
{
  if (initialized_) {
    return true;
  }

  sensor_name_ = sensor_info.name;
  environment_ = environment;
  if (environment_ != "real" && environment_ != "sim") {
    RCLCPP_ERROR(kLogger, "Unsupported environment '%s'", environment_.c_str());
    return false;
  }

  try {
    ip_address_ = get_param_or(sensor_info, "ip_address", ip_address_);
    port_ = get_int_param_or(sensor_info, "port", port_);
    ping_parameters_.start_mm = get_int_param_or(sensor_info, "start_mm", ping_parameters_.start_mm);
    ping_parameters_.end_mm = get_int_param_or(sensor_info, "end_mm", ping_parameters_.end_mm);
    ping_parameters_.sound_speed_m_s = static_cast<float>(
      get_double_param_or(sensor_info, "sound_speed_m_s", ping_parameters_.sound_speed_m_s));
    ping_parameters_.gain_index = static_cast<int16_t>(
      get_int_param_or(sensor_info, "gain_index", ping_parameters_.gain_index));
    ping_parameters_.msec_per_ping = static_cast<int16_t>(
      get_int_param_or(sensor_info, "msec_per_ping", ping_parameters_.msec_per_ping));
    ping_parameters_.target_ping_hz =
      get_int_param_or(sensor_info, "target_ping_hz", ping_parameters_.target_ping_hz);
    ping_parameters_.n_range_steps = static_cast<uint16_t>(
      std::clamp(get_int_param_or(sensor_info, "range_steps", ping_parameters_.n_range_steps), 200, 800));
    ping_parameters_.pulse_len_steps = static_cast<float>(
      get_double_param_or(sensor_info, "pulse_length_steps", ping_parameters_.pulse_len_steps));
    max_rx_bytes_per_read_ = static_cast<std::size_t>(
      std::max(1024, get_int_param_or(sensor_info, "max_rx_bytes_per_read", 65536)));
    max_messages_per_read_ = static_cast<std::size_t>(
      std::max(1, get_int_param_or(sensor_info, "max_messages_per_read", 64)));
    max_tx_queue_bytes_ = static_cast<std::size_t>(
      std::max(1024, get_int_param_or(sensor_info, "max_tx_queue_bytes", 65536)));
    silence_timeout_s_ = get_double_param_or(sensor_info, "silence_timeout_s", silence_timeout_s_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      kLogger,
      "Invalid multibeam parameter for sensor '%s': %s",
      sensor_name_.c_str(),
      e.what());
    return false;
  }

  ping_parameters_.enable_atof_data = true;
  ping_parameters_.enable_channel_data = false;
  ping_parameters_.reserved_for_raw_data = false;
  ping_parameters_.enable_yz_point_data = true;
  ping_parameters_.ping_enable = true;

  tcp_.configure(
    ip_address_,
    port_,
    max_rx_bytes_per_read_,
    max_messages_per_read_,
    max_tx_queue_bytes_);

  initialized_ = true;
  active_ = false;
  initialization_sent_ = false;
  return true;
}

bool MultibeamInterface::activate()
{
  if (!initialized_) {
    return false;
  }
  active_ = true;
  initialization_sent_ = false;
  last_rx_time_ = std::chrono::steady_clock::now();
  last_ping_time_ = last_rx_time_;
  return true;
}

bool MultibeamInterface::deactivate()
{
  if (tcp_.connected()) {
    auto stop = ping_parameters_;
    stop.ping_enable = false;
    (void)tcp_.enqueue(multibeam::encode_set_ping_parameters(stop));
    tcp_.flush_tx();
  }
  active_ = false;
  initialization_sent_ = false;
  tcp_.reset();
  parser_.clear();
  return true;
}

bool MultibeamInterface::cleanup()
{
  (void)deactivate();
  initialized_ = false;
  return true;
}

bool MultibeamInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  if (environment_ == "real") {
    const auto now = std::chrono::steady_clock::now();
    tcp_.service_connection(now);

    if (tcp_.state() == multibeam::TcpState::Initializing && !initialization_sent_) {
      send_initial_configuration();
    }

    tcp_.flush_tx();
    tcp_.receive(parser_);
    process_frames();

    if (
      tcp_.state() == multibeam::TcpState::Streaming &&
      std::chrono::duration<double>(now - last_rx_time_).count() > silence_timeout_s_)
    {
      initialization_sent_ = false;
      tcp_.reconnect_later(ETIMEDOUT);
    }
  }

  publish_states(states);
  return tcp_.connected() || environment_ == "sim";
}

void MultibeamInterface::send_initial_configuration()
{
  initialization_sent_ = true;
  (void)tcp_.enqueue(multibeam::encode_general_request(multibeam::kMsgProtocolVersion));
  (void)tcp_.enqueue(multibeam::encode_general_request(multibeam::kMsgDeviceInformation));
  (void)tcp_.enqueue(multibeam::encode_set_ping_parameters(ping_parameters_));
}

void MultibeamInterface::process_frames()
{
  multibeam::Frame frame;
  while (parser_.next(frame)) {
    handle_frame(frame);
  }
}

void MultibeamInterface::handle_frame(const multibeam::Frame & frame)
{
  last_rx_time_ = std::chrono::steady_clock::now();

  if (frame.message_id == multibeam::kMsgUtcRequest) {
    (void)tcp_.enqueue(multibeam::encode_utc_response(now_utc_ms(), 1000U));
    return;
  }

  if (frame.message_id == multibeam::kMsgAtofPointData) {
    multibeam::AtofPointData data;
    if (multibeam::decode_atof_point_data(frame.payload, data)) {
      handle_atof_point_data(data);
      tcp_.set_state(multibeam::TcpState::Streaming);
    } else {
      malformed_payloads_++;
    }
    return;
  }

  if (frame.message_id == multibeam::kMsgYzPointData) {
    multibeam::YzPointData data;
    if (multibeam::decode_yz_point_data(frame.payload, data)) {
      handle_yz_point_data(data);
      tcp_.set_state(multibeam::TcpState::Streaming);
    } else {
      malformed_payloads_++;
    }
    return;
  }

  if (frame.message_id == multibeam::kMsgAttitudeReport) {
    multibeam::AttitudeReport data;
    if (multibeam::decode_attitude_report(frame.payload, data)) {
      handle_attitude_report(data);
    } else {
      malformed_payloads_++;
    }
    return;
  }

  if (frame.message_id == multibeam::kMsgWaterStats) {
    multibeam::WaterStats data;
    if (multibeam::decode_water_stats(frame.payload, data)) {
      handle_water_stats(data);
    } else {
      malformed_payloads_++;
    }
  }
}

void MultibeamInterface::handle_atof_point_data(const multibeam::AtofPointData & data)
{
  const rclcpp::Time stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  reception_time_sec_ = static_cast<int32_t>(stamp.seconds());
  reception_time_nanosec_ =
    static_cast<uint32_t>(stamp.nanoseconds() - static_cast<int64_t>(reception_time_sec_) * 1000000000LL);

  ping_number_ = data.ping_number;
  power_up_time_ms_ = data.power_up_time_ms;
  device_utc_time_ms_ = data.utc_time_ms;
  listening_time_s_ = data.listening_time_s;
  sound_speed_m_s_ = data.sound_speed_m_s;
  acoustic_frequency_hz_ = data.acoustic_frequency_hz;
  pulse_duration_s_ = data.pulse_duration_s;
  flags_ = data.flags;
  reported_detection_count_ = data.reported_point_count;
  stored_detection_count_ = static_cast<uint16_t>(std::min(data.points.size(), kMaxDetections));
  truncated_ = data.reported_point_count > kMaxDetections;

  for (auto & detection : detections_) {
    detection = {};
  }

  for (std::size_t i = 0; i < stored_detection_count_; ++i) {
    detections_[i].angle_rad = data.points[i].angle_rad;
    detections_[i].time_of_flight_s = data.points[i].time_of_flight_s;
    detections_[i].power = data.points[i].power;
    detections_[i].point_type = data.points[i].point_type;
    detections_[i].reserved =
      static_cast<double>(data.points[i].reserved[0]) +
      static_cast<double>(data.points[i].reserved[1] << 8U) +
      static_cast<double>(data.points[i].reserved[2] << 16U);
  }

  if (truncated_) {
    truncated_pings_++;
  }
  valid_pings_++;
  ping_sequence_++;
  last_ping_time_ = std::chrono::steady_clock::now();
}

void MultibeamInterface::handle_yz_point_data(const multibeam::YzPointData & data)
{
  const rclcpp::Time stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  reception_time_sec_ = static_cast<int32_t>(stamp.seconds());
  reception_time_nanosec_ =
    static_cast<uint32_t>(stamp.nanoseconds() - static_cast<int64_t>(reception_time_sec_) * 1000000000LL);

  ping_number_ = data.ping_number;
  power_up_time_ms_ = data.power_up_time_ms;
  device_utc_time_ms_ = 0;
  listening_time_s_ = data.sound_speed_m_s > 0.0F ? 2.0F * data.end_m / data.sound_speed_m_s : 0.0F;
  sound_speed_m_s_ = data.sound_speed_m_s;
  acoustic_frequency_hz_ = 0;
  pulse_duration_s_ = 0.0F;
  flags_ = 0;
  reported_detection_count_ = data.reported_point_count;
  stored_detection_count_ = static_cast<uint16_t>(std::min(data.points.size(), kMaxDetections));
  truncated_ = data.reported_point_count > kMaxDetections;

  up_vector_x_ = data.up_vector_x;
  up_vector_y_ = data.up_vector_y;
  up_vector_z_ = data.up_vector_z;
  if (std::isfinite(data.water_temperature_c) && data.water_temperature_c > -999.0F &&
      std::isfinite(data.water_pressure_bar) && data.water_pressure_bar >= 0.0F)
  {
    temperature_c_ = data.water_temperature_c;
    pressure_bar_ = data.water_pressure_bar;
    water_valid_ = true;
    water_sequence_++;
  }

  for (auto & detection : detections_) {
    detection = {};
  }

  for (std::size_t i = 0; i < stored_detection_count_; ++i) {
    const float y = data.points[i].y_m;
    const float z = data.points[i].z_m;
    const float range = std::sqrt(y * y + z * z);
    detections_[i].angle_rad = std::atan2(y, -z);
    detections_[i].time_of_flight_s = data.sound_speed_m_s > 0.0F ?
      2.0F * range / data.sound_speed_m_s : 0.0F;
    detections_[i].power = 0.0;
    detections_[i].point_type = 0.0;
    detections_[i].reserved = 0.0;
  }

  if (truncated_) {
    truncated_pings_++;
  }
  valid_pings_++;
  ping_sequence_++;
  last_ping_time_ = std::chrono::steady_clock::now();
}

void MultibeamInterface::handle_attitude_report(const multibeam::AttitudeReport & data)
{
  up_vector_x_ = data.up_vector_x;
  up_vector_y_ = data.up_vector_y;
  up_vector_z_ = data.up_vector_z;
  attitude_utc_time_ms_ = data.utc_time_ms;
  attitude_power_up_time_ms_ = data.power_up_time_ms;
  attitude_sequence_++;
}

void MultibeamInterface::handle_water_stats(const multibeam::WaterStats & data)
{
  if (!std::isfinite(data.temperature_c) || !std::isfinite(data.pressure_bar) || data.pressure_bar < 0.0F) {
    return;
  }
  temperature_c_ = data.temperature_c;
  pressure_bar_ = data.pressure_bar;
  water_valid_ = true;
  water_sequence_++;
}

void MultibeamInterface::publish_states(std::unordered_map<std::string, double> & states)
{
  set_state(states, "ping_sequence", static_cast<double>(ping_sequence_));
  set_state(states, "reception_time.sec", static_cast<double>(reception_time_sec_));
  set_state(states, "reception_time.nanosec", static_cast<double>(reception_time_nanosec_));
  set_state(states, "ping_number", static_cast<double>(ping_number_));
  set_state(states, "power_up_time_ms", static_cast<double>(power_up_time_ms_));
  set_state(states, "device_utc_time_ms_hi", static_cast<double>(hi32(device_utc_time_ms_)));
  set_state(states, "device_utc_time_ms_lo", static_cast<double>(lo32(device_utc_time_ms_)));
  set_state(states, "listening_time_s", listening_time_s_);
  set_state(states, "sound_speed_m_s", sound_speed_m_s_);
  set_state(states, "acoustic_frequency_hz", static_cast<double>(acoustic_frequency_hz_));
  set_state(states, "pulse_duration_s", pulse_duration_s_);
  set_state(states, "flags", static_cast<double>(flags_));
  set_state(states, "reported_detection_count", static_cast<double>(reported_detection_count_));
  set_state(states, "stored_detection_count", static_cast<double>(stored_detection_count_));
  set_state(states, "truncated", truncated_ ? 1.0 : 0.0);

  for (std::size_t i = 0; i < kMaxDetections; ++i) {
    const auto prefix = "detection_" + std::to_string(i) + ".";
    set_state(states, prefix + "angle_rad", detections_[i].angle_rad);
    set_state(states, prefix + "time_of_flight_s", detections_[i].time_of_flight_s);
    set_state(states, prefix + "power", detections_[i].power);
    set_state(states, prefix + "point_type", detections_[i].point_type);
    set_state(states, prefix + "reserved", detections_[i].reserved);
  }

  set_state(states, "attitude_sequence", static_cast<double>(attitude_sequence_));
  set_state(states, "up_vector.x", up_vector_x_);
  set_state(states, "up_vector.y", up_vector_y_);
  set_state(states, "up_vector.z", up_vector_z_);
  set_state(states, "attitude_utc_time_ms_hi", static_cast<double>(hi32(attitude_utc_time_ms_)));
  set_state(states, "attitude_utc_time_ms_lo", static_cast<double>(lo32(attitude_utc_time_ms_)));
  set_state(states, "attitude_power_up_time_ms", static_cast<double>(attitude_power_up_time_ms_));

  set_state(states, "water_sequence", static_cast<double>(water_sequence_));
  set_state(states, "water_valid", water_valid_ ? 1.0 : 0.0);
  set_state(states, "temperature_c", temperature_c_);
  set_state(states, "pressure_bar", pressure_bar_);

  const auto now = std::chrono::steady_clock::now();
  set_state(states, "transport_state", static_cast<double>(static_cast<int>(tcp_.state())));
  set_state(states, "connected", tcp_.connected() ? 1.0 : 0.0);
  set_state(states, "valid_pings", static_cast<double>(valid_pings_));
  set_state(states, "reconnect_count", static_cast<double>(tcp_.stats().reconnects));
  set_state(states, "checksum_errors", static_cast<double>(parser_.counters().checksum_errors));
  set_state(states, "malformed_packets", static_cast<double>(parser_.counters().malformed_packets + malformed_payloads_));
  set_state(states, "unknown_ids", static_cast<double>(parser_.counters().unknown_ids));
  set_state(states, "truncated_pings", static_cast<double>(truncated_pings_));
  set_state(states, "rx_bytes", static_cast<double>(tcp_.stats().rx_bytes));
  set_state(states, "tx_bytes", static_cast<double>(tcp_.stats().tx_bytes));
  set_state(states, "last_errno", static_cast<double>(tcp_.stats().last_errno));
  set_state(states, "seconds_since_last_rx", std::chrono::duration<double>(now - last_rx_time_).count());
  set_state(states, "seconds_since_last_ping", std::chrono::duration<double>(now - last_ping_time_).count());
}

void MultibeamInterface::set_state(
  std::unordered_map<std::string, double> & states,
  const std::string & name,
  const double value)
{
  const auto it = states.find(name);
  if (it != states.end()) {
    it->second = value;
  }
}

uint64_t MultibeamInterface::now_utc_ms() const
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::MultibeamInterface,
  sura_hardware_interface::SensorInterfaceBase)
