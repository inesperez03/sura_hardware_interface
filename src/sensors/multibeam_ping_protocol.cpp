#include "sura_hardware_interface/sensors/multibeam_ping_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace sura_hardware_interface
{
namespace multibeam
{
namespace
{

void append_u8(std::vector<uint8_t> & bytes, const uint8_t value)
{
  bytes.push_back(value);
}

void append_bool(std::vector<uint8_t> & bytes, const bool value)
{
  append_u8(bytes, value ? 1U : 0U);
}

void append_u16(std::vector<uint8_t> & bytes, const uint16_t value)
{
  bytes.push_back(static_cast<uint8_t>(value & 0xffU));
  bytes.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
}

void append_i16(std::vector<uint8_t> & bytes, const int16_t value)
{
  append_u16(bytes, static_cast<uint16_t>(value));
}

void append_u32(std::vector<uint8_t> & bytes, const uint32_t value)
{
  bytes.push_back(static_cast<uint8_t>(value & 0xffU));
  bytes.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
  bytes.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
  bytes.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
}

void append_i32(std::vector<uint8_t> & bytes, const int32_t value)
{
  append_u32(bytes, static_cast<uint32_t>(value));
}

void append_u64(std::vector<uint8_t> & bytes, const uint64_t value)
{
  for (int shift = 0; shift < 64; shift += 8) {
    bytes.push_back(static_cast<uint8_t>((value >> shift) & 0xffU));
  }
}

void append_float(std::vector<uint8_t> & bytes, const float value)
{
  uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "float must be 32-bit");
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32(bytes, bits);
}

bool has_bytes(const std::vector<uint8_t> & bytes, const std::size_t offset, const std::size_t size)
{
  return offset <= bytes.size() && size <= bytes.size() - offset;
}

uint16_t read_u16_unchecked(const std::vector<uint8_t> & bytes, const std::size_t offset)
{
  return static_cast<uint16_t>(bytes[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(bytes[offset + 1]) << 8U);
}

bool read_u16(const std::vector<uint8_t> & bytes, std::size_t & offset, uint16_t & value)
{
  if (!has_bytes(bytes, offset, 2U)) {
    return false;
  }
  value = read_u16_unchecked(bytes, offset);
  offset += 2U;
  return true;
}

bool read_u32(const std::vector<uint8_t> & bytes, std::size_t & offset, uint32_t & value)
{
  if (!has_bytes(bytes, offset, 4U)) {
    return false;
  }
  value = static_cast<uint32_t>(bytes[offset]) |
          (static_cast<uint32_t>(bytes[offset + 1]) << 8U) |
          (static_cast<uint32_t>(bytes[offset + 2]) << 16U) |
          (static_cast<uint32_t>(bytes[offset + 3]) << 24U);
  offset += 4U;
  return true;
}

bool read_u64(const std::vector<uint8_t> & bytes, std::size_t & offset, uint64_t & value)
{
  if (!has_bytes(bytes, offset, 8U)) {
    return false;
  }
  value = 0;
  for (int shift = 0; shift < 64; shift += 8) {
    value |= static_cast<uint64_t>(bytes[offset++]) << shift;
  }
  return true;
}

bool read_float(const std::vector<uint8_t> & bytes, std::size_t & offset, float & value)
{
  uint32_t bits = 0;
  if (!read_u32(bytes, offset, bits)) {
    return false;
  }
  std::memcpy(&value, &bits, sizeof(value));
  return true;
}

uint16_t checksum(const std::vector<uint8_t> & bytes, const std::size_t count)
{
  uint16_t value = 0;
  for (std::size_t i = 0; i < count; ++i) {
    value = static_cast<uint16_t>(value + bytes[i]);
  }
  return value;
}

bool known_message_id(const uint16_t id)
{
  switch (id) {
    case kMsgAck:
    case kMsgNack:
    case kMsgDeviceInformation:
    case kMsgProtocolVersion:
    case kMsgGeneralRequest:
    case kMsgJsonWrapper:
    case kMsgUtcRequest:
    case kMsgUtcResponse:
    case kMsgWaterStats:
    case kMsgAttitudeReport:
    case kMsgYzPointData:
    case kMsgAtofPointData:
    case kMsgSetPingParameters:
      return true;
    default:
      return false;
  }
}

}  // namespace

Parser::Parser(const std::size_t max_payload_size)
: max_payload_size_(max_payload_size)
{
}

void Parser::append(const uint8_t * data, const std::size_t size)
{
  if (size == 0U) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + size);
  const std::size_t max_buffer_size = max_payload_size_ + kFrameHeaderSize + kFrameChecksumSize;
  if (buffer_.size() > max_buffer_size * 2U) {
    const std::size_t erase_count = buffer_.size() - max_buffer_size;
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(erase_count));
    counters_.bytes_dropped += erase_count;
    counters_.malformed_packets++;
  }
}

bool Parser::next(Frame & frame)
{
  while (buffer_.size() >= kFrameHeaderSize + kFrameChecksumSize) {
    const uint8_t header[] = {'B', 'R'};
    const auto it = std::search(buffer_.begin(), buffer_.end(), std::begin(header), std::end(header));
    if (it == buffer_.end()) {
      counters_.bytes_dropped += buffer_.size();
      buffer_.clear();
      return false;
    }

    if (it != buffer_.begin()) {
      const auto dropped = static_cast<std::size_t>(std::distance(buffer_.begin(), it));
      counters_.bytes_dropped += dropped;
      buffer_.erase(buffer_.begin(), it);
    }

    if (buffer_.size() < kFrameHeaderSize + kFrameChecksumSize) {
      return false;
    }

    const uint16_t payload_size = read_u16_unchecked(buffer_, 2U);
    if (payload_size > max_payload_size_) {
      buffer_.erase(buffer_.begin());
      counters_.malformed_packets++;
      counters_.bytes_dropped++;
      continue;
    }

    const std::size_t frame_size = kFrameHeaderSize + payload_size + kFrameChecksumSize;
    if (buffer_.size() < frame_size) {
      return false;
    }

    const uint16_t expected_checksum = checksum(buffer_, kFrameHeaderSize + payload_size);
    const uint16_t received_checksum = read_u16_unchecked(buffer_, kFrameHeaderSize + payload_size);
    if (expected_checksum != received_checksum) {
      buffer_.erase(buffer_.begin());
      counters_.checksum_errors++;
      counters_.bytes_dropped++;
      continue;
    }

    frame.message_id = read_u16_unchecked(buffer_, 4U);
    frame.source = buffer_[6U];
    frame.destination = buffer_[7U];
    frame.payload.assign(
      buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameHeaderSize),
      buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameHeaderSize + payload_size));
    buffer_.erase(
      buffer_.begin(),
      buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));

    if (!known_message_id(frame.message_id)) {
      counters_.unknown_ids++;
    }

    return true;
  }

  return false;
}

void Parser::clear()
{
  buffer_.clear();
}

const ParserCounters & Parser::counters() const
{
  return counters_;
}

std::size_t Parser::buffered_size() const
{
  return buffer_.size();
}

std::vector<uint8_t> encode_frame(
  const uint16_t message_id,
  const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> frame;
  if (payload.size() > std::numeric_limits<uint16_t>::max()) {
    return frame;
  }

  frame.reserve(kFrameHeaderSize + payload.size() + kFrameChecksumSize);
  append_u8(frame, 'B');
  append_u8(frame, 'R');
  append_u16(frame, static_cast<uint16_t>(payload.size()));
  append_u16(frame, message_id);
  append_u8(frame, 0);
  append_u8(frame, 0);
  frame.insert(frame.end(), payload.begin(), payload.end());
  append_u16(frame, checksum(frame, frame.size()));
  return frame;
}

std::vector<uint8_t> encode_general_request(const uint16_t requested_id)
{
  std::vector<uint8_t> payload;
  append_u16(payload, requested_id);
  return encode_frame(kMsgGeneralRequest, payload);
}

std::vector<uint8_t> encode_set_ping_parameters(const SetPingParameters & parameters)
{
  std::vector<uint8_t> payload;
  payload.reserve(44U);
  append_i32(payload, parameters.start_mm);
  append_i32(payload, parameters.end_mm);
  append_float(payload, parameters.sound_speed_m_s);
  append_i16(payload, parameters.gain_index);
  append_i16(payload, parameters.msec_per_ping);
  append_u16(payload, parameters.deprecated);
  append_u8(payload, parameters.diagnostic_injected_signal);
  append_bool(payload, parameters.ping_enable);
  append_bool(payload, parameters.enable_channel_data);
  append_bool(payload, parameters.reserved_for_raw_data);
  append_bool(payload, parameters.enable_yz_point_data);
  append_bool(payload, parameters.enable_atof_data);
  append_i32(payload, parameters.target_ping_hz);
  append_u16(payload, parameters.n_range_steps);
  append_u16(payload, parameters.reserved);
  append_float(payload, parameters.pulse_len_steps);
  return encode_frame(kMsgSetPingParameters, payload);
}

std::vector<uint8_t> encode_utc_response(
  const uint64_t utc_time_ms,
  const uint32_t accuracy_ms)
{
  std::vector<uint8_t> payload;
  append_u64(payload, utc_time_ms);
  append_u32(payload, accuracy_ms);
  return encode_frame(kMsgUtcResponse, payload);
}

bool decode_atof_point_data(const std::vector<uint8_t> & payload, AtofPointData & data)
{
  constexpr std::size_t fixed_size = 40U;
  if (payload.size() < fixed_size) {
    return false;
  }

  std::size_t offset = 0U;
  if (!read_u32(payload, offset, data.power_up_time_ms) ||
      !read_u64(payload, offset, data.utc_time_ms) ||
      !read_float(payload, offset, data.listening_time_s) ||
      !read_float(payload, offset, data.sound_speed_m_s) ||
      !read_u32(payload, offset, data.ping_number) ||
      !read_u32(payload, offset, data.acoustic_frequency_hz) ||
      !read_float(payload, offset, data.pulse_duration_s) ||
      !read_u32(payload, offset, data.flags) ||
      !read_u16(payload, offset, data.reported_point_count) ||
      !read_u16(payload, offset, data.reserved))
  {
    return false;
  }

  const std::size_t points_bytes = payload.size() - fixed_size;
  if (points_bytes % kAtofPointSize != 0U) {
    return false;
  }

  const std::size_t encoded_points = points_bytes / kAtofPointSize;
  if (data.reported_point_count > encoded_points) {
    return false;
  }

  data.points.clear();
  data.points.reserve(data.reported_point_count);
  for (std::size_t i = 0; i < data.reported_point_count; ++i) {
    AtofPoint point;
    if (!read_float(payload, offset, point.angle_rad) ||
        !read_float(payload, offset, point.time_of_flight_s) ||
        !read_float(payload, offset, point.power))
    {
      return false;
    }
    if (!has_bytes(payload, offset, 4U)) {
      return false;
    }
    point.point_type = payload[offset++];
    point.reserved[0] = payload[offset++];
    point.reserved[1] = payload[offset++];
    point.reserved[2] = payload[offset++];
    data.points.push_back(point);
  }

  return true;
}

bool decode_yz_point_data(const std::vector<uint8_t> & payload, YzPointData & data)
{
  constexpr std::size_t fixed_size = 100U;
  if (payload.size() < fixed_size) {
    return false;
  }

  std::size_t offset = 0U;
  uint32_t unused_u32 = 0;
  uint16_t unused_u16 = 0;
  float unused_float = 0.0F;

  if (!read_u32(payload, offset, data.power_up_time_ms) ||
      !read_u32(payload, offset, data.ping_number) ||
      !read_float(payload, offset, data.sound_speed_m_s) ||
      !read_float(payload, offset, data.up_vector_x) ||
      !read_float(payload, offset, data.up_vector_y) ||
      !read_float(payload, offset, data.up_vector_z))
  {
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    if (!read_float(payload, offset, unused_float)) {
      return false;
    }
  }

  for (int i = 0; i < 10; ++i) {
    if (!read_u32(payload, offset, unused_u32)) {
      return false;
    }
  }

  if (!read_float(payload, offset, data.water_temperature_c) ||
      !read_float(payload, offset, data.water_pressure_bar) ||
      !read_float(payload, offset, data.heave_m) ||
      !read_float(payload, offset, data.start_m) ||
      !read_float(payload, offset, data.end_m) ||
      !read_u16(payload, offset, unused_u16) ||
      !read_u16(payload, offset, data.reported_point_count))
  {
    return false;
  }

  const std::size_t points_bytes = payload.size() - fixed_size;
  if (points_bytes % (2U * sizeof(float)) != 0U) {
    return false;
  }

  const std::size_t encoded_points = points_bytes / (2U * sizeof(float));
  if (data.reported_point_count > encoded_points) {
    return false;
  }

  data.points.clear();
  data.points.reserve(data.reported_point_count);
  for (std::size_t i = 0; i < data.reported_point_count; ++i) {
    YzPoint point;
    if (!read_float(payload, offset, point.y_m) ||
        !read_float(payload, offset, point.z_m))
    {
      return false;
    }
    data.points.push_back(point);
  }

  return true;
}

bool decode_attitude_report(const std::vector<uint8_t> & payload, AttitudeReport & data)
{
  if (payload.size() < 36U) {
    return false;
  }
  std::size_t offset = 0U;
  return read_float(payload, offset, data.up_vector_x) &&
         read_float(payload, offset, data.up_vector_y) &&
         read_float(payload, offset, data.up_vector_z) &&
         read_float(payload, offset, data.reserved_1) &&
         read_float(payload, offset, data.reserved_2) &&
         read_float(payload, offset, data.reserved_3) &&
         read_u64(payload, offset, data.utc_time_ms) &&
         read_u32(payload, offset, data.power_up_time_ms);
}

bool decode_water_stats(const std::vector<uint8_t> & payload, WaterStats & data)
{
  if (payload.size() < 8U) {
    return false;
  }
  std::size_t offset = 0U;
  return read_float(payload, offset, data.temperature_c) &&
         read_float(payload, offset, data.pressure_bar);
}

}  // namespace multibeam
}  // namespace sura_hardware_interface
