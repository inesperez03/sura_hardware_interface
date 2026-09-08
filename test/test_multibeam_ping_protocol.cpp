#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "sura_hardware_interface/sensors/multibeam_ping_protocol.hpp"

namespace
{

using sura_hardware_interface::multibeam::AtofPointData;
using sura_hardware_interface::multibeam::Frame;
using sura_hardware_interface::multibeam::Parser;
using sura_hardware_interface::multibeam::SetPingParameters;
using sura_hardware_interface::multibeam::YzPointData;

template<typename T>
void append_le(std::vector<uint8_t> & bytes, T value)
{
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    bytes.push_back(static_cast<uint8_t>((static_cast<uint64_t>(value) >> (8U * i)) & 0xffU));
  }
}

void append_float(std::vector<uint8_t> & bytes, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_le(bytes, bits);
}

}  // namespace

TEST(MultibeamPingProtocol, EncodesGeneralRequest)
{
  const auto bytes = sura_hardware_interface::multibeam::encode_general_request(
    sura_hardware_interface::multibeam::kMsgAtofPointData);

  ASSERT_EQ(bytes.size(), 12U);
  EXPECT_EQ(bytes[0], 'B');
  EXPECT_EQ(bytes[1], 'R');
  EXPECT_EQ(bytes[2], 2U);
  EXPECT_EQ(bytes[3], 0U);
  EXPECT_EQ(bytes[4], 6U);
  EXPECT_EQ(bytes[5], 0U);
  EXPECT_EQ(bytes[8], 0xc4U);
  EXPECT_EQ(bytes[9], 0x0bU);
}

TEST(MultibeamPingProtocol, EncodesSetPingParameters)
{
  SetPingParameters params;
  params.ping_enable = true;
  const auto bytes = sura_hardware_interface::multibeam::encode_set_ping_parameters(params);

  ASSERT_EQ(bytes.size(), 46U);
  EXPECT_EQ(bytes[4], 0xcfU);
  EXPECT_EQ(bytes[5], 0x0bU);
  EXPECT_EQ(bytes[32], 0x80U);
  EXPECT_EQ(bytes[33], 0xa9U);
  EXPECT_EQ(bytes[34], 0x03U);
  EXPECT_EQ(bytes[35], 0x00U);
}

TEST(MultibeamPingProtocol, ParsesFragmentedAndGarbagePrefixedFrame)
{
  const auto frame = sura_hardware_interface::multibeam::encode_general_request(
    sura_hardware_interface::multibeam::kMsgWaterStats);

  Parser parser;
  const uint8_t garbage[] = {0x99, 0x88, 0x77};
  parser.append(garbage, sizeof(garbage));
  parser.append(frame.data(), 5U);

  Frame parsed;
  EXPECT_FALSE(parser.next(parsed));
  parser.append(frame.data() + 5U, frame.size() - 5U);
  ASSERT_TRUE(parser.next(parsed));
  EXPECT_EQ(parsed.message_id, sura_hardware_interface::multibeam::kMsgGeneralRequest);
  EXPECT_EQ(parser.counters().bytes_dropped, 3U);
}

TEST(MultibeamPingProtocol, RecoversAfterChecksumError)
{
  auto bad_frame = sura_hardware_interface::multibeam::encode_general_request(
    sura_hardware_interface::multibeam::kMsgWaterStats);
  bad_frame.back() ^= 0xffU;

  const auto good_frame = sura_hardware_interface::multibeam::encode_general_request(
    sura_hardware_interface::multibeam::kMsgAttitudeReport);

  Parser parser;
  parser.append(bad_frame.data(), bad_frame.size());
  parser.append(good_frame.data(), good_frame.size());

  Frame parsed;
  ASSERT_TRUE(parser.next(parsed));
  EXPECT_EQ(parsed.message_id, sura_hardware_interface::multibeam::kMsgGeneralRequest);
  EXPECT_GT(parser.counters().checksum_errors, 0U);
}

TEST(MultibeamPingProtocol, DecodesAtofPointData)
{
  std::vector<uint8_t> payload;
  append_le<uint32_t>(payload, 1234U);
  append_le<uint64_t>(payload, 5678U);
  append_float(payload, 0.25F);
  append_float(payload, 1500.0F);
  append_le<uint32_t>(payload, 42U);
  append_le<uint32_t>(payload, 240000U);
  append_float(payload, 0.001F);
  append_le<uint32_t>(payload, 7U);
  append_le<uint16_t>(payload, 1U);
  append_le<uint16_t>(payload, 0U);
  append_float(payload, 0.5F);
  append_float(payload, 0.01F);
  append_float(payload, 12.0F);
  payload.push_back(2U);
  payload.push_back(3U);
  payload.push_back(4U);
  payload.push_back(5U);

  AtofPointData data;
  ASSERT_TRUE(sura_hardware_interface::multibeam::decode_atof_point_data(payload, data));
  EXPECT_EQ(data.power_up_time_ms, 1234U);
  EXPECT_EQ(data.utc_time_ms, 5678U);
  EXPECT_EQ(data.ping_number, 42U);
  ASSERT_EQ(data.points.size(), 1U);
  EXPECT_FLOAT_EQ(data.points[0].angle_rad, 0.5F);
  EXPECT_FLOAT_EQ(data.points[0].time_of_flight_s, 0.01F);
  EXPECT_FLOAT_EQ(data.points[0].power, 12.0F);
  EXPECT_EQ(data.points[0].point_type, 2U);
}

TEST(MultibeamPingProtocol, DecodesYzPointData)
{
  std::vector<uint8_t> payload;
  append_le<uint32_t>(payload, 1234U);
  append_le<uint32_t>(payload, 42U);
  append_float(payload, 1500.0F);
  append_float(payload, 0.1F);
  append_float(payload, 0.2F);
  append_float(payload, 0.9F);
  append_float(payload, 0.0F);
  append_float(payload, 0.0F);
  append_float(payload, 0.0F);
  for (int i = 0; i < 10; ++i) {
    append_le<uint32_t>(payload, 0U);
  }
  append_float(payload, 12.5F);
  append_float(payload, 1.2F);
  append_float(payload, 0.0F);
  append_float(payload, 0.25F);
  append_float(payload, 10.0F);
  append_le<uint16_t>(payload, 0U);
  append_le<uint16_t>(payload, 1U);
  append_float(payload, 2.0F);
  append_float(payload, -3.0F);

  YzPointData data;
  ASSERT_TRUE(sura_hardware_interface::multibeam::decode_yz_point_data(payload, data));
  EXPECT_EQ(data.power_up_time_ms, 1234U);
  EXPECT_EQ(data.ping_number, 42U);
  EXPECT_FLOAT_EQ(data.sound_speed_m_s, 1500.0F);
  EXPECT_FLOAT_EQ(data.water_temperature_c, 12.5F);
  EXPECT_FLOAT_EQ(data.water_pressure_bar, 1.2F);
  ASSERT_EQ(data.points.size(), 1U);
  EXPECT_FLOAT_EQ(data.points[0].y_m, 2.0F);
  EXPECT_FLOAT_EQ(data.points[0].z_m, -3.0F);
}
