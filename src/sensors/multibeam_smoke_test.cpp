#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "sura_hardware_interface/sensors/multibeam_ping_protocol.hpp"
#include "sura_hardware_interface/sensors/multibeam_tcp_client.hpp"

namespace
{

struct Options
{
  std::string ip{"192.168.1.86"};
  int port{62312};
  int pings{0};
};

bool parse_args(int argc, char ** argv, Options & options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--ip" && i + 1 < argc) {
      options.ip = argv[++i];
    } else if (arg == "--port" && i + 1 < argc) {
      options.port = std::atoi(argv[++i]);
    } else if (arg == "--no-ping") {
      options.pings = 0;
    } else if (arg == "--pings" && i + 1 < argc) {
      options.pings = std::atoi(argv[++i]);
    } else {
      return false;
    }
  }
  return options.port > 0 && options.pings >= 0;
}

uint64_t utc_ms()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parse_args(argc, argv, options)) {
    std::cerr << "Usage: multibeam_smoke_test [--ip A.B.C.D] [--port N] "
              << "[--no-ping | --pings N]\n";
    return 2;
  }

  using sura_hardware_interface::multibeam::Frame;
  using sura_hardware_interface::multibeam::Parser;
  using sura_hardware_interface::multibeam::SetPingParameters;
  using sura_hardware_interface::multibeam::TcpClient;
  using sura_hardware_interface::multibeam::TcpState;

  TcpClient tcp;
  Parser parser;
  tcp.configure(options.ip, options.port, 65536U, 64U, 65536U);

  SetPingParameters params;
  params.ping_enable = options.pings > 0;
  params.enable_atof_data = true;
  params.enable_channel_data = false;
  params.reserved_for_raw_data = false;
  params.enable_yz_point_data = true;

  bool configured = false;
  int received_pings = 0;
  uint64_t checksum_errors = 0;
  uint64_t malformed_packets = 0;
  uint64_t unknown_ids = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);

  while (std::chrono::steady_clock::now() < deadline && received_pings < std::max(1, options.pings)) {
    const auto now = std::chrono::steady_clock::now();
    tcp.service_connection(now);
    if (tcp.state() == TcpState::Initializing && !configured) {
      configured = true;
      tcp.enqueue(sura_hardware_interface::multibeam::encode_general_request(
        sura_hardware_interface::multibeam::kMsgProtocolVersion));
      tcp.enqueue(sura_hardware_interface::multibeam::encode_general_request(
        sura_hardware_interface::multibeam::kMsgDeviceInformation));
      tcp.enqueue(sura_hardware_interface::multibeam::encode_set_ping_parameters(params));
    }

    tcp.flush_tx();
    tcp.receive(parser);

    if (options.pings == 0 && configured) {
      break;
    }

    Frame frame;
    while (parser.next(frame)) {
      if (frame.message_id == sura_hardware_interface::multibeam::kMsgUtcRequest) {
        tcp.enqueue(sura_hardware_interface::multibeam::encode_utc_response(utc_ms(), 1000U));
      }
      if (frame.message_id == sura_hardware_interface::multibeam::kMsgAtofPointData) {
        sura_hardware_interface::multibeam::AtofPointData data;
        if (sura_hardware_interface::multibeam::decode_atof_point_data(frame.payload, data)) {
          received_pings++;
          std::cout << "ping=" << data.ping_number
                    << " points=" << data.reported_point_count
                    << " sos=" << data.sound_speed_m_s
                    << " listening=" << data.listening_time_s << "\n";
        }
      }
      if (frame.message_id == sura_hardware_interface::multibeam::kMsgYzPointData) {
        sura_hardware_interface::multibeam::YzPointData data;
        if (sura_hardware_interface::multibeam::decode_yz_point_data(frame.payload, data)) {
          received_pings++;
          std::cout << "ping=" << data.ping_number
                    << " yz_points=" << data.reported_point_count
                    << " sos=" << data.sound_speed_m_s
                    << " range=" << data.start_m << ":" << data.end_m << "\n";
        }
      }
    }

    checksum_errors = parser.counters().checksum_errors;
    malformed_packets = parser.counters().malformed_packets;
    unknown_ids = parser.counters().unknown_ids;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (tcp.connected()) {
    params.ping_enable = false;
    tcp.enqueue(sura_hardware_interface::multibeam::encode_set_ping_parameters(params));
    tcp.flush_tx();
  }

  std::cout << "connected=" << (tcp.connected() ? "true" : "false")
            << " pings=" << received_pings
            << " rx_bytes=" << tcp.stats().rx_bytes
            << " tx_bytes=" << tcp.stats().tx_bytes
            << " checksum_errors=" << checksum_errors
            << " malformed_packets=" << malformed_packets
            << " unknown_ids=" << unknown_ids << "\n";

  return options.pings == 0 ? (configured && tcp.connected() ? 0 : 1) :
         (received_pings >= options.pings ? 0 : 1);
}
