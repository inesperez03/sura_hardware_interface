#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "sura_hardware_interface/sensors/multibeam_ping_protocol.hpp"

namespace sura_hardware_interface
{
namespace multibeam
{

enum class TcpState
{
  Disconnected = 0,
  Connecting = 1,
  Initializing = 2,
  Streaming = 3,
  Backoff = 4,
};

struct TcpStats
{
  uint64_t rx_bytes{0};
  uint64_t tx_bytes{0};
  uint64_t reconnects{0};
  int last_errno{0};
};

class TcpClient
{
public:
  TcpClient();
  ~TcpClient();

  void configure(
    const std::string & ip_address,
    int port,
    std::size_t max_rx_bytes_per_read,
    std::size_t max_messages_per_read,
    std::size_t max_tx_queue_bytes);

  void reset();
  void close();
  void set_state(TcpState state);
  TcpState state() const;
  bool connected() const;

  void service_connection(std::chrono::steady_clock::time_point now);
  bool enqueue(std::vector<uint8_t> bytes);
  void flush_tx();
  void receive(Parser & parser);
  void reconnect_later(int error_code);

  const TcpStats & stats() const;

private:
  struct PendingWrite
  {
    std::vector<uint8_t> bytes;
    std::size_t offset{0};
  };

  bool begin_connect(std::chrono::steady_clock::time_point now);
  void finish_connect(std::chrono::steady_clock::time_point now);
  void enter_backoff(std::chrono::steady_clock::time_point now, int error_code);
  void reset_backoff();

  std::string ip_address_{"192.168.1.86"};
  int port_{62312};
  int fd_{-1};
  TcpState state_{TcpState::Disconnected};

  std::size_t max_rx_bytes_per_read_{65536};
  std::size_t max_messages_per_read_{64};
  std::size_t max_tx_queue_bytes_{65536};
  std::size_t queued_tx_bytes_{0};
  std::deque<PendingWrite> tx_queue_;

  std::chrono::steady_clock::time_point connect_started_{};
  std::chrono::steady_clock::time_point next_attempt_{};
  std::chrono::milliseconds connect_timeout_{1000};
  std::chrono::milliseconds backoff_{1000};
  std::chrono::milliseconds max_backoff_{30000};

  TcpStats stats_;
};

}  // namespace multibeam
}  // namespace sura_hardware_interface
