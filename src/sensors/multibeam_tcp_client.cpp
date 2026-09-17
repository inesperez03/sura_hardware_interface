#include "sura_hardware_interface/sensors/multibeam_tcp_client.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sura_hardware_interface
{
namespace multibeam
{

TcpClient::TcpClient() = default;

TcpClient::~TcpClient()
{
  close();
}

void TcpClient::configure(
  const std::string & ip_address,
  const int port,
  const std::size_t max_rx_bytes_per_read,
  const std::size_t max_messages_per_read,
  const std::size_t max_tx_queue_bytes)
{
  ip_address_ = ip_address;
  port_ = port;
  max_rx_bytes_per_read_ = max_rx_bytes_per_read;
  max_messages_per_read_ = max_messages_per_read;
  max_tx_queue_bytes_ = max_tx_queue_bytes;
}

void TcpClient::reset()
{
  close();
  tx_queue_.clear();
  queued_tx_bytes_ = 0;
  state_ = TcpState::Disconnected;
  reset_backoff();
}

void TcpClient::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  if (state_ != TcpState::Backoff) {
    state_ = TcpState::Disconnected;
  }
}

void TcpClient::set_state(const TcpState state)
{
  state_ = state;
}

TcpState TcpClient::state() const
{
  return state_;
}

bool TcpClient::connected() const
{
  return fd_ >= 0 && (state_ == TcpState::Initializing || state_ == TcpState::Streaming);
}

void TcpClient::service_connection(const std::chrono::steady_clock::time_point now)
{
  if (state_ == TcpState::Backoff) {
    if (now < next_attempt_) {
      return;
    }
    state_ = TcpState::Disconnected;
  }

  if (state_ == TcpState::Disconnected) {
    (void)begin_connect(now);
    return;
  }

  if (state_ == TcpState::Connecting) {
    finish_connect(now);
  }
}

bool TcpClient::begin_connect(const std::chrono::steady_clock::time_point now)
{
  close();

  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0) {
    enter_backoff(now, errno);
    return false;
  }

  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    enter_backoff(now, errno);
    return false;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port_));
  if (::inet_pton(AF_INET, ip_address_.c_str(), &address.sin_addr) <= 0) {
    enter_backoff(now, errno);
    return false;
  }

  const int result = ::connect(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address));
  if (result == 0) {
    state_ = TcpState::Initializing;
    stats_.reconnects++;
    reset_backoff();
    return true;
  }

  if (errno != EINPROGRESS) {
    enter_backoff(now, errno);
    return false;
  }

  connect_started_ = now;
  state_ = TcpState::Connecting;
  return true;
}

void TcpClient::finish_connect(const std::chrono::steady_clock::time_point now)
{
  if (fd_ < 0) {
    enter_backoff(now, ENOTCONN);
    return;
  }

  if (now - connect_started_ > connect_timeout_) {
    enter_backoff(now, ETIMEDOUT);
    return;
  }

  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLOUT;
  const int ready = ::poll(&pfd, 1, 0);
  if (ready == 0) {
    return;
  }
  if (ready < 0) {
    enter_backoff(now, errno);
    return;
  }

  int socket_error = 0;
  socklen_t len = sizeof(socket_error);
  if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &socket_error, &len) < 0 || socket_error != 0) {
    enter_backoff(now, socket_error == 0 ? errno : socket_error);
    return;
  }

  state_ = TcpState::Initializing;
  stats_.reconnects++;
  reset_backoff();
}

bool TcpClient::enqueue(std::vector<uint8_t> bytes)
{
  if (bytes.empty() || bytes.size() > max_tx_queue_bytes_) {
    return false;
  }
  while (!tx_queue_.empty() && queued_tx_bytes_ + bytes.size() > max_tx_queue_bytes_) {
    queued_tx_bytes_ -= tx_queue_.front().bytes.size() - tx_queue_.front().offset;
    tx_queue_.pop_front();
  }
  if (queued_tx_bytes_ + bytes.size() > max_tx_queue_bytes_) {
    return false;
  }
  queued_tx_bytes_ += bytes.size();
  tx_queue_.push_back({std::move(bytes), 0U});
  return true;
}

void TcpClient::flush_tx()
{
  while (fd_ >= 0 && !tx_queue_.empty()) {
    auto & item = tx_queue_.front();
    const ssize_t sent = ::send(
      fd_,
      item.bytes.data() + item.offset,
      item.bytes.size() - item.offset,
      MSG_DONTWAIT);

    if (sent > 0) {
      const auto count = static_cast<std::size_t>(sent);
      item.offset += count;
      queued_tx_bytes_ -= count;
      stats_.tx_bytes += count;
      if (item.offset >= item.bytes.size()) {
        tx_queue_.pop_front();
      }
      continue;
    }

    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    }

    enter_backoff(std::chrono::steady_clock::now(), sent < 0 ? errno : ECONNRESET);
    return;
  }
}

void TcpClient::receive(Parser & parser)
{
  std::array<uint8_t, 4096> bytes{};
  std::size_t rx_total = 0;
  std::size_t reads = 0;

  while (fd_ >= 0 && rx_total < max_rx_bytes_per_read_ && reads < max_messages_per_read_) {
    const std::size_t limit = std::min(bytes.size(), max_rx_bytes_per_read_ - rx_total);
    const ssize_t received = ::recv(fd_, bytes.data(), limit, MSG_DONTWAIT);
    if (received > 0) {
      const auto count = static_cast<std::size_t>(received);
      parser.append(bytes.data(), count);
      rx_total += count;
      stats_.rx_bytes += count;
      reads++;
      continue;
    }

    if (received == 0) {
      enter_backoff(std::chrono::steady_clock::now(), ECONNRESET);
      return;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return;
    }

    enter_backoff(std::chrono::steady_clock::now(), errno);
    return;
  }
}

void TcpClient::reconnect_later(const int error_code)
{
  enter_backoff(std::chrono::steady_clock::now(), error_code);
}

const TcpStats & TcpClient::stats() const
{
  return stats_;
}

void TcpClient::enter_backoff(
  const std::chrono::steady_clock::time_point now,
  const int error_code)
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  stats_.last_errno = error_code;
  state_ = TcpState::Backoff;
  next_attempt_ = now + backoff_;
  backoff_ = std::min(backoff_ * 2, max_backoff_);
}

void TcpClient::reset_backoff()
{
  backoff_ = std::chrono::milliseconds(1000);
}

}  // namespace multibeam
}  // namespace sura_hardware_interface
