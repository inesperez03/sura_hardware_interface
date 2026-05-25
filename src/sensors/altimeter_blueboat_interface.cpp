#include "sura_hardware_interface/sensors/altimeter_blueboat_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iterator>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace sura_hardware_interface
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("AltimeterBlueboatInterface");

constexpr uint16_t kCommonProtocolVersion = 5;
constexpr uint16_t kPing1dSetRange = 1001;
constexpr uint16_t kPing1dSetSpeedOfSound = 1002;
constexpr uint16_t kPing1dSetModeAuto = 1003;
constexpr uint16_t kPing1dSetPingInterval = 1004;
constexpr uint16_t kPing1dSetGainSetting = 1005;
constexpr uint16_t kPing1dGeneralInfo = 1210;
constexpr uint16_t kPing1dDistanceSimple = 1211;
constexpr uint16_t kPing1dDistance = 1212;

std::string get_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & name,
  const std::string & hardware_name,
  const std::string & default_value)
{
  const auto sensor_it = sensor_info.parameters.find(name);
  if (sensor_it != sensor_info.parameters.end()) {
    return sensor_it->second;
  }

  const auto hardware_it = hardware_info.hardware_parameters.find(hardware_name);
  if (hardware_it != hardware_info.hardware_parameters.end()) {
    return hardware_it->second;
  }

  return default_value;
}

void set_state_if_exists(
  std::unordered_map<std::string, double> & states,
  const std::string & name,
  const double value)
{
  const auto it = states.find(name);
  if (it != states.end()) {
    it->second = value;
  }
}

bool baudrate_to_speed(const int baudrate, speed_t & speed)
{
  switch (baudrate) {
    case 4800:
      speed = B4800;
      return true;
    case 9600:
      speed = B9600;
      return true;
    case 19200:
      speed = B19200;
      return true;
    case 38400:
      speed = B38400;
      return true;
    case 57600:
      speed = B57600;
      return true;
    case 115200:
      speed = B115200;
      return true;
#ifdef B230400
    case 230400:
      speed = B230400;
      return true;
#endif
    default:
      return false;
  }
}

std::string sensor_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const std::string & default_value)
{
  const auto it = sensor_info.parameters.find(name);
  return it == sensor_info.parameters.end() ? default_value : it->second;
}

int parse_int_param(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const int default_value,
  const std::string & sensor_name)
{
  const std::string value = sensor_param_or(sensor_info, name, std::to_string(default_value));

  try {
    return std::stoi(value);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      kLogger,
      "Invalid %s '%s' for altimeter sensor '%s': %s. Using default %d",
      name.c_str(),
      value.c_str(),
      sensor_name.c_str(),
      ex.what(),
      default_value);
    return default_value;
  }
}

void append_u8(std::vector<uint8_t> & payload, const uint8_t value)
{
  payload.push_back(value);
}

void append_u16(std::vector<uint8_t> & payload, const uint16_t value)
{
  payload.push_back(static_cast<uint8_t>(value & 0xff));
  payload.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
}

void append_u32(std::vector<uint8_t> & payload, const uint32_t value)
{
  payload.push_back(static_cast<uint8_t>(value & 0xff));
  payload.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  payload.push_back(static_cast<uint8_t>((value >> 16) & 0xff));
  payload.push_back(static_cast<uint8_t>((value >> 24) & 0xff));
}

uint16_t read_u16(const std::vector<uint8_t> & payload, const std::size_t offset)
{
  return static_cast<uint16_t>(payload[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(payload[offset + 1]) << 8);
}

uint32_t read_u32(const std::vector<uint8_t> & payload, const std::size_t offset)
{
  return static_cast<uint32_t>(payload[offset]) |
         (static_cast<uint32_t>(payload[offset + 1]) << 8) |
         (static_cast<uint32_t>(payload[offset + 2]) << 16) |
         (static_cast<uint32_t>(payload[offset + 3]) << 24);
}

}  // namespace

bool AltimeterBlueboatInterface::initialize(
  const hardware_interface::ComponentInfo & sensor_info,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & environment,
  const rclcpp::Node::SharedPtr &)
{
  if (initialized_) {
    return true;
  }

  sensor_name_ = sensor_info.name;
  environment_ = environment;

  if (environment_ != "real" && environment_ != "sim") {
    RCLCPP_ERROR(
      kLogger,
      "Unsupported environment '%s' for altimeter sensor '%s'",
      environment_.c_str(),
      sensor_name_.c_str());
    return false;
  }

  serial_port_ = get_param_or(
    sensor_info,
    hardware_info,
    "serial_port",
    "altimeter_serial_port",
    serial_port_);

  const std::string baudrate = get_param_or(
    sensor_info,
    hardware_info,
    "baudrate",
    "altimeter_baudrate",
    std::to_string(baudrate_));

  try {
    baudrate_ = std::stoi(baudrate);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      kLogger,
      "Invalid baudrate '%s' for altimeter sensor '%s': %s. Using default %d",
      baudrate.c_str(),
      sensor_name_.c_str(),
      ex.what(),
      baudrate_);
  }

  timeout_ms_ = parse_int_param(sensor_info, "timeout_ms", timeout_ms_, sensor_name_);
  speed_of_sound_ = static_cast<uint32_t>(std::max(
    0,
    parse_int_param(sensor_info, "speed_of_sound", static_cast<int>(speed_of_sound_), sensor_name_)));
  ping_interval_ = static_cast<uint16_t>(std::clamp(
    parse_int_param(sensor_info, "ping_interval", ping_interval_, sensor_name_),
    0,
    static_cast<int>(std::numeric_limits<uint16_t>::max())));
  gain_setting_ = static_cast<uint8_t>(std::clamp(
    parse_int_param(sensor_info, "gain_setting", gain_setting_, sensor_name_),
    0,
    static_cast<int>(std::numeric_limits<uint8_t>::max())));
  scan_start_mm_ = static_cast<uint32_t>(std::max(
    0,
    parse_int_param(sensor_info, "scan_start", static_cast<int>(scan_start_mm_), sensor_name_)));
  scan_length_mm_ = static_cast<uint32_t>(std::max(
    0,
    parse_int_param(sensor_info, "scan_length", static_cast<int>(scan_length_mm_), sensor_name_)));
  mode_auto_ = static_cast<uint8_t>(std::clamp(
    parse_int_param(sensor_info, "mode_auto", mode_auto_, sensor_name_),
    0,
    static_cast<int>(std::numeric_limits<uint8_t>::max())));

  rx_buffer_.clear();
  last_altitude_ = 0.0;
  last_confidence_ = 0.0;
  last_scan_start_ = static_cast<double>(scan_start_mm_) / 1000.0;
  last_scan_length_ = static_cast<double>(scan_length_mm_) / 1000.0;
  last_gain_setting_ = static_cast<double>(gain_setting_);

  initialized_ = true;
  active_ = false;

  RCLCPP_INFO(
    kLogger,
    "Configured BlueBoat Ping1D altimeter sensor '%s': serial_port='%s', baudrate=%d, "
    "speed_of_sound=%u, ping_interval=%u, gain_setting=%u, scan_start=%u, scan_length=%u, "
    "mode_auto=%u",
    sensor_name_.c_str(),
    serial_port_.c_str(),
    baudrate_,
    speed_of_sound_,
    static_cast<unsigned>(ping_interval_),
    static_cast<unsigned>(gain_setting_),
    scan_start_mm_,
    scan_length_mm_,
    static_cast<unsigned>(mode_auto_));

  return true;
}

bool AltimeterBlueboatInterface::activate()
{
  if (!initialized_) {
    return false;
  }

#ifdef TARGET_RASPBERRY
  if (!open_serial()) {
    return false;
  }

  if (!initialize_ping() || !configure_ping()) {
    close_serial();
    return false;
  }
#endif

  active_ = true;
  return true;
}

bool AltimeterBlueboatInterface::deactivate()
{
  active_ = false;
  close_serial();
  return true;
}

bool AltimeterBlueboatInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  close_serial();
  rx_buffer_.clear();
  return true;
}

bool AltimeterBlueboatInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

#ifdef TARGET_RASPBERRY
  if (fd_ < 0) {
    return false;
  }

  request_distance();
#else
  last_altitude_ = 0.0;
  last_confidence_ = 0.0;
#endif

  set_state_if_exists(states, "altitude", last_altitude_);
  set_state_if_exists(states, "confidence", last_confidence_);
  set_state_if_exists(states, "scan_start", last_scan_start_);
  set_state_if_exists(states, "scan_length", last_scan_length_);
  set_state_if_exists(states, "gain_setting", last_gain_setting_);
  return true;
}

bool AltimeterBlueboatInterface::open_serial()
{
#ifdef TARGET_RASPBERRY
  speed_t speed{};
  if (!baudrate_to_speed(baudrate_, speed)) {
    RCLCPP_ERROR(kLogger, "Unsupported altimeter baudrate: %d", baudrate_);
    return false;
  }

  fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(
      kLogger,
      "Failed to open altimeter serial port %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(
      kLogger,
      "tcgetattr failed on altimeter serial port %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    close_serial();
    return false;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif

  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd_, TCIFLUSH);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      kLogger,
      "tcsetattr failed on altimeter serial port %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    close_serial();
    return false;
  }

  tcsendbreak(fd_, 0);
  usleep(1000);
  const uint8_t autobaud = 'U';
  (void)::write(fd_, &autobaud, 1);
#endif

  return true;
}

void AltimeterBlueboatInterface::close_serial()
{
#ifdef TARGET_RASPBERRY
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
#endif
}

bool AltimeterBlueboatInterface::initialize_ping()
{
#ifdef TARGET_RASPBERRY
  std::vector<uint8_t> payload;
  if (!request_message(kCommonProtocolVersion, payload)) {
    RCLCPP_ERROR(kLogger, "Failed to initialize Ping1D: no protocol_version response");
    return false;
  }

  if (!request_message(kPing1dGeneralInfo, payload)) {
    RCLCPP_ERROR(kLogger, "Failed to initialize Ping1D: no general_info response");
    return false;
  }

  return true;
#else
  return true;
#endif
}

bool AltimeterBlueboatInterface::configure_ping()
{
#ifdef TARGET_RASPBERRY
  std::vector<uint8_t> payload;

  payload.clear();
  append_u32(payload, speed_of_sound_);
  if (!send_set_message(kPing1dSetSpeedOfSound, payload)) {
    return false;
  }

  payload.clear();
  append_u16(payload, ping_interval_);
  if (!send_set_message(kPing1dSetPingInterval, payload)) {
    return false;
  }

  payload.clear();
  append_u8(payload, gain_setting_);
  if (!send_set_message(kPing1dSetGainSetting, payload)) {
    return false;
  }

  payload.clear();
  append_u32(payload, scan_start_mm_);
  append_u32(payload, scan_length_mm_);
  if (!send_set_message(kPing1dSetRange, payload)) {
    return false;
  }

  payload.clear();
  append_u8(payload, mode_auto_);
  if (!send_set_message(kPing1dSetModeAuto, payload)) {
    return false;
  }
#endif

  return true;
}

bool AltimeterBlueboatInterface::request_distance()
{
#ifdef TARGET_RASPBERRY
  std::vector<uint8_t> payload;
  if (request_message(kPing1dDistance, payload) && payload.size() >= 24) {
    last_altitude_ = static_cast<double>(read_u32(payload, 0)) / 1000.0;
    last_confidence_ = static_cast<double>(read_u16(payload, 4));
    last_scan_start_ = static_cast<double>(read_u32(payload, 12)) / 1000.0;
    last_scan_length_ = static_cast<double>(read_u32(payload, 16)) / 1000.0;
    last_gain_setting_ = static_cast<double>(read_u32(payload, 20));
    return true;
  }

  payload.clear();
  if (request_message(kPing1dDistanceSimple, payload) && payload.size() >= 5) {
    last_altitude_ = static_cast<double>(read_u32(payload, 0)) / 1000.0;
    last_confidence_ = static_cast<double>(payload[4]);
    return true;
  }

  RCLCPP_WARN_THROTTLE(
    kLogger,
    *rclcpp::Clock::make_shared(),
    2000,
    "No valid Ping1D distance response received");
  return false;
#else
  return true;
#endif
}

bool AltimeterBlueboatInterface::request_message(
  const uint16_t message_id,
  std::vector<uint8_t> & payload)
{
#ifdef TARGET_RASPBERRY
  if (!send_request(message_id)) {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    read_from_serial();
    if (take_message(message_id, payload)) {
      return true;
    }
    usleep(5000);
  }

  return false;
#else
  (void)message_id;
  (void)payload;
  return true;
#endif
}

bool AltimeterBlueboatInterface::send_request(const uint16_t message_id)
{
  return write_message(message_id, {});
}

bool AltimeterBlueboatInterface::send_set_message(
  const uint16_t message_id,
  const std::vector<uint8_t> & payload)
{
#ifdef TARGET_RASPBERRY
  if (!write_message(message_id, payload)) {
    RCLCPP_ERROR(kLogger, "Failed to send Ping1D set message %u", message_id);
    return false;
  }
#else
  (void)message_id;
  (void)payload;
#endif

  return true;
}

bool AltimeterBlueboatInterface::write_message(
  const uint16_t message_id,
  const std::vector<uint8_t> & payload)
{
#ifdef TARGET_RASPBERRY
  if (fd_ < 0 || payload.size() > std::numeric_limits<uint16_t>::max()) {
    return false;
  }

  std::vector<uint8_t> message;
  message.reserve(10 + payload.size());
  message.push_back('B');
  message.push_back('R');
  append_u16(message, static_cast<uint16_t>(payload.size()));
  append_u16(message, message_id);
  message.push_back(0);
  message.push_back(0);
  message.insert(message.end(), payload.begin(), payload.end());

  uint16_t checksum = 0;
  for (const uint8_t byte : message) {
    checksum = static_cast<uint16_t>(checksum + byte);
  }
  append_u16(message, checksum);

  std::size_t written = 0;
  while (written < message.size()) {
    const ssize_t result = ::write(
      fd_,
      message.data() + written,
      message.size() - written);

    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }

    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      usleep(1000);
      continue;
    }

    RCLCPP_WARN(
      kLogger,
      "Ping1D serial write failed: %s",
      std::strerror(errno));
    return false;
  }

  return true;
#else
  (void)message_id;
  (void)payload;
  return true;
#endif
}

bool AltimeterBlueboatInterface::read_from_serial()
{
#ifdef TARGET_RASPBERRY
  if (fd_ < 0) {
    return false;
  }

  bool read_any = false;
  char temp[256];

  while (true) {
    const ssize_t bytes_read = ::read(fd_, temp, sizeof(temp));

    if (bytes_read > 0) {
      read_any = true;
      rx_buffer_.insert(
        rx_buffer_.end(),
        reinterpret_cast<uint8_t *>(temp),
        reinterpret_cast<uint8_t *>(temp) + bytes_read);

      if (rx_buffer_.size() > 4096) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 2048);
      }

      continue;
    }

    if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        kLogger,
        *rclcpp::Clock::make_shared(),
        2000,
        "Altimeter serial read error: %s",
        std::strerror(errno));
    }

    break;
  }

  return read_any;
#else
  return false;
#endif
}

bool AltimeterBlueboatInterface::take_message(
  const uint16_t expected_message_id,
  std::vector<uint8_t> & payload)
{
  while (rx_buffer_.size() >= 10) {
    const uint8_t header_bytes[] = {'B', 'R'};
    const auto header = std::search(
      rx_buffer_.begin(),
      rx_buffer_.end(),
      std::begin(header_bytes),
      std::end(header_bytes));

    if (header == rx_buffer_.end()) {
      rx_buffer_.clear();
      return false;
    }

    if (header != rx_buffer_.begin()) {
      rx_buffer_.erase(rx_buffer_.begin(), header);
    }

    if (rx_buffer_.size() < 10) {
      return false;
    }

    const uint16_t payload_length = read_u16(rx_buffer_, 2);
    const std::size_t message_length = 8U + payload_length + 2U;
    if (rx_buffer_.size() < message_length) {
      return false;
    }

    uint16_t checksum = 0;
    for (std::size_t i = 0; i < 8U + payload_length; ++i) {
      checksum = static_cast<uint16_t>(checksum + rx_buffer_[i]);
    }

    const uint16_t received_checksum = read_u16(rx_buffer_, 8U + payload_length);
    if (checksum != received_checksum) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    const uint16_t message_id = read_u16(rx_buffer_, 4);
    if (message_id == expected_message_id) {
      payload.assign(rx_buffer_.begin() + 8, rx_buffer_.begin() + 8 + payload_length);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + message_length);
      return true;
    }

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + message_length);
  }

  return false;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::AltimeterBlueboatInterface,
  sura_hardware_interface::SensorInterfaceBase)
