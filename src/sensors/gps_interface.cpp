#include "sura_hardware_interface/sensors/gps_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace sura_hardware_interface
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("GpsInterface");

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
#ifdef B460800
    case 460800:
      speed = B460800;
      return true;
#endif
#ifdef B921600
    case 921600:
      speed = B921600;
      return true;
#endif
    default:
      return false;
  }
}

template<typename T>
T read_le(const std::vector<uint8_t> & data, const std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

bool check_ubx_checksum(const std::vector<uint8_t> & packet)
{
  if (packet.size() < 8) {
    return false;
  }

  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (std::size_t i = 2; i < packet.size() - 2; ++i) {
    ck_a = static_cast<uint8_t>(ck_a + packet[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }

  return ck_a == packet[packet.size() - 2] &&
         ck_b == packet[packet.size() - 1];
}

std::array<std::string, 15> split_gga(const std::string & line)
{
  std::array<std::string, 15> fields{};
  std::stringstream ss(line);
  std::string item;
  std::size_t i = 0;

  while (std::getline(ss, item, ',') && i < fields.size()) {
    fields[i++] = item;
  }

  return fields;
}

}  // namespace

bool GpsInterface::initialize(
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
    RCLCPP_ERROR(kLogger, "[GPS] Unsupported environment: %s", environment_.c_str());
    return false;
  }

  protocol_ = get_param_or(
    sensor_info,
    hardware_info,
    "protocol",
    "gps_protocol",
    protocol_);

  serial_port_ = get_param_or(
    sensor_info,
    hardware_info,
    "serial_port",
    "gps_serial_port",
    serial_port_);

  const std::string baudrate = get_param_or(
    sensor_info,
    hardware_info,
    "baudrate",
    "gps_baudrate",
    std::to_string(baudrate_));

  try {
    baudrate_ = std::stoi(baudrate);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      kLogger,
      "Invalid baudrate '%s' for GPS sensor '%s': %s. Using default %d",
      baudrate.c_str(),
      sensor_name_.c_str(),
      ex.what(),
      baudrate_);
  }

  line_buffer_.clear();
  byte_buffer_.clear();
  last_latitude_ = 0.0;
  last_longitude_ = 0.0;
  last_altitude_ = 0.0;
  last_valid_ = 0.0;

  initialized_ = true;
  active_ = false;

  RCLCPP_INFO(
    kLogger,
    "Configured GPS sensor '%s': protocol='%s', serial_port='%s', baudrate=%d",
    sensor_name_.c_str(),
    protocol_.c_str(),
    serial_port_.c_str(),
    baudrate_);

  return true;
}

bool GpsInterface::activate()
{
  if (!initialized_) {
    return false;
  }

#ifdef TARGET_RASPBERRY
  if (!open_serial()) {
    return false;
  }
#endif

  active_ = true;
  return true;
}

bool GpsInterface::deactivate()
{
  active_ = false;
  close_serial();
  return true;
}

bool GpsInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  close_serial();
  line_buffer_.clear();
  byte_buffer_.clear();
  return true;
}

bool GpsInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

#ifdef TARGET_RASPBERRY
  if (fd_ < 0) {
    return false;
  }

  read_from_serial();

  if (protocol_ == "nmea") {
    std::string line;
    while (read_line(line)) {
      double parsed_latitude = last_latitude_;
      double parsed_longitude = last_longitude_;
      double parsed_altitude = last_altitude_;
      double parsed_valid = last_valid_;
      if (parse_gga(
          line,
          parsed_latitude,
          parsed_longitude,
          parsed_altitude,
          parsed_valid))
      {
        last_latitude_ = parsed_latitude;
        last_longitude_ = parsed_longitude;
        last_altitude_ = parsed_altitude;
        last_valid_ = parsed_valid;
      }
    }
  } else if (protocol_ == "ubx") {
    std::vector<uint8_t> packet;
    while (try_extract_ubx_packet(packet)) {
      double parsed_latitude = last_latitude_;
      double parsed_longitude = last_longitude_;
      double parsed_altitude = last_altitude_;
      double parsed_valid = last_valid_;
      if (parse_nav_pvt(
          packet,
          parsed_latitude,
          parsed_longitude,
          parsed_altitude,
          parsed_valid))
      {
        last_latitude_ = parsed_latitude;
        last_longitude_ = parsed_longitude;
        last_altitude_ = parsed_altitude;
        last_valid_ = parsed_valid;
      }
    }
  } else {
    RCLCPP_ERROR_THROTTLE(
      kLogger,
      *rclcpp::Clock::make_shared(),
      5000,
      "Unknown GPS protocol '%s'. Use 'ubx' or 'nmea'.",
      protocol_.c_str());
  }
#else
  last_latitude_ = 0.0;
  last_longitude_ = 0.0;
  last_altitude_ = 0.0;
  last_valid_ = 0.0;
#endif

  set_state_if_exists(states, "gps.latitude", last_latitude_);
  set_state_if_exists(states, "gps.longitude", last_longitude_);
  set_state_if_exists(states, "gps.altitude", last_altitude_);
  set_state_if_exists(states, "gps.valid", last_valid_);

  return true;
}

bool GpsInterface::open_serial()
{
#ifdef TARGET_RASPBERRY
  speed_t speed{};
  if (!baudrate_to_speed(baudrate_, speed)) {
    RCLCPP_ERROR(kLogger, "Unsupported GPS baudrate: %d", baudrate_);
    return false;
  }

  fd_ = open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(
      kLogger,
      "Failed to open GPS serial port %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(
      kLogger,
      "tcgetattr failed on GPS serial port %s: %s",
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
      "tcsetattr failed on GPS serial port %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    close_serial();
    return false;
  }
#endif

  return true;
}

void GpsInterface::close_serial()
{
#ifdef TARGET_RASPBERRY
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
#endif
}

bool GpsInterface::read_from_serial()
{
#ifdef TARGET_RASPBERRY
  if (fd_ < 0) {
    return false;
  }

  bool read_any = false;
  uint8_t temp[512];

  while (true) {
    const ssize_t bytes_read = ::read(fd_, temp, sizeof(temp));

    if (bytes_read > 0) {
      read_any = true;

      if (protocol_ == "nmea") {
        line_buffer_.append(
          reinterpret_cast<const char *>(temp),
          static_cast<std::size_t>(bytes_read));

        if (line_buffer_.size() > 8192) {
          line_buffer_.erase(0, line_buffer_.size() - 4096);
        }
      } else {
        byte_buffer_.insert(byte_buffer_.end(), temp, temp + bytes_read);

        if (byte_buffer_.size() > 8192) {
          byte_buffer_.erase(byte_buffer_.begin(), byte_buffer_.end() - 4096);
        }
      }

      continue;
    }

    if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        kLogger,
        *rclcpp::Clock::make_shared(),
        2000,
        "GPS serial read error: %s",
        std::strerror(errno));
    }

    break;
  }

  return read_any;
#else
  return false;
#endif
}

bool GpsInterface::read_line(std::string & line)
{
  const std::size_t newline_pos = line_buffer_.find('\n');
  if (newline_pos == std::string::npos) {
    return false;
  }

  line = line_buffer_.substr(0, newline_pos);
  line_buffer_.erase(0, newline_pos + 1);

  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }

  return true;
}

double GpsInterface::nmea_to_decimal(
  const std::string & value,
  char direction) const
{
  if (value.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double raw = std::stod(value);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - static_cast<double>(degrees * 100);

  double decimal = static_cast<double>(degrees) + minutes / 60.0;

  if (direction == 'S' || direction == 'W') {
    decimal = -decimal;
  }

  return decimal;
}

bool GpsInterface::parse_gga(
  const std::string & line,
  double & latitude,
  double & longitude,
  double & altitude,
  double & valid) const
{
  if (line.rfind("$GPGGA", 0) != 0 && line.rfind("$GNGGA", 0) != 0) {
    return false;
  }

  const auto fields = split_gga(line);

  if (fields[2].empty() || fields[3].empty() || fields[4].empty() ||
      fields[5].empty() || fields[6].empty()) {
    return false;
  }

  try {
    const int fix_quality = std::stoi(fields[6]);
    valid = fix_quality == 0 ? 0.0 : 1.0;

    latitude = nmea_to_decimal(fields[2], fields[3][0]);
    longitude = nmea_to_decimal(fields[4], fields[5][0]);
    altitude = fields[9].empty() ?
      std::numeric_limits<double>::quiet_NaN() :
      std::stod(fields[9]);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      kLogger,
      "Failed to parse NMEA GGA line: %s. Error: %s",
      line.c_str(),
      ex.what());
    return false;
  }

  return true;
}

bool GpsInterface::try_extract_ubx_packet(std::vector<uint8_t> & packet)
{
  while (byte_buffer_.size() >= 2) {
    while (byte_buffer_.size() >= 2 &&
      !(byte_buffer_[0] == 0xB5 && byte_buffer_[1] == 0x62))
    {
      byte_buffer_.erase(byte_buffer_.begin());
    }

    if (byte_buffer_.size() < 6) {
      return false;
    }

    const uint16_t length =
      static_cast<uint16_t>(byte_buffer_[4]) |
      (static_cast<uint16_t>(byte_buffer_[5]) << 8);

    if (length > 1024) {
      byte_buffer_.erase(byte_buffer_.begin());
      continue;
    }

    const std::size_t total_size = 6u + static_cast<std::size_t>(length) + 2u;
    if (byte_buffer_.size() < total_size) {
      return false;
    }

    packet.assign(byte_buffer_.begin(), byte_buffer_.begin() + total_size);

    if (!check_ubx_checksum(packet)) {
      byte_buffer_.erase(byte_buffer_.begin());
      continue;
    }

    byte_buffer_.erase(byte_buffer_.begin(), byte_buffer_.begin() + total_size);
    return true;
  }

  return false;
}

bool GpsInterface::parse_nav_pvt(
  const std::vector<uint8_t> & packet,
  double & latitude,
  double & longitude,
  double & altitude,
  double & valid) const
{
  if (packet.size() < 8) {
    return false;
  }

  const uint8_t msg_class = packet[2];
  const uint8_t msg_id = packet[3];

  const uint16_t length =
    static_cast<uint16_t>(packet[4]) |
    (static_cast<uint16_t>(packet[5]) << 8);

  if (msg_class != 0x01 || msg_id != 0x07 || length < 92) {
    return false;
  }

  const std::size_t total_size = 6u + static_cast<std::size_t>(length) + 2u;
  if (packet.size() < total_size) {
    return false;
  }

  const std::size_t payload_offset = 6;
  const uint8_t fix_type = packet[payload_offset + 20];
  const uint8_t flags = packet[payload_offset + 21];

  const bool gnss_fix_ok = (flags & 0x01) != 0;
  const bool has_position_fix = gnss_fix_ok && fix_type >= 2;

  const int32_t lon_raw = read_le<int32_t>(packet, payload_offset + 24);
  const int32_t lat_raw = read_le<int32_t>(packet, payload_offset + 28);
  const int32_t h_msl_raw = read_le<int32_t>(packet, payload_offset + 36);

  latitude = static_cast<double>(lat_raw) * 1e-7;
  longitude = static_cast<double>(lon_raw) * 1e-7;
  altitude = static_cast<double>(h_msl_raw) / 1000.0;
  valid = has_position_fix ? 1.0 : 0.0;

  return true;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::GpsInterface,
  sura_hardware_interface::SensorInterfaceBase)
