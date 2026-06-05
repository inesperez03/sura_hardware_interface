#include "sura_hardware_interface/sensors/dvl75_interface.hpp"

#include <array>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace sura_hardware_interface
{

namespace
{

std::string get_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const std::string & default_value)
{
  const auto it = sensor_info.parameters.find(name);

  if (it == sensor_info.parameters.end()) {
    return default_value;
  }

  return it->second;
}

int get_int_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const int default_value)
{
  const auto it = sensor_info.parameters.find(name);

  if (it == sensor_info.parameters.end()) {
    return default_value;
  }

  return std::stoi(it->second);
}

double get_double_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const double default_value)
{
  const auto it = sensor_info.parameters.find(name);

  if (it == sensor_info.parameters.end()) {
    return default_value;
  }

  return std::stod(it->second);
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

}  // namespace

bool DvlInterface::initialize(
  const hardware_interface::ComponentInfo & sensor_info,
  const hardware_interface::HardwareInfo &,
  const std::string & environment,
  const rclcpp::Node::SharedPtr & sim_node)
{
  if (initialized_) {
    return true;
  }

  sensor_name_ = sensor_info.name;
  environment_ = environment;
  sim_node_ = sim_node;

  if (environment_ != "real" && environment_ != "sim") {
    std::cerr << "[DVL] Unsupported environment: " << environment_ << std::endl;
    return false;
  }

  try {
    read_rate_hz_ = get_double_param_or(
      sensor_info,
      "read_rate_hz",
      read_rate_hz_);

    sim_dvl_confidence_ = get_double_param_or(
      sensor_info,
      "sim_dvl_confidence",
      sim_dvl_confidence_);

    dvl_ip_ = get_param_or(
      sensor_info,
      "dvl_ip",
      dvl_ip_);

    listen_port_ = get_int_param_or(
      sensor_info,
      "dvl_listen_port",
      listen_port_);

    command_port_ = get_int_param_or(
      sensor_info,
      "dvl_command_port",
      command_port_);

    min_confidence_ = get_double_param_or(
      sensor_info,
      "dvl_min_confidence",
      min_confidence_);

    timeout_s_ = get_double_param_or(
      sensor_info,
      "dvl_timeout_s",
      timeout_s_);

    stonefish_topic_ = get_param_or(
      sensor_info,
      "stonefish_topic",
      stonefish_topic_);

    stonefish_altitude_topic_ = get_param_or(
      sensor_info,
      "stonefish_altitude_topic",
      stonefish_altitude_topic_);

    stonefish_gps_topic_ = get_param_or(
      sensor_info,
      "stonefish_gps_topic",
      stonefish_gps_topic_);
  } catch (const std::exception & e) {
    std::cerr << "[DVL] Error parsing parameters for sensor '"
              << sensor_name_ << "': " << e.what() << std::endl;
    return false;
  }

  if (environment_ == "real") {
    if (!setupSocket()) {
      return false;
    }
  }

  if (environment_ == "sim") {
    if (!sim_node_) {
      std::cerr << "[DVL] sim_node is null in simulation mode" << std::endl;
      return false;
    }

    if (stonefish_topic_.empty()) {
      std::cerr << "[DVL] Missing parameter 'stonefish_topic' in sim mode"
                << std::endl;
      return false;
    }

    if (stonefish_altitude_topic_.empty()) {
      std::cerr << "[DVL] Missing parameter 'stonefish_altitude_topic' in sim mode"
                << std::endl;
      return false;
    }

    if (stonefish_gps_topic_.empty()) {
      std::cerr << "[DVL] Missing parameter 'stonefish_gps_topic' in sim mode"
                << std::endl;
      return false;
    }

    dvl_sub_ =
      sim_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      stonefish_topic_,
      rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        last_rx_time_ = std::chrono::steady_clock::now();

        last_vx_ = msg->twist.linear.x;
        last_vy_ = msg->twist.linear.y;
        last_vz_ = msg->twist.linear.z;

        last_confidence_ = sim_dvl_confidence_;
      });

    altitude_sub_ =
      sim_node_->create_subscription<sensor_msgs::msg::Range>(
      stonefish_altitude_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Range::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        last_rx_time_ = std::chrono::steady_clock::now();

        last_distance_z_ = msg->range;
        last_distance_z_valid_ = true;
      });

    gps_sub_ =
      sim_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      stonefish_gps_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        last_rx_time_ = std::chrono::steady_clock::now();

        last_gps_latitude_ = msg->latitude;
        last_gps_longitude_ = msg->longitude;
        last_gps_altitude_ = msg->altitude;

        if (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
          last_gps_valid_ = 1.0;
        } else {
          last_gps_valid_ = 0.0;
        }
      });
  }

  initialized_ = true;
  active_ = false;

  return true;
}

bool DvlInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  if (environment_ == "real") {
    configureDvl();
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    last_rx_time_ = std::chrono::steady_clock::now();

    last_vx_ = 0.0;
    last_vy_ = 0.0;
    last_vz_ = 0.0;

    last_confidence_ = 0.0;

    last_distance_z_ = 0.0;
    last_distance_z_valid_ = false;

    last_gps_latitude_ = 0.0;
    last_gps_longitude_ = 0.0;
    last_gps_altitude_ = 0.0;
    last_gps_valid_ = 0.0;
  }

  active_ = true;
  return true;
}

bool DvlInterface::deactivate()
{
  active_ = false;
  return true;
}

bool DvlInterface::cleanup()
{
  active_ = false;
  initialized_ = false;

  if (environment_ == "real") {
    closeSocket();
  }

  dvl_sub_.reset();
  altitude_sub_.reset();
  gps_sub_.reset();
  sim_node_.reset();

  return true;
}

bool DvlInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  if (environment_ == "real") {
    while (receiveAndParseOnce()) {
    }
  }

  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(data_mutex_);

  const double age_s =
    std::chrono::duration<double>(now - last_rx_time_).count();

  if (age_s > timeout_s_) {
    set_state_if_exists(states, "linear_velocity.x", 0.0);
    set_state_if_exists(states, "linear_velocity.y", 0.0);
    set_state_if_exists(states, "linear_velocity.z", 0.0);

    set_state_if_exists(states, "angular_velocity.x", 0.0);
    set_state_if_exists(states, "angular_velocity.y", 0.0);
    set_state_if_exists(states, "angular_velocity.z", 0.0);

    set_state_if_exists(states, "distance_z", 0.0);
    set_state_if_exists(states, "confidence", 0.0);

    set_state_if_exists(states, "gps.latitude", 0.0);
    set_state_if_exists(states, "gps.longitude", 0.0);
    set_state_if_exists(states, "gps.altitude", 0.0);
    set_state_if_exists(states, "gps.valid", 0.0);

    return false;
  }

  set_state_if_exists(states, "linear_velocity.x", last_vx_);
  set_state_if_exists(states, "linear_velocity.y", last_vy_);
  set_state_if_exists(states, "linear_velocity.z", last_vz_);

  set_state_if_exists(states, "angular_velocity.x", 0.0);
  set_state_if_exists(states, "angular_velocity.y", 0.0);
  set_state_if_exists(states, "angular_velocity.z", 0.0);

  set_state_if_exists(states, "distance_z", last_distance_z_);
  set_state_if_exists(states, "confidence", last_confidence_);

  set_state_if_exists(states, "gps.latitude", last_gps_latitude_);
  set_state_if_exists(states, "gps.longitude", last_gps_longitude_);
  set_state_if_exists(states, "gps.altitude", last_gps_altitude_);
  set_state_if_exists(states, "gps.valid", last_gps_valid_);

  return true;
}

bool DvlInterface::setupSocket()
{
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (socket_fd_ < 0) {
    std::cerr << "[DVL] Error creating UDP socket: "
              << std::strerror(errno) << std::endl;
    return false;
  }

  int reuse = 1;

  if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    std::cerr << "[DVL] Error setting SO_REUSEADDR: "
              << std::strerror(errno) << std::endl;
    closeSocket();
    return false;
  }

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(static_cast<uint16_t>(listen_port_));

  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
    std::cerr << "[DVL] Error binding UDP socket on port "
              << listen_port_ << ": " << std::strerror(errno) << std::endl;
    closeSocket();
    return false;
  }

  const int flags = fcntl(socket_fd_, F_GETFL, 0);

  if (flags < 0) {
    std::cerr << "[DVL] Error getting socket flags: "
              << std::strerror(errno) << std::endl;
    closeSocket();
    return false;
  }

  if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    std::cerr << "[DVL] Error setting non-blocking socket: "
              << std::strerror(errno) << std::endl;
    closeSocket();
    return false;
  }

  std::cout << "[DVL] Listening UDP on port " << listen_port_ << std::endl;
  return true;
}

void DvlInterface::closeSocket()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

void DvlInterface::configureDvl()
{
  if (environment_ != "real") {
    return;
  }

  sendCommand("SEND-DVPDL ON");
  sendCommand("SEND-DVEXT ON");

  sendCommand("RETWEET-GPS ON");

  sendCommand("SEND-GPRMC OFF");

  sendCommand("RETWEET-IMU OFF");

  sendCommand("SET-SENSOR-ORIENTATION 0,0,0");
}

void DvlInterface::sendCommand(const std::string & command)
{
  if (environment_ != "real") {
    return;
  }

  const int cmd_socket = socket(AF_INET, SOCK_DGRAM, 0);

  if (cmd_socket < 0) {
    std::cerr << "[DVL] Error creating command socket: "
              << std::strerror(errno) << std::endl;
    return;
  }

  sockaddr_in dvl_addr{};
  dvl_addr.sin_family = AF_INET;
  dvl_addr.sin_port = htons(static_cast<uint16_t>(command_port_));

  if (inet_pton(AF_INET, dvl_ip_.c_str(), &dvl_addr.sin_addr) <= 0) {
    std::cerr << "[DVL] Invalid DVL IP: " << dvl_ip_ << std::endl;
    close(cmd_socket);
    return;
  }

  const std::string message = command + "\r\n";

  const ssize_t sent = sendto(
    cmd_socket,
    message.c_str(),
    message.size(),
    0,
    reinterpret_cast<sockaddr *>(&dvl_addr),
    sizeof(dvl_addr));

  if (sent < 0) {
    std::cerr << "[DVL] Error sending command '" << command
              << "': " << std::strerror(errno) << std::endl;
  } else {
    std::cout << "[DVL TX] " << command << std::endl;
  }

  close(cmd_socket);
}

bool DvlInterface::receiveAndParseOnce()
{
  if (socket_fd_ < 0) {
    return false;
  }

  std::array<char, 4096> buffer{};

  sockaddr_in sender_addr{};
  socklen_t sender_len = sizeof(sender_addr);

  const ssize_t n = recvfrom(
    socket_fd_,
    buffer.data(),
    buffer.size() - 1,
    0,
    reinterpret_cast<sockaddr *>(&sender_addr),
    &sender_len);

  if (n < 0) {
    if (errno == EWOULDBLOCK || errno == EAGAIN) {
      return false;
    }

    std::cerr << "[DVL] recvfrom error: "
              << std::strerror(errno) << std::endl;
    return false;
  }

  if (n == 0) {
    return false;
  }

  buffer[static_cast<size_t>(n)] = '\0';

  std::stringstream ss(std::string(buffer.data()));
  std::string line;

  bool parsed_any = false;

  while (std::getline(ss, line)) {
    if (line.empty()) {
      continue;
    }

    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    if (line.rfind("$DVPDL", 0) == 0) {
      double vx = 0.0;
      double vy = 0.0;
      double vz = 0.0;
      double confidence = 0.0;

      if (!parseDvpdL(line, vx, vy, vz, confidence)) {
        continue;
      }

      if (confidence < min_confidence_) {
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        last_rx_time_ = std::chrono::steady_clock::now();

        last_vx_ = vx;
        last_vy_ = vy;
        last_vz_ = vz;
        last_confidence_ = confidence;
      }

      parsed_any = true;
      continue;
    }

    if (line.rfind("$DVEXT", 0) == 0) {
      double distance_z = 0.0;
      bool valid_lock = false;

      if (!parseDvextDistanceZ(line, distance_z, valid_lock)) {
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        last_rx_time_ = std::chrono::steady_clock::now();

        last_distance_z_ = distance_z;
        last_distance_z_valid_ = valid_lock;
      }

      parsed_any = true;
      continue;
    }

    if (line.rfind("GPS:", 0) == 0) {
      const std::string nmea_line = line.substr(4);

      double latitude = last_gps_latitude_;
      double longitude = last_gps_longitude_;
      double altitude = 0.0;
      double valid = 0.0;

      if (!parseGprmc(nmea_line, latitude, longitude, altitude, valid)) {
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        last_rx_time_ = std::chrono::steady_clock::now();

        last_gps_latitude_ = latitude;
        last_gps_longitude_ = longitude;
        last_gps_altitude_ = altitude;
        last_gps_valid_ = valid;
      }

      parsed_any = true;
      continue;
    }
  }

  return parsed_any;
}

bool DvlInterface::parseDvpdL(
  const std::string & line,
  double & vx,
  double & vy,
  double & vz,
  double & confidence)
{
  const std::string clean = removeChecksum(line);

  std::vector<std::string> parts;
  std::stringstream ss(clean);
  std::string item;

  while (std::getline(ss, item, ',')) {
    parts.push_back(item);
  }

  if (parts.size() < 10 || parts[0] != "$DVPDL") {
    return false;
  }

  try {
    const double dt_us = std::stod(parts[2]);
    const double dt_s = dt_us / 1000000.0;

    if (dt_s <= 0.0) {
      return false;
    }

    const double pdx = std::stod(parts[6]);
    const double pdy = std::stod(parts[7]);
    const double pdz = std::stod(parts[8]);

    confidence = std::stod(parts[9]);

    vx = pdx / dt_s;
    vy = pdy / dt_s;
    vz = pdz / dt_s;

    return true;
  } catch (const std::exception & e) {
    std::cerr << "[DVL] Failed parsing DVPDL: "
              << e.what() << " | line: " << line << std::endl;
    return false;
  }
}

bool DvlInterface::parseDvextDistanceZ(
  const std::string & line,
  double & distance_z,
  bool & valid_lock)
{
  const std::string clean = removeChecksum(line);

  std::vector<std::string> parts;
  std::stringstream ss(clean);
  std::string item;

  while (std::getline(ss, item, ',')) {
    parts.push_back(item);
  }

  if (parts.empty() || parts[0] != "$DVEXT") {
    return false;
  }

  valid_lock = parts.size() > 1 && parts[1] == "T";

  for (auto it = parts.rbegin(); it != parts.rend(); ++it) {
    if (it->empty()) {
      continue;
    }

    try {
      distance_z = std::stod(*it);
      return true;
    } catch (const std::exception &) {
      continue;
    }
  }

  distance_z = 0.0;
  valid_lock = false;
  return true;
}

bool DvlInterface::parseGprmc(
  const std::string & line,
  double & latitude,
  double & longitude,
  double & altitude,
  double & valid)
{
  if (!hasValidNmeaChecksum(line)) {
    valid = 0.0;
    return false;
  }

  const std::string clean = removeChecksum(line);

  std::vector<std::string> parts;
  std::stringstream ss(clean);
  std::string item;

  while (std::getline(ss, item, ',')) {
    parts.push_back(item);
  }

  if (parts.size() < 7) {
    return false;
  }

  if (
    parts[0] != "$GPRMC" &&
    parts[0] != "$GNRMC" &&
    parts[0] != "$GARMC" &&
    parts[0] != "$GLRMC")
  {
    return false;
  }

  const std::string & status = parts[2];

  if (status != "A") {
    valid = 0.0;
    return true;
  }

  if (parts[3].empty() || parts[4].empty() || parts[5].empty() || parts[6].empty()) {
    valid = 0.0;
    return true;
  }

  if (
    (parts[4] != "N" && parts[4] != "S") ||
    (parts[6] != "E" && parts[6] != "W"))
  {
    valid = 0.0;
    return true;
  }

  try {
    latitude = nmeaCoordinateToDecimalDegrees(parts[3], parts[4], 90.0);
    longitude = nmeaCoordinateToDecimalDegrees(parts[5], parts[6], 180.0);

    altitude = 0.0;

    valid = 1.0;
    return true;
  } catch (const std::exception & e) {
    std::cerr << "[DVL] Failed parsing GPS RMC: "
              << e.what() << " | line: " << line << std::endl;

    latitude = 0.0;
    longitude = 0.0;
    altitude = 0.0;
    valid = 0.0;

    return true;
  }
}

double DvlInterface::nmeaCoordinateToDecimalDegrees(
  const std::string & value,
  const std::string & hemisphere,
  const double max_degrees)
{
  if (
    hemisphere != "N" && hemisphere != "S" &&
    hemisphere != "E" && hemisphere != "W")
  {
    throw std::invalid_argument("invalid NMEA hemisphere");
  }

  const double raw = std::stod(value);

  const double degrees = std::floor(raw / 100.0);
  const double minutes = raw - degrees * 100.0;

  if (
    !std::isfinite(raw) ||
    degrees < 0.0 ||
    degrees > max_degrees ||
    minutes < 0.0 ||
    minutes >= 60.0)
  {
    throw std::out_of_range("invalid NMEA coordinate");
  }

  double decimal = degrees + minutes / 60.0;

  if (decimal > max_degrees) {
    throw std::out_of_range("NMEA coordinate exceeds range");
  }

  if (hemisphere == "S" || hemisphere == "W") {
    decimal = -decimal;
  }

  return decimal;
}

bool DvlInterface::hasValidNmeaChecksum(const std::string & line)
{
  const auto start = line.find('$');
  const auto checksum_pos = line.find('*');

  if (start == std::string::npos || checksum_pos == std::string::npos) {
    return true;
  }

  if (checksum_pos <= start + 1 || checksum_pos + 2 >= line.size()) {
    return false;
  }

  const char high = line[checksum_pos + 1];
  const char low = line[checksum_pos + 2];

  if (
    !std::isxdigit(static_cast<unsigned char>(high)) ||
    !std::isxdigit(static_cast<unsigned char>(low)))
  {
    return false;
  }

  unsigned int calculated = 0;

  for (auto index = start + 1; index < checksum_pos; ++index) {
    calculated ^= static_cast<unsigned char>(line[index]);
  }

  const auto hex_value = [](const char c) -> unsigned int {
      if (c >= '0' && c <= '9') {
        return static_cast<unsigned int>(c - '0');
      }

      const char upper = static_cast<char>(
        std::toupper(static_cast<unsigned char>(c)));

      return static_cast<unsigned int>(upper - 'A' + 10);
    };

  const unsigned int expected = (hex_value(high) << 4) | hex_value(low);

  return calculated == expected;
}

std::string DvlInterface::removeChecksum(const std::string & line)
{
  const auto pos = line.find('*');

  if (pos == std::string::npos) {
    return line;
  }

  return line.substr(0, pos);
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::DvlInterface,
  sura_hardware_interface::SensorInterfaceBase)
