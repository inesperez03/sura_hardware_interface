#include "sura_hardware_interface/sensors/dvl75_interface.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>
#include <sstream>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sura_hardware_interface
{

bool DvlInterface::initialize(const hardware_interface::HardwareInfo & info)
{
  if (initialized_) {
    return true;
  }

  if (info.hardware_parameters.count("dvl_ip") > 0) {
    dvl_ip_ = info.hardware_parameters.at("dvl_ip");
  }

  if (info.hardware_parameters.count("dvl_listen_port") > 0) {
    listen_port_ = std::stoi(info.hardware_parameters.at("dvl_listen_port"));
  }

  if (info.hardware_parameters.count("dvl_command_port") > 0) {
    command_port_ = std::stoi(info.hardware_parameters.at("dvl_command_port"));
  }

  if (info.hardware_parameters.count("dvl_min_confidence") > 0) {
    min_confidence_ = std::stod(info.hardware_parameters.at("dvl_min_confidence"));
  }

  if (info.hardware_parameters.count("dvl_timeout_s") > 0) {
    timeout_s_ = std::stod(info.hardware_parameters.at("dvl_timeout_s"));
  }

  if (!setupSocket()) {
    return false;
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

  configureDvl();

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
  closeSocket();
  return true;
}

bool DvlInterface::read(
  double & linear_velocity_x,
  double & linear_velocity_y,
  double & linear_velocity_z,
  double & angular_velocity_x,
  double & angular_velocity_y,
  double & angular_velocity_z,
  double & distance_z,
  double & confidence,
  double & gps_latitude,
  double & gps_longitude,
  double & gps_altitude,
  double & gps_valid)
{
  if (!initialized_ || !active_) {
    return false;
  }

  while (receiveAndParseOnce()) {
  }

  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(data_mutex_);

  const double age_s =
    std::chrono::duration<double>(now - last_rx_time_).count();

  if (age_s > timeout_s_) {
    linear_velocity_x = 0.0;
    linear_velocity_y = 0.0;
    linear_velocity_z = 0.0;

    angular_velocity_x = 0.0;
    angular_velocity_y = 0.0;
    angular_velocity_z = 0.0;

    distance_z = 0.0;
    confidence = 0.0;

    gps_latitude = 0.0;
    gps_longitude = 0.0;
    gps_altitude = 0.0;
    gps_valid = 0.0;

    return false;
  }

  linear_velocity_x = last_vx_;
  linear_velocity_y = last_vy_;
  linear_velocity_z = last_vz_;

  angular_velocity_x = 0.0;
  angular_velocity_y = 0.0;
  angular_velocity_z = 0.0;

  distance_z = last_distance_z_;
  confidence = last_confidence_;

  gps_latitude = last_gps_latitude_;
  gps_longitude = last_gps_longitude_;
  gps_altitude = last_gps_altitude_;
  gps_valid = last_gps_valid_;

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
  sendCommand("SEND-DVPDL ON");
  sendCommand("SEND-DVEXT ON");

  sendCommand("RETWEET-GPS ON");

  sendCommand("SEND-GPRMC OFF");

  sendCommand("RETWEET-IMU OFF");

  // Montaje hacia abajo.
  sendCommand("SET-SENSOR-ORIENTATION 0,0,0");
}

void DvlInterface::sendCommand(const std::string & command)
{
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

      double latitude = 0.0;
      double longitude = 0.0;
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

  // $DVPDL,tu,dtu,adr,adp,ady,pdx,pdy,pdz,c
  if (parts.size() < 10 || parts[0] != "$DVPDL") {
    return false;
  }

  try {
    const double dt_us = std::stod(parts[2]);
    const double dt_s = dt_us / 1'000'000.0;

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

  try {
    latitude = nmeaCoordinateToDecimalDegrees(parts[3], parts[4]);
    longitude = nmeaCoordinateToDecimalDegrees(parts[5], parts[6]);

    // RMC no trae altitud. Si luego quieres altitud real, parsea GGA.
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
  const std::string & hemisphere)
{
  const double raw = std::stod(value);

  const double degrees = std::floor(raw / 100.0);
  const double minutes = raw - degrees * 100.0;

  double decimal = degrees + minutes / 60.0;

  if (hemisphere == "S" || hemisphere == "W") {
    decimal = -decimal;
  }

  return decimal;
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