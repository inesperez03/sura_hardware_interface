#include "sura_hardware_interface/sensors/dvla50_interface.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <exception>
#include <regex>
#include <string>
#include <unordered_map>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"

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
  return it == sensor_info.parameters.end() ? default_value : it->second;
}

int get_int_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const int default_value)
{
  const auto it = sensor_info.parameters.find(name);
  return it == sensor_info.parameters.end() ? default_value : std::stoi(it->second);
}

double get_double_param_or(
  const hardware_interface::ComponentInfo & sensor_info,
  const std::string & name,
  const double default_value)
{
  const auto it = sensor_info.parameters.find(name);
  return it == sensor_info.parameters.end() ? default_value : std::stod(it->second);
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

bool regex_double(const std::string & text, const std::string & key, double & value)
{
  const std::regex pattern(
    "\"" + key + "\"\\s*:\\s*(-?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][+-]?\\d+)?)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return false;
  }

  value = std::stod(match[1].str());
  return true;
}

bool regex_bool(const std::string & text, const std::string & key, bool & value)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*(true|false)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return false;
  }

  value = match[1].str() == "true";
  return true;
}

bool regex_string(const std::string & text, const std::string & key, std::string & value)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return false;
  }

  value = match[1].str();
  return true;
}

bool is_velocity_report(const std::string & json)
{
  std::string type;
  return regex_string(json, "type", type) && type == "velocity";
}

}  // namespace

bool DvlA50Interface::initialize(
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
    return false;
  }

  try {
    read_rate_hz_ = get_double_param_or(sensor_info, "read_rate_hz", read_rate_hz_);
    timeout_s_ = get_double_param_or(sensor_info, "dvl_timeout_s", timeout_s_);
    sim_fom_ = get_double_param_or(sensor_info, "sim_fom", sim_fom_);
    dvl_ip_ = get_param_or(sensor_info, "dvl_ip", dvl_ip_);
    dvl_port_ = get_int_param_or(sensor_info, "dvl_port", dvl_port_);
    stonefish_topic_ = get_param_or(sensor_info, "stonefish_topic", stonefish_topic_);
    stonefish_altitude_topic_ =
      get_param_or(sensor_info, "stonefish_altitude_topic", stonefish_altitude_topic_);
  } catch (const std::exception &) {
    return false;
  }

  if (environment_ == "sim") {
    if (!sim_node_) {
      return false;
    }

    if (!stonefish_topic_.empty()) {
      twist_sub_ =
        sim_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        stonefish_topic_,
        rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          last_rx_time_ = std::chrono::steady_clock::now();
          last_time_ =
            static_cast<double>(msg->header.stamp.sec) +
            static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
          last_vx_ = msg->twist.linear.x;
          last_vy_ = msg->twist.linear.y;
          last_vz_ = msg->twist.linear.z;
          last_fom_ = sim_fom_;
          last_velocity_valid_ = 1.0;
          last_status_ = 0.0;
          last_format_code_ = 0.0;
        });
    }

    if (!stonefish_altitude_topic_.empty()) {
      altitude_sub_ =
        sim_node_->create_subscription<sensor_msgs::msg::Range>(
        stonefish_altitude_topic_,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          last_rx_time_ = std::chrono::steady_clock::now();
          last_altitude_ = msg->range;
          for (auto & beam : last_beams_) {
            beam.distance = msg->range;
            beam.valid = 1.0;
          }
        });
    }
  }

  initialized_ = true;
  active_ = false;
  return true;
}

bool DvlA50Interface::activate()
{
  if (!initialized_) {
    return false;
  }

  if (environment_ == "real" && !setupConnection()) {
    return false;
  }

  active_ = true;
  return true;
}

bool DvlA50Interface::deactivate()
{
  active_ = false;
  closeConnection();
  return true;
}

bool DvlA50Interface::cleanup()
{
  active_ = false;
  closeConnection();
  twist_sub_.reset();
  altitude_sub_.reset();
  rx_buffer_.clear();
  initialized_ = false;
  return true;
}

bool DvlA50Interface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  if (environment_ == "real") {
    if (socket_fd_ < 0) {
      static auto last_reconnect_attempt = std::chrono::steady_clock::time_point{};
      const auto reconnect_now = std::chrono::steady_clock::now();

      if (reconnect_now - last_reconnect_attempt > std::chrono::seconds(1)) {
        last_reconnect_attempt = reconnect_now;

        if (!setupConnection()) {
          return false;
        }
      } else {
        return false;
      }
    }

    receiveAndParseAvailable();
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(data_mutex_);

  const double age_s =
    std::chrono::duration<double>(now - last_rx_time_).count();

  if (age_s > timeout_s_) {
    resetTimedOutStates(states);
    return false;
  }

  setStates(states);
  return true;
}

bool DvlA50Interface::setupConnection()
{
  closeConnection();

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    return false;
  }

  sockaddr_in dvl_addr{};
  dvl_addr.sin_family = AF_INET;
  dvl_addr.sin_port = htons(static_cast<uint16_t>(dvl_port_));

  if (inet_pton(AF_INET, dvl_ip_.c_str(), &dvl_addr.sin_addr) <= 0) {
    closeConnection();
    return false;
  }

  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    closeConnection();
    return false;
  }

  const int result = connect(
    socket_fd_,
    reinterpret_cast<sockaddr *>(&dvl_addr),
    sizeof(dvl_addr));

  if (result < 0 && errno != EINPROGRESS) {
    closeConnection();
    return false;
  }

  fd_set write_fds;
  FD_ZERO(&write_fds);
  FD_SET(socket_fd_, &write_fds);

  timeval timeout{};
  timeout.tv_sec = static_cast<int>(timeout_s_);
  timeout.tv_usec = static_cast<int>((timeout_s_ - timeout.tv_sec) * 1000000.0);

  const int ready = select(socket_fd_ + 1, nullptr, &write_fds, nullptr, &timeout);
  if (ready <= 0) {
    closeConnection();
    return false;
  }

  int socket_error = 0;
  socklen_t socket_error_len = sizeof(socket_error);
  if (getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_len) < 0 ||
      socket_error != 0)
  {
    closeConnection();
    return false;
  }

  return true;
}

void DvlA50Interface::closeConnection()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool DvlA50Interface::receiveAndParseAvailable()
{
  if (socket_fd_ < 0) {
    return false;
  }

  std::array<char, 4096> buffer{};
  std::string latest_velocity_report;
  bool has_latest_velocity_report = false;

  while (true) {
    const ssize_t n = recv(socket_fd_, buffer.data(), buffer.size(), 0);
    if (n < 0) {
      if (errno == EWOULDBLOCK || errno == EAGAIN) {
        break;
      }

      closeConnection();
      break;
    }

    if (n == 0) {
      closeConnection();
      break;
    }

    rx_buffer_.append(buffer.data(), static_cast<size_t>(n));

    size_t line_start = 0;
    size_t newline = rx_buffer_.find('\n', line_start);
    while (newline != std::string::npos) {
      std::string line = rx_buffer_.substr(line_start, newline - line_start);

      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      if (!line.empty() && is_velocity_report(line)) {
        latest_velocity_report = line;
        has_latest_velocity_report = true;
      }

      line_start = newline + 1;
      newline = rx_buffer_.find('\n', line_start);
    }

    if (line_start > 0) {
      rx_buffer_.erase(0, line_start);
    }

    if (rx_buffer_.size() > 65536) {
      rx_buffer_.clear();
    }
  }

  if (has_latest_velocity_report) {
    return parseVelocityReport(latest_velocity_report);
  }

  return false;
}

bool DvlA50Interface::parseVelocityReport(const std::string & json)
{
  std::string type;
  if (!regex_string(json, "type", type)) {
    return false;
  }

  if (type != "velocity") {
    return false;
  }

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double fom = 0.0;
  double altitude = 0.0;
  double report_time = 0.0;
  double status = 0.0;
  bool velocity_valid = false;

  if (!regex_double(json, "vx", vx)) {
    return false;
  }
  if (!regex_double(json, "vy", vy)) {
    return false;
  }
  if (!regex_double(json, "vz", vz)) {
    return false;
  }
  if (!regex_double(json, "fom", fom)) {
    return false;
  }
  if (!regex_double(json, "altitude", altitude)) {
    return false;
  }
  if (!regex_bool(json, "velocity_valid", velocity_valid)) {
    return false;
  }

  (void)regex_double(json, "time", report_time);
  (void)regex_double(json, "status", status);

  std::string format = "json";
  (void)regex_string(json, "format", format);

  std::array<BeamData, 4> beams{};
  const std::regex transducers_pattern("\"transducers\"\\s*:\\s*\\[(.*)\\]");
  std::smatch transducers_match;
  if (std::regex_search(json, transducers_match, transducers_pattern)) {
    const std::string transducers = transducers_match[1].str();
    const std::regex object_pattern("\\{([^\\}]*)\\}");
    auto object_begin = std::sregex_iterator(
      transducers.begin(), transducers.end(), object_pattern);
    const auto object_end = std::sregex_iterator();

    size_t index = 0;
    for (auto it = object_begin; it != object_end && index < beams.size(); ++it, ++index) {
      const std::string object = (*it)[1].str();
      bool beam_valid = false;
      (void)regex_double(object, "id", beams[index].id);
      (void)regex_double(object, "velocity", beams[index].velocity);
      (void)regex_double(object, "distance", beams[index].distance);
      (void)regex_double(object, "rssi", beams[index].rssi);
      (void)regex_double(object, "nsd", beams[index].nsd);
      (void)regex_bool(object, "beam_valid", beam_valid);
      beams[index].valid = beam_valid ? 1.0 : 0.0;
    }
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  last_rx_time_ = std::chrono::steady_clock::now();
  last_time_ = report_time;
  last_vx_ = vx;
  last_vy_ = vy;
  last_vz_ = vz;
  last_fom_ = fom;
  last_altitude_ = altitude;
  last_velocity_valid_ = velocity_valid ? 1.0 : 0.0;
  last_status_ = status;
  last_format_code_ = format.empty() ? 0.0 : 0.0;
  last_beams_ = beams;

  return true;
}

void DvlA50Interface::resetTimedOutStates(std::unordered_map<std::string, double> & states)
{
  set_state_if_exists(states, "linear_velocity.x", 0.0);
  set_state_if_exists(states, "linear_velocity.y", 0.0);
  set_state_if_exists(states, "linear_velocity.z", 0.0);
  set_state_if_exists(states, "fom", 0.0);
  set_state_if_exists(states, "altitude", 0.0);
  set_state_if_exists(states, "velocity_valid", 0.0);
  set_state_if_exists(states, "status", 0.0);
  set_state_if_exists(states, "time", 0.0);
  set_state_if_exists(states, "format_code", 0.0);
}

void DvlA50Interface::setStates(std::unordered_map<std::string, double> & states)
{
  set_state_if_exists(states, "linear_velocity.x", last_vx_);
  set_state_if_exists(states, "linear_velocity.y", last_vy_);
  set_state_if_exists(states, "linear_velocity.z", last_vz_);
  set_state_if_exists(states, "fom", last_fom_);
  set_state_if_exists(states, "altitude", last_altitude_);
  set_state_if_exists(states, "velocity_valid", last_velocity_valid_);
  set_state_if_exists(states, "status", last_status_);
  set_state_if_exists(states, "time", last_time_);
  set_state_if_exists(states, "format_code", last_format_code_);

  for (size_t i = 0; i < last_beams_.size(); ++i) {
    const std::string prefix = "beam" + std::to_string(i) + ".";
    set_state_if_exists(states, prefix + "id", last_beams_[i].id);
    set_state_if_exists(states, prefix + "velocity", last_beams_[i].velocity);
    set_state_if_exists(states, prefix + "distance", last_beams_[i].distance);
    set_state_if_exists(states, prefix + "rssi", last_beams_[i].rssi);
    set_state_if_exists(states, prefix + "nsd", last_beams_[i].nsd);
    set_state_if_exists(states, prefix + "valid", last_beams_[i].valid);
  }
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::DvlA50Interface,
  sura_hardware_interface::SensorInterfaceBase)
