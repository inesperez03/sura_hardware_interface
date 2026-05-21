#include "sura_hardware_interface/sensors/pressure_interface.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"

#ifdef TARGET_RASPBERRY

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <thread>

#endif

#include "sensor_msgs/msg/fluid_pressure.hpp"

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

#ifdef TARGET_RASPBERRY

class MS5837Local
{
public:
  bool init(int bus = 6, uint8_t address = 0x76)
  {
    bus_ = bus;
    address_ = address;

    const std::string device = "/dev/i2c-" + std::to_string(bus_);

    fd_ = ::open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
      return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    if (!write_byte(0x1E)) {
      close();
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if (!read_prom()) {
      close();
      return false;
    }

    uint32_t dummy_d1 = 0;
    uint32_t dummy_d2 = 0;
    conversion(0x48, dummy_d1);
    conversion(0x58, dummy_d2);

    return true;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool read()
  {
    uint32_t D1 = 0;
    uint32_t D2 = 0;

    if (!conversion(0x48, D1)) {
      return false;
    }

    if (!conversion(0x58, D2)) {
      return false;
    }

    if (D1 == 0 || D2 == 0) {
      return false;
    }

    const int32_t dT =
      static_cast<int32_t>(D2) - static_cast<int32_t>(C_[5]) * 256;

    int64_t SENS =
      static_cast<int64_t>(C_[1]) * 32768 +
      (static_cast<int64_t>(C_[3]) * dT) / 256;

    int64_t OFF =
      static_cast<int64_t>(C_[2]) * 65536 +
      (static_cast<int64_t>(C_[4]) * dT) / 128;

    int32_t TEMP =
      2000 + static_cast<int64_t>(dT) * C_[6] / 8388608;

    int64_t Ti = 0;
    int64_t OFFi = 0;
    int64_t SENSi = 0;

    if (TEMP < 2000) {
      Ti = 3 * static_cast<int64_t>(dT) * dT / 8589934592LL;
      OFFi = 3 * static_cast<int64_t>(TEMP - 2000) * (TEMP - 2000) / 2;
      SENSi = 5 * static_cast<int64_t>(TEMP - 2000) * (TEMP - 2000) / 8;

      if (TEMP < -1500) {
        OFFi += 7 * static_cast<int64_t>(TEMP + 1500) * (TEMP + 1500);
        SENSi += 4 * static_cast<int64_t>(TEMP + 1500) * (TEMP + 1500);
      }
    } else {
      Ti = 2 * static_cast<int64_t>(dT) * dT / 137438953472LL;
      OFFi = static_cast<int64_t>(TEMP - 2000) * (TEMP - 2000) / 16;
      SENSi = 0;
    }

    OFF -= OFFi;
    SENS -= SENSi;
    TEMP -= Ti;

    const int32_t P = static_cast<int32_t>(
      (((static_cast<int64_t>(D1) * SENS) / 2097152 - OFF) / 8192));

    pressure_mbar_ = P / 10.0;
    temperature_c_ = TEMP / 100.0;

    return pressure_mbar_ >= 500.0 && pressure_mbar_ <= 4000.0;
  }

  double pressure_mbar() const
  {
    return pressure_mbar_;
  }

  double pressure_pa() const
  {
    return pressure_mbar_ * 100.0;
  }

  double temperature_c() const
  {
    return temperature_c_;
  }

private:
  bool write_byte(uint8_t value)
  {
    if (fd_ < 0) {
      return false;
    }

    return ::write(fd_, &value, 1) == 1;
  }

  bool read_bytes(uint8_t command, uint8_t * data, size_t length)
  {
    if (fd_ < 0) {
      return false;
    }

    if (::write(fd_, &command, 1) != 1) {
      return false;
    }

    return ::read(fd_, data, length) == static_cast<ssize_t>(length);
  }

  bool read_prom()
  {
    for (uint8_t i = 0; i < 7; ++i) {
      uint8_t data[2] = {0, 0};

      if (!read_bytes(0xA0 + i * 2, data, 2)) {
        return false;
      }

      C_[i] = static_cast<uint16_t>((data[0] << 8) | data[1]);
    }

    C_[7] = 0;
    return true;
  }

  bool conversion(uint8_t command, uint32_t & result)
  {
    if (!write_byte(command)) {
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(12));

    uint8_t data[3] = {0, 0, 0};

    if (!read_bytes(0x00, data, 3)) {
      return false;
    }

    result =
      static_cast<uint32_t>(data[0]) << 16 |
      static_cast<uint32_t>(data[1]) << 8 |
      static_cast<uint32_t>(data[2]);

    return true;
  }

  int fd_{-1};
  int bus_{6};
  uint8_t address_{0x76};

  uint16_t C_[8]{};
  double pressure_mbar_{0.0};
  double temperature_c_{0.0};
};

#endif

}  // namespace

bool PressureInterface::initialize(
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
    std::cerr << "[Pressure] Unsupported environment: "
              << environment_ << std::endl;
    return false;
  }

  try {
    read_rate_hz_ = get_double_param_or(
      sensor_info,
      "read_rate_hz",
      read_rate_hz_);

    pressure_offset_pa_ = get_double_param_or(
      sensor_info,
      "pressure_offset_pa",
      pressure_offset_pa_);

    i2c_bus_ = get_int_param_or(
      sensor_info,
      "i2c_bus",
      i2c_bus_);

    i2c_address_ = static_cast<uint8_t>(
      get_int_param_or(sensor_info, "i2c_address", i2c_address_));

    stonefish_topic_ = get_param_or(
      sensor_info,
      "stonefish_topic",
      stonefish_topic_);
  } catch (const std::exception & e) {
    std::cerr << "[Pressure] Error parsing parameters for sensor '"
              << sensor_name_ << "': " << e.what() << std::endl;
    return false;
  }

#ifdef TARGET_RASPBERRY
  if (environment_ == "real") {
    pressure_sensor_ = std::make_unique<MS5837Local>();

    if (!pressure_sensor_->init(i2c_bus_, i2c_address_)) {
      std::cerr << "[Pressure] Failed to initialize MS5837 on I2C bus "
                << i2c_bus_ << " address 0x"
                << std::hex << static_cast<int>(i2c_address_)
                << std::dec << std::endl;
      pressure_sensor_.reset();
      return false;
    }
  }
#endif

  if (environment_ == "sim") {
    if (!sim_node_) {
      std::cerr << "[Pressure] sim_node is null in simulation mode"
                << std::endl;
      return false;
    }

    if (stonefish_topic_.empty()) {
      std::cerr << "[Pressure] Missing parameter 'stonefish_topic' in sim mode"
                << std::endl;
      return false;
    }

    pressure_sub_ =
      sim_node_->create_subscription<sensor_msgs::msg::FluidPressure>(
      stonefish_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::FluidPressure::SharedPtr msg)
      {
        last_pressure_pa_ = msg->fluid_pressure;
        has_last_pressure_ = true;
      });
  }

  initialized_ = true;
  active_ = false;

  return true;
}

bool PressureInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool PressureInterface::deactivate()
{
  active_ = false;
  return true;
}

bool PressureInterface::cleanup()
{
  active_ = false;
  initialized_ = false;

#ifdef TARGET_RASPBERRY
  if (pressure_sensor_) {
    pressure_sensor_->close();
    pressure_sensor_.reset();
  }
#endif

  pressure_sub_.reset();
  sim_node_.reset();

  last_pressure_pa_ = pressure_offset_pa_;
  has_last_pressure_ = false;

  return true;
}

bool PressureInterface::read(std::unordered_map<std::string, double> & states)
{
  if (!initialized_ || !active_) {
    return false;
  }

  double pressure_pa = pressure_offset_pa_;

  if (environment_ == "real") {
#ifdef TARGET_RASPBERRY
    if (pressure_sensor_ && pressure_sensor_->read()) {
      pressure_pa = pressure_sensor_->pressure_pa();
      last_pressure_pa_ = pressure_pa;
      has_last_pressure_ = true;
    } else if (has_last_pressure_) {
      pressure_pa = last_pressure_pa_;
    } else {
      return false;
    }
#else
    pressure_pa = pressure_offset_pa_;
#endif
  } else {
    if (has_last_pressure_) {
      pressure_pa = last_pressure_pa_;
    } else {
      pressure_pa = pressure_offset_pa_;
    }
  }

  set_state_if_exists(states, "fluid_pressure", pressure_pa);

  return true;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::PressureInterface,
  sura_hardware_interface::SensorInterfaceBase)
