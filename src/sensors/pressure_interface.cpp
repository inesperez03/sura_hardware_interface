#include "sura_hardware_interface/sensors/pressure_interface.hpp"

#ifdef TARGET_RASPBERRY

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

#endif

namespace sura_hardware_interface
{

#ifdef TARGET_RASPBERRY

namespace
{

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
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if (!read_prom()) {
      return false;
    }

    uint32_t dummy_d1 = 0;
    uint32_t dummy_d2 = 0;
    conversion(0x48, dummy_d1);
    conversion(0x58, dummy_d2);

    return true;
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

static MS5837Local sensor;
static bool initialized = false;
static double last_pressure_mbar = 1013.25;
static bool has_last_pressure = false;

}  // namespace

#endif

bool PressureInterface::read(double & pressure)
{
#ifdef TARGET_RASPBERRY
  if (!initialized) {
    if (!sensor.init(6, 0x76)) {
      if (has_last_pressure) {
        pressure = last_pressure_mbar;
        return true;
      }

      return false;
    }

    initialized = true;
  }

  if (!sensor.read()) {
    if (has_last_pressure) {
      pressure = last_pressure_mbar;
      return true;
    }

    return false;
  }

  pressure = sensor.pressure_mbar();
  last_pressure_mbar = pressure;
  has_last_pressure = true;

#else
  pressure = 0.0;
#endif

  return true;
}

}  // namespace sura_hardware_interface