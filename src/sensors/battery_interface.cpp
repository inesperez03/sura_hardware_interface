#include "sura_hardware_interface/sensors/battery_interface.hpp"

#include <limits>

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

namespace sura_hardware_interface
{

namespace
{

#ifdef TARGET_RASPBERRY
constexpr AdcChannel kVoltageChannel = AdcChannel::Ch3;
constexpr AdcChannel kCurrentChannel = AdcChannel::Ch2;

constexpr double kPsmVoltageMultiplier = 11.0;
constexpr double kPsmCurrentPerVolt = 37.8788;
constexpr double kPsmCurrentOffset = 0.330;
#endif

}  // namespace

bool BatteryInterface::initialize(const hardware_interface::HardwareInfo &)
{
  if (initialized_) {
    return true;
  }

#ifdef TARGET_RASPBERRY
  init();
#endif

  initialized_ = true;
  active_ = false;
  return true;
}

bool BatteryInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool BatteryInterface::deactivate()
{
  active_ = false;
  return true;
}

bool BatteryInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  return true;
}

bool BatteryInterface::read(
  double & voltage,
  double & current,
  double & present)
{
  if (!initialized_ || !active_) {
    return false;
  }

#ifdef TARGET_RASPBERRY
  const double voltage_adc = static_cast<double>(read_adc(kVoltageChannel));
  const double current_adc = static_cast<double>(read_adc(kCurrentChannel));

  voltage = voltage_adc * kPsmVoltageMultiplier;
  current = (current_adc - kPsmCurrentOffset) * kPsmCurrentPerVolt;

  if (current < 0.0) {
    current = 0.0;
  }

  present = 1.0;
#else
  voltage = std::numeric_limits<double>::quiet_NaN();
  current = std::numeric_limits<double>::quiet_NaN();
  present = 0.0;
#endif

  return true;
}

}  // namespace sura_hardware_interface
