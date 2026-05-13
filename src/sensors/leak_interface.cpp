#include "sura_hardware_interface/sensors/leak_interface.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

namespace sura_hardware_interface
{

bool LeakInterface::initialize(const hardware_interface::HardwareInfo &)
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

bool LeakInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool LeakInterface::deactivate()
{
  active_ = false;
  return true;
}

bool LeakInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  return true;
}

bool LeakInterface::read(double & leak)
{
  if (!initialized_ || !active_) {
    return false;
  }

#ifdef TARGET_RASPBERRY
  leak = read_leak() ? 1.0 : 0.0;
#else
  leak = 0.0;
#endif

  return true;
}

}  // namespace sura_hardware_interface