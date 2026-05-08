#pragma once

#include <hardware_interface/hardware_info.hpp>

namespace sura_hardware_interface
{

class BatteryInterface
{
public:
  bool initialize(const hardware_interface::HardwareInfo & info);
  bool activate();
  bool deactivate();
  bool cleanup();

  bool read(
    double & voltage,
    double & current,
    double & present);

private:
  bool initialized_{false};
  bool active_{false};
};

}  // namespace sura_hardware_interface
