#pragma once

#include <hardware_interface/hardware_info.hpp>

namespace sura_hardware_interface
{

class LeakInterface
{
public:
  bool initialize(const hardware_interface::HardwareInfo & info);
  bool activate();
  bool deactivate();
  bool cleanup();

  bool read(double & leak);

private:
  bool initialized_{false};
  bool active_{false};
};

}  // namespace sura_hardware_interface