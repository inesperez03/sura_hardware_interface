#pragma once

#include <string>

#include <hardware_interface/hardware_info.hpp>

namespace sura_hardware_interface
{

class LightInterface
{
public:
  bool initialize(
    const hardware_interface::HardwareInfo & info,
    const char * environment,
    int status_light_channel);
  bool activate();
  bool deactivate();
  bool cleanup();

  bool write(bool enabled);

private:
  std::string environment_{"real"};
  int status_light_channel_{1};
  bool initialized_{false};
  bool active_{false};
  bool navigator_initialized_{false};
  bool pwm_enabled_{false};
  double pwm_frequency_hz_{50.0};
};

}  // namespace sura_hardware_interface
