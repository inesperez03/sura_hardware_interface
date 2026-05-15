#pragma once

#include <string>

#include <hardware_interface/hardware_info.hpp>

namespace sura_hardware_interface
{

class LightsBluerovInterface
{
public:
  bool initialize(
    const hardware_interface::HardwareInfo & info,
    const char * environment,
    int lights_channel);
  bool activate();
  bool deactivate();
  bool cleanup();

  bool write(double pwm_us);

private:
  bool write_pwm_us(double pwm_us);

  std::string environment_{"real"};
  int lights_channel_{11};
  bool initialized_{false};
  bool active_{false};
  bool navigator_initialized_{false};
  bool pwm_enabled_{false};
  double pwm_frequency_hz_{50.0};
};

}  // namespace sura_hardware_interface
