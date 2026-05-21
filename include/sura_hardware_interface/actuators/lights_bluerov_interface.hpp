#pragma once

#include <string>
#include <unordered_map>

#include <hardware_interface/hardware_info.hpp>

#include "sura_hardware_interface/actuators/actuator_interface_base.hpp"

namespace sura_hardware_interface
{

class LightsBluerovInterface : public ActuatorInterfaceBase
{
public:
  bool initialize(
    const hardware_interface::ComponentInfo & actuator_info,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & environment) override;
  bool activate() override;
  bool deactivate() override;
  bool cleanup() override;

  bool read(
    const std::unordered_map<std::string, double> & commands,
    std::unordered_map<std::string, double> & states) override;

  bool write(
    const std::unordered_map<std::string, double> & commands,
    std::unordered_map<std::string, double> & states) override;

private:
  bool write_command(double pwm_us);

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
