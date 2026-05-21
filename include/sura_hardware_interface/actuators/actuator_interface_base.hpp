#pragma once

#include <string>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"

namespace sura_hardware_interface
{

class ActuatorInterfaceBase
{
public:
  virtual ~ActuatorInterfaceBase() = default;

  virtual bool initialize(
    const hardware_interface::ComponentInfo & actuator_info,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & environment) = 0;

  virtual bool activate() = 0;

  virtual bool deactivate() = 0;

  virtual bool cleanup() = 0;

  virtual bool read(
    const std::unordered_map<std::string, double> & commands,
    std::unordered_map<std::string, double> & states) = 0;

  virtual bool write(
    const std::unordered_map<std::string, double> & commands,
    std::unordered_map<std::string, double> & states) = 0;
};

}  // namespace sura_hardware_interface
