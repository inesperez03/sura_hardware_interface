#ifndef SURA_HARDWARE_INTERFACE_SENSORS_PRESSURE_INTERFACE_HPP_
#define SURA_HARDWARE_INTERFACE_SENSORS_PRESSURE_INTERFACE_HPP_

namespace sura_hardware_interface
{

class PressureInterface
{
public:
  PressureInterface() = default;
  ~PressureInterface() = default;

  bool read(double & pressure);
};

}  // namespace sura_hardware_interface

#endif