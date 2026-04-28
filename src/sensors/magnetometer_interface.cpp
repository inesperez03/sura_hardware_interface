#include "sura_hardware_interface/sensors/magnetometer_interface.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

namespace sura_hardware_interface
{

bool MagnetometerInterface::read(
  double & magnetic_field_x,
  double & magnetic_field_y,
  double & magnetic_field_z)
{
#ifdef TARGET_RASPBERRY
  const AxisData mag = read_mag();
  // ROTATION_YAW_270: x' = y, y' = -x, z' = z
  magnetic_field_x = static_cast<double>(mag.y);
  magnetic_field_y = -static_cast<double>(mag.x);
  magnetic_field_z = static_cast<double>(mag.z);
#else
  magnetic_field_x = 0.0;
  magnetic_field_y = 0.0;
  magnetic_field_z = 0.0;
#endif

  return true;
}

}  // namespace sura_hardware_interface
