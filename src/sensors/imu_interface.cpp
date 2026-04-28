#include "sura_hardware_interface/sensors/imu_interface.hpp"

#ifdef TARGET_RASPBERRY
#include "bindings.h"
#endif

namespace sura_hardware_interface
{

bool ImuInterface::initialize(const hardware_interface::HardwareInfo &)
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

bool ImuInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool ImuInterface::deactivate()
{
  active_ = false;
  return true;
}

bool ImuInterface::cleanup()
{
  active_ = false;
  initialized_ = false;
  return true;
}

bool ImuInterface::read(
  double & orientation_x,
  double & orientation_y,
  double & orientation_z,
  double & orientation_w,
  double & angular_velocity_x,
  double & angular_velocity_y,
  double & angular_velocity_z,
  double & linear_acceleration_x,
  double & linear_acceleration_y,
  double & linear_acceleration_z)
{
  if (!initialized_ || !active_) {
    return false;
  }

#ifdef TARGET_RASPBERRY
  const AxisData accel = read_accel();
  const AxisData gyro = read_gyro();
#else
#endif

  // Placeholder: la orientación real la calculará el filtro después
  orientation_x = 0.0;
  orientation_y = 0.0;
  orientation_z = 0.0;
  orientation_w = 1.0;

#ifdef TARGET_RASPBERRY
// Navigator IMU frame is SWD to converto to NED frame invert the X and Y axis
  angular_velocity_x = static_cast<double>(-gyro.x);
  angular_velocity_y = static_cast<double>(-gyro.y);
  angular_velocity_z = static_cast<double>(gyro.z);

  linear_acceleration_x = static_cast<double>(-accel.x);
  linear_acceleration_y = static_cast<double>(-accel.y);
  linear_acceleration_z = static_cast<double>(accel.z);
#else
  angular_velocity_x = 0.0;
  angular_velocity_y = 0.0;
  angular_velocity_z = 0.0;

  linear_acceleration_x = 0.0;
  linear_acceleration_y = 0.0;
  linear_acceleration_z = 0.0;
#endif

  return true;
}

}  // namespace sura_hardware_interface
