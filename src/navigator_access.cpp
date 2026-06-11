#include "sura_hardware_interface/navigator_access.hpp"

#ifdef TARGET_RASPBERRY
#include <mutex>

#include "bindings.h"

namespace sura_hardware_interface::navigator_access
{

std::mutex & mutex()
{
  static std::mutex navigator_mutex;
  return navigator_mutex;
}

void initialize_once()
{
  static std::once_flag init_flag;
  std::call_once(
    init_flag,
    []()
    {
      std::lock_guard<std::mutex> lock(mutex());
      set_raspberry_pi_version(Raspberry::Pi4);
      set_navigator_version(NavigatorVersion::Version1);
      init();
    });
}

}  // namespace sura_hardware_interface::navigator_access
#endif
