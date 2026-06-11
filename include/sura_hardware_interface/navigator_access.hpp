#pragma once

#include <mutex>
#include <utility>

namespace sura_hardware_interface::navigator_access
{

#ifdef TARGET_RASPBERRY
void initialize_once();
std::mutex & mutex();

template<typename Callable>
decltype(auto) call(Callable && callable)
{
  std::lock_guard<std::mutex> lock(mutex());
  return std::forward<Callable>(callable)();
}
#else
inline void initialize_once() {}

template<typename Callable>
decltype(auto) call(Callable && callable)
{
  return std::forward<Callable>(callable)();
}
#endif

}  // namespace sura_hardware_interface::navigator_access
