cmake_minimum_required(VERSION 3.8)
project(sura_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stonefish_ros2 REQUIRED)

include(ExternalProject)

set(ZIP_URL "https://github.com/bluerobotics/navigator-lib/releases/latest/download/cpp.zip")
set(NAVIGATOR_DIR "${CMAKE_BINARY_DIR}/navigator_zip_contents")

# Detect architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7")
  set(NAVIGATOR_ARCH "armv7-unknown-linux-gnueabihf")
  set(TARGET_RASPBERRY TRUE)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(NAVIGATOR_ARCH "aarch64-unknown-linux-gnu")
  set(TARGET_RASPBERRY TRUE)
else()
  message(WARNING "Building sura_hardware_interface in desktop mode on unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  set(TARGET_RASPBERRY FALSE)
endif()

# Only download and link navigator-lib on Raspberry-compatible targets
if(TARGET_RASPBERRY)
  ExternalProject_Add(
    navigator_lib
    URL ${ZIP_URL}
    DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/navigator_download"
    SOURCE_DIR ${NAVIGATOR_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    TEST_COMMAND ""
  )

  set(NAVIGATOR_LIB "${NAVIGATOR_DIR}/${NAVIGATOR_ARCH}/libbluerobotics_navigator.so")
endif()

add_library(${PROJECT_NAME} SHARED
  src/thrusters/thrusters_system.cpp
  src/thrusters/thruster_mapper.cpp
  src/sensors/sensors_system.cpp
  src/sensors/imu_interface.cpp
  src/sensors/magnetometer_interface.cpp
  src/sensors/pressure_interface.cpp
  src/sensors/battery_interface.cpp
  src/sensors/dvl75_interface.cpp
)

if(TARGET_RASPBERRY)
  add_dependencies(${PROJECT_NAME} navigator_lib)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

if(TARGET_RASPBERRY)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${NAVIGATOR_DIR}
  )

  target_link_libraries(${PROJECT_NAME}
    ${NAVIGATOR_LIB}
  )

  target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_RASPBERRY)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE TARGET_DESKTOP)
endif()

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  stonefish_ros2
)

pluginlib_export_plugin_description_file(
  hardware_interface
  sura_hardware_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES sura_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()
