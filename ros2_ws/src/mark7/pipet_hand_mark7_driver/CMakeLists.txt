cmake_minimum_required(VERSION 3.8)
project(pipet_hand_mark7_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pipet_hand_mark7_msgs REQUIRED)

# serial_port 라이브러리 (POSIX only, ROS 의존성 없음)
add_library(mark7_serial SHARED
  src/serial_port.cpp
)
target_include_directories(mark7_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mark7_serial PUBLIC cxx_std_17)

install(TARGETS mark7_serial
  EXPORT export_mark7_serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_mark7_serial HAS_LIBRARY_TARGET)

# Hardware Interface 라이브러리 (ros2_control 플러그인)
add_library(mark7_hardware SHARED
  src/mark7_system_hardware.cpp
)
target_include_directories(mark7_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mark7_hardware PUBLIC cxx_std_17)
target_link_libraries(mark7_hardware mark7_serial)
ament_target_dependencies(mark7_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  pipet_hand_mark7_msgs
)

install(TARGETS mark7_hardware
  EXPORT export_mark7_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_mark7_hardware HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

pluginlib_export_plugin_description_file(hardware_interface mark7_hardware_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
