cmake_minimum_required(VERSION 3.8)
project(panda_mjk_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mujoco_ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(franka_hardware REQUIRED)

add_library(panda_mjk_hardware SHARED
  src/mujoco_franka_system_interface.cpp
)

target_include_directories(panda_mjk_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(panda_mjk_hardware PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  panda_mjk_hardware
  rclcpp
  hardware_interface
  mujoco_ros2_control
  pluginlib
  franka_semantic_components
  franka_hardware
)

pluginlib_export_plugin_description_file(hardware_interface panda_mjk_hardware_plugins.xml)

install(TARGETS
  panda_mjk_hardware
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES panda_mjk_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
