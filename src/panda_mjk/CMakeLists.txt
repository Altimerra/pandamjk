cmake_minimum_required(VERSION 3.8)
project(panda_mjk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(mujoco_ros2_control REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(franka_description REQUIRED)
find_package(joy REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Install directories
install(
  PROGRAMS scripts/interactive_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# Custom teleop node
add_executable(cartesian_teleop src/cartesian_teleop.cpp)
target_include_directories(cartesian_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(cartesian_teleop PUBLIC c_std_99 cxx_std_17)  # Require C++17
ament_target_dependencies(
  cartesian_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  control_msgs
  rclcpp_action
)

install(TARGETS
  cartesian_teleop
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
