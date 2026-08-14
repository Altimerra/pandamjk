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
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install directories
install(
  DIRECTORY launch config urdf scripts
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/collision_behavior_shim.py
  scripts/gripper_sim_shim.py
  DESTINATION lib/${PROJECT_NAME}
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

# Interactive marker -> MoveIt Servo bridge node
add_executable(servo_interactive_marker src/servo_interactive_marker.cpp)
target_compile_features(servo_interactive_marker PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  servo_interactive_marker
  rclcpp
  geometry_msgs
  visualization_msgs
  interactive_markers
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  servo_interactive_marker
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
