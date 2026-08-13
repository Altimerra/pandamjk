#ifndef PANDA_MJK_HARDWARE__MUJOCO_FRANKA_SYSTEM_INTERFACE_HPP_
#define PANDA_MJK_HARDWARE__MUJOCO_FRANKA_SYSTEM_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <array>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mujoco_ros2_control/mujoco_system_interface.hpp"

namespace panda_mjk_hardware {

class MujocoFrankaSystemInterface : public mujoco_ros2_control::MujocoSystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocoFrankaSystemInterface)

  ~MujocoFrankaSystemInterface() override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Dummy buffers for Franka specific GPIOs
  std::array<double, 16> cartesian_pose_state_{};
  std::array<double, 16> cartesian_pose_command_{};

  std::array<double, 6> cartesian_velocity_command_{};

  std::array<double, 49> mass_matrix_{};
  std::array<double, 7> coriolis_{};
  std::array<double, 7> gravity_{};
  std::array<double, 42> zero_jacobian_{};

  double robot_time_{0.0};

  // F/T sensor data
  std::array<double, 6> ft_sensor_state_{};

  // Dummy pointers for franka_semantic_components bit_cast. franka_semantic_components::
  // FrankaRobotModel reinterprets these as live franka_hardware::Model* / franka::RobotState*
  // and calls real methods on them (e.g. robot_model_->mass(*robot_state_)) -- there is no
  // sim-side object we can point them at without a real libfranka model, so they stay null.
  // Nothing in this package's controllers.yaml uses FrankaRobotModel: joint_impedance/
  // move_to_start work off plain position+velocity state and effort commands, and
  // gravity_compensation uses the F/T sensor GPIO below, not this. A controller that does
  // need it (e.g. model_example_controller) would segfault dereferencing these; don't spawn one.
  double dummy_robot_model_ptr_{0.0};
  double dummy_robot_state_ptr_{0.0};

  // mjModel/mjData are heavy owned copies handed back by get_model()/get_data() -- cache them
  // across calls instead of re-allocating (and leaking, since MujocoSystemInterface never frees
  // a pointer we didn't pass back in) a fresh copy on every read() cycle.
  mjModel * cached_model_{nullptr};
  mjData * cached_data_{nullptr};
};

}  // namespace panda_mjk_hardware

#endif  // PANDA_MJK_HARDWARE__MUJOCO_FRANKA_SYSTEM_INTERFACE_HPP_
