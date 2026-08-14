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
#include "franka/robot_state.h"
#include "panda_mjk_hardware/mujoco_franka_model.hpp"

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
  // ---- Cartesian pose / velocity interfaces ----
  std::array<double, 16> cartesian_pose_state_{};
  std::array<double, 16> cartesian_pose_command_{};
  std::array<double, 6> cartesian_velocity_command_{};

  // ---- Dynamics buffers (computed from MuJoCo, also cached in mujoco_model_) ----
  std::array<double, 49> mass_matrix_{};
  std::array<double, 7> coriolis_{};
  std::array<double, 7> gravity_{};
  std::array<double, 42> zero_jacobian_{};

  double robot_time_{0.0};

  // ---- F/T sensor data ----
  std::array<double, 6> ft_sensor_state_{};

  // ---- Simulation-side FrankaRobotModel support ----
  // MujocoFrankaModel subclasses franka_hardware::Model and overrides the
  // virtual methods to return MuJoCo-computed dynamics.  Controllers that use
  // FrankaRobotModel (e.g. model_example_controller) will invoke these
  // overrides via C++ virtual dispatch.
  MujocoFrankaModel mujoco_model_;
  franka_hardware::Model* mujoco_model_ptr_{&mujoco_model_};

  // Simulation-side RobotState populated each read() cycle with joint data
  // from MuJoCo.  FrankaRobotModel bit_casts the state interface value to
  // franka::RobotState* and passes it to Model::mass(*robot_state_) etc.
  franka::RobotState sim_robot_state_{};
  franka::RobotState* sim_robot_state_ptr_{&sim_robot_state_};

  // ---- Elbow interfaces ----
  // elbow[0] = joint 3 position (rad), elbow[1] = joint 4 flip sign (+1/-1)
  static constexpr double kElbowFlipThreshold = -0.467002423653011;
  std::array<double, 2> elbow_state_{0.0, 1.0};
  std::array<double, 2> elbow_command_{0.0, 1.0};

  // mjModel/mjData are heavy owned copies handed back by get_model()/get_data() -- cache them
  // across calls instead of re-allocating (and leaking, since MujocoSystemInterface never frees
  // a pointer we didn't pass back in) a fresh copy on every read() cycle.
  mjModel * cached_model_{nullptr};
  mjData * cached_data_{nullptr};
};

}  // namespace panda_mjk_hardware

#endif  // PANDA_MJK_HARDWARE__MUJOCO_FRANKA_SYSTEM_INTERFACE_HPP_
