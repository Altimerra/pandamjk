#include "panda_mjk_hardware/mujoco_franka_system_interface.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace panda_mjk_hardware {

MujocoFrankaSystemInterface::~MujocoFrankaSystemInterface() {
  if (cached_data_ != nullptr) {
    mj_deleteData(cached_data_);
  }
  if (cached_model_ != nullptr) {
    mj_deleteModel(cached_model_);
  }
}

std::vector<hardware_interface::StateInterface> MujocoFrankaSystemInterface::export_state_interfaces() {
  auto state_interfaces = mujoco_ros2_control::MujocoSystemInterface::export_state_interfaces();

  // Export robot_time
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "fer", "robot_time", &robot_time_));
      
  // Export robot_model and robot_state pointers
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "fer", "robot_model", &dummy_robot_model_ptr_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "fer", "robot_state", &dummy_robot_state_ptr_));

  // Export cartesian_pose_state (16 elements)
  for (size_t i = 0; i < 16; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "fer" + std::to_string(i), "cartesian_pose_state", &cartesian_pose_state_[i]));
  }

  // Export ft_sensor data (force.x, force.y, force.z, torque.x, torque.y, torque.z)
  state_interfaces.emplace_back(hardware_interface::StateInterface("fer_tcp", "force.x", &ft_sensor_state_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("fer_tcp", "force.y", &ft_sensor_state_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("fer_tcp", "force.z", &ft_sensor_state_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("fer_tcp", "torque.x", &ft_sensor_state_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("fer_tcp", "torque.y", &ft_sensor_state_[4]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("fer_tcp", "torque.z", &ft_sensor_state_[5]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoFrankaSystemInterface::export_command_interfaces() {
  auto command_interfaces = mujoco_ros2_control::MujocoSystemInterface::export_command_interfaces();

  // Export cartesian_pose_command (16 elements)
  for (size_t i = 0; i < 16; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "fer" + std::to_string(i), "cartesian_pose_command", &cartesian_pose_command_[i]));
  }

  // Export cartesian_velocity (6 elements)
  std::vector<std::string> vel_names = {"vx", "vy", "vz", "wx", "wy", "wz"};
  for (size_t i = 0; i < 6; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "fer" + vel_names[i], "cartesian_velocity", &cartesian_velocity_command_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type MujocoFrankaSystemInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period) {
  
  auto ret = mujoco_ros2_control::MujocoSystemInterface::read(time, period);
  if (ret != hardware_interface::return_type::OK) {
    return ret;
  }

  // get_model()/get_data() allocate into the pointer only when it's null, then refresh a copy
  // into the existing buffer on later calls -- reuse the cached members instead of local
  // pointers reset to nullptr every cycle, which would allocate (and leak) a fresh mjModel/
  // mjData every single read() at the control loop's rate.
  get_model(cached_model_);
  get_data(cached_data_);
  mjModel* m = cached_model_;
  mjData* d = cached_data_;

  if (m != nullptr && d != nullptr) {
    robot_time_ = d->time;

    // We need nv * nv elements for mj_fullM
    std::vector<mjtNum> M(m->nv * m->nv, 0.0);
    mj_fullM(m, M.data(), d->qM);

    // Find the DOFs for fer_joint1 through fer_joint7
    std::vector<int> dofs;
    for (int i = 1; i <= 7; ++i) {
      std::string j_name = "fer_joint" + std::to_string(i);
      int j_id = mj_name2id(m, mjOBJ_JOINT, j_name.c_str());
      if (j_id >= 0) {
        dofs.push_back(m->jnt_dofadr[j_id]);
      }
    }

    // If we found all 7 joints
    if (dofs.size() == 7) {
      // 1. Mass Matrix (column-major 7x7)
      for (size_t col = 0; col < 7; ++col) {
        for (size_t row = 0; row < 7; ++row) {
          mass_matrix_[col * 7 + row] = M[dofs[row] * m->nv + dofs[col]];
        }
      }

      // 2. Coriolis (+ gravity -- MuJoCo's qfrc_bias combines both) vector
      for (size_t i = 0; i < 7; ++i) {
        coriolis_[i] = d->qfrc_bias[dofs[i]];
      }

      // Gravity vector g(q) alone: inverse dynamics (RNE) with velocity zeroed drops the
      // Coriolis/centrifugal term, leaving just gravity. mj_rne only reads d->qvel here
      // (flg_acc=0 means d->qacc isn't used) and the body poses already cached in `d` from
      // the last physics step, so no extra mj_forward is needed.
      {
        std::vector<mjtNum> saved_qvel(d->qvel, d->qvel + m->nv);
        std::fill(d->qvel, d->qvel + m->nv, 0.0);
        std::vector<mjtNum> gravity_bias(m->nv, 0.0);
        mj_rne(m, d, 0, gravity_bias.data());
        std::copy(saved_qvel.begin(), saved_qvel.end(), d->qvel);
        for (size_t i = 0; i < 7; ++i) {
          gravity_[i] = gravity_bias[dofs[i]];
        }
      }

      // 3. Jacobian for the end effector
      int tcp_site_id = mj_name2id(m, mjOBJ_SITE, "tcp");
      if (tcp_site_id >= 0) {
        std::vector<mjtNum> jacp(3 * m->nv, 0.0);
        std::vector<mjtNum> jacr(3 * m->nv, 0.0);
        mj_jacSite(m, d, jacp.data(), jacr.data(), tcp_site_id);
        
        // Franka expects a 6x7 Jacobian, column-major
        // row 0-2: linear, row 3-5: angular
        for (size_t col = 0; col < 7; ++col) {
          zero_jacobian_[col * 6 + 0] = jacp[0 * m->nv + dofs[col]];
          zero_jacobian_[col * 6 + 1] = jacp[1 * m->nv + dofs[col]];
          zero_jacobian_[col * 6 + 2] = jacp[2 * m->nv + dofs[col]];
          zero_jacobian_[col * 6 + 3] = jacr[0 * m->nv + dofs[col]];
          zero_jacobian_[col * 6 + 4] = jacr[1 * m->nv + dofs[col]];
          zero_jacobian_[col * 6 + 5] = jacr[2 * m->nv + dofs[col]];
        }
      }
      
      // 4. Force/Torque Sensor
      int ft_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "force_sensor");
      int tq_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "torque_sensor");
      if (ft_sensor_id >= 0 && tq_sensor_id >= 0) {
        int ft_adr = m->sensor_adr[ft_sensor_id];
        int tq_adr = m->sensor_adr[tq_sensor_id];
        ft_sensor_state_[0] = d->sensordata[ft_adr + 0];
        ft_sensor_state_[1] = d->sensordata[ft_adr + 1];
        ft_sensor_state_[2] = d->sensordata[ft_adr + 2];
        ft_sensor_state_[3] = d->sensordata[tq_adr + 0];
        ft_sensor_state_[4] = d->sensordata[tq_adr + 1];
        ft_sensor_state_[5] = d->sensordata[tq_adr + 2];
      }
    }
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoFrankaSystemInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period) {
  // Real Franka FCI torque control is "on top of" the robot's own internal gravity
  // compensation: a controller's effort command is the torque *beyond* what's needed to hold
  // the arm up. Gravity compensation for the arm is handled natively by MuJoCo itself
  // (gravcomp="1" on link1..link7/hand in panda.xml cancels each body's own weight every
  // physics step) rather than here: hardware_interface::CommandInterface's copy constructor is
  // deliberately deleted ("to avoid simultaneous writes to the same resource"), so there is no
  // supported way for this class to intercept and modify the value a controller writes to the
  // base class's effort command interfaces before the base class's write() reads it.

  // Compute IK for Cartesian commands if they are active, then pass to base write

  return mujoco_ros2_control::MujocoSystemInterface::write(time, period);
}

}  // namespace panda_mjk_hardware

PLUGINLIB_EXPORT_CLASS(panda_mjk_hardware::MujocoFrankaSystemInterface, hardware_interface::SystemInterface)
