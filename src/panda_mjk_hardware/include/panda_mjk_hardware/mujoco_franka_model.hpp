// Copyright (c) 2024 PandaMJK Contributors
// Licensed under the Apache License, Version 2.0
#ifndef PANDA_MJK_HARDWARE__MUJOCO_FRANKA_MODEL_HPP_
#define PANDA_MJK_HARDWARE__MUJOCO_FRANKA_MODEL_HPP_

#include <array>
#include <cmath>
#include <cstring>

#include "franka_hardware/model.hpp"
#include "franka/model.h"
#include "franka/robot_state.h"

namespace panda_mjk_hardware {

/**
 * Simulation-side subclass of franka_hardware::Model that returns dynamics
 * computed by MuJoCo instead of delegating to the real libfranka model.
 *
 * franka_hardware::Model exposes virtual overloads of mass(), coriolis(),
 * gravity(), pose(), bodyJacobian(), and zeroJacobian() that take a
 * const franka::RobotState&.  FrankaRobotModel (the semantic component used
 * by controllers) calls exactly these overloads.  By overriding them here we
 * can return MuJoCo-computed quantities without touching franka_ros2.
 *
 * The MujocoFrankaSystemInterface owns this object and refreshes the cached
 * dynamics buffers every read() cycle; the overrides simply return copies of
 * those buffers.
 */
class MujocoFrankaModel : public franka_hardware::Model {
 public:
  MujocoFrankaModel() : franka_hardware::Model() {}
  ~MujocoFrankaModel() override = default;

  // ---- Dynamics buffers set by MujocoFrankaSystemInterface::read() ----

  /// 7x7 mass matrix, column-major.  Written by the hardware interface.
  std::array<double, 49> cached_mass{};

  /// Coriolis + gravity bias vector (7).  Written by the hardware interface.
  std::array<double, 7> cached_coriolis{};

  /// Pure gravity vector (7).  Written by the hardware interface.
  std::array<double, 7> cached_gravity{};

  /// 6x7 zero (base-frame) Jacobian at the TCP, column-major.
  std::array<double, 42> cached_zero_jacobian{};

  /// Per-frame FK poses.  Index is static_cast<int>(franka::Frame).
  /// Frames: kJoint1..kJoint7 (0-6), kFlange (7), kEndEffector (8), kStiffness (9).
  std::array<std::array<double, 16>, 10> cached_frame_poses{};

  // ---- Virtual overrides ----

  [[nodiscard]] std::array<double, 49> mass(
      const franka::RobotState& /*robot_state*/) const override {
    return cached_mass;
  }

  [[nodiscard]] std::array<double, 7> coriolis(
      const franka::RobotState& /*robot_state*/) const override {
    return cached_coriolis;
  }

  [[nodiscard]] std::array<double, 7> gravity(
      const franka::RobotState& /*robot_state*/) const override {
    return cached_gravity;
  }

  [[nodiscard]] std::array<double, 7> gravity(
      const franka::RobotState& /*robot_state*/,
      const std::array<double, 3>& /*gravity_earth*/) const override {
    // MuJoCo gravity is always -9.81 m/s^2 in z; ignore the argument and
    // return the cached value (which already reflects MuJoCo's gravity).
    return cached_gravity;
  }

  [[nodiscard]] std::array<double, 16> pose(
      franka::Frame frame,
      const franka::RobotState& /*robot_state*/) const override {
    int idx = static_cast<int>(frame);
    if (idx >= 0 && idx < static_cast<int>(cached_frame_poses.size())) {
      return cached_frame_poses[idx];
    }
    // Fallback: identity
    return {1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1};
  }

  [[nodiscard]] std::array<double, 42> zeroJacobian(
      franka::Frame frame,
      const franka::RobotState& /*robot_state*/) const override {
    // We only compute the TCP Jacobian from MuJoCo.  For kEndEffector /
    // kFlange / kStiffness this is a good approximation (TCP ≈ EE for the
    // Panda).  For intermediate frames we'd need per-frame Jacobians, which
    // MuJoCo can compute but we don't cache yet — return the TCP Jacobian as
    // a best-effort value.
    (void)frame;
    return cached_zero_jacobian;
  }

  [[nodiscard]] std::array<double, 42> bodyJacobian(
      franka::Frame frame,
      const franka::RobotState& /*robot_state*/) const override {
    // J_body = Ad(T_frame^{-1}) * J_zero
    // For a rigid body: Ad(T^{-1}) transforms a 6D spatial vector from the
    // base frame to the body frame.

    int idx = static_cast<int>(frame);
    std::array<double, 16> T;
    if (idx >= 0 && idx < static_cast<int>(cached_frame_poses.size())) {
      T = cached_frame_poses[idx];
    } else {
      T = {1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1};
    }

    // T is 4x4 column-major.  Extract R (3x3 rotation) and p (3x1 position).
    // T = [R p; 0 1], column-major: T[0..2]=col0, T[4..6]=col1, T[8..10]=col2, T[12..14]=p
    // R^T (transpose of R)
    double Rt[9];  // row-major for convenience
    Rt[0] = T[0]; Rt[1] = T[4]; Rt[2] = T[8];
    Rt[3] = T[1]; Rt[4] = T[5]; Rt[5] = T[9];
    Rt[6] = T[2]; Rt[7] = T[6]; Rt[8] = T[10];

    double px = T[12], py = T[13], pz = T[14];

    // skew(p) = [  0  -pz  py ]
    //           [  pz  0  -px ]
    //           [ -py  px  0  ]
    // -R^T * skew(p) computed element-wise:
    double neg_Rt_skew[9];
    // skew(p) columns: s0=[0, pz, -py], s1=[-pz, 0, px], s2=[py, -px, 0]
    // -R^T * skew(p): column j = -R^T * skew_col_j
    neg_Rt_skew[0] = -(Rt[0]*0    + Rt[1]*pz   + Rt[2]*(-py));
    neg_Rt_skew[3] = -(Rt[3]*0    + Rt[4]*pz   + Rt[5]*(-py));
    neg_Rt_skew[6] = -(Rt[6]*0    + Rt[7]*pz   + Rt[8]*(-py));

    neg_Rt_skew[1] = -(Rt[0]*(-pz) + Rt[1]*0   + Rt[2]*px);
    neg_Rt_skew[4] = -(Rt[3]*(-pz) + Rt[4]*0   + Rt[5]*px);
    neg_Rt_skew[7] = -(Rt[6]*(-pz) + Rt[7]*0   + Rt[8]*px);

    neg_Rt_skew[2] = -(Rt[0]*py   + Rt[1]*(-px) + Rt[2]*0);
    neg_Rt_skew[5] = -(Rt[3]*py   + Rt[4]*(-px) + Rt[5]*0);
    neg_Rt_skew[8] = -(Rt[6]*py   + Rt[7]*(-px) + Rt[8]*0);

    // Ad(T^{-1}) = [ R^T         -R^T*skew(p) ]   (6x6)
    //              [  0              R^T        ]
    //
    // J_body (6x7) = Ad(T^{-1}) * J_zero (6x7)
    // J_zero is column-major 6x7: column j at offset j*6.
    // Rows 0-2 = linear part, rows 3-5 = angular part.

    const auto& Jz = cached_zero_jacobian;
    std::array<double, 42> Jb{};

    for (int j = 0; j < 7; ++j) {
      double v_lin[3] = {Jz[j*6+0], Jz[j*6+1], Jz[j*6+2]};
      double v_ang[3] = {Jz[j*6+3], Jz[j*6+4], Jz[j*6+5]};

      // body linear = R^T * lin + (-R^T*skew(p)) * ang
      for (int r = 0; r < 3; ++r) {
        Jb[j*6 + r] = Rt[r*3+0]*v_lin[0] + Rt[r*3+1]*v_lin[1] + Rt[r*3+2]*v_lin[2]
                     + neg_Rt_skew[r*3+0]*v_ang[0] + neg_Rt_skew[r*3+1]*v_ang[1] + neg_Rt_skew[r*3+2]*v_ang[2];
      }
      // body angular = R^T * ang
      for (int r = 0; r < 3; ++r) {
        Jb[j*6 + 3 + r] = Rt[r*3+0]*v_ang[0] + Rt[r*3+1]*v_ang[1] + Rt[r*3+2]*v_ang[2];
      }
    }

    return Jb;
  }
};

}  // namespace panda_mjk_hardware

#endif  // PANDA_MJK_HARDWARE__MUJOCO_FRANKA_MODEL_HPP_
