// servo_interactive_marker.cpp
//
// C++ bridge node between an RViz interactive marker and MoveIt Servo, per the
// MoveIt Servo + JointGroupPositionController plan.
//
//   1. Spawns a 6-DOF interactive_markers::InteractiveMarkerServer in RViz.
//   2. Tracks the marker's pose (drag to reposition, release to command).
//   3. On release, runs a proportional (P) controller on the Cartesian error
//      between the live end-effector pose (from TF) and the marker target.
//   4. Streams geometry_msgs::msg::TwistStamped to Servo's ~/delta_twist_cmds.
//
// Why velocity streaming rather than a one-shot pose command: ros-humble
// moveit_servo does not reliably drive the arm from its pose_target_cmds topic,
// so a released target is reached by continuously re-computing a proportional
// velocity toward it -- the same input path Servo uses for live teleop. Servo
// halts on its own if it stops receiving commands (incoming_command_timeout),
// so we only stream while there is an unreached target and stop once the EE is
// within tolerance, letting the arm hold position. This mirrors the behaviour
// arrived at for the earlier Python interactive_teleop node.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace
{
double clampAbs(double value, double limit)
{
  return std::max(-limit, std::min(limit, value));
}
}  // namespace

class ServoInteractiveMarker : public rclcpp::Node
{
public:
  ServoInteractiveMarker() : Node("servo_interactive_marker")
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "link0");
    ee_frame_ = this->declare_parameter<std::string>("ee_frame", "link8");
    kp_linear_ = this->declare_parameter<double>("kp_linear", 5.0);
    kp_angular_ = this->declare_parameter<double>("kp_angular", 2.0);
    max_linear_vel_ = this->declare_parameter<double>("max_linear_vel", 0.2);
    max_angular_vel_ = this->declare_parameter<double>("max_angular_vel", 0.5);
    position_tolerance_ = this->declare_parameter<double>("position_tolerance", 0.005);
    orientation_tolerance_ = this->declare_parameter<double>("orientation_tolerance", 0.05);
    seek_timeout_ = this->declare_parameter<double>("seek_timeout", 5.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    // get_node_*_interface() are valid inside the constructor (no
    // shared_from_this needed), so the server can be built here directly.
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "servo_interactive_marker",
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_topics_interface(),
      this->get_node_services_interface());

    // 50 Hz control loop. The marker is spawned lazily on the first tick that
    // has a valid EE transform, so we don't block the constructor waiting on TF.
    timer_ = this->create_wall_timer(20ms, std::bind(&ServoInteractiveMarker::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "servo_interactive_marker started; waiting for initial TF (%s -> %s)...",
                base_frame_.c_str(), ee_frame_.c_str());
  }

private:
  bool lookupEe(geometry_msgs::msg::TransformStamped & out)
  {
    try {
      out = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void initMarker(const geometry_msgs::msg::TransformStamped & initial_tf)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = base_frame_;
    int_marker.name = kMarkerName;
    int_marker.description = "6-DOF Servo Target";
    int_marker.scale = 0.2f;

    int_marker.pose.position.x = initial_tf.transform.translation.x;
    int_marker.pose.position.y = initial_tf.transform.translation.y;
    int_marker.pose.position.z = initial_tf.transform.translation.z;
    int_marker.pose.orientation = initial_tf.transform.rotation;
    target_pose_ = int_marker.pose;

    // Always-visible box so the marker is easy to grab.
    visualization_msgs::msg::Marker box;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.scale.x = box.scale.y = box.scale.z = 0.05;
    box.color.r = 0.0;
    box.color.g = 1.0;
    box.color.b = 0.0;
    box.color.a = 0.5;

    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box);
    int_marker.controls.push_back(box_control);

    // 6-DOF: a MOVE_AXIS + ROTATE_AXIS control about each of X, Z, Y.
    addAxisControls(int_marker, 1.0, 0.0, 0.0, "x");
    addAxisControls(int_marker, 0.0, 1.0, 0.0, "z");
    addAxisControls(int_marker, 0.0, 0.0, 1.0, "y");

    server_->insert(int_marker);
    server_->setCallback(
      int_marker.name,
      std::bind(&ServoInteractiveMarker::processFeedback, this, std::placeholders::_1));
    server_->applyChanges();

    marker_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "TF received; interactive marker spawned. Drag and release to command the arm.");
  }

  void addAxisControls(visualization_msgs::msg::InteractiveMarker & int_marker,
                       double qx, double qy, double qz, const std::string & suffix)
  {
    visualization_msgs::msg::InteractiveMarkerControl rotate;
    rotate.orientation.w = 1.0;
    rotate.orientation.x = qx;
    rotate.orientation.y = qy;
    rotate.orientation.z = qz;
    rotate.name = "rotate_" + suffix;
    rotate.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate);

    visualization_msgs::msg::InteractiveMarkerControl move;
    move.orientation.w = 1.0;
    move.orientation.x = qx;
    move.orientation.y = qy;
    move.orientation.z = qz;
    move.name = "move_" + suffix;
    move.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move);
  }

  void processFeedback(
    visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
  {
    using Fb = visualization_msgs::msg::InteractiveMarkerFeedback;
    if (feedback->event_type == Fb::POSE_UPDATE) {
      target_pose_ = feedback->pose;
      is_dragging_ = true;
    } else if (feedback->event_type == Fb::MOUSE_UP) {
      // Only start commanding the arm once the target is released, so dragging
      // just repositions the marker and the robot moves to where it was left.
      is_dragging_ = false;
      pending_target_ = true;
      seek_start_time_ = this->now();
    }
  }

  void controlLoop()
  {
    geometry_msgs::msg::TransformStamped trans;
    if (!lookupEe(trans)) {
      return;
    }

    if (!marker_initialized_) {
      initMarker(trans);
      return;
    }

    if (is_dragging_) {
      // Marker is being moved live in RViz; don't command until it's released.
      return;
    }

    if (!pending_target_) {
      syncMarkerToEe(trans);
      return;
    }

    // A target was released and hasn't been reached. Give up if it takes too
    // long: the target may be outside the reachable workspace, in which case
    // the error never shrinks and we'd otherwise stream max-speed commands
    // forever.
    const double elapsed = (this->now() - seek_start_time_).seconds();
    if (elapsed > seek_timeout_) {
      RCLCPP_WARN(this->get_logger(), "Target not reached within seek_timeout; giving up (out of reach?).");
      pending_target_ = false;
      return;
    }

    const double dx = target_pose_.position.x - trans.transform.translation.x;
    const double dy = target_pose_.position.y - trans.transform.translation.y;
    const double dz = target_pose_.position.z - trans.transform.translation.z;

    // Orientation error as a rotation vector (axis * shortest angle).
    tf2::Quaternion q_target, q_current;
    tf2::fromMsg(target_pose_.orientation, q_target);
    tf2::fromMsg(trans.transform.rotation, q_current);
    tf2::Quaternion q_err = q_target * q_current.inverse();
    q_err.normalize();
    double angle = q_err.getAngle();          // [0, 2*pi)
    if (angle > M_PI) {
      angle -= 2.0 * M_PI;                     // take the shorter way around
    }
    tf2::Vector3 axis = q_err.getAxis();
    const double ex = axis.x() * angle;
    const double ey = axis.y() * angle;
    const double ez = axis.z() * angle;

    const double lin_err = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double ang_err = std::sqrt(ex * ex + ey * ey + ez * ez);
    if (lin_err <= position_tolerance_ && ang_err <= orientation_tolerance_) {
      pending_target_ = false;   // reached -- stop; Servo halts on command timeout and holds
      return;
    }

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = base_frame_;
    twist.twist.linear.x = clampAbs(kp_linear_ * dx, max_linear_vel_);
    twist.twist.linear.y = clampAbs(kp_linear_ * dy, max_linear_vel_);
    twist.twist.linear.z = clampAbs(kp_linear_ * dz, max_linear_vel_);
    twist.twist.angular.x = clampAbs(kp_angular_ * ex, max_angular_vel_);
    twist.twist.angular.y = clampAbs(kp_angular_ * ey, max_angular_vel_);
    twist.twist.angular.z = clampAbs(kp_angular_ * ez, max_angular_vel_);
    twist_pub_->publish(twist);
  }

  // Idle: snap the marker back to the real EE to prevent drift. Only push an
  // update when the pose meaningfully changed -- calling setPose/applyChanges
  // on every 50 Hz tick rebuilds the marker's clickable object in RViz
  // constantly, which makes it flicker and blocks clicking/dragging it.
  void syncMarkerToEe(const geometry_msgs::msg::TransformStamped & trans)
  {
    const auto & t = trans.transform.translation;
    const auto & r = trans.transform.rotation;
    if (synced_ &&
        std::abs(t.x - last_pos_[0]) < 1e-4 &&
        std::abs(t.y - last_pos_[1]) < 1e-4 &&
        std::abs(t.z - last_pos_[2]) < 1e-4 &&
        std::abs(r.x - last_quat_[0]) < 1e-4 &&
        std::abs(r.y - last_quat_[1]) < 1e-4 &&
        std::abs(r.z - last_quat_[2]) < 1e-4 &&
        std::abs(r.w - last_quat_[3]) < 1e-4) {
      return;
    }

    target_pose_.position.x = t.x;
    target_pose_.position.y = t.y;
    target_pose_.position.z = t.z;
    target_pose_.orientation = r;
    server_->setPose(kMarkerName, target_pose_);
    server_->applyChanges();

    last_pos_ = {t.x, t.y, t.z};
    last_quat_ = {r.x, r.y, r.z, r.w};
    synced_ = true;
  }

  static constexpr const char * kMarkerName = "servo_marker";

  // Parameters
  std::string base_frame_;
  std::string ee_frame_;
  double kp_linear_{};
  double kp_angular_{};
  double max_linear_vel_{};
  double max_angular_vel_{};
  double position_tolerance_{};
  double orientation_tolerance_{};
  double seek_timeout_{};

  // ROS interfaces
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool marker_initialized_{false};
  bool is_dragging_{false};
  bool pending_target_{false};
  bool synced_{false};
  rclcpp::Time seek_start_time_;
  geometry_msgs::msg::Pose target_pose_;
  std::array<double, 3> last_pos_{};
  std::array<double, 4> last_quat_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoInteractiveMarker>());
  rclcpp::shutdown();
  return 0;
}
