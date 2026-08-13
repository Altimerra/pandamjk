#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class CartesianTeleop : public rclcpp::Node {
public:
  CartesianTeleop() : Node("cartesian_teleop") {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&CartesianTeleop::joy_callback, this, std::placeholders::_1));

    gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      this, "/panda_gripper_controller/gripper_cmd"); // Change to real hardware topic if use_sim=false

    RCLCPP_INFO(this->get_logger(), "Cartesian Teleop Node Started.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() < 6 || msg->buttons.size() < 6) {
      RCLCPP_WARN(this->get_logger(), "Joy message has insufficient axes/buttons.");
      return;
    }

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "fer_link0";

    // Soft brake (B button - typically index 1)
    if (msg->buttons[1] == 1) {
      // Publish zero twist
      twist_pub_->publish(twist_msg);
      return;
    }

    // D-Pad for Master Speed Scaling (axes[6] or axes[7] depending on driver)
    // Here we'll just implement a simple fixed scale for demonstration
    double speed_scale = 1.0; 

    // ---- Translation (Left Hand) ----
    // Left Stick: Up/Down -> X, Left/Right -> Y
    twist_msg.twist.linear.x = msg->axes[1] * speed_scale;
    twist_msg.twist.linear.y = msg->axes[0] * speed_scale;

    // Left Trigger: Analog throttle (usually axes 2). Ranges from 1.0 (unpressed) to -1.0 (pressed).
    // Convert to 0.0 to 1.0 range:
    double left_trigger = (1.0 - msg->axes[2]) / 2.0;
    
    // Left Bumper: Direction Modifier (typically button 4)
    if (msg->buttons[4] == 1) {
      twist_msg.twist.linear.z = -left_trigger * speed_scale; // DOWN
    } else {
      twist_msg.twist.linear.z = left_trigger * speed_scale;  // UP
    }

    // ---- Orientation (Right Hand) ----
    // Right Stick: Up/Down -> Pitch (Y axis rotation), Left/Right -> Yaw (Z axis rotation)
    // Note: Axis mapping depends on exactly how we define tool frame relative to world frame.
    // Assuming standard:
    twist_msg.twist.angular.y = msg->axes[4] * speed_scale; // Pitch
    twist_msg.twist.angular.z = msg->axes[3] * speed_scale; // Yaw

    // Right Trigger: Roll Analog throttle (usually axes 5).
    double right_trigger = (1.0 - msg->axes[5]) / 2.0;

    // Right Bumper: Direction Modifier (typically button 5)
    if (msg->buttons[5] == 1) {
      twist_msg.twist.angular.x = -right_trigger * speed_scale; // Roll CCW
    } else {
      twist_msg.twist.angular.x = right_trigger * speed_scale;  // Roll CW
    }

    // ---- Null-Space / Elbow Clutch (Right Stick L/R while holding X button (index 2)) ----
    if (msg->buttons[2] == 1) {
      // If we had a delta_joint_cmds publisher, we would publish to joint7 here.
      // For now, we nullify cartesian yaw since Right Stick L/R is being used for elbow.
      twist_msg.twist.angular.z = 0.0;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Elbow clutch active (Joint 7)");
    }

    // Publish the Cartesian Twist
    twist_pub_->publish(twist_msg);

    // ---- Gripper Control (Face Buttons A and Y) ----
    // A button (index 0) = Close, Y button (index 3) = Open
    if (msg->buttons[0] == 1 || msg->buttons[3] == 1) {
      handle_gripper(msg->buttons[3] == 1); // True if Open
    }
  }

  void handle_gripper(bool open) {
    if (!gripper_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Gripper action server not available");
      return;
    }

    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = open ? 0.08 : 0.0; // 8cm open, 0cm closed
    goal_msg.command.max_effort = 20.0;

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    gripper_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianTeleop>());
  rclcpp::shutdown();
  return 0;
}
