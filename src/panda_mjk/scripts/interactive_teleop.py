#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import numpy as np
from scipy.spatial.transform import Rotation as R

class VisualTeleop(Node):
    def __init__(self):
        super().__init__('visual_teleop')

        # Parameters
        self.declare_parameter('base_frame', 'link0')
        self.declare_parameter('ee_frame', 'link8')
        self.declare_parameter('kp_linear', 5.0)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 0.5)

        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for MoveIt Servo
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Interactive Marker Server
        self.server = InteractiveMarkerServer(self, 'teleop_target')
        self.target_pose = None
        self.is_dragging = False
        
        # Wait for initial TF
        self.get_logger().info("Waiting for initial TF...")
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
                break
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
                
        self.get_logger().info("TF received. Spawning marker...")

        # Setup Marker
        self.init_marker(trans)

        # Control Loop
        self.timer = self.create_timer(0.02, self.control_loop) # 50 Hz

    def init_marker(self, initial_tf):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.base_frame
        int_marker.name = "teleop_marker"
        int_marker.description = "6-DOF Teleop Target"
        int_marker.scale = 0.2

        int_marker.pose.position.x = initial_tf.transform.translation.x
        int_marker.pose.position.y = initial_tf.transform.translation.y
        int_marker.pose.position.z = initial_tf.transform.translation.z
        int_marker.pose.orientation = initial_tf.transform.rotation

        self.target_pose = int_marker.pose

        # Add a visual box
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.05
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.5

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # 6-DOF controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        from interactive_markers.interactive_marker_server import InteractiveMarkerFeedback
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.target_pose = feedback.pose
            self.is_dragging = True
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # When released, we could snap it back, but let's leave it so it finishes servoing.
            self.is_dragging = False

    def control_loop(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
        except Exception:
            return

        if not self.is_dragging:
            # If not dragging, smoothly snap the marker back to the real EE to prevent drift
            self.target_pose.position.x = trans.transform.translation.x
            self.target_pose.position.y = trans.transform.translation.y
            self.target_pose.position.z = trans.transform.translation.z
            self.target_pose.orientation = trans.transform.rotation
            self.server.setPose("teleop_marker", self.target_pose)
            self.server.applyChanges()
            return

        # Calculate error
        dx = self.target_pose.position.x - trans.transform.translation.x
        dy = self.target_pose.position.y - trans.transform.translation.y
        dz = self.target_pose.position.z - trans.transform.translation.z

        q_target = R.from_quat([self.target_pose.orientation.x, self.target_pose.orientation.y, 
                                self.target_pose.orientation.z, self.target_pose.orientation.w])
        q_current = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, 
                                 trans.transform.rotation.z, trans.transform.rotation.w])
        
        # Angular error (rotation vector)
        error_r = (q_target * q_current.inv()).as_rotvec()

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame

        # Proportional control with limits
        twist.twist.linear.x = np.clip(self.kp_linear * dx, -self.max_linear_vel, self.max_linear_vel)
        twist.twist.linear.y = np.clip(self.kp_linear * dy, -self.max_linear_vel, self.max_linear_vel)
        twist.twist.linear.z = np.clip(self.kp_linear * dz, -self.max_linear_vel, self.max_linear_vel)

        twist.twist.angular.x = np.clip(self.kp_angular * error_r[0], -self.max_angular_vel, self.max_angular_vel)
        twist.twist.angular.y = np.clip(self.kp_angular * error_r[1], -self.max_angular_vel, self.max_angular_vel)
        twist.twist.angular.z = np.clip(self.kp_angular * error_r[2], -self.max_angular_vel, self.max_angular_vel)

        # Deadband to avoid jitter
        if np.linalg.norm([dx, dy, dz]) > 0.005 or np.linalg.norm(error_r) > 0.05:
            self.twist_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VisualTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
