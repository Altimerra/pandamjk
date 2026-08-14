#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStateDebugger(Node):
    def __init__(self):
        super().__init__('joint_state_debugger')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscribed to /joint_states. Waiting for messages...')

    def listener_callback(self, msg):
        self.get_logger().info(f"Received JointState message with {len(msg.name)} joints: {msg.name}")
        
        # Check size mismatches (this is exactly what robot_state_publisher checks)
        if len(msg.name) != len(msg.position):
            if len(msg.position) == 0 and len(msg.velocity) == 0 and len(msg.effort) == 0:
                self.get_logger().info("Message is OK (only names, no data).")
            else:
                self.get_logger().error(f"MISMATCH ERROR: name.size()={len(msg.name)} != position.size()={len(msg.position)}")
                return
                
        # Check for NaNs
        has_nan = False
        for i, pos in enumerate(msg.position):
            if math.isnan(pos):
                self.get_logger().error(f"NaN ERROR: position is NaN for joint: {msg.name[i]}")
                has_nan = True
                
        for i, vel in enumerate(msg.velocity):
            if math.isnan(vel):
                self.get_logger().warn(f"NaN warning: velocity is NaN for joint: {msg.name[i]}")
                
        for i, eff in enumerate(msg.effort):
            if math.isnan(eff):
                self.get_logger().warn(f"NaN warning: effort is NaN for joint: {msg.name[i]}")
                
        if not has_nan and len(msg.name) == len(msg.position):
            self.get_logger().info("Message is VALID according to robot_state_publisher checks.")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
