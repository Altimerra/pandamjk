#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStateDebugger(Node):
    def __init__(self):
        super().__init__('debug_joint_states')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        publishers = self.get_publishers_info_by_topic('/joint_states')
        for pub in publishers:
            self.get_logger().info(f"Publisher: {pub.node_name()} from node namespace {pub.node_namespace()}")
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscribed to /joint_states. Waiting for messages...')

    def listener_callback(self, msg):
        if len(msg.position) == 10:
            self.get_logger().error(f"*** FOUND MESSAGE WITH 10 POSITIONS! ***")
            self.get_logger().error(f"Names: {msg.name}")
            self.get_logger().info(f"Positions: {msg.position}")
            self.get_logger().info(f"Velocities: {msg.velocity}")
            self.get_logger().info(f"Efforts: {msg.effort}")
        
        # Check size mismatches (this is exactly what robot_state_publisher checks)
        if len(msg.name) != len(msg.position):
            if len(msg.position) == 0 and len(msg.velocity) == 0 and len(msg.effort) == 0:
                pass # Message is OK
            else:
                self.get_logger().error(f"MISMATCH ERROR: name.size()={len(msg.name)} != position.size()={len(msg.position)}")

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
