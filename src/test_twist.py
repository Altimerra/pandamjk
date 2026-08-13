#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TwistTester(Node):
    def __init__(self):
        super().__init__('test_twist_node')
        
        # use_sim_time set via CLI
        
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        # Publish at 50 Hz
        self.timer_ = self.create_timer(0.02, self.timer_callback)
        self.start_time_ = self.get_clock().now()
        
    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time_).nanoseconds / 1e9
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link0'
        
        if elapsed < 2.0:
            # First 2 seconds: move +X
            msg.twist.linear.x = 0.05
            self.get_logger().info('Moving +X')
        elif elapsed < 4.0:
            # Next 2 seconds: move -X
            msg.twist.linear.x = -0.05
            self.get_logger().info('Moving -X')
        else:
            # Stop
            msg.twist.linear.x = 0.0
            self.get_logger().info('Stopping')
            
        self.publisher_.publish(msg)
        
        if elapsed > 4.5:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TwistTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
