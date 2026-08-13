#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import sys

def main():
    rclpy.init()
    node = Node('servo_pose_tester')
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    pub = node.create_publisher(PoseStamped, '/servo_node/pose_target_cmds', 10)
    
    node.get_logger().info("Waiting for TF link0 -> link8...")
    
    target_sent = False
    for i in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            trans = tf_buffer.lookup_transform('link0', 'link8', rclpy.time.Time())
            
            target = PoseStamped()
            target.header.frame_id = 'link0'
            target.header.stamp = node.get_clock().now().to_msg()
            
            # Command it to move 5cm in +X
            target.pose.position.x = trans.transform.translation.x + 0.05
            target.pose.position.y = trans.transform.translation.y
            target.pose.position.z = trans.transform.translation.z
            target.pose.orientation = trans.transform.rotation
            
            node.get_logger().info(f"Publishing target pose:\n{target.pose.position}")
            pub.publish(target)
            target_sent = True
            break
        except Exception as e:
            if i % 10 == 0:
                node.get_logger().info(f"Still waiting for TF... ({e})")
                
    if target_sent:
        # allow it to publish
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        node.get_logger().error("Failed to get TF and publish.")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
