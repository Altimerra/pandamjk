#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetFullCollisionBehavior

class CollisionBehaviorShim(Node):
    def __init__(self):
        super().__init__('collision_behavior_shim')
        self.srv = self.create_service(SetFullCollisionBehavior, '/service_server/set_full_collision_behavior', self.handle_service)
        self.get_logger().info('Collision Behavior Shim is ready.')

    def handle_service(self, request, response):
        self.get_logger().info('Received SetFullCollisionBehavior request, ignoring in simulation.')
        # Assuming the service response needs some fields populated (check actual definition)
        # We'll just return an empty response which is usually enough for a bool success field if it has one.
        # Note: If the actual franka_msgs response has specific fields, you must set them here.
        return response

def main(args=None):
    rclpy.init(args=args)
    shim = CollisionBehaviorShim()
    rclpy.spin(shim)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
